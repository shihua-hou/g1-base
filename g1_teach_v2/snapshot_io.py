"""Snapshot capture, persistence, and joint-space transition helpers.

Snapshots still save only the target joint angles, but snapshot capture itself
now follows a small teaching flow: soften -> hand-guide -> save -> release.
"""

import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from .joints import UPPER_JOINTS, WAIST_SUPPORT_JOINTS, get_group_joints, with_waist_support
from .profiles import get_playback_profile, get_snapshot_capture_profile
from .takeover_debug import create_takeover_debug_state, emit_takeover_debug_snapshot, emit_takeover_debug_tick


@dataclass
class Snapshot:
    path: Path
    group: str
    joints: list
    q: list
    meta: dict
    tau_est: list | None = None


def _lerp_vector(a, b, t):
    return [(1.0 - t) * float(x) + t * float(y) for x, y in zip(a, b)]


def _lerp_scalar(a, b, t):
    return (1.0 - t) * float(a) + t * float(b)


def _normalize_snapshot_easing(easing):
    if easing is None:
        return "minimum_jerk"

    value = str(easing).strip().lower()
    aliases = {
        "min_jerk": "minimum_jerk",
        "minimum-jerk": "minimum_jerk",
        "minjerk": "minimum_jerk",
        "quint": "quintic",
        "poly5": "quintic",
    }
    value = aliases.get(value, value)
    allowed = {"linear", "smoothstep", "minimum_jerk", "quintic"}
    if value not in allowed:
        raise ValueError(f"unsupported snapshot easing: {easing}")
    return value


def _apply_snapshot_easing(raw, easing):
    easing = _normalize_snapshot_easing(easing)
    if easing == "linear":
        return raw
    if easing == "smoothstep":
        return raw * raw * (3.0 - 2.0 * raw)
    # Minimum-jerk trajectory: zero velocity/acceleration at both ends and
    # smoother progress through the middle than plain linear interpolation.
    return raw * raw * raw * (10.0 - 15.0 * raw + 6.0 * raw * raw)


def _compute_snapshot_trajectory_point(q_start, q_target, raw, duration, mode):
    mode = _normalize_snapshot_easing(mode)
    alpha = _apply_snapshot_easing(raw, "minimum_jerk" if mode == "quintic" else mode)

    q_cmd = _lerp_vector(q_start, q_target, alpha)
    dq_cmd = [0.0] * len(q_cmd)
    return q_cmd, dq_cmd, alpha


def _attach_partial_result(exc, joints, q_final, kp, kd, control_dt):
    """Stash the latest commanded pose on an in-flight exception.

    kp/kd may be scalars or per-joint gain vectors; both are stored as-is so
    the hold step in script_runner uses the correct per-joint stiffness.
    """
    exc.partial_result = {
        "joints": list(joints),
        "q_final": list(q_final),
        "kp": kp,
        "kd": kd,
        "control_dt": control_dt,
    }
    return exc


def _build_snapshot_command_context(session, snapshot, source_q, source_result=None):
    """Expand partial upper-body snapshots to the full upper command set."""
    target_joints = [int(joint) for joint in snapshot.joints]
    upper_joints = list(UPPER_JOINTS)
    arm_target_joints = [joint for joint in target_joints if joint in upper_joints]
    command_joints = list(target_joints)
    source_joints_for_mapping = list(target_joints)
    if arm_target_joints:
        command_joints = with_waist_support(upper_joints if target_joints != upper_joints else target_joints)
        source_joints_for_mapping = arm_target_joints

    if command_joints == target_joints and source_joints_for_mapping == target_joints:
        return command_joints, list(source_q), None

    hold_q = None
    if source_result is not None:
        result_joints = [int(v) for v in source_result["joints"]]
        if result_joints == command_joints:
            hold_q = [float(v) for v in source_result["q_final"]]

    if hold_q is None:
        hold_q = list(session.get_joint_q(command_joints))

    joint_to_command_idx = {int(joint): idx for idx, joint in enumerate(command_joints)}
    source_to_command_indices = [
        (source_idx, joint_to_command_idx[int(joint)])
        for source_idx, joint in enumerate(target_joints)
        if int(joint) in joint_to_command_idx
        and int(joint) in source_joints_for_mapping
    ]

    command_source_q = list(hold_q)
    for source_idx, command_idx in source_to_command_indices:
        command_source_q[command_idx] = float(source_q[source_idx])

    return command_joints, command_source_q, (hold_q, source_to_command_indices)


def _expand_snapshot_command_q(target_q, hold_context):
    if hold_context is None:
        return list(target_q)

    hold_q, source_to_command_indices = hold_context
    q_cmd = list(hold_q)
    for source_idx, command_idx in source_to_command_indices:
        q_cmd[command_idx] = float(target_q[source_idx])
    return q_cmd


def _expand_snapshot_command_dq(target_dq, hold_context):
    if hold_context is None:
        return list(target_dq)

    hold_q, source_to_command_indices = hold_context
    dq_cmd = [0.0] * len(hold_q)
    for source_idx, command_idx in source_to_command_indices:
        dq_cmd[command_idx] = float(target_dq[source_idx])
    return dq_cmd


def _expand_snapshot_command_tau(target_tau, hold_context, command_count):
    if target_tau is None:
        return None
    if hold_context is None:
        return [float(value) for value in target_tau]

    tau_cmd = [0.0] * command_count
    _hold_q, source_to_command_indices = hold_context
    for source_idx, command_idx in source_to_command_indices:
        tau_cmd[command_idx] = float(target_tau[source_idx])
    return tau_cmd


def _add_tau_ff(base, extra):
    if extra is None:
        return base
    if isinstance(base, (int, float)):
        return [float(base) + float(value) for value in extra]
    return [float(a) + float(b) for a, b in zip(base, extra)]


def _scale_vector(values, scale):
    return [float(value) * float(scale) for value in values]


def _resolve_takeover_support(step_idx, fade_steps, tau_seed, takeover_kp, takeover_kd, kp, kd):
    if step_idx is None or fade_steps <= 0 or tau_seed is None:
        return kp, kd, 0.0, None

    fade = 1.0 if fade_steps == 1 else min(1.0, float(step_idx) / float(fade_steps - 1))
    zero_tau = [0.0] * len(tau_seed)
    return (
        _lerp_scalar(takeover_kp, kp, fade),
        _lerp_scalar(takeover_kd, kd, fade),
        _lerp_vector(tau_seed, zero_tau, fade),
        fade,
    )


def _support_local_indices(command_joints):
    return [idx for idx, joint in enumerate(command_joints) if int(joint) in WAIST_SUPPORT_JOINTS]


def _build_joint_gain_vector(command_joints, base_gain, support_gain):
    support_idx = _support_local_indices(command_joints)
    if not support_idx:
        return base_gain
    values = [float(base_gain)] * len(command_joints)
    for idx in support_idx:
        values[idx] = float(support_gain)
    return values


def _resolve_support_axis_gain(base_gain, takeover_gain, fade):
    if fade is None:
        return float(base_gain)
    return _lerp_scalar(float(takeover_gain), float(base_gain), float(fade))


def save_snapshot(path, group, joints, q, source="capture-snapshot", tau_est=None):
    """Save a simplified snapshot schema and return the reloaded object."""
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "kind": "snapshot",
        "version": 1,
        "group": group,
        "joints": [int(v) for v in joints],
        "q": [float(v) for v in q],
        "meta": {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "source": source,
        },
    }
    if tau_est is not None:
        payload["tau_est"] = [float(v) for v in tau_est]
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    return load_snapshot(path)


def load_snapshot(path):
    """Load and validate a snapshot json file."""
    path = Path(path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"snapshot file not found: {path}")

    with path.open("r", encoding="utf-8-sig") as handle:
        payload = json.load(handle)

    if payload.get("kind") != "snapshot":
        raise ValueError(f"snapshot kind must be 'snapshot': {path}")
    if int(payload.get("version", 0)) != 1:
        raise ValueError(f"unsupported snapshot version in {path}")

    group = str(payload["group"])
    joints = [int(v) for v in payload["joints"]]
    q = [float(v) for v in payload["q"]]
    if len(joints) != len(q):
        raise ValueError(f"snapshot q length mismatch in {path}")

    meta = payload.get("meta", {})
    tau_est = None
    if "tau_est" in payload:
        try:
            tau_values = [float(v) for v in payload["tau_est"]]
        except (TypeError, ValueError):
            print(f"[SNAPSHOT] warning: ignoring invalid tau_est in {path}")
        else:
            if len(tau_values) != len(joints):
                print(f"[SNAPSHOT] warning: ignoring tau_est length mismatch in {path}")
            else:
                tau_est = tau_values

    return Snapshot(path=path, group=group, joints=joints, q=q, meta=meta, tau_est=tau_est)


def describe_snapshot(snapshot, duration=1.5, profile_name="upper_hold", easing=None):
    summary = (
        f"snapshot={snapshot.path.name} group={snapshot.group} joints={len(snapshot.joints)} "
        f"duration={duration:.2f}s profile={profile_name}"
    )
    if easing is not None:
        summary += f" easing={_normalize_snapshot_easing(easing)}"
    return summary


def _wait_for_snapshot_confirm(stop_event):
    """Block in a helper thread until the operator confirms the current pose."""
    try:
        input("[SNAPSHOT] hand-guide the arm to the target pose, then press ENTER to save...")
    finally:
        stop_event.set()


def capture_snapshot(session, out_path, group="upper"):
    """Soften the selected joints, let the operator move them by hand, then save."""
    joints = get_group_joints(group)
    command_joints = with_waist_support(joints) if group in {"left", "right", "both", "upper"} else list(joints)
    support_local_idx = [idx for idx, joint in enumerate(command_joints) if joint in WAIST_SUPPORT_JOINTS]
    target_local_idx = [idx for idx, joint in enumerate(command_joints) if joint in joints]
    profile = get_snapshot_capture_profile()
    hold_time = float(profile["hold_time"])
    blend_time = float(profile["blend_time"])
    control_dt = float(profile["control_dt"])
    kp_hold = float(profile["kp_hold"])
    kd_hold = float(profile["kd_hold"])
    kp_soft = float(profile["kp_soft"])
    kd_soft = float(profile["kd_soft"])
    support_kp = float(profile.get("waist_support_kp", 30.0))
    support_kd = float(profile.get("waist_support_kd", 1.0))

    session.wait_lowstate()
    q_hold = list(session.get_joint_q(command_joints))
    q_last = list(q_hold)

    print(f"[SNAPSHOT] capture group={group} joints={len(joints)}")
    print("[SNAPSHOT] phase 1: hold current pose")
    print("[SNAPSHOT] phase 2: soften arm for hand-guiding")

    try:
        hold_steps = max(1, int(hold_time / control_dt))
        for _ in range(hold_steps):
            kp_cmd = [support_kp if idx in support_local_idx else kp_hold for idx in range(len(command_joints))]
            kd_cmd = [support_kd if idx in support_local_idx else kd_hold for idx in range(len(command_joints))]
            session.send_arm_q(command_joints, q_hold, kp=kp_cmd, kd=kd_cmd)
            q_last = list(q_hold)
            time.sleep(control_dt)

        blend_steps = max(1, int(blend_time / control_dt))
        for step in range(blend_steps):
            raw = (step + 1) / blend_steps
            s = raw * raw
            q_now = session.get_joint_q(command_joints)
            q_cmd = _lerp_vector(q_hold, q_now, s)
            kp = _lerp_scalar(kp_hold, kp_soft, s)
            kd = _lerp_scalar(kd_hold, kd_soft, s)
            for idx in support_local_idx:
                q_cmd[idx] = q_hold[idx]
            kp_cmd = [support_kp if idx in support_local_idx else kp for idx in range(len(command_joints))]
            kd_cmd = [support_kd if idx in support_local_idx else kd for idx in range(len(command_joints))]
            session.send_arm_q(command_joints, q_cmd, kp=kp_cmd, kd=kd_cmd)
            q_last = list(q_cmd)
            time.sleep(control_dt)

        stop_event = threading.Event()
        input_thread = threading.Thread(target=_wait_for_snapshot_confirm, args=(stop_event,), daemon=True)
        input_thread.start()

        while not stop_event.is_set():
            # Keep refreshing q_cmd to the current measured pose so the operator can
            # freely hand-guide the arm while the controller stays soft and engaged.
            q_now = session.get_joint_q(command_joints)
            for idx in support_local_idx:
                q_now[idx] = q_hold[idx]
            kp_cmd = [support_kp if idx in support_local_idx else kp_soft for idx in range(len(command_joints))]
            kd_cmd = [support_kd if idx in support_local_idx else kd_soft for idx in range(len(command_joints))]
            session.send_arm_q(command_joints, q_now, kp=kp_cmd, kd=kd_cmd)
            q_last = list(q_now)
            time.sleep(control_dt)

        q_save_full = session.get_joint_q(command_joints)
        tau_save_full = session.get_joint_tau_est(command_joints)
        q_save = [q_save_full[idx] for idx in target_local_idx]
        tau_save = [tau_save_full[idx] for idx in target_local_idx]
        snapshot = save_snapshot(out_path, group=group, joints=joints, q=q_save, tau_est=tau_save)
        print(f"[SNAPSHOT] saved {snapshot.path}")
        return snapshot
    finally:
        print("[SNAPSHOT] release arm_sdk...")
        session.release_arm_sdk(
            command_joints,
            q_last,
            release_time=float(profile["release_time"]),
            dt=control_dt,
            kp=[support_kp if idx in support_local_idx else kp_soft for idx in range(len(command_joints))],
            kd=[support_kd if idx in support_local_idx else kd_soft for idx in range(len(command_joints))],
        )


def capture_current_snapshot(session, out_path, group="upper", settle_time=0.0):
    """Save the current measured pose without taking over arm_sdk."""
    if settle_time < 0:
        raise ValueError("settle_time must be >= 0")

    joints = get_group_joints(group)
    session.wait_lowstate()

    print(f"[SNAPSHOT] capture-current group={group} joints={len(joints)}")
    if settle_time > 0:
        print(f"[SNAPSHOT] waiting {settle_time:.2f}s for the pose to settle...")
        time.sleep(settle_time)

    q_now = session.get_joint_q(joints)
    tau_now = session.get_joint_tau_est(joints)
    snapshot = save_snapshot(
        out_path,
        group=group,
        joints=joints,
        q=q_now,
        source="capture-current-state",
        tau_est=tau_now,
    )
    print(f"[SNAPSHOT] saved {snapshot.path}")
    return snapshot


def goto_snapshot(
    session,
    snapshot,
    duration=1.5,
    profile_name="upper_hold",
    dry_run=False,
    release=True,
    source_q=None,
    source_result=None,
    easing="quintic",
    debug_takeover=False,
    takeover_tau_ff=True,
    hold=False,
    hold_time=0.0,
    snapshot_tau_ff=False,
):
    """Move from the current measured pose to a target snapshot in joint space."""
    if duration <= 0:
        raise ValueError("duration must be > 0")
    if hold_time < 0:
        raise ValueError("hold_time must be >= 0")

    profile = get_playback_profile(profile_name, snapshot.group)
    control_dt = float(profile["control_dt"])
    kp = profile["kp"]
    kd = profile["kd"]
    takeover_time = float(profile.get("takeover_time", 0.0))
    takeover_fade_time = float(profile.get("takeover_fade_time", 0.0))
    takeover_kp = float(profile.get("takeover_kp", kp))
    takeover_kd = float(profile.get("takeover_kd", kd))
    waist_support_kp = float(profile.get("waist_support_kp", kp))
    waist_support_kd = float(profile.get("waist_support_kd", kd))
    waist_support_takeover_kp = float(profile.get("waist_support_takeover_kp", waist_support_kp))
    waist_support_takeover_kd = float(profile.get("waist_support_takeover_kd", waist_support_kd))
    easing = _normalize_snapshot_easing(easing)

    if dry_run:
        print(
            f"[DRY-RUN] "
            f"{describe_snapshot(snapshot, duration=duration, profile_name=profile_name, easing=easing)}"
        )
        if takeover_tau_ff:
            print("[DRY-RUN] takeover tau_ff enabled for first arm_sdk handoff")
        if snapshot_tau_ff:
            print(
                "[DRY-RUN] snapshot tau_ff "
                + ("available" if snapshot.tau_est is not None else "requested but snapshot has no tau_est")
            )
        if hold or hold_time > 0:
            hold_desc = "until Ctrl+C" if hold else f"{hold_time:.2f}s"
            print(f"[DRY-RUN] final pose hold {hold_desc}")
        return {
            "joints": list(snapshot.joints),
            "q_final": list(snapshot.q),
            "kp": kp,
            "kd": kd,
            "control_dt": control_dt,
        }

    if source_q is None:
        source_q = list(session.get_joint_q(snapshot.joints))
    else:
        if len(source_q) != len(snapshot.joints):
            raise ValueError("source_q length must match snapshot joint count")
        source_q = [float(v) for v in source_q]
    command_joints, source_q_cmd, hold_context = _build_snapshot_command_context(
        session, snapshot, source_q, source_result=source_result
    )
    q_target = _expand_snapshot_command_q(snapshot.q, hold_context)
    snapshot_tau_cmd = _expand_snapshot_command_tau(
        snapshot.tau_est if snapshot_tau_ff else None,
        hold_context,
        len(command_joints),
    )
    q_last = list(source_q_cmd)
    steps = max(1, int(duration / control_dt))
    support_step = 0 if takeover_tau_ff and source_result is None else None
    fade_steps = max(1, int(takeover_fade_time / control_dt)) if support_step is not None and takeover_fade_time > 0 else 0
    tau_seed = None
    debug_state = create_takeover_debug_state(
        debug_takeover and source_result is None,
        control_dt,
        snapshot.path.name,
    )

    try:
        print(
            f"[SNAPSHOT] {describe_snapshot(snapshot, duration=duration, profile_name=profile_name, easing=easing)}"
        )
        if hold_context is not None:
            print("[SNAPSHOT] preserving non-target upper joints during partial-upper snapshot...")
        if snapshot_tau_ff:
            if snapshot_tau_cmd is None:
                print("[SNAPSHOT] snapshot tau_ff requested but this snapshot has no tau_est; using position control only")
            else:
                print("[SNAPSHOT] snapshot tau_ff enabled from saved tau_est")
        if debug_state is not None:
            emit_takeover_debug_snapshot(
                session,
                snapshot.path.name,
                "pre_takeover",
                command_joints,
                q_cmd=source_q_cmd,
                extra={"easing": easing},
            )
        if source_result is None:
            # Give arm_sdk a short, stiffer takeover hold before we start moving.
            takeover_steps = max(1, int(takeover_time / control_dt))
            if takeover_tau_ff:
                print("[SNAPSHOT] takeover tau_ff enabled using live tau_est...")
            for _ in range(takeover_steps):
                tau_ff = session.get_joint_tau_est(command_joints) if takeover_tau_ff else 0.0
                if takeover_tau_ff:
                    tau_seed = [float(value) for value in tau_ff]
                kp_send = _build_joint_gain_vector(command_joints, takeover_kp, waist_support_takeover_kp)
                kd_send = _build_joint_gain_vector(command_joints, takeover_kd, waist_support_takeover_kd)
                session.send_arm_q(
                    command_joints,
                    source_q_cmd,
                    kp=kp_send,
                    kd=kd_send,
                    tau_ff=tau_ff,
                )
                q_last = list(source_q_cmd)
                time.sleep(control_dt)
                emit_takeover_debug_tick(
                    session,
                    debug_state,
                    "takeover_hold",
                    command_joints,
                    q_cmd=q_last,
                    extra=(
                        {"tau_ff_cmd": [round(float(value), 6) for value in tau_ff]}
                        if takeover_tau_ff
                        else None
                    ),
                )
        for step in range(steps):
            # Keep snapshot playback position-only; explicit dq targets made the
            # controller feel too damped during the move on the real robot.
            raw = 1.0 if steps == 1 else step / (steps - 1)
            q_cmd_local, dq_cmd_local, alpha = _compute_snapshot_trajectory_point(source_q_cmd, q_target, raw, duration, easing)
            q_cmd = list(q_cmd_local)
            dq_cmd = _expand_snapshot_command_dq(dq_cmd_local, hold_context)
            q_last = list(q_cmd)
            kp_cmd, kd_cmd, tau_ff_cmd, support_fade = _resolve_takeover_support(
                support_step,
                fade_steps,
                tau_seed,
                takeover_kp,
                takeover_kd,
                kp,
                kd,
            )
            kp_send = _build_joint_gain_vector(
                command_joints,
                kp_cmd,
                _resolve_support_axis_gain(waist_support_kp, waist_support_takeover_kp, support_fade),
            )
            kd_send = _build_joint_gain_vector(
                command_joints,
                kd_cmd,
                _resolve_support_axis_gain(waist_support_kd, waist_support_takeover_kd, support_fade),
            )
            if snapshot_tau_cmd is not None:
                tau_ff_cmd = _add_tau_ff(tau_ff_cmd, _scale_vector(snapshot_tau_cmd, alpha))
            session.send_arm_q(
                command_joints,
                q_cmd,
                kp=kp_send,
                kd=kd_send,
                dq_target=dq_cmd,
                tau_ff=tau_ff_cmd,
            )
            if support_step is not None:
                support_step += 1
            time.sleep(control_dt)
            debug_extra = {
                "alpha": round(alpha, 6),
                "kp_cmd": round(float(kp_cmd), 6),
                "kd_cmd": round(float(kd_cmd), 6),
            }
            if support_fade is not None:
                debug_extra["support_fade"] = round(float(support_fade), 6)
                debug_extra["tau_ff_cmd"] = [round(float(value), 6) for value in tau_ff_cmd]
            emit_takeover_debug_tick(
                session,
                debug_state,
                "snapshot_blend",
                command_joints,
                q_cmd=q_last,
                extra=debug_extra,
            )
        if hold or hold_time > 0:
            hold_desc = "until Ctrl+C" if hold else f"{hold_time:.2f}s"
            print(f"[SNAPSHOT] holding final pose {hold_desc}...")
            kp_hold_vec = _build_joint_gain_vector(command_joints, kp, waist_support_kp)
            kd_hold_vec = _build_joint_gain_vector(command_joints, kd, waist_support_kd)
            hold_start = time.time()
            try:
                while hold or time.time() - hold_start < hold_time:
                    session.send_arm_q(
                        command_joints,
                        q_last,
                        kp=kp_hold_vec,
                        kd=kd_hold_vec,
                        tau_ff=snapshot_tau_cmd if snapshot_tau_cmd is not None else 0.0,
                    )
                    time.sleep(control_dt)
            except KeyboardInterrupt:
                print("\n[SNAPSHOT] final pose hold stopped by user")
        # Return per-joint gain vectors so script_runner's hold step preserves
        # the same waist_support_kp stiffness rather than falling back to base kp.
        kp_vec = _build_joint_gain_vector(command_joints, kp, waist_support_kp)
        kd_vec = _build_joint_gain_vector(command_joints, kd, waist_support_kd)
        return {
            "joints": list(command_joints),
            "q_final": list(q_last),
            "kp": kp_vec,
            "kd": kd_vec,
            "control_dt": control_dt,
        }
    except BaseException as exc:
        kp_vec = _build_joint_gain_vector(command_joints, kp, waist_support_kp)
        kd_vec = _build_joint_gain_vector(command_joints, kd, waist_support_kd)
        raise _attach_partial_result(
            exc,
            command_joints,
            q_last,
            kp_vec,
            kd_vec,
            control_dt,
        )
    finally:
        if release:
            # Single snapshot playback releases here; scripted playback defers release to the outer layer.
            print("[SNAPSHOT] release arm_sdk...")
            session.release_arm_sdk(
                command_joints,
                q_last,
                release_time=profile["release_time"],
                dt=control_dt,
                kp=_build_joint_gain_vector(command_joints, kp, waist_support_kp),
                kd=_build_joint_gain_vector(command_joints, kd, waist_support_kd),
            )
