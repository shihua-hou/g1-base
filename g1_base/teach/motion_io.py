"""Legacy motion loader and continuous trajectory playback.

This module keeps the original g1_teach idea: load time-ordered `.jsonl`
joint trajectories, then replay them with 100 Hz-ish linear interpolation.
"""

import json
import time
from dataclasses import dataclass
from pathlib import Path

from .joints import UPPER_JOINTS, WAIST_SUPPORT_JOINTS, WAIST_YAW_JOINTS, get_group_joints, with_waist_support
from .profiles import get_playback_profile
from .takeover_debug import create_takeover_debug_state, emit_takeover_debug_snapshot, emit_takeover_debug_tick


@dataclass
class MotionTrajectory:
    path: Path
    group: str
    joints: list
    times: list
    qs: list


def _lerp_vector(a, b, t):
    return [(1.0 - t) * float(x) + t * float(y) for x, y in zip(a, b)]


def _lerp_scalar(a, b, t):
    return (1.0 - t) * float(a) + t * float(b)


def _normalize_motion_interpolation(interpolation):
    if interpolation is None:
        return "linear"

    value = str(interpolation).strip().lower()
    aliases = {
        "cubic_hermite": "cubic",
        "cubic-hermite": "cubic",
        "hermite": "cubic",
        "spline": "cubic",
    }
    value = aliases.get(value, value)
    allowed = {"linear", "cubic"}
    if value not in allowed:
        raise ValueError(f"unsupported motion interpolation: {interpolation}")
    return value


def _compute_motion_tangents(times, qs):
    dim = len(qs[0]) if qs else 0
    count = len(times)
    tangents = []

    for idx in range(count):
        if count == 1:
            tangents.append([0.0] * dim)
            continue

        if idx == 0:
            dt = float(times[1]) - float(times[0])
            if dt <= 0:
                tangents.append([0.0] * dim)
            else:
                tangents.append([(float(qs[1][j]) - float(qs[0][j])) / dt for j in range(dim)])
            continue

        if idx == count - 1:
            dt = float(times[-1]) - float(times[-2])
            if dt <= 0:
                tangents.append([0.0] * dim)
            else:
                tangents.append([(float(qs[-1][j]) - float(qs[-2][j])) / dt for j in range(dim)])
            continue

        dt = float(times[idx + 1]) - float(times[idx - 1])
        if dt <= 0:
            tangents.append([0.0] * dim)
        else:
            tangents.append(
                [(float(qs[idx + 1][j]) - float(qs[idx - 1][j])) / dt for j in range(dim)]
            )

    return tangents


def _hermite_vector(q0, q1, m0, m1, dt, alpha):
    a2 = alpha * alpha
    a3 = a2 * alpha
    h00 = 2.0 * a3 - 3.0 * a2 + 1.0
    h10 = a3 - 2.0 * a2 + alpha
    h01 = -2.0 * a3 + 3.0 * a2
    h11 = a3 - a2
    return [
        h00 * float(q0v) + h10 * dt * float(m0v) + h01 * float(q1v) + h11 * dt * float(m1v)
        for q0v, q1v, m0v, m1v in zip(q0, q1, m0, m1)
    ]


def _find_implicit_lead_in_target(trajectory, tol=1e-4):
    """Return the first frame that meaningfully moves away from frame 0."""
    if len(trajectory.times) < 2:
        return None

    base_q = list(trajectory.qs[0])
    base_t = float(trajectory.times[0])
    for idx in range(1, len(trajectory.times)):
        if float(trajectory.times[idx]) <= base_t:
            continue
        q = trajectory.qs[idx]
        if any(abs(float(qv) - float(bv)) > tol for qv, bv in zip(q, base_q)):
            return idx
    return 1


def _attach_partial_result(exc, joints, q_final, kp, kd, control_dt):
    """Stash the latest commanded pose on an in-flight exception."""
    exc.partial_result = {
        "joints": list(joints),
        "q_final": list(q_final),
        "kp": kp,
        "kd": kd,
        "control_dt": control_dt,
    }
    return exc


def _build_motion_command_context(session, trajectory, source_q, source_result=None, force_hold_joints=None):
    """Expand partial upper-body motions to the full upper command set.

    force_hold_joints: joint IDs whose trajectory values are ignored;
    they are held at the initial readback position regardless of the recorded data.
    """
    target_joints = [int(joint) for joint in trajectory.joints]
    upper_joints = list(UPPER_JOINTS)
    arm_target_joints = [joint for joint in target_joints if joint in upper_joints]
    command_joints = list(target_joints)
    source_joints_for_mapping = list(target_joints)
    if arm_target_joints:
        command_joints = with_waist_support(upper_joints if target_joints != upper_joints else target_joints)
        source_joints_for_mapping = arm_target_joints

    # Exclude force-held joints from trajectory mapping so they stay at initial position.
    force_hold_set = set(int(j) for j in force_hold_joints) if force_hold_joints else set()
    if force_hold_set:
        source_joints_for_mapping = [j for j in source_joints_for_mapping if j not in force_hold_set]
        # Ensure force-held joints appear in command_joints so they receive explicit commands.
        for j in sorted(force_hold_set):
            if j not in command_joints:
                command_joints = [j] + command_joints

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
    source_joints_set = set(source_joints_for_mapping)
    source_to_command_indices = [
        (source_idx, joint_to_command_idx[int(joint)])
        for source_idx, joint in enumerate(target_joints)
        if int(joint) in joint_to_command_idx
        and int(joint) in source_joints_set
    ]

    command_source_q = list(hold_q)
    for source_idx, command_idx in source_to_command_indices:
        command_source_q[command_idx] = float(source_q[source_idx])

    return command_joints, command_source_q, (hold_q, source_to_command_indices)


def _expand_motion_command_q(target_q, hold_context):
    if hold_context is None:
        return list(target_q)

    hold_q, source_to_command_indices = hold_context
    q_cmd = list(hold_q)
    for source_idx, command_idx in source_to_command_indices:
        q_cmd[command_idx] = float(target_q[source_idx])
    return q_cmd


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


def _yaw_local_indices(command_joints):
    return [idx for idx, joint in enumerate(command_joints) if int(joint) in WAIST_YAW_JOINTS]


def _build_joint_gain_vector(command_joints, base_gain, support_gain, yaw_gain=None):
    support_idx = _support_local_indices(command_joints)
    yaw_idx = _yaw_local_indices(command_joints) if yaw_gain is not None else []
    if not support_idx and not yaw_idx:
        return base_gain
    values = [float(base_gain)] * len(command_joints)
    for idx in support_idx:
        values[idx] = float(support_gain)
    for idx in yaw_idx:
        values[idx] = float(yaw_gain)
    return values


def _resolve_support_axis_gain(base_gain, takeover_gain, fade):
    if fade is None:
        return float(base_gain)
    return _lerp_scalar(float(takeover_gain), float(base_gain), float(fade))


def _can_play_arm_sequence(session, debug_state=None):
    return debug_state is None and callable(getattr(session, "play_arm_sequence", None))


def _arm_sequence_frame(q, kp, kd, dq_target=0.0, tau_ff=0.0):
    return {
        "q": [float(v) for v in q],
        "kp": kp,
        "kd": kd,
        "dq_target": dq_target,
        "tau_ff": tau_ff,
    }


def load_motion(path):
    """Load a legacy jsonl motion file and validate its basic timing schema."""
    path = Path(path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"motion file not found: {path}")

    rows = []
    with path.open("r", encoding="utf-8-sig") as handle:
        for line_no, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON on line {line_no} in {path}") from exc
            rows.append(row)

    if not rows:
        raise ValueError(f"empty motion file: {path}")

    required = {"t", "group", "joints", "q"}
    missing = required - rows[0].keys()
    if missing:
        raise ValueError(f"motion file missing required fields {sorted(missing)}: {path}")

    group = str(rows[0]["group"])
    joints = [int(v) for v in rows[0]["joints"]]
    q_dim = len(joints)
    times = []
    qs = []
    prev_t = None

    for idx, row in enumerate(rows, start=1):
        if str(row.get("group")) != group:
            raise ValueError(f"group mismatch on row {idx} in {path}")
        if [int(v) for v in row.get("joints", [])] != joints:
            raise ValueError(f"joints mismatch on row {idx} in {path}")

        t = float(row["t"])
        q = [float(v) for v in row["q"]]
        if len(q) != q_dim:
            raise ValueError(f"q length mismatch on row {idx} in {path}")
        if prev_t is not None and t < prev_t:
            raise ValueError(f"time must be non-decreasing on row {idx} in {path}")

        times.append(t)
        qs.append(q)
        prev_t = t

    return MotionTrajectory(path=path, group=group, joints=joints, times=times, qs=qs)


def describe_motion(
    trajectory,
    profile_name="upper_playback",
    speed=1.0,
    blend_time=None,
    interpolation=None,
):
    duration = trajectory.times[-1] if trajectory.times else 0.0
    summary = (
        f"motion={trajectory.path.name} group={trajectory.group} joints={len(trajectory.joints)} "
        f"frames={len(trajectory.times)} duration={duration:.3f}s profile={profile_name} speed={speed:.2f}"
    )
    if blend_time is not None:
        summary += f" blend_time={float(blend_time):.2f}s"
    if interpolation is not None:
        summary += f" interpolation={_normalize_motion_interpolation(interpolation)}"
    return summary


def play_motion(
    session,
    trajectory,
    profile_name="upper_playback",
    speed=1.0,
    dry_run=False,
    release=True,
    source_q=None,
    source_result=None,
    blend_time=None,
    interpolation="linear",
    debug_takeover=False,
    takeover_tau_ff=True,
    cancel_event=None,
):
    """Replay a loaded trajectory, optionally in dry-run mode."""
    if speed <= 0:
        raise ValueError("speed must be > 0")

    profile = get_playback_profile(profile_name, trajectory.group)
    control_dt = float(profile["control_dt"])
    if blend_time is None:
        blend_time = float(profile["blend_time"])
    else:
        blend_time = float(blend_time)
    if blend_time < 0:
        raise ValueError("blend_time must be >= 0")
    interpolation = _normalize_motion_interpolation(interpolation)
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
    # waist_yaw_kp/kd: optional, only set in upper_waist_lock; None means use base kp/kd for joint 12
    _raw_yaw_kp = profile.get("waist_yaw_kp")
    waist_yaw_kp = float(_raw_yaw_kp) if _raw_yaw_kp is not None else None
    _raw_yaw_kd = profile.get("waist_yaw_kd")
    waist_yaw_kd = float(_raw_yaw_kd) if _raw_yaw_kd is not None else None
    _raw_yaw_tkp = profile.get("waist_yaw_takeover_kp")
    waist_yaw_takeover_kp = float(_raw_yaw_tkp) if _raw_yaw_tkp is not None else waist_yaw_kp
    _raw_yaw_tkd = profile.get("waist_yaw_takeover_kd")
    waist_yaw_takeover_kd = float(_raw_yaw_tkd) if _raw_yaw_tkd is not None else waist_yaw_kd
    # waist_yaw_hold: ignore trajectory data for WaistYaw joints, hold at initial position
    waist_yaw_hold = bool(profile.get("waist_yaw_hold", False))
    force_hold_joints = list(WAIST_YAW_JOINTS) if waist_yaw_hold else None
    tangents = _compute_motion_tangents(trajectory.times, trajectory.qs) if interpolation == "cubic" else None

    if dry_run:
        print(
            f"[DRY-RUN] "
            f"{describe_motion(trajectory, profile_name=profile_name, speed=speed, blend_time=blend_time, interpolation=interpolation)}"
        )
        if takeover_tau_ff:
            print("[DRY-RUN] takeover tau_ff enabled for first arm_sdk handoff")
        return {
            "joints": list(trajectory.joints),
            "q_final": list(trajectory.qs[-1]),
            "kp": kp,
            "kd": kd,
            "control_dt": control_dt,
        }

    if source_q is None:
        q_source = list(session.get_joint_q(trajectory.joints))
    else:
        if len(source_q) != len(trajectory.joints):
            raise ValueError("source_q length must match motion joint count")
        q_source = [float(v) for v in source_q]

    command_joints, q_source_cmd, hold_context = _build_motion_command_context(
        session, trajectory, q_source, source_result=source_result, force_hold_joints=force_hold_joints
    )
    q_last = list(q_source_cmd)
    q_start = _expand_motion_command_q(trajectory.qs[0], hold_context)
    use_explicit_blend = blend_time > 0
    explicit_blend_steps = max(1, int(blend_time / control_dt)) if use_explicit_blend else 0
    implicit_lead_in_idx = None if use_explicit_blend else _find_implicit_lead_in_target(trajectory)
    support_step = 0 if takeover_tau_ff and source_result is None else None
    fade_steps = max(1, int(takeover_fade_time / control_dt)) if support_step is not None and takeover_fade_time > 0 else 0
    tau_seed = None
    debug_state = create_takeover_debug_state(
        debug_takeover and source_result is None,
        control_dt,
        trajectory.path.name,
    )

    try:
        print(
            f"[PLAY] "
            f"{describe_motion(trajectory, profile_name=profile_name, speed=speed, blend_time=blend_time, interpolation=interpolation)}"
        )
        if hold_context is not None:
            print("[PLAY] preserving non-target upper joints during partial-upper motion...")
        if debug_state is not None:
            emit_takeover_debug_snapshot(
                session,
                trajectory.path.name,
                "pre_takeover",
                command_joints,
                q_cmd=q_source_cmd,
                extra={"interpolation": interpolation},
            )

        if _can_play_arm_sequence(session, debug_state):
            frames = []
            if source_result is None:
                print("[PLAY] arm_sdk takeover hold...")
                takeover_steps = max(1, int(takeover_time / control_dt))
                if takeover_tau_ff:
                    print("[PLAY] takeover tau_ff enabled using live tau_est...")
                    tau_seed = [float(value) for value in session.get_joint_tau_est(command_joints)]
                kp_send = _build_joint_gain_vector(
                    command_joints,
                    takeover_kp,
                    waist_support_takeover_kp,
                    yaw_gain=waist_yaw_takeover_kp,
                )
                kd_send = _build_joint_gain_vector(
                    command_joints,
                    takeover_kd,
                    waist_support_takeover_kd,
                    yaw_gain=waist_yaw_takeover_kd,
                )
                for _ in range(takeover_steps):
                    frames.append(
                        _arm_sequence_frame(
                            q_source_cmd,
                            kp_send,
                            kd_send,
                            tau_ff=tau_seed if takeover_tau_ff else 0.0,
                        )
                    )
                q_last = list(q_source_cmd)

            if use_explicit_blend:
                print("[PLAY] blend to first frame...")
                for step in range(explicit_blend_steps):
                    alpha = 1.0 if explicit_blend_steps == 1 else step / (explicit_blend_steps - 1)
                    q_cmd = _lerp_vector(q_source_cmd, q_start, alpha)
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
                        _resolve_support_axis_gain(
                            waist_support_kp,
                            waist_support_takeover_kp,
                            support_fade,
                        ),
                        yaw_gain=(
                            _resolve_support_axis_gain(
                                waist_yaw_kp,
                                waist_yaw_takeover_kp,
                                support_fade,
                            )
                            if waist_yaw_kp is not None
                            else None
                        ),
                    )
                    kd_send = _build_joint_gain_vector(
                        command_joints,
                        kd_cmd,
                        _resolve_support_axis_gain(
                            waist_support_kd,
                            waist_support_takeover_kd,
                            support_fade,
                        ),
                        yaw_gain=(
                            _resolve_support_axis_gain(
                                waist_yaw_kd,
                                waist_yaw_takeover_kd,
                                support_fade,
                            )
                            if waist_yaw_kd is not None
                            else None
                        ),
                    )
                    frames.append(
                        _arm_sequence_frame(
                            q_cmd,
                            kp_send,
                            kd_send,
                            tau_ff=tau_ff_cmd,
                        )
                    )
                    if support_step is not None:
                        support_step += 1
            elif implicit_lead_in_idx is not None:
                print(f"[PLAY] implicit lead-in using motion frames 0 -> {implicit_lead_in_idx}...")

            print("[PLAY] playback...")
            idx = 0
            tick = 0
            final_t = trajectory.times[-1]
            lead_in_idx = implicit_lead_in_idx

            while True:
                now = tick * control_dt * speed
                if now >= final_t:
                    q_last = _expand_motion_command_q(trajectory.qs[-1], hold_context)
                    frames.append(
                        _arm_sequence_frame(
                            q_last,
                            _build_joint_gain_vector(
                                command_joints,
                                kp,
                                waist_support_kp,
                                yaw_gain=waist_yaw_kp,
                            ),
                            _build_joint_gain_vector(
                                command_joints,
                                kd,
                                waist_support_kd,
                                yaw_gain=waist_yaw_kd,
                            ),
                        )
                    )
                    break

                if lead_in_idx is not None:
                    lead_t = float(trajectory.times[lead_in_idx])
                    if now < lead_t:
                        alpha = 0.0 if lead_t <= 0 else now / lead_t
                        alpha = max(0.0, min(1.0, alpha))
                        q_lead_target = _expand_motion_command_q(trajectory.qs[lead_in_idx], hold_context)
                        q_cmd = _lerp_vector(q_source_cmd, q_lead_target, alpha)
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
                            _resolve_support_axis_gain(
                                waist_support_kp,
                                waist_support_takeover_kp,
                                support_fade,
                            ),
                            yaw_gain=(
                                _resolve_support_axis_gain(
                                    waist_yaw_kp,
                                    waist_yaw_takeover_kp,
                                    support_fade,
                                )
                                if waist_yaw_kp is not None
                                else None
                            ),
                        )
                        kd_send = _build_joint_gain_vector(
                            command_joints,
                            kd_cmd,
                            _resolve_support_axis_gain(
                                waist_support_kd,
                                waist_support_takeover_kd,
                                support_fade,
                            ),
                            yaw_gain=(
                                _resolve_support_axis_gain(
                                    waist_yaw_kd,
                                    waist_yaw_takeover_kd,
                                    support_fade,
                                )
                                if waist_yaw_kd is not None
                                else None
                            ),
                        )
                        frames.append(
                            _arm_sequence_frame(
                                q_cmd,
                                kp_send,
                                kd_send,
                                tau_ff=tau_ff_cmd,
                            )
                        )
                        if support_step is not None:
                            support_step += 1
                        tick += 1
                        continue

                    idx = max(idx, lead_in_idx)
                    lead_in_idx = None

                while idx + 1 < len(trajectory.times) and trajectory.times[idx + 1] <= now:
                    idx += 1

                if idx >= len(trajectory.times) - 1:
                    q_cmd = list(trajectory.qs[-1])
                else:
                    t0 = trajectory.times[idx]
                    t1 = trajectory.times[idx + 1]
                    q0 = trajectory.qs[idx]
                    q1 = trajectory.qs[idx + 1]
                    alpha = 0.0 if t1 <= t0 else (now - t0) / (t1 - t0)
                    alpha = max(0.0, min(1.0, alpha))
                    if interpolation == "cubic":
                        q_target = _hermite_vector(
                            q0,
                            q1,
                            tangents[idx],
                            tangents[idx + 1],
                            float(t1 - t0),
                            alpha,
                        )
                    else:
                        q_target = _lerp_vector(q0, q1, alpha)
                    q_cmd = _expand_motion_command_q(q_target, hold_context)

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
                    _resolve_support_axis_gain(
                        waist_support_kp,
                        waist_support_takeover_kp,
                        support_fade,
                    ),
                    yaw_gain=(
                        _resolve_support_axis_gain(
                            waist_yaw_kp,
                            waist_yaw_takeover_kp,
                            support_fade,
                        )
                        if waist_yaw_kp is not None
                        else None
                    ),
                )
                kd_send = _build_joint_gain_vector(
                    command_joints,
                    kd_cmd,
                    _resolve_support_axis_gain(
                        waist_support_kd,
                        waist_support_takeover_kd,
                        support_fade,
                    ),
                    yaw_gain=(
                        _resolve_support_axis_gain(
                            waist_yaw_kd,
                            waist_yaw_takeover_kd,
                            support_fade,
                        )
                        if waist_yaw_kd is not None
                        else None
                    ),
                )
                frames.append(
                    _arm_sequence_frame(
                        q_cmd,
                        kp_send,
                        kd_send,
                        tau_ff=tau_ff_cmd,
                    )
                )
                if support_step is not None:
                    support_step += 1
                tick += 1

            sequence_result = session.play_arm_sequence(
                command_joints,
                frames,
                dt=control_dt,
                release=False,
                cancel_event=cancel_event,
            )
            q_last = list(sequence_result.get("q_final", q_last))
            print("[PLAY] playback finished")
            return {
                "joints": list(command_joints),
                "q_final": q_last,
                "kp": sequence_result.get("kp")
                or _build_joint_gain_vector(command_joints, kp, waist_support_kp, yaw_gain=waist_yaw_kp),
                "kd": sequence_result.get("kd")
                or _build_joint_gain_vector(command_joints, kd, waist_support_kd, yaw_gain=waist_yaw_kd),
                "control_dt": control_dt,
            }

        if source_result is None:
            print("[PLAY] arm_sdk takeover hold...")
            takeover_steps = max(1, int(takeover_time / control_dt))
            if takeover_tau_ff:
                print("[PLAY] takeover tau_ff enabled using live tau_est...")
            for _ in range(takeover_steps):
                if cancel_event and cancel_event.is_set():
                    print("[PLAY] 收到取消信号，中断 takeover hold")
                    break
                tau_ff = session.get_joint_tau_est(command_joints) if takeover_tau_ff else 0.0
                if takeover_tau_ff:
                    tau_seed = [float(value) for value in tau_ff]
                kp_send = _build_joint_gain_vector(command_joints, takeover_kp, waist_support_takeover_kp, yaw_gain=waist_yaw_takeover_kp)
                kd_send = _build_joint_gain_vector(command_joints, takeover_kd, waist_support_takeover_kd, yaw_gain=waist_yaw_takeover_kd)
                session.send_arm_q(
                    command_joints,
                    q_source_cmd,
                    kp=kp_send,
                    kd=kd_send,
                    tau_ff=tau_ff,
                )
                q_last = list(q_source_cmd)
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

            if cancel_event and cancel_event.is_set():
                return {
                    "joints": list(command_joints),
                    "q_final": list(q_last),
                    "kp": kp,
                    "kd": kd,
                    "control_dt": control_dt,
                }

        if use_explicit_blend:
            print("[PLAY] blend to first frame...")
            for step in range(explicit_blend_steps):
                if cancel_event and cancel_event.is_set():
                    print("[PLAY] 收到取消信号，中断 blend")
                    break
                # Start by explicitly holding the current pose for one control tick so arm_sdk
                # can take over cleanly before we begin pulling toward the recorded first frame.
                alpha = 1.0 if explicit_blend_steps == 1 else step / (explicit_blend_steps - 1)
                q_cmd = _lerp_vector(q_source_cmd, q_start, alpha)
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
                    yaw_gain=_resolve_support_axis_gain(waist_yaw_kp, waist_yaw_takeover_kp, support_fade) if waist_yaw_kp is not None else None,
                )
                kd_send = _build_joint_gain_vector(
                    command_joints,
                    kd_cmd,
                    _resolve_support_axis_gain(waist_support_kd, waist_support_takeover_kd, support_fade),
                    yaw_gain=_resolve_support_axis_gain(waist_yaw_kd, waist_yaw_takeover_kd, support_fade) if waist_yaw_kd is not None else None,
                )
                session.send_arm_q(command_joints, q_cmd, kp=kp_send, kd=kd_send, tau_ff=tau_ff_cmd)
                if support_step is not None:
                    support_step += 1
                time.sleep(control_dt)
                debug_extra = {"alpha": round(alpha, 6), "kp_cmd": round(float(kp_cmd), 6), "kd_cmd": round(float(kd_cmd), 6)}
                if support_fade is not None:
                    debug_extra["support_fade"] = round(float(support_fade), 6)
                    debug_extra["tau_ff_cmd"] = [round(float(value), 6) for value in tau_ff_cmd]
                emit_takeover_debug_tick(
                    session,
                    debug_state,
                    "motion_blend",
                    command_joints,
                    q_cmd=q_last,
                    extra=debug_extra,
                )
            if cancel_event and cancel_event.is_set():
                return {
                    "joints": list(command_joints),
                    "q_final": list(q_last),
                    "kp": kp,
                    "kd": kd,
                    "control_dt": control_dt,
                }
        elif implicit_lead_in_idx is not None:
            print(f"[PLAY] implicit lead-in using motion frames 0 -> {implicit_lead_in_idx}...")

        print("[PLAY] playback...")
        start = time.time()
        idx = 0
        final_t = trajectory.times[-1]

        while True:
            if cancel_event and cancel_event.is_set():
                print("[PLAY] 收到取消信号，中断播放")
                break
            now = (time.time() - start) * speed
            if now >= final_t:
                q_last = _expand_motion_command_q(trajectory.qs[-1], hold_context)
                session.send_arm_q(
                    command_joints,
                    q_last,
                    kp=_build_joint_gain_vector(command_joints, kp, waist_support_kp, yaw_gain=waist_yaw_kp),
                    kd=_build_joint_gain_vector(command_joints, kd, waist_support_kd, yaw_gain=waist_yaw_kd),
                )
                break

            if implicit_lead_in_idx is not None:
                lead_t = float(trajectory.times[implicit_lead_in_idx])
                if now < lead_t:
                    alpha = 0.0 if lead_t <= 0 else now / lead_t
                    alpha = max(0.0, min(1.0, alpha))
                    q_lead_target = _expand_motion_command_q(trajectory.qs[implicit_lead_in_idx], hold_context)
                    q_cmd = _lerp_vector(q_source_cmd, q_lead_target, alpha)
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
                        yaw_gain=_resolve_support_axis_gain(waist_yaw_kp, waist_yaw_takeover_kp, support_fade) if waist_yaw_kp is not None else None,
                    )
                    kd_send = _build_joint_gain_vector(
                        command_joints,
                        kd_cmd,
                        _resolve_support_axis_gain(waist_support_kd, waist_support_takeover_kd, support_fade),
                        yaw_gain=_resolve_support_axis_gain(waist_yaw_kd, waist_yaw_takeover_kd, support_fade) if waist_yaw_kd is not None else None,
                    )
                    session.send_arm_q(command_joints, q_cmd, kp=kp_send, kd=kd_send, tau_ff=tau_ff_cmd)
                    if support_step is not None:
                        support_step += 1
                    time.sleep(control_dt)
                    debug_extra = {
                        "play_t": round(now, 6),
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
                        "motion_implicit_lead",
                        command_joints,
                        q_cmd=q_last,
                        extra=debug_extra,
                    )
                    continue

                idx = max(idx, implicit_lead_in_idx)
                implicit_lead_in_idx = None

            while idx + 1 < len(trajectory.times) and trajectory.times[idx + 1] <= now:
                idx += 1

            if idx >= len(trajectory.times) - 1:
                q_cmd = list(trajectory.qs[-1])
            else:
                t0 = trajectory.times[idx]
                t1 = trajectory.times[idx + 1]
                q0 = trajectory.qs[idx]
                q1 = trajectory.qs[idx + 1]
                alpha = 0.0 if t1 <= t0 else (now - t0) / (t1 - t0)
                alpha = max(0.0, min(1.0, alpha))
                if interpolation == "cubic":
                    q_target = _hermite_vector(q0, q1, tangents[idx], tangents[idx + 1], float(t1 - t0), alpha)
                else:
                    q_target = _lerp_vector(q0, q1, alpha)
                q_cmd = _expand_motion_command_q(q_target, hold_context)

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
                yaw_gain=_resolve_support_axis_gain(waist_yaw_kp, waist_yaw_takeover_kp, support_fade) if waist_yaw_kp is not None else None,
            )
            kd_send = _build_joint_gain_vector(
                command_joints,
                kd_cmd,
                _resolve_support_axis_gain(waist_support_kd, waist_support_takeover_kd, support_fade),
                yaw_gain=_resolve_support_axis_gain(waist_yaw_kd, waist_yaw_takeover_kd, support_fade) if waist_yaw_kd is not None else None,
            )
            session.send_arm_q(command_joints, q_cmd, kp=kp_send, kd=kd_send, tau_ff=tau_ff_cmd)
            if support_step is not None:
                support_step += 1
            time.sleep(control_dt)
            debug_extra = {
                "play_t": round(now, 6),
                "kp_cmd": round(float(kp_cmd), 6),
                "kd_cmd": round(float(kd_cmd), 6),
            }
            if support_fade is not None:
                debug_extra["support_fade"] = round(float(support_fade), 6)
                debug_extra["tau_ff_cmd"] = [round(float(value), 6) for value in tau_ff_cmd]
            emit_takeover_debug_tick(
                session,
                debug_state,
                "motion_playback",
                command_joints,
                q_cmd=q_last,
                extra=debug_extra,
            )

        print("[PLAY] playback finished")
        kp_vec = _build_joint_gain_vector(command_joints, kp, waist_support_kp, yaw_gain=waist_yaw_kp)
        kd_vec = _build_joint_gain_vector(command_joints, kd, waist_support_kd, yaw_gain=waist_yaw_kd)
        return {
            "joints": list(command_joints),
            "q_final": list(q_last),
            "kp": kp_vec,
            "kd": kd_vec,
            "control_dt": control_dt,
        }
    except BaseException as exc:
        kp_vec = _build_joint_gain_vector(command_joints, kp, waist_support_kp, yaw_gain=waist_yaw_kp)
        kd_vec = _build_joint_gain_vector(command_joints, kd, waist_support_kd, yaw_gain=waist_yaw_kd)
        raise _attach_partial_result(exc, command_joints, q_last, kp_vec, kd_vec, control_dt)
    finally:
        if release:
            print("[PLAY] release arm_sdk...")
            session.release_arm_sdk(
                command_joints,
                q_last,
                release_time=profile["release_time"],
                dt=control_dt,
                kp=_build_joint_gain_vector(command_joints, kp, waist_support_kp, yaw_gain=waist_yaw_kp),
                kd=_build_joint_gain_vector(command_joints, kd, waist_support_kd, yaw_gain=waist_yaw_kd),
            )
