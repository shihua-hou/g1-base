"""Recording modes built on top of the original g1_teach workflow."""

import time
from pathlib import Path

import numpy as np

from .audio_io import (
    DEFAULT_AUDIO_VOLUME,
    AudioPlayback,
    SUPPORTED_AUDIO_BACKENDS,
    describe_audio_step,
    normalize_audio_volume,
)
from .joints import G1JointIndex, RIGHT_ARM_JOINTS, UPPER_JOINTS, WAIST_SUPPORT_JOINTS, get_group_joints, with_waist_support
from .profiles import get_record_profile
from .robot_io import RobotSession


def _lerp(a, b, t):
    return a + (b - a) * t


def _save_jsonl_row(handle, row):
    import json

    handle.write(json.dumps(row) + "\n")
    handle.flush()


def _idx_in(base_joints, sub_joints):
    base_map = {joint: idx for idx, joint in enumerate(base_joints)}
    return [base_map[j] for j in sub_joints]


def _apply_joint_locks(q_cmd, kp, kd, q_lock, locked_local_idx, lock_kp, lock_kd):
    for idx in locked_local_idx:
        q_cmd[idx] = q_lock[idx]
        kp[idx] = max(kp[idx], lock_kp)
        kd[idx] = max(kd[idx], lock_kd)


def _build_auto_hold_state(profile, active_local_idx, enabled):
    if not enabled or not active_local_idx:
        return None

    return {
        "mode": "drag",
        "active_local_idx": list(active_local_idx),
        "q_ref": None,
        "still_since": None,
        "velocity_enter": float(profile["auto_hold_velocity_enter"]),
        "velocity_exit": float(profile["auto_hold_velocity_exit"]),
        "position_exit": float(profile["auto_hold_position_exit"]),
        "dwell_time": float(profile["auto_hold_dwell_time"]),
    }


def _update_auto_hold_state(state, q_now, dq_now, now):
    if state is None:
        return None

    active_idx = state["active_local_idx"]
    max_dq = max(abs(float(dq_now[idx])) for idx in active_idx)

    if state["mode"] == "drag":
        if max_dq <= state["velocity_enter"]:
            if state["still_since"] is None:
                state["still_since"] = now
            elif now - state["still_since"] >= state["dwell_time"]:
                state["mode"] = "hold"
                state["q_ref"] = np.array(q_now, dtype=float)
                state["still_since"] = None
                return "hold"
        else:
            state["still_since"] = None
        return None

    q_ref = state["q_ref"]
    max_err = max(abs(float(q_now[idx] - q_ref[idx])) for idx in active_idx)
    if max_dq >= state["velocity_exit"] or max_err >= state["position_exit"]:
        state["mode"] = "drag"
        state["still_since"] = None
        return "drag"
    return None


def _apply_auto_hold_targets(q_cmd, kp, kd, state, kp_hold, kd_hold):
    if state is None or state["mode"] != "hold":
        return

    for idx in state["active_local_idx"]:
        q_cmd[idx] = state["q_ref"][idx]
        kp[idx] = max(kp[idx], kp_hold[idx])
        kd[idx] = max(kd[idx], kd_hold[idx])


def _prepend_waist_support(values, support_value):
    return np.array([float(support_value)] * len(WAIST_SUPPORT_JOINTS) + list(values), dtype=float)


def _format_upper_status(q_values, joints, include_wrists=False):
    joint_to_local = {joint: idx for idx, joint in enumerate(joints)}

    def _get(joint):
        return float(q_values[joint_to_local[joint]])

    parts = []
    if G1JointIndex.WaistYaw in joint_to_local:
        parts.append(f"W_yaw={_get(G1JointIndex.WaistYaw):+.2f}")
    parts.append(
        f"L_sh={_get(G1JointIndex.LeftShoulderPitch):+.2f} L_el={_get(G1JointIndex.LeftElbow):+.2f}"
    )
    if include_wrists:
        parts.append(
            f"L_wr={_get(G1JointIndex.LeftWristRoll):+.2f}/{_get(G1JointIndex.LeftWristPitch):+.2f}/{_get(G1JointIndex.LeftWristYaw):+.2f}"
        )
    parts.append(
        f"R_sh={_get(G1JointIndex.RightShoulderPitch):+.2f} R_el={_get(G1JointIndex.RightElbow):+.2f}"
    )
    if include_wrists:
        parts.append(
            f"R_wr={_get(G1JointIndex.RightWristRoll):+.2f}/{_get(G1JointIndex.RightWristPitch):+.2f}/{_get(G1JointIndex.RightWristYaw):+.2f}"
        )
    return " ".join(parts)


def _as_path(out_path):
    path = Path(out_path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _resolve_control_dt(default_dt, override_dt):
    if override_dt is None:
        return float(default_dt)
    override_dt = float(override_dt)
    if override_dt <= 0:
        raise ValueError("control_dt must be > 0")
    return override_dt


def _start_record_music(
    music_path,
    *,
    backend="auto",
    delay=0.0,
    start="recording",
    record_start_delay=0.0,
    stream_name="music",
    volume=DEFAULT_AUDIO_VOLUME,
):
    if not music_path:
        return None

    normalized_backend = str(backend or "auto").strip().lower()
    if normalized_backend not in SUPPORTED_AUDIO_BACKENDS:
        raise ValueError(f"music backend must be one of {', '.join(SUPPORTED_AUDIO_BACKENDS)}")

    start_mode = str(start or "recording").strip().lower()
    if start_mode not in {"recording", "command"}:
        raise ValueError("music start must be 'recording' or 'command'")

    audio_path = Path(music_path).expanduser().resolve()
    if not audio_path.exists():
        raise FileNotFoundError(f"music file not found: {audio_path}")

    user_delay = float(delay or 0.0)
    effective_delay = user_delay
    if start_mode == "recording":
        effective_delay += float(record_start_delay)
    if effective_delay < 0:
        raise ValueError(
            "music delay starts before the record command; use a smaller negative delay "
            "or --music-start command"
        )
    volume = normalize_audio_volume(volume)

    print(
        "[RECORD] music "
        + describe_audio_step(
            audio_path,
            backend=normalized_backend,
            delay=effective_delay,
            async_play=True,
            stream_name=stream_name,
            volume=volume,
        )
        + f" start={start_mode} offset={user_delay:+.2f}s"
    )
    return AudioPlayback(
        audio_path,
        backend=normalized_backend,
        delay=effective_delay,
        stream_name=stream_name,
        async_play=True,
        volume=volume,
    ).start()


def _stop_record_music(playback):
    if playback is None:
        return
    playback.cancel()
    try:
        playback.join()
    except BaseException as exc:
        print(f"[RECORD] music playback error: {exc}")


def record_motion(
    iface,
    mode,
    out_path,
    group="upper",
    auto_hold=False,
    control_dt=None,
    music_path=None,
    music_backend="auto",
    music_delay=0.0,
    music_start="recording",
    music_stream_name="music",
    music_volume=DEFAULT_AUDIO_VOLUME,
):
    """Dispatch the requested record mode to its concrete implementation."""
    music_options = {
        "music_path": music_path,
        "music_backend": music_backend,
        "music_delay": music_delay,
        "music_start": music_start,
        "music_stream_name": music_stream_name,
        "music_volume": music_volume,
    }
    if mode == "raw":
        return _record_raw(iface, group, out_path, control_dt=control_dt, **music_options)
    if mode == "teach_upper":
        if group != "upper":
            raise ValueError("mode 'teach_upper' only supports group=upper in v1")
        return _record_teach_upper(
            iface,
            out_path,
            auto_hold=auto_hold,
            control_dt_override=control_dt,
            **music_options,
        )
    if mode == "right_hold_left":
        if group not in {"upper", "right"}:
            raise ValueError("mode 'right_hold_left' only supports group=upper or group=right")
        return _record_right_hold_left(
            iface,
            out_path,
            output_group=group,
            auto_hold=auto_hold,
            control_dt_override=control_dt,
            **music_options,
        )
    if mode == "lock_forearm":
        if group != "upper":
            raise ValueError("mode 'lock_forearm' only supports group=upper in v1")
        if auto_hold:
            raise ValueError("mode 'lock_forearm' does not support --auto-hold")
        return _record_lock_forearm(iface, out_path, control_dt_override=control_dt, **music_options)
    raise ValueError(f"unknown record mode: {mode}")


def _record_raw(
    iface,
    group,
    out_path,
    control_dt=None,
    music_path=None,
    music_backend="auto",
    music_delay=0.0,
    music_start="recording",
    music_stream_name="music",
    music_volume=DEFAULT_AUDIO_VOLUME,
):
    path = _as_path(out_path)
    joints = get_group_joints(group)
    session = RobotSession(iface=iface, enable_pub=False)
    session.wait_lowstate()

    # Raw mode only samples state and never takes over arm_sdk.
    dt = _resolve_control_dt(1.0 / 50.0, control_dt)
    t0 = time.time()
    music_playback = _start_record_music(
        music_path,
        backend=music_backend,
        delay=music_delay,
        start=music_start,
        record_start_delay=0.0,
        stream_name=music_stream_name,
        volume=music_volume,
    )
    count = 0

    print(f"[RECORD] raw group={group} -> {path}")
    print(f"[RECORD] control_dt={dt:.4f}s (~{1.0 / dt:.1f} Hz)")
    print("[RECORD] Ctrl+C to stop")

    try:
        with path.open("w", encoding="utf-8") as handle:
            while True:
                now = time.time()
                row = {
                    "t": now - t0,
                    "group": group,
                    "joints": joints,
                    "q": session.get_joint_q(joints),
                    "dq": session.get_joint_dq(joints),
                    "tau_est": session.get_joint_tau_est(joints),
                }
                _save_jsonl_row(handle, row)
                count += 1
                if count % 10 == 0:
                    print(f"\r\033[K[RECORD] {group} frame={count} t={row['t']:.2f}s", end="", flush=True)
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[RECORD] raw recording stopped")
    finally:
        _stop_record_music(music_playback)


def _record_teach_upper(
    iface,
    out_path,
    auto_hold=False,
    control_dt_override=None,
    music_path=None,
    music_backend="auto",
    music_delay=0.0,
    music_start="recording",
    music_stream_name="music",
    music_volume=DEFAULT_AUDIO_VOLUME,
):
    path = _as_path(out_path)
    profile = get_record_profile("teach_upper")
    joints = with_waist_support(UPPER_JOINTS)
    output_joints = list(UPPER_JOINTS)
    output_local_idx = _idx_in(joints, output_joints)

    hold_time = float(profile["hold_time"])
    blend_time = float(profile["blend_time"])
    control_dt = _resolve_control_dt(profile["control_dt"], control_dt_override)
    record_start_delay = hold_time + blend_time
    lock_kp = float(profile["lock_kp"])
    lock_kd = float(profile["lock_kd"])

    kp_hold = _prepend_waist_support(profile["kp_hold"], profile.get("waist_support_kp_hold", 30.0))
    kd_hold = _prepend_waist_support(profile["kd_hold"], profile.get("waist_support_kd_hold", 1.0))
    kp_final = _prepend_waist_support(profile["kp_final"], profile.get("waist_support_kp_final", 30.0))
    kd_final = _prepend_waist_support(profile["kd_final"], profile.get("waist_support_kd_final", 1.0))

    session = RobotSession(iface=iface, enable_pub=True)
    session.wait_lowstate()
    q_hold = np.array(session.get_joint_q(joints), dtype=float)
    q_lock = q_hold.copy()
    support_local_idx = _idx_in(joints, WAIST_SUPPORT_JOINTS)
    locked_local_idx = support_local_idx + _idx_in(joints, profile["locked_joints"])
    active_local_idx = [idx for idx in range(len(joints)) if idx not in locked_local_idx]
    auto_hold_state = _build_auto_hold_state(profile, active_local_idx, enabled=auto_hold)
    q_last = q_hold.copy()

    print(f"[RECORD] teach_upper -> {path}")
    print("[RECORD] phase 1: hold current pose")
    print("[RECORD] phase 2: slowly blend to easy-drag mode")
    print(f"[RECORD] hold waist support + lock WristPitch/WristYaw local idx: {locked_local_idx}")
    print(f"[RECORD] control_dt={control_dt:.4f}s (~{1.0 / control_dt:.1f} Hz)")
    if auto_hold_state is not None:
        print("[RECORD] auto-hold enabled: arm will freeze the current pose after it settles")
    print("[RECORD] Ctrl+C to stop, save, and release arm_sdk.")

    t0 = time.time()
    music_playback = _start_record_music(
        music_path,
        backend=music_backend,
        delay=music_delay,
        start=music_start,
        record_start_delay=record_start_delay,
        stream_name=music_stream_name,
        volume=music_volume,
    )
    count = 0
    recording_started = False

    try:
        with path.open("w", encoding="utf-8") as handle:
            while True:
                elapsed = time.time() - t0
                q_now = np.array(session.get_joint_q(joints), dtype=float)

                if elapsed < hold_time:
                    q_cmd = q_hold.copy()
                    kp = kp_hold.copy()
                    kd = kd_hold.copy()
                elif elapsed < hold_time + blend_time:
                    raw = (elapsed - hold_time) / blend_time
                    s = raw * raw
                    q_cmd = (1.0 - s) * q_hold + s * q_now
                    kp = _lerp(kp_hold, kp_final, s)
                    kd = _lerp(kd_hold, kd_final, s)
                else:
                    q_cmd = q_now.copy()
                    kp = kp_final.copy()
                    kd = kd_final.copy()

                dq_now = None
                if auto_hold_state is not None and elapsed >= record_start_delay:
                    dq_now = np.array(session.get_joint_dq(joints), dtype=float)
                    transition = _update_auto_hold_state(auto_hold_state, q_now, dq_now, elapsed)
                    if transition == "hold":
                        print("\n[RECORD] auto-hold -> hold")
                    elif transition == "drag":
                        print("\n[RECORD] auto-hold -> drag")
                    _apply_auto_hold_targets(q_cmd, kp, kd, auto_hold_state, kp_hold, kd_hold)

                # Teach mode holds waist roll/pitch and keeps both wrist pitch/yaw pairs fixed.
                _apply_joint_locks(q_cmd, kp, kd, q_lock, locked_local_idx, lock_kp, lock_kd)

                q_last = q_cmd.copy()
                session.send_arm_q(joints, q_cmd.tolist(), kp=kp.tolist(), kd=kd.tolist())

                if elapsed >= record_start_delay:
                    if not recording_started:
                        print("\n[RECORD] recording started")
                        recording_started = True

                    q_meas = session.get_joint_q(joints)
                    dq_meas = session.get_joint_dq(joints)
                    tau_meas = session.get_joint_tau_est(joints)
                    if auto_hold_state is not None and auto_hold_state["mode"] == "hold":
                        for idx in auto_hold_state["active_local_idx"]:
                            q_meas[idx] = float(auto_hold_state["q_ref"][idx])
                            dq_meas[idx] = 0.0
                    for idx in locked_local_idx:
                        q_meas[idx] = float(q_lock[idx])
                        dq_meas[idx] = 0.0

                    row = {
                        "t": elapsed - record_start_delay,
                        "group": "upper",
                        "joints": output_joints,
                        "q": [q_meas[idx] for idx in output_local_idx],
                        "dq": [dq_meas[idx] for idx in output_local_idx],
                        "tau_est": [tau_meas[idx] for idx in output_local_idx],
                    }
                    _save_jsonl_row(handle, row)
                    count += 1
                    if count % 10 == 0:
                        print(
                            "\r\033[K" + _format_upper_status(row["q"], joints),
                            end="",
                            flush=True,
                        )

                time.sleep(control_dt)
    except KeyboardInterrupt:
        print("\n[RECORD] interrupted by user")
    finally:
        _stop_record_music(music_playback)
        print("[RECORD] release arm_sdk...")
        session.release_arm_sdk(
            joints,
            q_last.tolist(),
            dt=control_dt,
            kp=kp_final.tolist(),
            kd=kd_final.tolist(),
        )
        print("[RECORD] done")


def _record_right_hold_left(
    iface,
    out_path,
    output_group="upper",
    auto_hold=False,
    control_dt_override=None,
    music_path=None,
    music_backend="auto",
    music_delay=0.0,
    music_start="recording",
    music_stream_name="music",
    music_volume=DEFAULT_AUDIO_VOLUME,
):
    path = _as_path(out_path)
    profile = get_record_profile("right_hold_left")
    joints = with_waist_support(UPPER_JOINTS)
    idx_right = _idx_in(joints, RIGHT_ARM_JOINTS)
    output_joints = list(UPPER_JOINTS) if output_group == "upper" else list(RIGHT_ARM_JOINTS)
    output_local_idx = _idx_in(joints, output_joints)

    hold_time = float(profile["hold_time"])
    blend_time = float(profile["blend_time"])
    control_dt = _resolve_control_dt(profile["control_dt"], control_dt_override)
    record_start_delay = hold_time + blend_time
    lock_kp = float(profile["lock_kp"])
    lock_kd = float(profile["lock_kd"])

    kp_hold = _prepend_waist_support(profile["kp_hold"], profile.get("waist_support_kp_hold", 30.0))
    kd_hold = _prepend_waist_support(profile["kd_hold"], profile.get("waist_support_kd_hold", 1.0))
    kp_drag = kp_hold.copy()
    kd_drag = kd_hold.copy()
    kp_drag[idx_right] = np.array(profile["kp_drag_right"], dtype=float)
    kd_drag[idx_right] = np.array(profile["kd_drag_right"], dtype=float)

    session = RobotSession(iface=iface, enable_pub=True)
    session.wait_lowstate()
    q_hold = np.array(session.get_joint_q(joints), dtype=float)
    q_lock = q_hold.copy()
    support_local_idx = _idx_in(joints, WAIST_SUPPORT_JOINTS)
    locked_local_idx = support_local_idx + _idx_in(joints, profile["locked_joints"])
    active_local_idx = [idx for idx in idx_right if idx not in locked_local_idx]
    auto_hold_state = _build_auto_hold_state(profile, active_local_idx, enabled=auto_hold)
    q_last = q_hold.copy()

    print(f"[RECORD] right_hold_left -> {path}")
    print(
        f"[RECORD] mode: LEFT arm hold, RIGHT arm easy-drag, save group={output_group} "
        f"({len(output_joints)} joints)"
    )
    print(f"[RECORD] hold waist support + lock active WristPitch/WristYaw local idx: {locked_local_idx}")
    print(f"[RECORD] control_dt={control_dt:.4f}s (~{1.0 / control_dt:.1f} Hz)")
    if auto_hold_state is not None:
        print("[RECORD] auto-hold enabled: active arm will freeze the current pose after it settles")
    print("[RECORD] Ctrl+C to stop, save, and release arm_sdk.")

    t0 = time.time()
    music_playback = _start_record_music(
        music_path,
        backend=music_backend,
        delay=music_delay,
        start=music_start,
        record_start_delay=record_start_delay,
        stream_name=music_stream_name,
        volume=music_volume,
    )
    count = 0
    recording_started = False

    try:
        with path.open("w", encoding="utf-8") as handle:
            while True:
                elapsed = time.time() - t0
                q_now = np.array(session.get_joint_q(joints), dtype=float)

                q_cmd = q_hold.copy()
                kp = kp_hold.copy()
                kd = kd_hold.copy()

                if elapsed < hold_time:
                    q_cmd[idx_right] = q_hold[idx_right]
                    kp[idx_right] = kp_hold[idx_right]
                    kd[idx_right] = kd_hold[idx_right]
                elif elapsed < hold_time + blend_time:
                    raw = (elapsed - hold_time) / blend_time
                    s = raw * raw
                    q_cmd[idx_right] = (1.0 - s) * q_hold[idx_right] + s * q_now[idx_right]
                    kp[idx_right] = _lerp(kp_hold[idx_right], kp_drag[idx_right], s)
                    kd[idx_right] = _lerp(kd_hold[idx_right], kd_drag[idx_right], s)
                else:
                    q_cmd[idx_right] = q_now[idx_right]
                    kp[idx_right] = kp_drag[idx_right]
                    kd[idx_right] = kd_drag[idx_right]

                dq_now = None
                if auto_hold_state is not None and elapsed >= record_start_delay:
                    dq_now = np.array(session.get_joint_dq(joints), dtype=float)
                    transition = _update_auto_hold_state(auto_hold_state, q_now, dq_now, elapsed)
                    if transition == "hold":
                        print("\n[RECORD] auto-hold -> hold")
                    elif transition == "drag":
                        print("\n[RECORD] auto-hold -> drag")
                    _apply_auto_hold_targets(q_cmd, kp, kd, auto_hold_state, kp_hold, kd_hold)

                # Single-arm mode holds waist roll/pitch and locks the active right wrist pitch/yaw pair.
                _apply_joint_locks(q_cmd, kp, kd, q_lock, locked_local_idx, lock_kp, lock_kd)

                q_last = q_cmd.copy()
                session.send_arm_q(joints, q_cmd.tolist(), kp=kp.tolist(), kd=kd.tolist())

                if elapsed >= record_start_delay:
                    if not recording_started:
                        print("\n[RECORD] recording started")
                        recording_started = True

                    q_meas = np.array(session.get_joint_q(joints), dtype=float)
                    dq_meas = np.array(session.get_joint_dq(joints), dtype=float)
                    tau_meas = np.array(session.get_joint_tau_est(joints), dtype=float)
                    if auto_hold_state is not None and auto_hold_state["mode"] == "hold":
                        for idx in auto_hold_state["active_local_idx"]:
                            q_meas[idx] = float(auto_hold_state["q_ref"][idx])
                            dq_meas[idx] = 0.0

                    q_rec = q_hold.copy()
                    q_rec[idx_right] = q_meas[idx_right]
                    q_rec[locked_local_idx] = q_lock[locked_local_idx]

                    dq_rec = np.zeros_like(q_rec)
                    dq_rec[idx_right] = dq_meas[idx_right]
                    dq_rec[locked_local_idx] = 0.0

                    tau_rec = np.zeros_like(q_rec)
                    tau_rec[idx_right] = tau_meas[idx_right]

                    q_out = q_rec[output_local_idx].tolist()
                    dq_out = dq_rec[output_local_idx].tolist()
                    tau_out = tau_rec[output_local_idx].tolist()

                    row = {
                        "t": elapsed - record_start_delay,
                        "group": output_group,
                        "joints": output_joints,
                        "q": q_out,
                        "dq": dq_out,
                        "tau_est": tau_out,
                    }
                    _save_jsonl_row(handle, row)
                    count += 1
                    if count % 10 == 0:
                        print(
                            "\r\033[K" + _format_upper_status(q_rec, joints),
                            end="",
                            flush=True,
                        )

                time.sleep(control_dt)
    except KeyboardInterrupt:
        print("\n[RECORD] interrupted by user")
    finally:
        _stop_record_music(music_playback)
        print("[RECORD] release arm_sdk...")
        session.release_arm_sdk(
            joints,
            q_last.tolist(),
            dt=control_dt,
            kp=kp_hold.tolist(),
            kd=kd_hold.tolist(),
        )
        print("[RECORD] done")


def _record_lock_forearm(
    iface,
    out_path,
    control_dt_override=None,
    music_path=None,
    music_backend="auto",
    music_delay=0.0,
    music_start="recording",
    music_stream_name="music",
    music_volume=DEFAULT_AUDIO_VOLUME,
):
    path = _as_path(out_path)
    profile = get_record_profile("lock_forearm")
    joints = with_waist_support(UPPER_JOINTS)
    output_joints = list(UPPER_JOINTS)
    output_local_idx = _idx_in(joints, output_joints)

    hold_time = float(profile["hold_time"])
    blend_time = float(profile["blend_time"])
    control_dt = _resolve_control_dt(profile["control_dt"], control_dt_override)
    record_start_delay = hold_time + blend_time

    kp_hold = _prepend_waist_support(profile["kp_hold"], profile.get("waist_support_kp_hold", 30.0))
    kd_hold = _prepend_waist_support(profile["kd_hold"], profile.get("waist_support_kd_hold", 1.0))
    kp_final = _prepend_waist_support(profile["kp_final"], profile.get("waist_support_kp_final", 30.0))
    kd_final = _prepend_waist_support(profile["kd_final"], profile.get("waist_support_kd_final", 1.0))
    lock_kp = float(profile["lock_kp"])
    lock_kd = float(profile["lock_kd"])

    joint_to_local = {joint: idx for idx, joint in enumerate(joints)}
    locked_local_idx = [joint_to_local[j] for j in WAIST_SUPPORT_JOINTS if j in joint_to_local]
    locked_local_idx += [joint_to_local[j] for j in profile["locked_joints"] if j in joint_to_local]

    session = RobotSession(iface=iface, enable_pub=True)
    session.wait_lowstate()
    q_hold = np.array(session.get_joint_q(joints), dtype=float)
    q_lock = q_hold.copy()
    q_last = q_hold.copy()

    print(f"[RECORD] lock_forearm -> {path}")
    print(f"[RECORD] locked forearm local idx: {locked_local_idx}")
    print(f"[RECORD] control_dt={control_dt:.4f}s (~{1.0 / control_dt:.1f} Hz)")
    print("[RECORD] Ctrl+C to stop, save, and release arm_sdk.")

    t0 = time.time()
    music_playback = _start_record_music(
        music_path,
        backend=music_backend,
        delay=music_delay,
        start=music_start,
        record_start_delay=record_start_delay,
        stream_name=music_stream_name,
        volume=music_volume,
    )
    count = 0
    recording_started = False

    try:
        with path.open("w", encoding="utf-8") as handle:
            while True:
                elapsed = time.time() - t0
                q_now = np.array(session.get_joint_q(joints), dtype=float)

                if elapsed < hold_time:
                    q_cmd = q_hold.copy()
                    kp = kp_hold.copy()
                    kd = kd_hold.copy()
                elif elapsed < hold_time + blend_time:
                    raw = (elapsed - hold_time) / blend_time
                    s = raw * raw
                    q_cmd = (1.0 - s) * q_hold + s * q_now
                    kp = _lerp(kp_hold, kp_final, s)
                    kd = _lerp(kd_hold, kd_final, s)
                else:
                    q_cmd = q_now.copy()
                    kp = kp_final.copy()
                    kd = kd_final.copy()

                # Keep waist roll/pitch and all configured wrist axes pinned near their start angles.
                _apply_joint_locks(q_cmd, kp, kd, q_lock, locked_local_idx, lock_kp, lock_kd)

                q_last = q_cmd.copy()
                session.send_arm_q(joints, q_cmd.tolist(), kp=kp.tolist(), kd=kd.tolist())

                if elapsed >= record_start_delay:
                    if not recording_started:
                        print("\n[RECORD] recording started")
                        recording_started = True

                    q_meas = session.get_joint_q(joints)
                    dq_meas = session.get_joint_dq(joints)
                    tau_meas = session.get_joint_tau_est(joints)
                    for idx in locked_local_idx:
                        q_meas[idx] = float(q_lock[idx])
                        dq_meas[idx] = 0.0

                    row = {
                        "t": elapsed - record_start_delay,
                        "group": "upper",
                        "joints": output_joints,
                        "q": [q_meas[idx] for idx in output_local_idx],
                        "dq": [dq_meas[idx] for idx in output_local_idx],
                        "tau_est": [tau_meas[idx] for idx in output_local_idx],
                    }
                    _save_jsonl_row(handle, row)
                    count += 1
                    if count % 10 == 0:
                        print(
                            "\r\033[K" + _format_upper_status(row["q"], joints, include_wrists=True),
                            end="",
                            flush=True,
                        )

                time.sleep(control_dt)
    except KeyboardInterrupt:
        print("\n[RECORD] interrupted by user")
    finally:
        _stop_record_music(music_playback)
        print("[RECORD] release arm_sdk...")
        session.release_arm_sdk(
            joints,
            q_last.tolist(),
            dt=control_dt,
            kp=kp_final.tolist(),
            kd=kd_final.tolist(),
        )
        print("[RECORD] done")
