"""Sequential script runner for snapshot/motion/hold/hand/audio composition.

Scripts are treated as a blocking event stream: resolve each step in order,
execute snapshot/motion/hold/hand/audio, or recursively expand nested scripts.
"""

import json
import time
from pathlib import Path

from .audio_io import (
    DEFAULT_AUDIO_VOLUME,
    AudioPlayback,
    SUPPORTED_AUDIO_BACKENDS,
    describe_audio_step,
    normalize_audio_volume,
)
from .hand_adapters import (
    HAND_DEFAULT_DURATION,
    describe_hand_step,
    normalize_hand_targets,
    validate_hand_preset,
)
from .motion_io import describe_motion, load_motion, play_motion
from .snapshot_io import describe_snapshot, goto_snapshot, load_snapshot


ALLOWED_STEP_TYPES = {"snapshot", "motion", "hold", "script", "hand", "audio"}


def _resolve_path(base_dir, raw_path):
    """Resolve a step path relative to the current script file."""
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return candidate.resolve()
        
    # If base_dir is a 'scripts' directory inside 'movement', we can resolve relative to 'movement' instead.
    if base_dir.name == "scripts" and base_dir.parent.name == "movement":
        return (base_dir.parent / candidate).resolve()
        
    return (base_dir / candidate).resolve()


def _resolve_optional_float(payload, *keys):
    for key in keys:
        if key in payload:
            return float(payload[key])
    return None


def _resolve_optional_text(payload, *keys):
    for key in keys:
        if key in payload:
            value = payload[key]
            return None if value is None else str(value)
    return None


def _validate_hand_step(step, step_index, path):
    has_preset = "preset" in step
    has_targets = "targets" in step
    if has_preset == has_targets:
        raise ValueError(f"hand step {step_index} in {path} must set exactly one of preset or targets")

    if "duration" in step and float(step["duration"]) <= 0:
        raise ValueError(f"hand step {step_index} in {path} requires duration > 0")

    if has_preset:
        validate_hand_preset(step["preset"])
        return

    normalize_hand_targets(step["targets"])


def _validate_audio_step(step, step_index, path):
    if "path" not in step:
        raise ValueError(f"audio step {step_index} in {path} requires a path")
    if "delay" in step and float(step["delay"]) < 0:
        raise ValueError(f"audio step {step_index} in {path} requires delay >= 0")
    if "timeout" in step and float(step["timeout"]) <= 0:
        raise ValueError(f"audio step {step_index} in {path} requires timeout > 0")


def _attach_script_result(exc, result):
    """Propagate the latest safe release target through nested script errors."""
    if result is not None:
        exc.script_result = {
            "joints": list(result["joints"]),
            "q_final": list(result["q_final"]),
            "kp": result["kp"],
            "kd": result["kd"],
            "control_dt": float(result.get("control_dt", 0.01)),
        }
    return exc


def _extract_source_q(last_result, joints):
    """Reuse the last commanded pose when it covers the next step's joints."""
    if last_result is None:
        return None

    result_joints = [int(v) for v in last_result["joints"]]
    target_joints = [int(v) for v in joints]
    if result_joints == target_joints:
        return list(last_result["q_final"])

    result_index = {joint: idx for idx, joint in enumerate(result_joints)}
    try:
        return [float(last_result["q_final"][result_index[joint]]) for joint in target_joints]
    except KeyError:
        return None


def load_script(path):
    """Load a script json file and validate the minimal execution schema."""
    path = Path(path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"script file not found: {path}")

    with path.open("r", encoding="utf-8-sig") as handle:
        payload = json.load(handle)

    if payload.get("kind") != "script":
        raise ValueError(f"script kind must be 'script': {path}")
    if int(payload.get("version", 0)) != 1:
        raise ValueError(f"unsupported script version in {path}")

    steps = payload.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError(f"script steps must be a non-empty list: {path}")

    for idx, step in enumerate(steps, start=1):
        step_type = step.get("type")
        if step_type not in ALLOWED_STEP_TYPES:
            raise ValueError(f"unsupported step type on step {idx} in {path}: {step_type}")

        repeat = int(step.get("repeat", 1))
        if repeat <= 0:
            raise ValueError(f"repeat must be a positive integer on step {idx} in {path}")

        if step_type in {"snapshot", "motion", "script"} and "path" not in step:
            raise ValueError(f"step {idx} in {path} requires a path")
        if step_type == "hold" and "duration" not in step:
            raise ValueError(f"hold step {idx} in {path} requires duration")
        if step_type == "hand":
            _validate_hand_step(step, idx, path)
        if step_type == "audio":
            _validate_audio_step(step, idx, path)

    defaults = payload.get("defaults", {})
    default_blend_time = _resolve_optional_float(defaults, "blend_time", "blendtime")
    if default_blend_time is not None and default_blend_time < 0:
        raise ValueError(f"script default blend_time must be >= 0: {path}")

    return path, defaults, steps


def run_script(
    session,
    path,
    cli_profile=None,
    dry_run=False,
    release=True,
    _depth=0,
    hand_adapter=None,
    hand_backend_name="none",
    debug_takeover=False,
    takeover_tau_ff=True,
    audio_backend="auto",
    audio_volume=DEFAULT_AUDIO_VOLUME,
):
    """Run one script file, optionally printing the plan instead of touching the robot."""
    script_path, defaults, steps = load_script(path)
    base_dir = script_path.parent
    resolved_profile = cli_profile or defaults.get("profile") or "upper_playback"
    snapshot_duration = float(defaults.get("snapshot_duration", 1.5))
    motion_speed = float(defaults.get("motion_speed", 1.0))
    hand_duration = float(defaults.get("hand_duration", HAND_DEFAULT_DURATION))
    default_blend_time = _resolve_optional_float(defaults, "blend_time", "blendtime")
    default_motion_interpolation = _resolve_optional_text(
        defaults, "motion_interpolation", "motion_interp", "motion_interpolate"
    )
    default_snapshot_easing = _resolve_optional_text(
        defaults, "snapshot_ease", "snapshot_easing", "snapshot_ease_mode"
    )
    default_audio_backend = (
        _resolve_optional_text(defaults, "audio_backend", "audio_output") or audio_backend
    )
    default_audio_backend = str(default_audio_backend).strip().lower()
    default_audio_stream = _resolve_optional_text(defaults, "audio_stream", "audio_stream_name") or "music"
    default_audio_volume = _resolve_optional_float(defaults, "audio_volume", "music_volume", "volume")
    if default_audio_volume is None:
        default_audio_volume = audio_volume
    default_audio_volume = normalize_audio_volume(default_audio_volume)
    if default_audio_backend not in SUPPORTED_AUDIO_BACKENDS:
        raise ValueError(
            f"script default audio_backend must be one of {', '.join(SUPPORTED_AUDIO_BACKENDS)}: {script_path}"
        )
    if hand_duration <= 0:
        raise ValueError(f"script default hand_duration must be > 0: {script_path}")
    if default_blend_time is not None and default_blend_time < 0:
        raise ValueError(f"script default blend_time must be >= 0: {script_path}")
    indent = "  " * _depth

    if dry_run:
        default_blend_time_label = "profile" if default_blend_time is None else f"{default_blend_time:.2f}s"
        default_motion_interpolation_label = default_motion_interpolation or "linear"
        default_snapshot_easing_label = default_snapshot_easing or "minimum_jerk"
        print(
            f"{indent}[DRY-RUN] script={script_path.name} profile={resolved_profile} "
            f"default_snapshot_duration={snapshot_duration:.2f} "
            f"default_motion_speed={motion_speed:.2f} "
            f"default_blend_time={default_blend_time_label} "
            f"default_motion_interp={default_motion_interpolation_label} "
            f"default_snapshot_ease={default_snapshot_easing_label} "
            f"default_hand_duration={hand_duration:.2f} "
            f"hand_backend={hand_backend_name} "
            f"audio_backend={default_audio_backend} "
            f"audio_volume={default_audio_volume}"
        )

    last_result = None
    audio_playbacks = []
    run_error = None

    try:
        for step_index, step in enumerate(steps, start=1):
            repeat = int(step.get("repeat", 1))
            for repeat_idx in range(repeat):
                step_type = step["type"]

                if step_type == "hold":
                    duration = float(step["duration"])
                    if dry_run:
                        print(
                            f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                            f"type=hold duration={duration:.2f}s"
                        )
                    else:
                        print(f"{indent}[SCRIPT] hold {duration:.2f}s")
                        if last_result is None:
                            time.sleep(duration)
                        else:
                            hold_dt = float(last_result.get("control_dt", 0.01))
                            hold_steps = max(1, int(duration / hold_dt))
                            for _ in range(hold_steps):
                                session.send_arm_q(
                                    last_result["joints"],
                                    last_result["q_final"],
                                    kp=last_result["kp"],
                                    kd=last_result["kd"],
                                )
                                time.sleep(hold_dt)
                    continue

                if step_type == "hand":
                    duration = float(step.get("duration", hand_duration))
                    if dry_run:
                        print(
                            f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                            f"type=hand {describe_hand_step(step, duration)}"
                        )
                    else:
                        if hand_adapter is None:
                            raise RuntimeError(
                                "script contains hand steps but no hand backend is active; "
                                "rerun with --hand-backend inspire_ftp_right"
                            )
                        if "preset" in step:
                            hand_adapter.apply_preset(step["preset"], duration=duration)
                        else:
                            hand_adapter.send_targets(step["targets"], duration=duration)
                    continue

                if step_type == "audio":
                    resolved_path = _resolve_path(base_dir, step["path"])
                    if not resolved_path.exists():
                        raise FileNotFoundError(f"audio file not found: {resolved_path}")
                    step_audio_backend = _resolve_optional_text(step, "backend", "audio_backend")
                    if step_audio_backend is None:
                        step_audio_backend = default_audio_backend
                    step_audio_backend = str(step_audio_backend).strip().lower()
                    if step_audio_backend not in SUPPORTED_AUDIO_BACKENDS:
                        raise ValueError(
                            f"audio step {step_index} in {script_path} has unsupported backend: {step_audio_backend}"
                        )
                    delay = float(step.get("delay", 0.0))
                    async_play = bool(step.get("async", True))
                    stream_name = _resolve_optional_text(step, "stream", "stream_name")
                    if stream_name is None:
                        stream_name = default_audio_stream
                    timeout = float(step.get("timeout", 10.0))
                    step_audio_volume = _resolve_optional_float(step, "volume", "audio_volume")
                    if step_audio_volume is None:
                        step_audio_volume = default_audio_volume
                    step_audio_volume = normalize_audio_volume(step_audio_volume)

                    if dry_run:
                        print(
                            f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                            f"type=audio "
                            f"{describe_audio_step(resolved_path, backend=step_audio_backend, delay=delay, async_play=async_play, stream_name=stream_name, volume=step_audio_volume)}"
                        )
                    else:
                        print(
                            f"{indent}[SCRIPT] audio "
                            f"{describe_audio_step(resolved_path, backend=step_audio_backend, delay=delay, async_play=async_play, stream_name=stream_name, volume=step_audio_volume)}"
                        )
                        playback = AudioPlayback(
                            resolved_path,
                            backend=step_audio_backend,
                            delay=delay,
                            stream_name=stream_name,
                            timeout=timeout,
                            async_play=async_play,
                            volume=step_audio_volume,
                        ).start()
                        if async_play:
                            audio_playbacks.append(playback)
                    continue

                # Every non-hold/non-hand/non-audio step resolves to a concrete file path.
                resolved_path = _resolve_path(base_dir, step["path"])

                if step_type == "motion":
                    speed = float(step.get("speed", motion_speed))
                    step_blend_time = _resolve_optional_float(step, "blend_time", "blendtime")
                    if step_blend_time is None:
                        step_blend_time = default_blend_time
                    step_motion_interpolation = _resolve_optional_text(
                        step,
                        "interpolation",
                        "interp",
                        "motion_interpolation",
                        "motion_interp",
                    )
                    if step_motion_interpolation is None:
                        step_motion_interpolation = default_motion_interpolation
                    if step_motion_interpolation is None:
                        step_motion_interpolation = "linear"
                    if step_blend_time is not None and step_blend_time < 0:
                        raise ValueError(
                            f"motion step {step_index} in {script_path} requires blend_time >= 0"
                        )
                    trajectory = load_motion(resolved_path)
                    # Reuse the last commanded pose as the next segment's start so scripted
                    # transitions stay continuous instead of re-sampling a slightly sagged arm.
                    source_q = _extract_source_q(last_result, trajectory.joints)
                    if dry_run:
                        print(
                            f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                            f"type=motion "
                            f"{describe_motion(trajectory, profile_name=resolved_profile, speed=speed, blend_time=step_blend_time, interpolation=step_motion_interpolation)}"
                        )
                    else:
                        last_result = play_motion(
                            session,
                            trajectory,
                            profile_name=resolved_profile,
                            speed=speed,
                            dry_run=False,
                            release=False,
                            source_q=source_q,
                            source_result=last_result,
                            blend_time=step_blend_time,
                            interpolation=step_motion_interpolation,
                            debug_takeover=debug_takeover,
                            takeover_tau_ff=takeover_tau_ff,
                        )
                    continue

                if step_type == "snapshot":
                    duration_raw = step.get("duration") or step.get("snapshot_duration") or snapshot_duration
                    duration = float(duration_raw)
                    step_snapshot_easing = _resolve_optional_text(
                        step, "easing", "ease", "snapshot_ease", "snapshot_easing"
                    )
                    if step_snapshot_easing is None:
                        step_snapshot_easing = default_snapshot_easing
                    if step_snapshot_easing is None:
                        step_snapshot_easing = "minimum_jerk"
                    snapshot = load_snapshot(resolved_path)
                    # When chaining snapshots, keep continuity by starting from the last
                    # commanded pose rather than a freshly drooped readback when possible.
                    source_q = _extract_source_q(last_result, snapshot.joints)
                    if dry_run:
                        print(
                            f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                            f"type=snapshot "
                            f"{describe_snapshot(snapshot, duration=duration, profile_name=resolved_profile, easing=step_snapshot_easing)}"
                        )
                    else:
                        last_result = goto_snapshot(
                            session,
                            snapshot,
                            duration=duration,
                            profile_name=resolved_profile,
                            dry_run=False,
                            release=False,
                            source_q=source_q,
                            source_result=last_result,
                            easing=step_snapshot_easing,
                            debug_takeover=debug_takeover,
                            takeover_tau_ff=takeover_tau_ff,
                        )
                    continue

                if dry_run:
                    print(
                        f"{indent}  step={step_index} repeat={repeat_idx + 1}/{repeat} "
                        f"type=script path={resolved_path.name}"
                    )
                    run_script(
                        session=None,
                        path=resolved_path,
                        cli_profile=cli_profile,
                        dry_run=True,
                        release=False,
                        _depth=_depth + 1,
                        hand_adapter=None,
                        hand_backend_name=hand_backend_name,
                        debug_takeover=debug_takeover,
                        takeover_tau_ff=takeover_tau_ff,
                        audio_backend=audio_backend,
                        audio_volume=audio_volume,
                    )
                else:
                    nested_result = run_script(
                        session=session,
                        path=resolved_path,
                        cli_profile=cli_profile,
                        dry_run=False,
                        release=False,
                        _depth=_depth + 1,
                        hand_adapter=hand_adapter,
                        hand_backend_name=hand_backend_name,
                        debug_takeover=debug_takeover,
                        takeover_tau_ff=takeover_tau_ff,
                        audio_backend=audio_backend,
                        audio_volume=audio_volume,
                    )
                    if nested_result is not None:
                        last_result = nested_result

        return last_result
    except BaseException as exc:
        run_error = exc
        for playback in audio_playbacks:
            playback.cancel()
        partial = getattr(exc, "partial_result", None) or getattr(exc, "script_result", None)
        if partial is not None:
            last_result = {
                "joints": list(partial["joints"]),
                "q_final": list(partial["q_final"]),
                "kp": partial["kp"],
                "kd": partial["kd"],
                "control_dt": float(partial.get("control_dt", 0.01)),
            }
        raise _attach_script_result(exc, last_result)
    finally:
        if release and not dry_run and last_result is not None:
            # Only release arm_sdk once at the outermost level so nested scripts do not "drop"
            # control after every step.
            print(f"{indent}[SCRIPT] release arm_sdk...")
            session.release_arm_sdk(
                last_result["joints"],
                last_result["q_final"],
                kp=last_result["kp"],
                kd=last_result["kd"],
            )
        if not dry_run:
            audio_error = None
            for playback in audio_playbacks:
                try:
                    playback.join()
                except BaseException as exc:
                    if audio_error is None:
                        audio_error = exc
            if audio_error is not None and run_error is None:
                raise audio_error
