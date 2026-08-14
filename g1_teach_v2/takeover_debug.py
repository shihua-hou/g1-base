"""Helpers for optional arm takeover diagnostics."""

import json


DEBUG_TAKEOVER_WINDOW = 0.30


def _round_scalar(value, digits=6):
    return round(float(value), digits)


def _round_vector(values, digits=6):
    return [_round_scalar(value, digits=digits) for value in values]


def _extract_imu_rpy(low_state):
    imu_state = getattr(low_state, "imu_state", None)
    rpy = getattr(imu_state, "rpy", None)
    if rpy is None:
        return None
    try:
        return _round_vector(rpy)
    except TypeError:
        return None


def _build_sample(session, joints, q_cmd=None):
    low_state = session.wait_lowstate()
    joints = [int(joint) for joint in joints]
    q_meas = [float(low_state.motor_state[joint].q) for joint in joints]
    dq_meas = [float(low_state.motor_state[joint].dq) for joint in joints]
    tau_est = [float(low_state.motor_state[joint].tau_est) for joint in joints]
    sample = {
        "mode_machine": int(getattr(low_state, "mode_machine", -1)),
        "imu_rpy": _extract_imu_rpy(low_state),
        "joints": joints,
        "q_meas": _round_vector(q_meas),
        "dq_meas": _round_vector(dq_meas),
        "tau_est": _round_vector(tau_est),
    }
    if q_cmd is not None:
        q_cmd = [float(value) for value in q_cmd]
        sample["q_cmd"] = _round_vector(q_cmd)
        sample["q_cmd_err"] = _round_vector([cmd - meas for cmd, meas in zip(q_cmd, q_meas)])
    return sample


def create_takeover_debug_state(enabled, control_dt, label):
    if not enabled:
        return None
    max_ticks = max(1, int(DEBUG_TAKEOVER_WINDOW / float(control_dt)))
    stride = max(1, max_ticks // 6)
    return {
        "label": str(label),
        "tick": 0,
        "max_ticks": max_ticks,
        "stride": stride,
    }


def emit_takeover_debug_snapshot(session, label, phase, joints, q_cmd=None, extra=None):
    payload = {
        "label": str(label),
        "phase": str(phase),
        "tick": None,
    }
    payload.update(_build_sample(session, joints, q_cmd=q_cmd))
    if extra:
        payload.update(extra)
    print(f"[TAKEOVER-DEBUG] {json.dumps(payload, ensure_ascii=True)}")


def emit_takeover_debug_tick(session, state, phase, joints, q_cmd=None, extra=None):
    if state is None:
        return

    tick = int(state["tick"])
    if tick >= int(state["max_ticks"]):
        return

    should_log = tick == 0 or tick == int(state["max_ticks"]) - 1 or tick % int(state["stride"]) == 0
    if should_log:
        payload = {
            "label": state["label"],
            "phase": str(phase),
            "tick": tick,
        }
        payload.update(_build_sample(session, joints, q_cmd=q_cmd))
        if extra:
            payload.update(extra)
        print(f"[TAKEOVER-DEBUG] {json.dumps(payload, ensure_ascii=True)}")

    state["tick"] = tick + 1
