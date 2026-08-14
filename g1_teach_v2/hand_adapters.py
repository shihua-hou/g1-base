"""Optional DDS-backed hand adapters for script playback.

This module keeps hand-specific topics, message types, preset definitions, and
value mappings away from the arm-focused teaching core so `g1_teach_v2` can
stay arm-first while still gaining scriptable dexterous-hand support.
"""

import time


HAND_BACKENDS = ("none", "inspire_ftp_right")
HAND_CHANNELS = (
    "pinky",
    "ring",
    "middle",
    "index",
    "thumb_bend",
    "thumb_rotation",
)

HAND_OPEN_ANGLE = 200
HAND_CLOSE_ANGLE = 800
HAND_DEFAULT_DURATION = 0.4
HAND_DEFAULT_FPS = 20.0
HAND_READY_TIMEOUT = 2.0
HAND_ANGLE_MODE = 0b0001
HAND_THUMB_ROTATION_NEUTRAL = 0.5

# 第一版先提供稳定的绝对手型，方便演讲脚本直接复用。
HAND_PRESETS = {
    "open_all": {
        "pinky": 0.0,
        "ring": 0.0,
        "middle": 0.0,
        "index": 0.0,
        "thumb_bend": 0.0,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
    "close_all": {
        "pinky": 1.0,
        "ring": 1.0,
        "middle": 1.0,
        "index": 1.0,
        "thumb_bend": 1.0,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
    "relaxed": {
        "pinky": 0.35,
        "ring": 0.35,
        "middle": 0.35,
        "index": 0.35,
        "thumb_bend": 0.35,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
    "count_1": {
        "pinky": 1.0,
        "ring": 1.0,
        "middle": 1.0,
        "index": 0.0,
        "thumb_bend": 1.0,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
    "count_2": {
        "pinky": 1.0,
        "ring": 1.0,
        "middle": 0.0,
        "index": 0.0,
        "thumb_bend": 1.0,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
    "count_3": {
        "pinky": 1.0,
        "ring": 0.0,
        "middle": 0.0,
        "index": 0.0,
        "thumb_bend": 1.0,
        "thumb_rotation": HAND_THUMB_ROTATION_NEUTRAL,
    },
}


def _clamp01(value):
    return max(0.0, min(1.0, float(value)))


def normalize_hand_targets(targets):
    """Validate one partial hand-target dictionary and coerce values to float."""
    if not isinstance(targets, dict) or not targets:
        raise ValueError("hand targets must be a non-empty object")

    normalized = {}
    for key, value in targets.items():
        if key not in HAND_CHANNELS:
            raise ValueError(f"unsupported hand target key: {key}")
        value = float(value)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"hand target {key} must be within [0.0, 1.0]")
        normalized[key] = value
    return normalized


def validate_hand_preset(name):
    """Return a normalized preset name or raise if it is unknown."""
    name = str(name).strip()
    if name not in HAND_PRESETS:
        raise ValueError(f"unknown hand preset: {name}")
    return name


def format_hand_targets(targets):
    """Format targets in a stable channel order for logs and dry-run output."""
    parts = []
    for key in HAND_CHANNELS:
        if key in targets:
            parts.append(f"{key}={float(targets[key]):.2f}")
    return ", ".join(parts)


def describe_hand_step(step, duration):
    """Describe one hand step without requiring any live DDS dependencies."""
    if "preset" in step:
        return f"preset={step['preset']} duration={duration:.2f}s"
    return f"targets={format_hand_targets(step['targets'])} duration={duration:.2f}s"


class InspireFtpRightAdapter:
    """DDS-only adapter for the current Inspire FTP right-hand bridge."""

    backend_name = "inspire_ftp_right"

    def __init__(self, fps=HAND_DEFAULT_FPS, ready_timeout=HAND_READY_TIMEOUT):
        # 延迟导入 SDK 类型，这样 arm-only 和 dry-run 场景不依赖因时手环境。
        from inspire_sdkpy import inspire_dds, inspire_hand_defaut
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber

        self._ctrl_factory = inspire_hand_defaut.get_inspire_hand_ctrl
        self._pub = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self._pub.Init()
        self._sub = ChannelSubscriber("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)
        self._sub.Init(self._state_callback, 10)

        self._fps = float(fps)
        self._ready_timeout = float(ready_timeout)
        self._state_msg = None
        self._last_targets = dict(HAND_PRESETS["relaxed"])

    def _state_callback(self, msg):
        self._state_msg = msg

    def _wait_ready(self, timeout=None):
        timeout = self._ready_timeout if timeout is None else float(timeout)
        start = time.time()
        while self._state_msg is None:
            if time.time() - start >= timeout:
                raise RuntimeError(
                    "timed out waiting for rt/inspire_hand/state/r; "
                    "start inspire_hand_ws/inspire_hand_sdk/example/Headless_driver_485_r.py first"
                )
            time.sleep(0.05)

    def _angles_to_normalized(self, angle_values):
        span = float(HAND_CLOSE_ANGLE - HAND_OPEN_ANGLE)
        if span <= 0.0:
            raise ValueError("invalid Inspire hand angle range")

        normalized = {}
        for key, angle in zip(HAND_CHANNELS, angle_values):
            normalized[key] = _clamp01((float(angle) - HAND_OPEN_ANGLE) / span)
        return normalized

    def _normalized_to_angles(self, targets_norm):
        span = float(HAND_CLOSE_ANGLE - HAND_OPEN_ANGLE)
        return [
            int(round(HAND_OPEN_ANGLE + _clamp01(targets_norm[key]) * span))
            for key in HAND_CHANNELS
        ]

    def _build_ctrl_msg(self, targets_norm):
        msg = self._ctrl_factory()
        msg.angle_set = self._normalized_to_angles(targets_norm)
        msg.mode = HAND_ANGLE_MODE
        return msg

    def _transition_to(self, full_targets, duration):
        if duration <= 0:
            raise ValueError("hand step duration must be > 0")

        state = self.read_state()
        source = state["normalized"] if state["normalized"] else dict(self._last_targets)
        steps = max(1, int(duration * self._fps))
        sleep_dt = duration / steps

        # 用短线性过渡代替“一次性跳目标”，这样数手势时更平顺一些。
        for step in range(steps):
            alpha = (step + 1) / steps
            interpolated = {
                key: (1.0 - alpha) * float(source[key]) + alpha * float(full_targets[key])
                for key in HAND_CHANNELS
            }
            self._pub.Write(self._build_ctrl_msg(interpolated))
            time.sleep(sleep_dt)

        self._last_targets = dict(full_targets)

    def is_ready(self):
        return self._state_msg is not None

    def read_state(self):
        self._wait_ready()
        angle_values = [int(v) for v in self._state_msg.angle_act]
        normalized = self._angles_to_normalized(angle_values) if angle_values else {}
        return {
            "angle_act": angle_values,
            "normalized": normalized,
        }

    def send_targets(self, targets_norm, duration=HAND_DEFAULT_DURATION):
        targets_norm = normalize_hand_targets(targets_norm)
        base_targets = self.read_state()["normalized"] or dict(self._last_targets)
        full_targets = dict(base_targets)
        full_targets.update(targets_norm)
        self._transition_to(full_targets, float(duration))
        return {
            "backend": self.backend_name,
            "targets": dict(full_targets),
            "duration": float(duration),
        }

    def apply_preset(self, name, duration=HAND_DEFAULT_DURATION):
        name = validate_hand_preset(name)
        full_targets = dict(HAND_PRESETS[name])
        self._transition_to(full_targets, float(duration))
        return {
            "backend": self.backend_name,
            "preset": name,
            "targets": dict(full_targets),
            "duration": float(duration),
        }

    def close(self):
        # 第一版没有额外线程，保留空 close 统一未来 backend 生命周期接口。
        return None


def create_hand_adapter(name):
    """Create one optional hand backend for script playback."""
    name = str(name or "none")
    if name == "none":
        return None
    if name == "inspire_ftp_right":
        return InspireFtpRightAdapter()
    raise ValueError(f"unknown hand backend: {name}")
