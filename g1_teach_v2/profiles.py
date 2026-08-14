"""Runtime tuning profiles for recording and playback.

The profile layer keeps control gains and timing out of the command handlers so
the same settings can be reused by motion playback, snapshots, and record modes.
"""

from .joints import G1JointIndex


PLAYBACK_PROFILE_NAMES = {"upper_playback", "upper_hold", "upper_waist_lock"}
RECORD_PROFILE_NAMES = {"teach_upper", "right_hold_left", "lock_forearm"}


SNAPSHOT_CAPTURE_PROFILE = {
    "hold_time": 0.8,
    "blend_time": 1.8,
    "control_dt": 0.01,
    "kp_hold": 35.0,
    "kd_hold": 0.6,
    "kp_soft": 8.0,
    "kd_soft": 0.22,
    "release_time": 1.0,
}


RECORD_PROFILES = {
    "teach_upper": {
        "hold_time": 1.0,
        "blend_time": 3.0,
        "control_dt": 0.01,
        "locked_joints": [
            G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
        ],
        "lock_kp": 60.0,
        "lock_kd": 1.5,
        "auto_hold_velocity_enter": 0.12,
        "auto_hold_velocity_exit": 0.18,
        "auto_hold_position_exit": 0.035,
        "auto_hold_dwell_time": 0.05,
        "kp_hold": [
            20.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
        ],
        "kd_hold": [
            0.20,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
        ],
        "kp_final": [
            9.0,
            11.0,
            11.0,
            11.0,
            11.0,
            9.0,
            10.0,
            10.0,
            11.0,
            11.0,
            11.0,
            11.0,
            7.0,
            10.0,
            10.0,
        ],
        "kd_final": [
            0.40,
            0.45,
            0.45,
            0.45,
            0.40,
            0.28,
            0.35,
            0.35,
            0.45,
            0.45,
            0.45,
            0.40,
            0.24,
            0.35,
            0.35,
        ],
    },
    "right_hold_left": {
        "hold_time": 1.0,
        "blend_time": 3.0,
        "control_dt": 0.01,
        "locked_joints": [
            G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
        ],
        "lock_kp": 60.0,
        "lock_kd": 1.5,
        "auto_hold_velocity_enter": 0.12,
        "auto_hold_velocity_exit": 0.18,
        "auto_hold_position_exit": 0.035,
        "auto_hold_dwell_time": 0.05,
        "kp_hold": [20.0] + [45.0] * 7 + [45.0] * 7,
        "kd_hold": [0.40] + [1.20] * 7 + [1.20] * 7,
        # Bias the active right arm toward lighter hand-guiding so "hand to chest"
        # style motions need less effort, while keeping wrist lock axes unchanged.
        "kp_drag_right": [9.0, 9.0, 8.5, 8.0, 6.0, 12.0, 12.0],
        "kd_drag_right": [0.26, 0.26, 0.24, 0.22, 0.18, 0.30, 0.30],
    },
    "lock_forearm": {
        "hold_time": 1.0,
        "blend_time": 3.0,
        "control_dt": 0.01,
        "kp_hold": [
            20.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
            42.0,
        ],
        "kd_hold": [
            0.20,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
            0.40,
        ],
        "kp_final": [
            10.0,
            12.0,
            12.0,
            12.0,
            12.0,
            42.0,
            42.0,
            42.0,
            12.0,
            12.0,
            12.0,
            12.0,
            42.0,
            42.0,
            42.0,
        ],
        "kd_final": [
            0.50,
            0.55,
            0.55,
            0.55,
            0.50,
            0.40,
            0.40,
            0.40,
            0.55,
            0.55,
            0.55,
            0.50,
            0.40,
            0.40,
            0.40,
        ],
        "locked_joints": [
            G1JointIndex.LeftWristRoll,
            G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightWristRoll,
            G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
        ],
        "lock_kp": 55.0,
        "lock_kd": 1.2,
    },
}


def get_playback_profile(name, group):
    """Return a lightweight playback profile for the requested joint group."""
    if name not in PLAYBACK_PROFILE_NAMES:
        raise ValueError(f"unknown playback profile: {name}")

    if name == "upper_playback":
        # Use slightly stiffer gains and more blend time when both arms move.
        # WaistRoll(13)/WaistPitch(14) 默认强锁死（kp=200），防止手臂运动触发平衡步进。
        # WaistYaw(12) 使用普通 kp，保留正常偏转自由度。
        return {
            "name": name,
            "control_dt": 0.01,
            "kp": 60.0 if group == "both" else 40.0,
            "kd": 1.2,
            "waist_support_kp": 200.0,          # 13/14 强锁死
            "waist_support_kd": 13.0,
            "blend_time": 2.0 if group == "both" else 1.5,
            "takeover_time": 0.18 if group in {"upper", "both"} else 0.12,
            "takeover_fade_time": 0.30 if group in {"upper", "both"} else 0.18,
            "takeover_kp": 65.0 if group in {"upper", "both"} else 52.0,
            "takeover_kd": 2.1,
            "waist_support_takeover_kp": 220.0,  # 接管阶段同步提高
            "waist_support_takeover_kd": 7.0,
            "motion_speed": 1.0,
            "release_time": 1.0,
        }

    if name == "upper_waist_lock":
        # 全腰锁死模式：在 upper_playback 基础上额外锁死 WaistYaw(12)。
        # 适用于打鼓等手臂大幅运动会带动腰部旋转的场景。
        # joint 12/13/14 全部强锁死，防止任何腰部自由度漂移。
        # waist_yaw_hold=True 表示回放时左除轨迹中 joint 12 的数据，将其锁死在初始位置。
        return {
            "name": name,
            "control_dt": 0.01,
            "kp": 60.0 if group == "both" else 40.0,
            "kd": 1.2,
            "waist_support_kp": 200.0,          # 13/14 强锁死
            "waist_support_kd": 6.0,
            "waist_yaw_kp": 200.0,              # 12 额外强锁死
            "waist_yaw_kd": 6.0,
            "waist_yaw_hold": True,             # 回放时差除 joint 12 轨迹，将其持守在初始位置
            "blend_time": 2.0 if group == "both" else 1.5,
            "takeover_time": 0.18 if group in {"upper", "both"} else 0.12,
            "takeover_fade_time": 0.30 if group in {"upper", "both"} else 0.18,
            "takeover_kp": 65.0 if group in {"upper", "both"} else 52.0,
            "takeover_kd": 2.1,
            "waist_support_takeover_kp": 220.0,
            "waist_support_takeover_kd": 7.0,
            "waist_yaw_takeover_kp": 220.0,
            "waist_yaw_takeover_kd": 7.0,
            "motion_speed": 1.0,
            "release_time": 1.0,
        }

    # upper_hold
    return {
        "name": name,
        "control_dt": 0.01,
        "kp": 40.0,
        "kd": 1.2,
        "waist_support_kp": 90.0,
        "waist_support_kd": 3.0,
        "blend_time": 1.5,
        "takeover_time": 0.25 if group in {"upper", "both"} else 0.15,
        "takeover_fade_time": 0.30 if group in {"upper", "both"} else 0.20,
        "takeover_kp": 65.0 if group in {"upper", "both"} else 52.0,
        "takeover_kd": 2.1,
        "waist_support_takeover_kp": 110.0,
        "waist_support_takeover_kd": 3.8,
        "motion_speed": 1.0,
        "release_time": 1.0,
    }


def get_record_profile(name):
    """Return one of the built-in record-mode tuning presets."""
    if name not in RECORD_PROFILES:
        raise ValueError(f"unknown record profile: {name}")
    return RECORD_PROFILES[name]


def get_snapshot_capture_profile():
    """Return the interactive soft-teach profile used by capture-snapshot."""
    return dict(SNAPSHOT_CAPTURE_PROFILE)
