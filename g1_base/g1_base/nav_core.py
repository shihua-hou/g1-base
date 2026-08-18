import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from g1_base.common import (
    clamp,
    config_file,
    load_waypoints_from_yaml,
    normalize_angle,
    package_share_dir,
    quaternion_dict_from_yaw,
    route_file,
    yaml,
)
from g1_base.bt_tools import DEFAULT_NAV_TO_POSE_BT, inspect_bt_file, missing_bt_plugins
from g1_base.logging import get_logger as _get_file_logger
from g1_base.unitree_sdk_bridge import UnitreeSdkBridge

_nav_logger = _get_file_logger("nav_core")

try:
    from rclpy.parameter_client import AsyncParameterClient
except ImportError:  # pragma: no cover - AsyncParameterClient is unavailable on ROS 2 Humble
    AsyncParameterClient = None


@dataclass(frozen=True)
class WalkingModeConfig:
    """行走模式参数，区分固定腰部与解开腰部。"""

    mode_name: str = "locked_waist"
    speed_mode: int = 1
    linear_speed: float = 0.35
    fast_max_vel_x: float = 1.04
    fast_max_vel_theta: float = 1.2
    fast_acc_lim_x: float = 2.0
    fast_acc_lim_theta: float = 1.0
    slow_max_vel_x: float = 0.46
    slow_max_vel_theta: float = 0.8
    slow_acc_lim_x: float = 0.5
    slow_acc_lim_theta: float = 0.8
    max_cmd_step_vx: float = 0.10
    max_cmd_step_vy: float = 0.05
    max_cmd_step_wz: float = 0.15
    min_vx_threshold: float = 0.25
    min_vx_bypass_wz: float = 0.25
    rotate_wz_base: float = 0.785
    rotate_wz_min: float = 0.2
    rotate_wz_max: float = 1.2
    move_deadline_margin: float = 2.0
    rotate_deadline_margin: float = 1.0
    rotate_recovery_min_wz: float = 0.2
    rotate_recovery_max_wz: float = 1.0
    rotate_recovery_gain: float = 2.5
    config_path: str = ""


def load_walking_mode_config(path=None):
    """从 config/walking_mode.yaml 加载行走模式配置。"""
    config_path = os.path.expanduser(
        path
        or os.environ.get("G1_WALKING_MODE_FILE", "")
        or config_file("walking_mode.yaml")
    )
    if not os.path.exists(config_path):
        _nav_logger.info("[walking_mode] 配置文件不存在: %s，使用默认值", config_path)
        return WalkingModeConfig(config_path=config_path)
    if yaml is None:
        _nav_logger.warning("[walking_mode] PyYAML 不可用，使用默认值")
        return WalkingModeConfig(config_path=config_path)

    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return WalkingModeConfig(config_path=config_path)

    mode_name = str(data.get("walking_mode", "locked_waist")).strip()
    mode_data = data.get(mode_name, {})
    if not isinstance(mode_data, dict):
        mode_data = {}

    def _f(key, default):
        return _config_float(mode_data.get(key), default)

    def _i(key, default):
        return _config_int(mode_data.get(key), default)

    default = WalkingModeConfig()
    cfg = WalkingModeConfig(
        mode_name=mode_name,
        speed_mode=_i("speed_mode", default.speed_mode),
        linear_speed=_f("linear_speed", default.linear_speed),
        fast_max_vel_x=_f("fast_max_vel_x", default.fast_max_vel_x),
        fast_max_vel_theta=_f("fast_max_vel_theta", default.fast_max_vel_theta),
        fast_acc_lim_x=_f("fast_acc_lim_x", default.fast_acc_lim_x),
        fast_acc_lim_theta=_f("fast_acc_lim_theta", default.fast_acc_lim_theta),
        slow_max_vel_x=_f("slow_max_vel_x", default.slow_max_vel_x),
        slow_max_vel_theta=_f("slow_max_vel_theta", default.slow_max_vel_theta),
        slow_acc_lim_x=_f("slow_acc_lim_x", default.slow_acc_lim_x),
        slow_acc_lim_theta=_f("slow_acc_lim_theta", default.slow_acc_lim_theta),
        max_cmd_step_vx=_f("max_cmd_step_vx", default.max_cmd_step_vx),
        max_cmd_step_vy=_f("max_cmd_step_vy", default.max_cmd_step_vy),
        max_cmd_step_wz=_f("max_cmd_step_wz", default.max_cmd_step_wz),
        min_vx_threshold=_f("min_vx_threshold", default.min_vx_threshold),
        min_vx_bypass_wz=_f("min_vx_bypass_wz", default.min_vx_bypass_wz),
        rotate_wz_base=_f("rotate_wz_base", default.rotate_wz_base),
        rotate_wz_min=_f("rotate_wz_min", default.rotate_wz_min),
        rotate_wz_max=_f("rotate_wz_max", default.rotate_wz_max),
        move_deadline_margin=_f("move_deadline_margin", default.move_deadline_margin),
        rotate_deadline_margin=_f("rotate_deadline_margin", default.rotate_deadline_margin),
        rotate_recovery_min_wz=_f("rotate_recovery_min_wz", default.rotate_recovery_min_wz),
        rotate_recovery_max_wz=_f("rotate_recovery_max_wz", default.rotate_recovery_max_wz),
        rotate_recovery_gain=_f("rotate_recovery_gain", default.rotate_recovery_gain),
        config_path=config_path,
    )
    _nav_logger.info(
        "[walking_mode] 已加载配置: mode=%s, speed_mode=%d, linear_speed=%.2f, "
        "fast_vx=%.2f, slow_vx=%.2f, file=%s",
        cfg.mode_name, cfg.speed_mode, cfg.linear_speed,
        cfg.fast_max_vel_x, cfg.slow_max_vel_x, config_path,
    )
    return cfg


SPEED_MODE = 1
MIN_VX_THRESHOLD = 0.25
MIN_VX_BYPASS_WZ = 0.25
PLANNER_TIMEOUT = 0.5
PLANNER_CMD_TOPIC = "/cmd_vel"
CONTROL_LOOP_HZ = 20.0
ROTATE_RECOVERY_TRIGGER_REQUESTS = 2
REVERSE_REPLAN_COOLDOWN = 3.0
ROTATE_RECOVERY_TIMEOUT = 2.0
ROTATE_RECOVERY_COOLDOWN = 1.0
ROTATE_RECOVERY_MAX_RETRIES = 2
ROTATE_RECOVERY_YAW_THRESHOLD = math.radians(15.0)
ROTATE_RECOVERY_MIN_WZ = 0.2
ROTATE_RECOVERY_MAX_WZ = 1.0
ROTATE_RECOVERY_GAIN = 2.5
ROTATE_RECOVERY_FALLBACK_YAW = math.radians(30.0)
ROTATE_RECOVERY_MIN_PLANNER_WZ = 0.15
YIELD_HOLD_TIMEOUT = 0.7
UNSMOOTHED_COMMAND_SOURCES = {
    "manual",
    "rotate_stop",
    "rotate_to_yaw_settle",
    "close_obstacle_hold",
    "goal_occupied_wait",
}
MAX_CMD_STEP_VX = 0.10
MAX_CMD_STEP_VY = 0.05
MAX_CMD_STEP_WZ = 0.15
LATERAL_ASSIST_SCAN_TOPIC = "/scan"
LATERAL_ASSIST_TRIGGER_DISTANCE = 2.50
LATERAL_ASSIST_CLEAR_DISTANCE = 2.70
LATERAL_ASSIST_FRONT_HALF_WIDTH = 0.45
LATERAL_ASSIST_SIDE_LOOKAHEAD = 2.80
LATERAL_ASSIST_SIDE_MIN_CLEARANCE = 0.55
LATERAL_ASSIST_SIDE_BAND_MIN = 0.35
LATERAL_ASSIST_SIDE_BAND_MAX = 1.20
LATERAL_ASSIST_VY = 0.22
LATERAL_ASSIST_MAX_FORWARD_VX = 0.15
LATERAL_ASSIST_SCAN_TIMEOUT = 0.6
CLOSE_OBSTACLE_EVENT_TOPIC = "/g1_safety/close_obstacle_event"
YIELD_PROMPT_EVENT_TOPIC = "/g1_safety/yield_prompt_event"
CLOSE_OBSTACLE_MIN_RANGE = 0.05
CLOSE_OBSTACLE_TRIGGER_DISTANCE = 0.35
CLOSE_OBSTACLE_RELEASE_DISTANCE = 0.75
CLOSE_OBSTACLE_FRONT_HALF_WIDTH = 0.30
CLOSE_OBSTACLE_FRONT_ANGLE_MIN = -0.66
CLOSE_OBSTACLE_FRONT_ANGLE_MAX = 0.70
CLOSE_OBSTACLE_MIN_POINTS = 3
CLOSE_OBSTACLE_TRIGGER_PERCENTILE = 0.10
CLOSE_OBSTACLE_MIN_CONSECUTIVE_FRAMES = 2
CLOSE_OBSTACLE_CLEAR_DURATION = 1.0
CLOSE_OBSTACLE_REPLAN_ON_CLEAR = True
CLOSE_OBSTACLE_REPLAN_COOLDOWN = 3.0
CLOSE_OBSTACLE_REPLAN_GOAL_MARGIN = 0.15
GOAL_OCCUPANCY_ENABLED = True
GOAL_OCCUPANCY_START_DISTANCE = 1.5
GOAL_OCCUPANCY_WAIT_TIMEOUT = 10.0
GOAL_OCCUPANCY_CLEAR_DURATION = 1.0
GOAL_OCCUPANCY_SPEAK_INTERVAL = 4.0
GOAL_OCCUPANCY_COSTMAP_TOPIC = "/local_costmap/costmap_raw"
GOAL_OCCUPANCY_COSTMAP_TIMEOUT = 1.0
GOAL_OCCUPANCY_RADIUS = 0.35
GOAL_OCCUPANCY_COST_THRESHOLD = 253
GOAL_OCCUPANCY_MIN_OCCUPIED_CELLS = 2
GOAL_OCCUPANCY_RETRY_HOLD_DURATION = 6.0
GOAL_OCCUPANCY_RETRY_WINDOW = 5.0
GOAL_OCCUPANCY_MAX_RETRIES = 2

PATH_ALIGN_MIN_POINTS = 2
PATH_ALIGN_MIN_SEGMENT = 0.15
PATH_ALIGN_TIMEOUT = 4.0
PATH_ALIGN_YAW_THRESHOLD = math.radians(15.0)
FORCED_REPLAN_MAX_ATTEMPTS = 6
STALL_REPLAN_TIMEOUT = 4.0
STALL_REPLAN_DISTANCE_EPS = 0.12
STALL_REPLAN_MIN_GOAL_DISTANCE = 0.8
STALL_EXECUTED_VX_EPS = 0.05
STALL_EXECUTED_WZ_EPS = 0.18
PLANNER_SILENT_ABORT_TIMEOUT = 5.0
SDK_SLOW_ABORT_P95_MS = 500.0
SDK_SLOW_ABORT_MIN_ACTIVE_SAMPLES = 5
SDK_SLOW_ABORT_ACTIVE_SAMPLE_MAX_AGE = 2.0
INITIAL_PLANNER_CMD_GRACE = 3.0

FAST_MAX_VEL_X = 1.04
SLOW_MAX_VEL_X = 0.46
FAST_MAX_VEL_THETA = 1.2
SLOW_MAX_VEL_THETA = 0.8
FAST_ACC_LIM_X = 2.0
SLOW_ACC_LIM_X = 0.5
FAST_ACC_LIM_THETA = 1.0
SLOW_ACC_LIM_THETA = 0.8
FAST_PATH_DISTANCE_BIAS = 18.0
SLOW_PATH_DISTANCE_BIAS = 18.0
FAST_GOAL_DISTANCE_BIAS = 16.0
SLOW_GOAL_DISTANCE_BIAS = 22.0
FAST_OCCDIST_SCALE = 10.0
SLOW_OCCDIST_SCALE = 10.0
FAST_FOOTPRINT_SCALE = 14.0
SLOW_FOOTPRINT_SCALE = 14.0
SLOWDOWN_START_DISTANCE = 2.0
SLOWDOWN_FULL_PROFILE_DISTANCE = 0.6
SLOWDOWN_PROFILE_STEP = 0.1

NAV_MANAGER_READY_TOPIC = "/navigation_manager/ready"
NAV_MANAGER_ENSURE_SERVICE = "/navigation_manager/ensure_ready"
NAV_MANAGER_DISCOVERY_TIMEOUT = 5.0
NAV_MANAGER_READY_TOPIC_TIMEOUT = 2.0


@dataclass(frozen=True)
class MotionPolicyConfig:
    global_allow_lateral_motion: bool = False
    lateral_assist_enabled: bool = True
    lateral_assist_scan_topic: str = LATERAL_ASSIST_SCAN_TOPIC
    lateral_assist_trigger_distance: float = LATERAL_ASSIST_TRIGGER_DISTANCE
    lateral_assist_clear_distance: float = LATERAL_ASSIST_CLEAR_DISTANCE
    lateral_assist_front_half_width: float = LATERAL_ASSIST_FRONT_HALF_WIDTH
    lateral_assist_side_lookahead: float = LATERAL_ASSIST_SIDE_LOOKAHEAD
    lateral_assist_side_min_clearance: float = LATERAL_ASSIST_SIDE_MIN_CLEARANCE
    lateral_assist_side_band_min: float = LATERAL_ASSIST_SIDE_BAND_MIN
    lateral_assist_side_band_max: float = LATERAL_ASSIST_SIDE_BAND_MAX
    lateral_assist_vy: float = LATERAL_ASSIST_VY
    lateral_assist_max_forward_vx: float = LATERAL_ASSIST_MAX_FORWARD_VX
    lateral_assist_scan_timeout: float = LATERAL_ASSIST_SCAN_TIMEOUT
    close_obstacle_guard_enabled: bool = True
    close_obstacle_scan_topic: str = LATERAL_ASSIST_SCAN_TOPIC
    close_obstacle_min_range: float = CLOSE_OBSTACLE_MIN_RANGE
    close_obstacle_trigger_distance: float = CLOSE_OBSTACLE_TRIGGER_DISTANCE
    close_obstacle_release_distance: float = CLOSE_OBSTACLE_RELEASE_DISTANCE
    close_obstacle_front_half_width: float = CLOSE_OBSTACLE_FRONT_HALF_WIDTH
    close_obstacle_front_angle_min: float = CLOSE_OBSTACLE_FRONT_ANGLE_MIN
    close_obstacle_front_angle_max: float = CLOSE_OBSTACLE_FRONT_ANGLE_MAX
    close_obstacle_min_points: int = CLOSE_OBSTACLE_MIN_POINTS
    close_obstacle_trigger_percentile: float = CLOSE_OBSTACLE_TRIGGER_PERCENTILE
    close_obstacle_min_consecutive_frames: int = CLOSE_OBSTACLE_MIN_CONSECUTIVE_FRAMES
    close_obstacle_clear_duration: float = CLOSE_OBSTACLE_CLEAR_DURATION
    close_obstacle_replan_on_clear: bool = CLOSE_OBSTACLE_REPLAN_ON_CLEAR
    close_obstacle_replan_cooldown: float = CLOSE_OBSTACLE_REPLAN_COOLDOWN
    close_obstacle_replan_goal_margin: float = CLOSE_OBSTACLE_REPLAN_GOAL_MARGIN
    goal_occupancy_enabled: bool = GOAL_OCCUPANCY_ENABLED
    goal_occupancy_start_distance: float = GOAL_OCCUPANCY_START_DISTANCE
    goal_occupancy_wait_timeout: float = GOAL_OCCUPANCY_WAIT_TIMEOUT
    goal_occupancy_clear_duration: float = GOAL_OCCUPANCY_CLEAR_DURATION
    goal_occupancy_speak_interval: float = GOAL_OCCUPANCY_SPEAK_INTERVAL
    goal_occupancy_costmap_topic: str = GOAL_OCCUPANCY_COSTMAP_TOPIC
    goal_occupancy_costmap_timeout: float = GOAL_OCCUPANCY_COSTMAP_TIMEOUT
    goal_occupancy_radius: float = GOAL_OCCUPANCY_RADIUS
    goal_occupancy_cost_threshold: int = GOAL_OCCUPANCY_COST_THRESHOLD
    goal_occupancy_min_occupied_cells: int = GOAL_OCCUPANCY_MIN_OCCUPIED_CELLS
    goal_occupancy_retry_hold_duration: float = GOAL_OCCUPANCY_RETRY_HOLD_DURATION
    goal_occupancy_retry_window: float = GOAL_OCCUPANCY_RETRY_WINDOW
    goal_occupancy_max_retries: int = GOAL_OCCUPANCY_MAX_RETRIES
    global_max_vel_y: float = 0.10
    global_min_vel_y: float = -0.10
    global_vy_samples: int = 3
    global_acc_lim_y: float = 0.15
    global_decel_lim_y: float = -0.15
    config_path: str = ""


@dataclass(frozen=True)
class NavigationProfileLateralValues:
    max_vel_y: float
    min_vel_y: float
    vy_samples: int
    acc_lim_y: float
    decel_lim_y: float


@dataclass(frozen=True)
class GoalOccupancyWaitState:
    active: bool = False
    started_at: float = 0.0
    clear_since: float = 0.0
    retry_count: int = 0
    retrying_until: float = 0.0
    last_retry_at: float = 0.0


def _config_bool(value, default):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _config_float(value, default):
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _config_ratio(value, default):
    return max(0.0, min(1.0, _config_float(value, default)))


def _config_int(value, default):
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _percentile(values, ratio):
    sorted_values = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not sorted_values:
        return float("inf")
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = max(0.0, min(1.0, float(ratio))) * (len(sorted_values) - 1)
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    fraction = rank - low
    return sorted_values[low] * (1.0 - fraction) + sorted_values[high] * fraction


def _section(data, name):
    value = data.get(name, {}) if isinstance(data, dict) else {}
    return value if isinstance(value, dict) else {}


def load_motion_policy_config(path=None):
    default = MotionPolicyConfig(config_path=str(path or config_file("motion_policy.yaml")))
    config_path = os.path.expanduser(path or os.environ.get("G1_MOTION_POLICY_FILE", default.config_path))
    if not os.path.exists(config_path):
        return MotionPolicyConfig(config_path=config_path)
    if yaml is None:
        return MotionPolicyConfig(config_path=config_path)

    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return MotionPolicyConfig(config_path=config_path)

    motion = _section(data, "motion")
    assist = _section(data, "lateral_assist")
    close_guard = _section(data, "close_obstacle_guard")
    goal_occupancy = _section(data, "goal_occupancy")
    global_lateral = _section(data, "global_lateral_motion")

    return MotionPolicyConfig(
        global_allow_lateral_motion=_config_bool(
            motion.get(
                "global_allow_lateral_motion",
                data.get("global_allow_lateral_motion"),
            ),
            default.global_allow_lateral_motion,
        ),
        lateral_assist_enabled=_config_bool(
            assist.get("enabled"), default.lateral_assist_enabled
        ),
        lateral_assist_scan_topic=str(
            assist.get("scan_topic", default.lateral_assist_scan_topic)
        ),
        lateral_assist_trigger_distance=_config_float(
            assist.get("trigger_distance"), default.lateral_assist_trigger_distance
        ),
        lateral_assist_clear_distance=_config_float(
            assist.get("clear_distance"), default.lateral_assist_clear_distance
        ),
        lateral_assist_front_half_width=_config_float(
            assist.get("front_half_width"), default.lateral_assist_front_half_width
        ),
        lateral_assist_side_lookahead=_config_float(
            assist.get("side_lookahead"), default.lateral_assist_side_lookahead
        ),
        lateral_assist_side_min_clearance=_config_float(
            assist.get("side_min_clearance"),
            default.lateral_assist_side_min_clearance,
        ),
        lateral_assist_side_band_min=_config_float(
            assist.get("side_band_min"), default.lateral_assist_side_band_min
        ),
        lateral_assist_side_band_max=_config_float(
            assist.get("side_band_max"), default.lateral_assist_side_band_max
        ),
        lateral_assist_vy=_config_float(assist.get("vy"), default.lateral_assist_vy),
        lateral_assist_max_forward_vx=_config_float(
            assist.get("max_forward_vx"), default.lateral_assist_max_forward_vx
        ),
        lateral_assist_scan_timeout=_config_float(
            assist.get("scan_timeout"), default.lateral_assist_scan_timeout
        ),
        close_obstacle_guard_enabled=_config_bool(
            close_guard.get("enabled"), default.close_obstacle_guard_enabled
        ),
        close_obstacle_scan_topic=str(
            close_guard.get(
                "scan_topic",
                assist.get("scan_topic", default.close_obstacle_scan_topic),
            )
        ),
        close_obstacle_min_range=_config_float(
            close_guard.get("min_range"), default.close_obstacle_min_range
        ),
        close_obstacle_trigger_distance=_config_float(
            close_guard.get("trigger_distance"),
            default.close_obstacle_trigger_distance,
        ),
        close_obstacle_release_distance=_config_float(
            close_guard.get("release_distance"),
            default.close_obstacle_release_distance,
        ),
        close_obstacle_front_half_width=_config_float(
            close_guard.get("front_half_width"),
            default.close_obstacle_front_half_width,
        ),
        close_obstacle_front_angle_min=_config_float(
            close_guard.get("front_angle_min"),
            default.close_obstacle_front_angle_min,
        ),
        close_obstacle_front_angle_max=_config_float(
            close_guard.get("front_angle_max"),
            default.close_obstacle_front_angle_max,
        ),
        close_obstacle_min_points=max(
            1,
            _config_int(
                close_guard.get("min_points"),
                default.close_obstacle_min_points,
            ),
        ),
        close_obstacle_trigger_percentile=_config_ratio(
            close_guard.get("trigger_percentile"),
            default.close_obstacle_trigger_percentile,
        ),
        close_obstacle_min_consecutive_frames=max(
            1,
            _config_int(
                close_guard.get("min_consecutive_frames"),
                default.close_obstacle_min_consecutive_frames,
            ),
        ),
        close_obstacle_clear_duration=_config_float(
            close_guard.get("clear_duration"), default.close_obstacle_clear_duration
        ),
        close_obstacle_replan_on_clear=_config_bool(
            close_guard.get("replan_on_clear"),
            default.close_obstacle_replan_on_clear,
        ),
        close_obstacle_replan_cooldown=max(
            0.0,
            _config_float(
                close_guard.get("replan_cooldown"),
                default.close_obstacle_replan_cooldown,
            ),
        ),
        close_obstacle_replan_goal_margin=max(
            0.0,
            _config_float(
                close_guard.get("replan_goal_margin"),
                default.close_obstacle_replan_goal_margin,
            ),
        ),
        goal_occupancy_enabled=_config_bool(
            goal_occupancy.get("enabled"), default.goal_occupancy_enabled
        ),
        goal_occupancy_start_distance=_config_float(
            goal_occupancy.get("start_distance"),
            default.goal_occupancy_start_distance,
        ),
        goal_occupancy_wait_timeout=_config_float(
            goal_occupancy.get("wait_timeout"),
            default.goal_occupancy_wait_timeout,
        ),
        goal_occupancy_clear_duration=_config_float(
            goal_occupancy.get("clear_duration"),
            default.goal_occupancy_clear_duration,
        ),
        goal_occupancy_speak_interval=_config_float(
            goal_occupancy.get("speak_interval"),
            default.goal_occupancy_speak_interval,
        ),
        goal_occupancy_costmap_topic=str(
            goal_occupancy.get(
                "costmap_topic",
                default.goal_occupancy_costmap_topic,
            )
        ),
        goal_occupancy_costmap_timeout=_config_float(
            goal_occupancy.get("costmap_timeout"),
            default.goal_occupancy_costmap_timeout,
        ),
        goal_occupancy_radius=max(
            0.0,
            _config_float(
                goal_occupancy.get("radius"),
                default.goal_occupancy_radius,
            ),
        ),
        goal_occupancy_cost_threshold=max(
            0,
            _config_int(
                goal_occupancy.get("cost_threshold"),
                default.goal_occupancy_cost_threshold,
            ),
        ),
        goal_occupancy_min_occupied_cells=max(
            1,
            _config_int(
                goal_occupancy.get("min_occupied_cells"),
                default.goal_occupancy_min_occupied_cells,
            ),
        ),
        goal_occupancy_retry_hold_duration=max(
            0.0,
            _config_float(
                goal_occupancy.get("retry_hold_duration"),
                default.goal_occupancy_retry_hold_duration,
            ),
        ),
        goal_occupancy_retry_window=max(
            0.0,
            _config_float(
                goal_occupancy.get("retry_window"),
                default.goal_occupancy_retry_window,
            ),
        ),
        goal_occupancy_max_retries=max(
            0,
            _config_int(
                goal_occupancy.get("max_retries"),
                default.goal_occupancy_max_retries,
            ),
        ),
        global_max_vel_y=abs(
            _config_float(global_lateral.get("max_vel_y"), default.global_max_vel_y)
        ),
        global_min_vel_y=-abs(
            _config_float(global_lateral.get("min_vel_y"), default.global_min_vel_y)
        ),
        global_vy_samples=max(
            1, _config_int(global_lateral.get("vy_samples"), default.global_vy_samples)
        ),
        global_acc_lim_y=abs(
            _config_float(global_lateral.get("acc_lim_y"), default.global_acc_lim_y)
        ),
        global_decel_lim_y=-abs(
            _config_float(
                global_lateral.get("decel_lim_y"), default.global_decel_lim_y
            )
        ),
        config_path=config_path,
    )


def _env_flag(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _parameter_service_name(remote_node_name, service_name):
    node_name = remote_node_name.rstrip("/")
    if not node_name.startswith("/"):
        node_name = f"/{node_name}"
    return f"{node_name}/{service_name}"


class HumbleAsyncParameterClientCompat:
    """Small AsyncParameterClient subset for ROS 2 Humble."""

    def __init__(self, node, remote_node_name):
        self._set_parameters_client = node.create_client(
            SetParameters,
            _parameter_service_name(remote_node_name, "set_parameters"),
        )

    def wait_for_services(self, timeout_sec=None):
        return self._set_parameters_client.wait_for_service(timeout_sec=timeout_sec)

    def set_parameters(self, parameters, callback=None):
        request = SetParameters.Request()
        request.parameters = [
            parameter.to_parameter_msg()
            if isinstance(parameter, Parameter)
            else parameter
            for parameter in parameters
        ]
        future = self._set_parameters_client.call_async(request)
        if callback is not None:
            future.add_done_callback(callback)
        return future


def lerp(start, end, blend):
    return start + (end - start) * blend


def build_rotation_command(
    yaw_diff, max_abs=1.2, min_abs=0.2, gain=2.5, stop_threshold=0.05
):
    if abs(yaw_diff) < stop_threshold:
        return 0.0
    cmd_wz = clamp(yaw_diff * gain, -max_abs, max_abs)
    if 0 < abs(cmd_wz) < min_abs:
        return min_abs if cmd_wz > 0 else -min_abs
    return cmd_wz


def profile_blend_for_distance(distance_to_goal):
    if distance_to_goal >= SLOWDOWN_START_DISTANCE:
        return 0.0
    if distance_to_goal <= SLOWDOWN_FULL_PROFILE_DISTANCE:
        return 1.0
    span = SLOWDOWN_START_DISTANCE - SLOWDOWN_FULL_PROFILE_DISTANCE
    return (SLOWDOWN_START_DISTANCE - distance_to_goal) / span


def quantize_profile_blend(blend):
    blend = clamp(blend, 0.0, 1.0)
    stepped = round(blend / SLOWDOWN_PROFILE_STEP) * SLOWDOWN_PROFILE_STEP
    return round(clamp(stepped, 0.0, 1.0), 2)


def build_navigation_profile(blend, walking_mode=None):
    blend = clamp(blend, 0.0, 1.0)
    wm = walking_mode
    fast_vx = wm.fast_max_vel_x if wm else FAST_MAX_VEL_X
    slow_vx = wm.slow_max_vel_x if wm else SLOW_MAX_VEL_X
    fast_vt = wm.fast_max_vel_theta if wm else FAST_MAX_VEL_THETA
    slow_vt = wm.slow_max_vel_theta if wm else SLOW_MAX_VEL_THETA
    fast_ax = wm.fast_acc_lim_x if wm else FAST_ACC_LIM_X
    slow_ax = wm.slow_acc_lim_x if wm else SLOW_ACC_LIM_X
    fast_at = wm.fast_acc_lim_theta if wm else FAST_ACC_LIM_THETA
    slow_at = wm.slow_acc_lim_theta if wm else SLOW_ACC_LIM_THETA
    return {
        "blend": blend,
        "max_vel_x": lerp(fast_vx, slow_vx, blend),
        "max_vel_theta": lerp(fast_vt, slow_vt, blend),
        "acc_lim_x": lerp(fast_ax, slow_ax, blend),
        "acc_lim_theta": lerp(fast_at, slow_at, blend),
        "path_distance_bias": lerp(FAST_PATH_DISTANCE_BIAS, SLOW_PATH_DISTANCE_BIAS, blend),
        "goal_distance_bias": lerp(FAST_GOAL_DISTANCE_BIAS, SLOW_GOAL_DISTANCE_BIAS, blend),
        "occdist_scale": lerp(FAST_OCCDIST_SCALE, SLOW_OCCDIST_SCALE, blend),
        "footprint_scale": lerp(FAST_FOOTPRINT_SCALE, SLOW_FOOTPRINT_SCALE, blend),
    }


def _double_parameter(name, value):
    return Parameter(name, Parameter.Type.DOUBLE, float(value))


def _integer_parameter(name, value):
    return Parameter(name, Parameter.Type.INTEGER, int(value))


def _double_array_parameter(name, values):
    return Parameter(name, Parameter.Type.DOUBLE_ARRAY, [float(value) for value in values])


class MotionController:
    def __init__(self, node, loco_client):
        self.node = node
        self.loco = loco_client
        self.lock = threading.RLock()
        self.log_times = {}
        self.motion_policy = getattr(node, "motion_policy", None) or load_motion_policy_config()
        self.walking_mode = getattr(node, "walking_mode", None) or load_walking_mode_config()

        self.planner_cmd = (0.0, 0.0, 0.0)
        self.planner_stamp = 0.0
        self.manual_active = False
        self.manual_cmd = (0.0, 0.0, 0.0)
        self.manual_deadline = 0.0
        self.manual_source = "manual"
        self.last_source = None
        self.last_sent_cmd = (0.0, 0.0, 0.0)
        self.last_sent_source = "idle"
        self.stop_latched = False
        self.stop_latch_reason = ""
        self.navigation_failure_reason = ""
        self.navigation_goal_active = False
        self.reverse_request_streak = 0
        self.rotate_recovery_active = False
        self.rotate_recovery_target_yaw = 0.0
        self.rotate_recovery_target_source = ""
        self.rotate_recovery_deadline = 0.0
        self.rotate_recovery_retry_count = 0
        self.rotate_recovery_cooldown = 0.0
        self.rotate_recovery_heading_provider = None
        self.yield_hold_active = False
        self.yield_hold_deadline = 0.0
        self.yield_hold_event_count = 0
        self.yield_hold_consumed_count = 0
        self.close_obstacle_clear_event_count = 0
        self.close_obstacle_clear_consumed_count = 0
        self.reverse_replan_event_count = 0
        self.reverse_replan_consumed_count = 0
        self.reverse_replan_cooldown = 0.0
        self.lateral_assist_active = False
        self.lateral_assist_side = 1
        self.lateral_assist_stamp = 0.0
        self.lateral_assist_front_min = float("inf")
        self.lateral_assist_left_clearance = float("inf")
        self.lateral_assist_right_clearance = float("inf")
        self.close_obstacle_active = False
        self.close_obstacle_front_min = float("inf")
        self.close_obstacle_hit_count = 0
        self.close_obstacle_clear_since = 0.0

        self.executed_pub = node.create_publisher(Twist, "/cmd_vel_executed", 10)
        self.source_pub = node.create_publisher(String, "/motion_source", 10)
        self.close_obstacle_event_pub = node.create_publisher(
            String, CLOSE_OBSTACLE_EVENT_TOPIC, 10
        )
        self.subscription = node.create_subscription(
            Twist, PLANNER_CMD_TOPIC, self._planner_cb, 10
        )
        self.scan_subscription = None
        if (
            self.motion_policy.lateral_assist_enabled
            or self.motion_policy.close_obstacle_guard_enabled
        ):
            scan_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.scan_subscription = node.create_subscription(
                LaserScan,
                self._scan_topic(),
                self._scan_cb,
                scan_qos,
            )
        self.timer = node.create_timer(1.0 / CONTROL_LOOP_HZ, self._loop)
        self._loop_paused = False
        self._pause_lock = threading.Lock()
        self._pause_depth = 0
        self._set_speed_mode()
        self.node.get_logger().info(
            "[motion] 已接管 /cmd_vel，并直接下发宇树 SDK；"
            f"policy={self.motion_policy.config_path}, "
            f"global_allow_lateral={self.motion_policy.global_allow_lateral_motion}, "
            f"lateral_assist={self.motion_policy.lateral_assist_enabled}, "
            f"close_obstacle_guard={self.motion_policy.close_obstacle_guard_enabled}, "
            f"walking_mode={self.walking_mode.mode_name}"
        )

    def _scan_topic(self):
        policy = self.motion_policy
        if policy.close_obstacle_guard_enabled:
            return policy.close_obstacle_scan_topic
        return policy.lateral_assist_scan_topic

    def _reset_loco_health_snapshot(self):
        reset_health = getattr(self.loco, "ResetHealthSnapshot", None)
        if reset_health is None:
            return
        try:
            reset_health()
        except Exception as exc:
            self._log_throttle("warning", 2.0, f"[motion] SDK 健康统计重置失败: {exc}")

    def pause_loop(self, source="paused"):
        t_enter = time.monotonic()
        with self._pause_lock:
            self._pause_depth += 1
            if self._pause_depth > 1:
                _nav_logger.info(
                    "[motion] pause_loop reentrant depth=%d", self._pause_depth
                )
                return
            self._loop_paused = True
        t_flagged = time.monotonic()
        try:
            self.loco.Move(0.0, 0.0, 0.0)
        except Exception as exc:
            _nav_logger.warning("[motion] pause 兜底 Move(0) 失败: %s", exc)
        t_moved = time.monotonic()
        with self.lock:
            self.last_sent_cmd = (0.0, 0.0, 0.0)
            self.last_sent_source = source
        _nav_logger.info(
            "[motion] 控制循环暂停 (%s)  flag=%.1fms  watchdog_move=%.1fms",
            source,
            (t_flagged - t_enter) * 1000,
            (t_moved - t_flagged) * 1000,
        )

    def resume_loop(self):
        with self._pause_lock:
            if self._pause_depth == 0:
                return
            self._pause_depth -= 1
            if self._pause_depth > 0:
                return
            self._loop_paused = False
        self.node.get_logger().info("[motion] 控制循环恢复")

    def _log_throttle(self, level, period, message):
        now = time.time()
        last = self.log_times.get(message, 0.0)
        if now - last < period:
            return
        self.log_times[message] = now
        getattr(self.node.get_logger(), level)(message)

    def _set_speed_mode(self):
        wm = self.walking_mode
        ret = self.loco.SetSpeedMode(wm.speed_mode)
        speed_table = {0: "1.0", 1: "2.0", 2: "2.7", 3: "3.0"}
        max_speed = speed_table.get(wm.speed_mode, "未知")
        if ret == 0:
            self.node.get_logger().info(
                f"[motion] 速度模式已设置: mode={wm.speed_mode}, "
                f"最高 {max_speed} m/s (walking_mode={wm.mode_name})"
            )
        else:
            self.node.get_logger().warning(f"[motion] 设置速度模式失败, 返回码: {ret}")

    def _planner_cb(self, msg):
        reverse_replan_requested = False
        with self.lock:
            if self.stop_latched:
                return
            self.planner_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.planner_stamp = time.time()
            if msg.linear.x < 0.0:
                self.reverse_request_streak += 1
                if (
                    self.navigation_goal_active
                    and self.reverse_request_streak == ROTATE_RECOVERY_TRIGGER_REQUESTS
                    and self.planner_stamp >= self.reverse_replan_cooldown
                ):
                    self.reverse_replan_event_count += 1
                    self.reverse_replan_cooldown = self.planner_stamp + REVERSE_REPLAN_COOLDOWN
                    reverse_replan_requested = True
            else:
                self.reverse_request_streak = 0

        if reverse_replan_requested:
            self._log_throttle(
                "warning",
                1.0,
                "[motion] 检测到连续后退请求，准备改为前向重规划",
            )

    def _scan_cb(self, msg):
        policy = self.motion_policy
        if not (
            policy.lateral_assist_enabled
            or policy.close_obstacle_guard_enabled
        ):
            return

        now = time.time()
        angle_increment = float(msg.angle_increment)
        if abs(angle_increment) < 1e-9:
            return

        with self.lock:
            was_active = self.lateral_assist_active
            previous_side = self.lateral_assist_side

        front_limit = (
            policy.lateral_assist_clear_distance
            if was_active
            else policy.lateral_assist_trigger_distance
        )
        lateral_range_min = max(float(msg.range_min), 0.05)
        close_range_min = max(0.0, policy.close_obstacle_min_range)
        range_max = float(msg.range_max)
        front_min = float("inf")
        left_clearance = float("inf")
        right_clearance = float("inf")
        close_front_values = []
        close_front_limit = max(
            policy.close_obstacle_trigger_distance,
            policy.close_obstacle_release_distance,
        )
        close_angle_min = min(
            policy.close_obstacle_front_angle_min,
            policy.close_obstacle_front_angle_max,
        )
        close_angle_max = max(
            policy.close_obstacle_front_angle_min,
            policy.close_obstacle_front_angle_max,
        )

        angle = float(msg.angle_min)
        for raw_range in msg.ranges:
            scan_range = float(raw_range)
            if not math.isfinite(scan_range) or scan_range > range_max:
                angle += angle_increment
                continue

            x = scan_range * math.cos(angle)
            y = scan_range * math.sin(angle)

            if scan_range >= close_range_min:
                if (
                    0.0 < x <= close_front_limit
                    and abs(y) <= policy.close_obstacle_front_half_width
                    and close_angle_min <= angle <= close_angle_max
                ):
                    close_front_values.append(x)

            if scan_range >= lateral_range_min:
                if (
                    0.0 < x <= front_limit
                    and abs(y) <= policy.lateral_assist_front_half_width
                ):
                    front_min = min(front_min, x)

                if (
                    0.0 < x <= policy.lateral_assist_side_lookahead
                    and policy.lateral_assist_side_band_min
                    <= abs(y)
                    <= policy.lateral_assist_side_band_max
                ):
                    if y > 0.0:
                        left_clearance = min(left_clearance, scan_range)
                    else:
                        right_clearance = min(right_clearance, scan_range)

            angle += angle_increment

        if policy.close_obstacle_guard_enabled:
            close_front_min = (
                min(close_front_values) if close_front_values else float("inf")
            )
            trigger_blocked = self._close_obstacle_cluster_blocked(
                close_front_values,
                policy.close_obstacle_trigger_distance,
            )
            release_blocked = self._close_obstacle_cluster_blocked(
                close_front_values,
                max(
                    policy.close_obstacle_release_distance,
                    policy.close_obstacle_trigger_distance,
                ),
            )
            self._update_close_obstacle_guard(
                now,
                close_front_min,
                trigger_blocked=trigger_blocked,
                release_blocked=release_blocked,
            )

        if not policy.lateral_assist_enabled:
            return

        blocked = front_min <= front_limit
        side = previous_side if previous_side in (-1, 1) else 1
        if blocked:
            if left_clearance > right_clearance:
                side = 1
            elif right_clearance > left_clearance:
                side = -1
            selected_clearance = left_clearance if side > 0 else right_clearance
            if selected_clearance < policy.lateral_assist_side_min_clearance:
                blocked = False

        with self.lock:
            previous_active = self.lateral_assist_active
            previous_side = self.lateral_assist_side
            self.lateral_assist_active = blocked
            self.lateral_assist_side = side
            self.lateral_assist_stamp = now
            self.lateral_assist_front_min = front_min
            self.lateral_assist_left_clearance = left_clearance
            self.lateral_assist_right_clearance = right_clearance

        if blocked and (not previous_active or previous_side != side):
            side_label = "left" if side > 0 else "right"
            self.node.get_logger().info(
                "[motion] 前方近障碍触发横移辅助 "
                f"(front_x={front_min:.2f}m, side={side_label}, "
                f"left_clear={left_clearance:.2f}, right_clear={right_clearance:.2f})"
            )
        elif previous_active and not blocked:
            self.node.get_logger().info(
                f"[motion] 横移辅助关闭 (front_x={front_min:.2f}m)"
            )

    def _reset_close_obstacle_locked(self):
        self.close_obstacle_active = False
        self.close_obstacle_front_min = float("inf")
        self.close_obstacle_hit_count = 0
        self.close_obstacle_clear_since = 0.0

    def _reset_close_obstacle_clear_event_locked(self):
        self.close_obstacle_clear_event_count = 0
        self.close_obstacle_clear_consumed_count = 0

    def _publish_close_obstacle_event(self, state, front_min):
        try:
            payload = {
                "state": state,
                "front_x": None if not math.isfinite(front_min) else round(front_min, 3),
                "timestamp": time.time(),
            }
            self.close_obstacle_event_pub.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:
            self._log_throttle(
                "warning", 2.0, f"[motion] 近障碍事件发布失败: {exc}"
            )

    def _urgent_stop_for_close_obstacle(self):
        try:
            emergency_stop = getattr(self.loco, "EmergencyStop", None)
            if emergency_stop is not None:
                emergency_stop("close_obstacle")
                return
            self.loco.Move(0.0, 0.0, 0.0, source="close_obstacle_hold")
            stop_move = getattr(self.loco, "StopMove", None)
            if stop_move is not None:
                stop_move()
        except Exception as exc:
            self._log_throttle("error", 1.0, f"[motion] 近障碍急停下发失败: {exc}")

    def _close_obstacle_cluster_blocked(self, front_values, distance_limit):
        policy = self.motion_policy
        finite_values = [
            float(value)
            for value in front_values
            if math.isfinite(float(value))
        ]
        close_count = sum(1 for value in finite_values if value <= distance_limit)
        if close_count < policy.close_obstacle_min_points:
            return False
        return (
            _percentile(finite_values, policy.close_obstacle_trigger_percentile)
            <= distance_limit
        )

    def _update_close_obstacle_guard(
        self,
        now,
        front_min,
        *,
        trigger_blocked,
        release_blocked,
    ):
        policy = self.motion_policy
        enter_guard = False
        release_guard = False
        with self.lock:
            if not self.navigation_goal_active:
                self._reset_close_obstacle_locked()
                return

            self.close_obstacle_front_min = front_min
            if self.close_obstacle_active:
                self.close_obstacle_hit_count = 0
                if release_blocked:
                    self.close_obstacle_clear_since = 0.0
                elif self.close_obstacle_clear_since <= 0.0:
                    self.close_obstacle_clear_since = now
                elif now - self.close_obstacle_clear_since >= policy.close_obstacle_clear_duration:
                    self._reset_close_obstacle_locked()
                    if policy.close_obstacle_replan_on_clear:
                        self.close_obstacle_clear_event_count += 1
                    release_guard = True
            elif trigger_blocked:
                self.close_obstacle_hit_count += 1
                if (
                    self.close_obstacle_hit_count
                    >= policy.close_obstacle_min_consecutive_frames
                ):
                    self.close_obstacle_active = True
                    self.close_obstacle_clear_since = 0.0
                    self.close_obstacle_hit_count = 0
                    self._reset_rotate_recovery_locked(
                        reset_retry_count=True, reset_cooldown=True
                    )
                    self._reset_yield_hold_locked()
                    enter_guard = True
            else:
                self.close_obstacle_hit_count = 0
                self.close_obstacle_clear_since = 0.0

        if enter_guard:
            self.node.get_logger().warning(
                "[motion] 近距离障碍触发安全停 "
                f"(front_x={front_min:.2f}m, "
                f"threshold={policy.close_obstacle_trigger_distance:.2f}m)"
            )
            self._publish_close_obstacle_event("blocked", front_min)
            self._urgent_stop_for_close_obstacle()
        elif release_guard:
            self.node.get_logger().info("[motion] 近距离障碍已清除，恢复导航控制")
            self._publish_close_obstacle_event("cleared", front_min)

    def set_rotate_recovery_heading_provider(self, provider):
        with self.lock:
            self.rotate_recovery_heading_provider = provider

    def _reset_rotate_recovery_locked(
        self, reset_retry_count=False, reset_cooldown=False
    ):
        self.reverse_request_streak = 0
        self.rotate_recovery_active = False
        self.rotate_recovery_target_yaw = 0.0
        self.rotate_recovery_target_source = ""
        self.rotate_recovery_deadline = 0.0
        if reset_retry_count:
            self.rotate_recovery_retry_count = 0
        if reset_cooldown:
            self.rotate_recovery_cooldown = 0.0

    def _reset_yield_hold_locked(self):
        self.yield_hold_active = False
        self.yield_hold_deadline = 0.0

    def begin_navigation_goal(self, release_stop_latch=True):
        with self.lock:
            if release_stop_latch:
                self.stop_latched = False
                self.stop_latch_reason = ""
                self.navigation_failure_reason = ""
            self.navigation_goal_active = True
            self.planner_cmd = (0.0, 0.0, 0.0)
            self.planner_stamp = 0.0
            self.reverse_request_streak = 0
            self._reset_rotate_recovery_locked(
                reset_retry_count=True, reset_cooldown=True
            )
            self._reset_yield_hold_locked()
            self._reset_close_obstacle_locked()
            self._reset_close_obstacle_clear_event_locked()
            self.reverse_replan_cooldown = 0.0
        self._reset_loco_health_snapshot()

    def end_navigation_goal(self):
        with self.lock:
            self.navigation_goal_active = False
            self._reset_rotate_recovery_locked(
                reset_retry_count=True, reset_cooldown=True
            )
            self._reset_yield_hold_locked()
            self._reset_close_obstacle_locked()
            self._reset_close_obstacle_clear_event_locked()
            self.reverse_replan_cooldown = 0.0

    def is_stop_latched(self):
        with self.lock:
            return bool(self.stop_latched)

    def release_stop_latch(self, reason=""):
        with self.lock:
            was_latched = self.stop_latched
            self.stop_latched = False
            self.stop_latch_reason = ""
            if was_latched:
                self.last_sent_source = "idle"
        if was_latched:
            self.node.get_logger().info(
                f"[motion] 停止闩锁已释放 ({reason or 'resume'})"
            )

    def emergency_stop_latched(self, reason="stop"):
        source = "stop_latched"
        with self.lock:
            self.stop_latched = True
            self.stop_latch_reason = str(reason or "stop")
            self.navigation_goal_active = False
            self.planner_cmd = (0.0, 0.0, 0.0)
            self.planner_stamp = 0.0
            self.manual_active = False
            self.manual_cmd = (0.0, 0.0, 0.0)
            self.manual_deadline = 0.0
            self.last_sent_cmd = (0.0, 0.0, 0.0)
            self.last_sent_source = source
            self.reverse_replan_cooldown = 0.0
            self.reverse_request_streak = 0
            self._reset_rotate_recovery_locked(reset_retry_count=True, reset_cooldown=True)
            self._reset_yield_hold_locked()
            self._reset_close_obstacle_locked()
            self._reset_close_obstacle_clear_event_locked()

        self.node.get_logger().warning(
            f"[motion] 硬停闩锁触发 ({self.stop_latch_reason})"
        )
        self._publish_command_state(0.0, 0.0, 0.0, source)
        try:
            emergency_stop = getattr(self.loco, "EmergencyStop", None)
            if emergency_stop is not None:
                emergency_stop(self.stop_latch_reason)
            else:
                self.loco.Move(0.0, 0.0, 0.0, source=source)
                stop_move = getattr(self.loco, "StopMove", None)
                if stop_move is not None:
                    stop_move()
        except Exception as exc:
            self._log_throttle("error", 1.0, f"[motion] 硬停 SDK 下发失败: {exc}")

    def set_navigation_failure_reason(self, reason):
        with self.lock:
            self.navigation_failure_reason = str(reason or "")

    def time_since_planner_cmd(self):
        with self.lock:
            if self.planner_stamp <= 0.0:
                return float("inf")
            return time.time() - self.planner_stamp

    def _lateral_assist_active_locked(self, now):
        return bool(
            self.navigation_goal_active
            and self.motion_policy.lateral_assist_enabled
            and self.lateral_assist_active
            and self.lateral_assist_side in (-1, 1)
            and now - self.lateral_assist_stamp
            <= self.motion_policy.lateral_assist_scan_timeout
        )

    def _lateral_assist_snapshot(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            active = self._lateral_assist_active_locked(now)
            return {
                "active": active,
                "side": self.lateral_assist_side,
                "front_min": self.lateral_assist_front_min,
                "left_clearance": self.lateral_assist_left_clearance,
                "right_clearance": self.lateral_assist_right_clearance,
                "scan_age": now - self.lateral_assist_stamp
                if self.lateral_assist_stamp > 0.0
                else float("inf"),
            }

    def get_command_snapshot(self):
        with self.lock:
            now = time.time()
            planner_stamp = self.planner_stamp
            planner_cmd = self.planner_cmd
            executed_cmd = self.last_sent_cmd
            output_source = self.last_sent_source
            manual_active = self.manual_active
            rotate_recovery_active = self.rotate_recovery_active
            yield_hold_active = self.yield_hold_active
            close_obstacle_active = self.close_obstacle_active
            close_obstacle_front_min = self.close_obstacle_front_min
            lateral_assist_active = self._lateral_assist_active_locked(now)
            lateral_assist_side = self.lateral_assist_side
            lateral_assist_front_min = self.lateral_assist_front_min
            stop_latched = self.stop_latched
            navigation_failure_reason = self.navigation_failure_reason

        planner_age = float("inf")
        if planner_stamp > 0.0:
            planner_age = max(0.0, time.time() - planner_stamp)

        return {
            "planner_cmd": planner_cmd,
            "planner_age": planner_age,
            "executed_cmd": executed_cmd,
            "output_source": output_source,
            "manual_active": manual_active,
            "rotate_recovery_active": rotate_recovery_active,
            "yield_hold_active": yield_hold_active,
            "close_obstacle_active": close_obstacle_active,
            "close_obstacle_front_min": close_obstacle_front_min,
            "lateral_assist_active": lateral_assist_active,
            "lateral_assist_side": lateral_assist_side,
            "lateral_assist_front_min": lateral_assist_front_min,
            "stop_latched": stop_latched,
            "navigation_failure_reason": navigation_failure_reason,
        }

    def _enter_yield_hold(self, now, reason):
        with self.lock:
            self.yield_hold_active = True
            self.yield_hold_deadline = now + YIELD_HOLD_TIMEOUT
            self.rotate_recovery_active = False
            self.rotate_recovery_target_yaw = 0.0
            self.rotate_recovery_target_source = ""
            self.rotate_recovery_deadline = 0.0
            self.reverse_request_streak = 0
            self.yield_hold_event_count += 1

        self.node.get_logger().warning(
            f"[motion] 进入让行静止 ({reason})，暂停 {YIELD_HOLD_TIMEOUT:.1f}s 等待障碍离开"
        )

    def consume_yield_hold_event(self):
        with self.lock:
            if self.yield_hold_consumed_count == self.yield_hold_event_count:
                return False
            self.yield_hold_consumed_count = self.yield_hold_event_count
            return True

    def consume_close_obstacle_clear_event(self):
        with self.lock:
            if (
                self.close_obstacle_clear_consumed_count
                == self.close_obstacle_clear_event_count
            ):
                return False
            self.close_obstacle_clear_consumed_count = (
                self.close_obstacle_clear_event_count
            )
            return True

    def consume_reverse_replan_event(self):
        with self.lock:
            if self.reverse_replan_consumed_count == self.reverse_replan_event_count:
                return False
            self.reverse_replan_consumed_count = self.reverse_replan_event_count
            return True

    def _finish_rotate_recovery(self, now, reason, warn=False):
        with self.lock:
            if not self.rotate_recovery_active:
                return
            retry_count = self.rotate_recovery_retry_count
            target_source = self.rotate_recovery_target_source
            self.rotate_recovery_active = False
            self.rotate_recovery_target_yaw = 0.0
            self.rotate_recovery_target_source = ""
            self.rotate_recovery_deadline = 0.0
            self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
            self.reverse_request_streak = 0

        log = self.node.get_logger().warning if warn else self.node.get_logger().info
        log(
            f"[motion] 旋转恢复结束 ({reason}, attempt={retry_count}, target={target_source})"
        )

    def _compute_rotate_recovery_command(self, now, get_pose_snapshot):
        with self.lock:
            if not self.rotate_recovery_active:
                return None
            target_yaw = self.rotate_recovery_target_yaw
            deadline = self.rotate_recovery_deadline

        _, _, current_yaw = get_pose_snapshot()
        yaw_diff = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_diff) <= ROTATE_RECOVERY_YAW_THRESHOLD:
            self._finish_rotate_recovery(now, "yaw_aligned")
            return None

        if now >= deadline:
            self._finish_rotate_recovery(now, "timeout", warn=True)
            return None

        wm = self.walking_mode
        cmd_wz = build_rotation_command(
            yaw_diff,
            max_abs=wm.rotate_recovery_max_wz,
            min_abs=wm.rotate_recovery_min_wz,
            gain=wm.rotate_recovery_gain,
        )
        return 0.0, 0.0, cmd_wz, "rotate_recovery"

    def _maybe_start_rotate_recovery(self, now, get_pose_snapshot):
        enter_yield_hold = False
        with self.lock:
            if self.rotate_recovery_active or not self.navigation_goal_active:
                return False
            if self.reverse_request_streak < ROTATE_RECOVERY_TRIGGER_REQUESTS:
                return False
            if self.yield_hold_active or now < self.rotate_recovery_cooldown:
                return False
            if self.rotate_recovery_retry_count >= ROTATE_RECOVERY_MAX_RETRIES:
                enter_yield_hold = True
            else:
                heading_provider = self.rotate_recovery_heading_provider

        if enter_yield_hold:
            self._enter_yield_hold(now, "rotate_recovery_exhausted")
            return False

        if heading_provider is None:
            self._log_throttle(
                "warning", 2.0, "[motion] 未配置旋转恢复朝向提供器，无法接管后退恢复"
            )
            with self.lock:
                self.reverse_request_streak = 0
                self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
            return False

        try:
            heading_result = heading_provider()
        except Exception as exc:
            self._log_throttle("warning", 2.0, f"[motion] 获取旋转恢复朝向失败: {exc}")
            with self.lock:
                self.reverse_request_streak = 0
                self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
            return False

        if heading_result is None:
            self._log_throttle("warning", 2.0, "[motion] 当前无法确定旋转恢复朝向，先跳过本次接管")
            with self.lock:
                self.reverse_request_streak = 0
                self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
            return False

        target_yaw, target_source = heading_result
        _, _, current_yaw = get_pose_snapshot()
        yaw_diff = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_diff) <= ROTATE_RECOVERY_YAW_THRESHOLD:
            with self.lock:
                planner_wz = self.planner_cmd[2]

            if abs(planner_wz) >= ROTATE_RECOVERY_MIN_PLANNER_WZ:
                target_yaw = normalize_angle(
                    current_yaw
                    + math.copysign(ROTATE_RECOVERY_FALLBACK_YAW, planner_wz)
                )
                target_source = "planner_turn_fallback"
                yaw_diff = normalize_angle(target_yaw - current_yaw)
            else:
                with self.lock:
                    self.reverse_request_streak = 0
                    self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
                self._log_throttle(
                    "info",
                    1.0,
                    "[motion] planner 请求后退，但当前朝向已接近路径方向，跳过旋转恢复",
                )
                return False

        with self.lock:
            self.rotate_recovery_active = True
            self.rotate_recovery_target_yaw = target_yaw
            self.rotate_recovery_target_source = target_source
            self.rotate_recovery_deadline = now + ROTATE_RECOVERY_TIMEOUT
            self.rotate_recovery_retry_count += 1
            self.reverse_request_streak = 0
            retry_count = self.rotate_recovery_retry_count

        self.node.get_logger().info(
            "[motion] planner 连续请求后退，切换为旋转恢复 "
            f"(attempt={retry_count}/{ROTATE_RECOVERY_MAX_RETRIES}, "
            f"target={target_source}, yaw_error={math.degrees(yaw_diff):.1f}°)"
        )
        return True

    def set_manual_velocity(self, vx, vy, wz, timeout=0.3, source="manual"):
        deadline = float("inf") if timeout is None else time.time() + max(0.0, timeout)
        with self.lock:
            self.manual_active = True
            self.manual_cmd = (vx, vy, wz)
            self.manual_deadline = deadline
            self.manual_source = source

    def hold_position(self, source="hold"):
        self.set_manual_velocity(0.0, 0.0, 0.0, timeout=None, source=source)

    def clear_manual_override(self):
        with self.lock:
            self.manual_active = False
            self.manual_cmd = (0.0, 0.0, 0.0)
            self.manual_deadline = 0.0
            self.manual_source = "manual"
            self._reset_rotate_recovery_locked()
            self._reset_yield_hold_locked()

    def _sanitize_command(self, vx, vy, wz, source):
        if source == "planner":
            policy = self.motion_policy
            assist = self._lateral_assist_snapshot()
            if vx < 0.0:
                self._log_throttle(
                    "info", 1.0, "[motion] 自动导航禁止倒车：负向 planner 指令已改为停止/转向恢复"
                )
                vx = 0.0
                vy = 0.0
            elif assist["active"] and (
                not policy.global_allow_lateral_motion or abs(vy) <= 1e-3
            ):
                side = assist["side"]
                side_label = "left" if side > 0 else "right"
                vx = clamp(
                    max(vx, 0.0), 0.0, policy.lateral_assist_max_forward_vx
                )
                vy = side * policy.lateral_assist_vy
                self._log_throttle(
                    "info",
                    1.0,
                    "[motion] 前方障碍横移辅助 "
                    f"side={side_label}, front_x={assist['front_min']:.2f}m, "
                    f"cmd=({vx:.3f}, {vy:.3f}, {wz:.3f})",
                )
                return vx, vy, wz, "lateral_assist"
            elif policy.global_allow_lateral_motion:
                original_vy = vy
                vy = clamp(vy, policy.global_min_vel_y, policy.global_max_vel_y)
                if abs(original_vy - vy) > 1e-6:
                    self._log_throttle(
                        "info",
                        2.0,
                        f"[motion] 全局横移限速: {original_vy:.3f} -> {vy:.3f}",
                    )
            elif abs(vy) > 1e-3:
                self._log_throttle(
                    "info",
                    2.0,
                    f"[motion] 横移辅助未触发，自动导航 vy 清零: {vy:.3f} -> 0.000",
                )
                vy = 0.0

        wm = self.walking_mode
        if 0.0 < vx < wm.min_vx_threshold and abs(wz) < wm.min_vx_bypass_wz:
            original_vx = vx
            vx = wm.min_vx_threshold
            self._log_throttle("info", 2.0, f"[motion] vx 钳制: {original_vx:.3f} -> {vx:.3f}")
        return vx, vy, wz, source

    @staticmethod
    def _limit_axis_step(target, previous, max_step):
        delta = target - previous
        if delta > max_step:
            return previous + max_step
        if delta < -max_step:
            return previous - max_step
        return target

    def _smooth_command(self, vx, vy, wz):
        wm = self.walking_mode
        prev_vx, prev_vy, prev_wz = self.last_sent_cmd
        limited_vx = self._limit_axis_step(vx, prev_vx, wm.max_cmd_step_vx)
        limited_vy = self._limit_axis_step(vy, prev_vy, wm.max_cmd_step_vy)
        limited_wz = self._limit_axis_step(wz, prev_wz, wm.max_cmd_step_wz)

        if (
            abs(limited_vx - vx) > 1e-6
            or abs(limited_vy - vy) > 1e-6
            or abs(limited_wz - wz) > 1e-6
        ):
            self._log_throttle(
                "info",
                1.0,
                "[motion] 斜率限制 "
                f"target=({vx:.3f}, {vy:.3f}, {wz:.3f}) "
                f"send=({limited_vx:.3f}, {limited_vy:.3f}, {limited_wz:.3f})",
            )

        return limited_vx, limited_vy, limited_wz

    def _select_command(self, get_pose_snapshot):
        now = time.time()
        with self.lock:
            if self.stop_latched:
                self.manual_active = False
                self.manual_cmd = (0.0, 0.0, 0.0)
                self.manual_deadline = 0.0
                self._reset_rotate_recovery_locked(reset_retry_count=True, reset_cooldown=True)
                self._reset_yield_hold_locked()
                return 0.0, 0.0, 0.0, "stop_latched"

            if self.close_obstacle_active:
                self._reset_rotate_recovery_locked(
                    reset_retry_count=True, reset_cooldown=True
                )
                self._reset_yield_hold_locked()
                self.reverse_request_streak = 0
                return 0.0, 0.0, 0.0, "close_obstacle_hold"

            if self.manual_active and now >= self.manual_deadline:
                self.manual_active = False

            if self.manual_active:
                vx, vy, wz = self.manual_cmd
                source = self.manual_source
                self._reset_rotate_recovery_locked()
                self._reset_yield_hold_locked()
                return self._sanitize_command(vx, vy, wz, source)

            planner_fresh = now - self.planner_stamp <= PLANNER_TIMEOUT
            if not planner_fresh:
                if self._lateral_assist_active_locked(now):
                    self._reset_rotate_recovery_locked()
                    self.reverse_request_streak = 0
                    return self._sanitize_command(0.0, 0.0, 0.0, "planner")
                self._reset_rotate_recovery_locked()
                self._reset_yield_hold_locked()
                return self._sanitize_command(0.0, 0.0, 0.0, "planner_timeout")

            if self.yield_hold_active:
                if now < self.yield_hold_deadline:
                    return self._sanitize_command(0.0, 0.0, 0.0, "yield_hold")

                self._reset_yield_hold_locked()
                self.rotate_recovery_retry_count = 0
                self.rotate_recovery_cooldown = now + ROTATE_RECOVERY_COOLDOWN
                self.reverse_request_streak = 0
                self.node.get_logger().info("[motion] 让行静止结束，恢复 planner 控制")

            if self._lateral_assist_active_locked(now):
                self._reset_rotate_recovery_locked()
                self.reverse_request_streak = 0

        recovery_cmd = self._compute_rotate_recovery_command(now, get_pose_snapshot)
        if recovery_cmd is not None:
            return self._sanitize_command(*recovery_cmd)

        if self._maybe_start_rotate_recovery(now, get_pose_snapshot):
            recovery_cmd = self._compute_rotate_recovery_command(now, get_pose_snapshot)
            if recovery_cmd is not None:
                return self._sanitize_command(*recovery_cmd)

        with self.lock:
            if now - self.planner_stamp <= PLANNER_TIMEOUT:
                vx, vy, wz = self.planner_cmd
                source = "planner"
            else:
                vx, vy, wz = 0.0, 0.0, 0.0
                source = "planner_timeout"
                self._reset_rotate_recovery_locked()

        return self._sanitize_command(vx, vy, wz, source)

    def _publish_command_state(self, vx, vy, wz, source):
        executed_msg = Twist()
        executed_msg.linear.x = vx
        executed_msg.linear.y = vy
        executed_msg.angular.z = wz
        self.executed_pub.publish(executed_msg)
        self.source_pub.publish(String(data=source))

    def _loop(self):
        if self._loop_paused:
            return
        vx, vy, wz, source = self._select_command(self.node.get_pose_snapshot)
        if not source.startswith("g1_control_") and source not in UNSMOOTHED_COMMAND_SOURCES:
            vx, vy, wz = self._smooth_command(vx, vy, wz)
        with self.lock:
            previous_cmd = self.last_sent_cmd
            previous_source = self.last_sent_source
            self.last_sent_cmd = (vx, vy, wz)
            self.last_sent_source = source

        if source != self.last_source:
            self.node.get_logger().info(f"[motion] 控制源切换 -> {source}")
            self.last_source = source

        self._publish_command_state(vx, vy, wz, source)

        suppress_sdk_move = (
            source == "planner_timeout"
            and previous_source == "planner_timeout"
            and max(abs(vx), abs(vy), abs(wz)) <= 1e-6
            and max(abs(value) for value in previous_cmd) <= 1e-6
        )
        if suppress_sdk_move:
            return

        try:
            queue_move = getattr(self.loco, "QueueMove", None)
            if queue_move is not None:
                queue_move(vx, vy, wz, source=source)
            else:
                self.loco.Move(vx, vy, wz, source=source)
        except Exception as exc:
            self._log_throttle("error", 2.0, f"[motion] SDK Move 失败: {exc}")

    def shutdown(self):
        self.set_manual_velocity(0.0, 0.0, 0.0, timeout=0.3, source="shutdown")
        for _ in range(5):
            try:
                self.loco.Move(0.0, 0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.02)
        try:
            self.loco.StopMove()
        except Exception as exc:
            self.node.get_logger().warning(f"[motion] StopMove 调用失败: {exc}")


def _get_default_trajectory_params(group):
    if group == "both":
        return {
            "kp": 45.0,
            "kd": 1.2,
            "speed_scale": 0.9,
            "blend_time": 2.0,
        }
    return {
        "kp": 40.0,
        "kd": 1.2,
        "speed_scale": 1.5,
        "blend_time": 1.5,
    }


class RobotController:
    def __init__(
        self,
        node,
        network_interface,
        enable_audio=None,
        enable_arm=None,
    ):
        self.node = node
        self._sdk_network_interface = network_interface
        sdk_python_bin = os.environ.get("G1_SDK_PYTHON_BIN") or os.environ.get(
            "G1_NAV_PYTHON_BIN"
        )
        sdk_domain_id = int(os.environ.get("G1_SDK_DOMAIN_ID", "0"))
        self._sdk_domain_id = sdk_domain_id
        if enable_audio is None:
            enable_audio = not _env_flag("G1_DISABLE_AUDIO", default=False)
        if enable_arm is None:
            enable_arm = not _env_flag("G1_DISABLE_ARM", default=False)

        node.get_logger().info("正在初始化运动、语音和动作系统...")
        node.get_logger().info(
            "SDK 子进程参数: "
            f"python={sdk_python_bin or 'current'}, "
            f"domain_id={sdk_domain_id}, "
            f"audio={'on' if enable_audio else 'off'}, "
            f"arm={'on' if enable_arm else 'off'}"
        )
        self.sdk_bridge = UnitreeSdkBridge(
            node,
            network_interface,
            python_executable=sdk_python_bin or sys.executable,
            domain_id=sdk_domain_id,
            enable_audio=enable_audio,
            enable_arm=enable_arm,
        )

        self.loco = self.sdk_bridge.loco_client
        self.motion = MotionController(node, self.loco)
        self.audio_client = self.sdk_bridge.audio_client
        self.arm_client = self.sdk_bridge.arm_client

        self._wakeup_audio()
        self._action_lock = threading.Lock()
        self._is_squatting = False
        self.global_plan_points = []
        self.plan_lock = threading.Lock()
        self.current_goal_xy = None
        self.goal_lock = threading.Lock()
        self.motion.set_rotate_recovery_heading_provider(
            self.get_rotate_recovery_heading
        )
        node.get_logger().info("运动、语音和动作系统初始化完成")

    def update_global_plan(self, path_msg):
        with self.plan_lock:
            self.global_plan_points = [
                (pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses
            ]

    def clear_global_plan(self):
        with self.plan_lock:
            self.global_plan_points = []

    def set_current_navigation_goal(self, goal_x, goal_y):
        with self.goal_lock:
            self.current_goal_xy = (goal_x, goal_y)

    def clear_current_navigation_goal(self):
        with self.goal_lock:
            self.current_goal_xy = None

    def get_path_heading_near_pose(self, current_x, current_y):
        with self.plan_lock:
            points = list(self.global_plan_points)

        if len(points) < PATH_ALIGN_MIN_POINTS:
            return None

        sample_count = min(len(points), 8)
        start_index = min(
            range(sample_count),
            key=lambda idx: math.hypot(points[idx][0] - current_x, points[idx][1] - current_y),
        )
        start_x, start_y = points[start_index]

        for next_x, next_y in points[start_index + 1 :]:
            if math.hypot(next_x - start_x, next_y - start_y) >= PATH_ALIGN_MIN_SEGMENT:
                return math.atan2(next_y - start_y, next_x - start_x)
        return None

    def get_recommended_navigation_heading(self, current_x, current_y):
        path_heading = self.get_path_heading_near_pose(current_x, current_y)
        if path_heading is not None:
            return path_heading, "path_heading"
        return None

    def get_rotate_recovery_heading(self):
        current_x, current_y, _ = self.node.get_pose_snapshot()
        return self.get_recommended_navigation_heading(current_x, current_y)

    def _wakeup_audio(self):
        if self.audio_client is None:
            return
        self.node.get_logger().info("正在唤醒音频硬件...")
        for index in range(10):
            try:
                self.audio_client.GetVolume()
                self.node.get_logger().info("音频服务已连接")
                break
            except Exception:
                self.node.get_logger().warning(f"等待音频服务... ({index + 1}/10)")
                time.sleep(1)
        try:
            self.audio_client.SetVolume(100)
        except Exception:
            pass
        time.sleep(0.5)

    def speak(self, text, voice_id=0):
        """念一段话。返回 True 表示确实交给音频硬件了。

        原来这个函数吞掉所有失败只写日志——巡航讲解那条路径无所谓，
        但网页上点了「播报」得知道到底响没响，否则只能趴到容器日志里找。
        """
        if self.audio_client is None:
            self.node.get_logger().info(f"[speech-disabled] {text}")
            return False
        try:
            self.node.get_logger().info(f"说: {text}")
            self.audio_client.TtsMaker(text, int(voice_id))
            return True
        except Exception as exc:
            self.node.get_logger().error(f"语音播放失败: {exc}")
            return False

    # ── 音量：网页上的音量条走这里 ──
    #
    # AudioClient.GetVolume() 各版本返回形状不一样：见过 (code, {"volume": 80})、
    # (code, {"name": "volume", "value": 80})，也见过直接返回数字。
    # 猜错了界面上就是个假数字，所以这里逐层剥，剥不出来就老实回 None。
    @staticmethod
    def _parse_volume(raw):
        if raw is None:
            return None
        if isinstance(raw, (int, float)):
            return int(raw)
        if isinstance(raw, (tuple, list)):
            # (code, data) 形式：code 非 0 表示失败
            if len(raw) == 2 and isinstance(raw[0], int):
                if raw[0] != 0:
                    return None
                return RobotController._parse_volume(raw[1])
            for item in raw:
                got = RobotController._parse_volume(item)
                if got is not None:
                    return got
            return None
        if isinstance(raw, dict):
            for key in ("volume", "value", "Volume", "Value"):
                if key in raw:
                    return RobotController._parse_volume(raw[key])
            return None
        if isinstance(raw, str):
            try:
                return int(float(raw.strip()))
            except ValueError:
                return None
        return None

    def get_volume(self):
        """返回 0-100 的音量；音频不可用或读不出来返回 None。"""
        if self.audio_client is None:
            return None
        try:
            value = self._parse_volume(self.audio_client.GetVolume())
        except Exception as exc:
            self.node.get_logger().warning(f"读取音量失败: {exc}")
            return None
        if value is None:
            return None
        return max(0, min(100, int(value)))

    def set_volume(self, volume):
        """设置音量，返回设置后实际读回的值（读不回就返回请求值）。"""
        if self.audio_client is None:
            raise RuntimeError("音频客户端不可用")
        target = max(0, min(100, int(volume)))
        self.audio_client.SetVolume(target)
        return self.get_volume() if self.get_volume() is not None else target

    def perform_interaction(self, text, action_id):
        """到点讲解：做个动作 + 念一段话 + 复位。

        动作和讲解各自可缺省：巡航点只填了讲解词就只说话，只填了动作就
        只做动作。原来这里对空参数不设防——action_id 为 None 时
        ExecuteAction 抛异常被吞掉，接着念空串、再白做一次复位动作，
        平白多花五秒还看不出为什么。
        """
        text = str(text or "").strip()
        try:
            action_id = int(action_id) if action_id is not None else 0
        except (TypeError, ValueError):
            action_id = 0
        do_action = self.arm_client is not None and action_id > 0
        self.node.get_logger().info(
            f"执行交互: action_id={action_id if do_action else '无'} 讲解={'有' if text else '无'}"
        )
        if not do_action and not text:
            return

        if do_action:
            try:
                self.arm_client.ExecuteAction(action_id)
                time.sleep(2.0)
            except Exception as exc:
                self.node.get_logger().error(f"动作执行失败: {exc}")

        if text:
            self.speak(text)
            # TtsMaker 是异步的，没有"念完了"的回调，只能按字数估时间等它。
            # 0.195s/字 是现场调出来的经验值，宁可多等一点也别把话打断。
            estimated_speech_time = len(text) * 0.195
            self.node.get_logger().info(
                f"预计讲解时长 {estimated_speech_time:.1f}s，等待讲解结束"
            )
            time.sleep(estimated_speech_time)

        # 没做动作就没什么好复位的
        if do_action:
            try:
                self.arm_client.ExecuteAction(99)
                time.sleep(3.0)
            except Exception as exc:
                self.node.get_logger().error(f"复位失败: {exc}")

    @staticmethod
    def _load_preset_action_config():
        """加载 config/preset_actions.json，返回 {action_id_int: {"name": ..., "duration_sec": ...}}。"""
        try:
            path = config_file("preset_actions.json")
            with open(path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            config = {}
            for key, value in raw.items():
                if key.startswith("_"):
                    continue
                try:
                    config[int(key)] = value
                except (ValueError, TypeError):
                    pass
            return config
        except Exception as exc:
            _nav_logger.warning("[arm_action_safe] 加载 preset_actions.json 失败: %s", exc)
            return {}

    def _get_preset_duration(self, action_id):
        """获取预设动作的配置时长（秒），未配置则返回 None。"""
        if not hasattr(self, "_preset_config"):
            self._preset_config = self._load_preset_action_config()
        entry = self._preset_config.get(int(action_id))
        if entry is None:
            return None
        return float(entry.get("duration_sec", 0))

    def execute_arm_action_safe(self, action_id, reset_after=True):
        if self.arm_client is None:
            raise RuntimeError("手臂能力不可用")

        action_id = int(action_id)
        t_total = time.monotonic()
        t0 = time.monotonic()
        duration = self._get_preset_duration(action_id)
        _nav_logger.info(
            "[arm_action_safe] T_preset_lookup=%.3fs (id=%d, duration=%s)",
            time.monotonic() - t0, action_id, duration,
        )

        t0 = time.monotonic()
        with self._action_lock:
            t_locked = time.monotonic()
            _nav_logger.info(
                "[arm_action_safe] T_action_lock_acquire=%.3fs",
                t_locked - t0,
            )
            t0 = time.monotonic()
            self.motion.pause_loop("arm_action_hold")
            _nav_logger.info(
                "[arm_action_safe] T_pause_loop=%.3fs",
                time.monotonic() - t0,
            )
            try:
                if duration is not None and duration > 0 and action_id != 99:
                    # ── 定时模式：后台线程发动作 SDK，主线程等播放时长后同步发 99 复位 ──
                    playback_duration = duration
                    _nav_logger.info(
                        "[arm_action_safe] ExecuteAction(%d) 定时模式, "
                        "playback=%.2fs (preset duration=%.2fs)",
                        action_id, playback_duration, duration,
                    )
                    sdk_error = [None]

                    def _run_sdk():
                        try:
                            self.arm_client.ExecuteAction(action_id)
                        except Exception as exc:
                            sdk_error[0] = exc
                            _nav_logger.warning(
                                "[arm_action_safe] 后台 ExecuteAction(%d) 异常: %s",
                                action_id, exc,
                            )

                    sdk_thread = threading.Thread(target=_run_sdk, daemon=True)
                    sdk_thread.start()
                    time.sleep(playback_duration)
                    _nav_logger.info(
                        "[arm_action_safe] 播放 %.2fs 结束，同步发 99 复位",
                        playback_duration,
                    )

                    if reset_after:
                        t_reset = time.monotonic()
                        try:
                            reset_result = self.arm_client.ExecuteAction(99)
                            _nav_logger.info(
                                "[arm_action_safe] ExecuteAction(99) 返回 %s, 耗时 %.3fs",
                                reset_result, time.monotonic() - t_reset,
                            )
                        except Exception as exc:
                            _nav_logger.warning(
                                "[arm_action_safe] 同步复位(99) 异常: %s", exc,
                            )
                else:
                    # ── 同步模式（复位动作 99 或无配置时长的动作）──
                    t_sdk = time.monotonic()
                    result = self.arm_client.ExecuteAction(action_id)
                    _nav_logger.info(
                        "[arm_action_safe] ExecuteAction(%d) 返回 %s，耗时 %.3fs",
                        action_id, result, time.monotonic() - t_sdk,
                    )
                    if result not in (None, 0, 3104):
                        raise RuntimeError(
                            f"手臂动作 {action_id} 执行失败，返回码: {result}"
                        )

                _nav_logger.info("[arm_action_safe] 总耗时 %.3fs", time.monotonic() - t_total)
                return {
                    "status": "success",
                    "message": f"手臂动作 {action_id} 完成",
                }
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def play_trajectory_safe(
        self, rows, params=None, hold_secs=0.0, release_time=1.0, dt=0.01,
    ):
        if self.arm_client is None:
            raise RuntimeError("手臂能力不可用")

        if params is None:
            group = rows[0].get("group", "unknown") if rows else "unknown"
            params = _get_default_trajectory_params(group)

        with self._action_lock:
            self.motion.pause_loop("trajectory_hold")
            try:
                result = self.arm_client.PlayTrajectory(
                    rows=rows,
                    params=params,
                    hold_secs=hold_secs,
                    release_time=release_time,
                    dt=dt,
                )
                return {
                    "status": "success",
                    "message": (
                        f"轨迹播放完成，{result.get('frame_count', 0)} 帧，"
                        f"耗时 {result.get('elapsed', 0):.2f}s"
                    ),
                    "result": result,
                }
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def execute_custom_action_safe(self, action_name, wait=5.0):
        """执行 APP 端录制的示教动作（Custom Action）。"""
        if self.arm_client is None:
            raise RuntimeError("手臂能力不可用")

        with self._action_lock:
            self.motion.pause_loop("custom_action_hold")
            try:
                _nav_logger.info(
                    "[custom_action_safe] ExecuteCustomAction(%s)", action_name,
                )
                code = self.arm_client.ExecuteCustomAction(action_name)
                if code not in (None, 0):
                    from teach.arm_action import format_error
                    raise RuntimeError(
                        f"示教动作 {action_name} 执行失败: {format_error(code)}"
                    )
                if wait and wait > 0:
                    time.sleep(wait)
                return {
                    "status": "success",
                    "message": f"示教动作 {action_name} 已执行",
                }
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def run_movement_script_safe(self, script_rel_path, movement_base_dir, cancel_event=None):
        """执行 config/movement/ 中的编排脚本。"""
        from pathlib import Path as _Path
        from teach.robot_io import RobotSession
        from teach.script_runner import run_script

        abs_path = str((_Path(movement_base_dir) / script_rel_path).resolve())
        _nav_logger.info(
            "[movement_script_safe] run_script(%s)", abs_path,
        )

        session = RobotSession(
            self.sdk_bridge,
        )
        with self._action_lock:
            self.motion.pause_loop("movement_script_hold")
            try:
                run_script(
                    session,
                    abs_path,
                    movement_dir=movement_base_dir,
                    release=True,
                    cancel_event=cancel_event,
                )
                return {
                    "status": "success",
                    "message": f"脚本 {script_rel_path} 执行完成",
                }
            except Exception as exc:
                _nav_logger.error(
                    "[movement_script_safe] 脚本执行失败: %s", exc,
                )
                raise
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def rotate_to_yaw(self, target_yaw, max_abs=0.8, min_abs=0.10, gain=1.6):
        self.node.get_logger().info(
            f"开始原地旋转修正航向至 {math.degrees(target_yaw):.1f}°"
        )
        timeout = time.time() + 20.0
        converge_count = 0
        converge_threshold = 0.10
        converge_required = 5

        while rclpy.ok() and time.time() < timeout:
            self.node.spin_for(0.05)
            try:
                _, _, current_yaw = self.node.lookup_current_pose()
            except RuntimeError:
                continue

            yaw_diff = normalize_angle(target_yaw - current_yaw)
            if abs(yaw_diff) < converge_threshold:
                converge_count += 1
                self.motion.set_manual_velocity(
                    0.0, 0.0, 0.0, timeout=0.25, source="rotate_to_yaw_settle"
                )
                if converge_count >= converge_required:
                    self.node.get_logger().info(
                        f"航向对齐完成，残差 {math.degrees(yaw_diff):.1f}°"
                    )
                    break
                continue
            else:
                converge_count = 0

            cmd_wz = build_rotation_command(
                yaw_diff,
                max_abs=max_abs,
                min_abs=min_abs,
                gain=gain,
            )
            self.motion.set_manual_velocity(
                0.0, 0.0, cmd_wz, timeout=0.25, source="rotate_to_yaw"
            )

        self.motion.set_manual_velocity(0.0, 0.0, 0.0, timeout=0.3, source="rotate_stop")
        time.sleep(0.3)
        self.motion.clear_manual_override()

    def squat_safe(self):
        """安全蹲下：暂停行走循环 → 调 SDK → 恢复行走循环。"""
        with self._action_lock:
            self.motion.pause_loop("squat_hold")
            try:
                _nav_logger.info("[squat_safe] 开始执行蹲下")
                self.loco.StandUp2Squat()
                time.sleep(10.0)  # 实测蹲下约 10s
                self._is_squatting = True
                _nav_logger.info("[squat_safe] 蹲下完成")
                return {"status": "success", "message": "已蹲下"}
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def stand_up_safe(self):
        """安全站起来：暂停行走循环 → 调 SDK → 恢复行走循环。"""
        with self._action_lock:
            self.motion.pause_loop("stand_up_hold")
            try:
                _nav_logger.info("[stand_up_safe] 开始执行站起来")
                self.loco.Squat2StandUp()
                time.sleep(9.0)  # 实测站立约 9s
                self._is_squatting = False
                _nav_logger.info("[stand_up_safe] 站起来完成")
                return {"status": "success", "message": "已站起来"}
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    # 零力矩检查阈值 (Nm)
    ZERO_TORQUE_THRESHOLD = 0.5

    def damp_safe(self):
        """切换到阻尼模式 (FSM 1)：暂停行走循环 → 检查零力矩 → 调 SDK Damp。

        安全规则：只有在所有关节力矩接近零时才允许切换到阻尼模式，
        防止机器人在承载状态下进入阻尼模式导致摔倒。
        """
        with self._action_lock:
            self.motion.pause_loop("damp_hold")
            try:
                # ── 零力矩前置检查 ──
                _nav_logger.info("[damp_safe] 检查关节力矩...")
                try:
                    torque_result = self.loco.CheckZeroTorque(
                        threshold=self.ZERO_TORQUE_THRESHOLD
                    )
                except Exception as exc:
                    _nav_logger.error(
                        "[damp_safe] 力矩检查失败: %s，拒绝切换阻尼模式", exc
                    )
                    return {
                        "status": "error",
                        "message": f"力矩检查失败: {exc}，无法切换阻尼模式",
                    }

                if not torque_result.get("is_zero", False):
                    max_tau = torque_result.get("max_abs_tau", "?")
                    details = torque_result.get("details", "")
                    msg = (
                        f"关节力矩不为零 (最大 |τ|={max_tau}Nm)，"
                        f"无法切换阻尼模式。{details}"
                    )
                    _nav_logger.warning("[damp_safe] 拒绝: %s", msg)
                    return {"status": "error", "message": msg}

                _nav_logger.info(
                    "[damp_safe] 零力矩检查通过 (max_abs_tau=%.4f)，切换到阻尼模式",
                    torque_result.get("max_abs_tau", 0),
                )
                self.loco.Damp()
                time.sleep(1.0)
                _nav_logger.info("[damp_safe] 阻尼模式切换完成")
                return {"status": "success", "message": "已切换到阻尼模式"}
            finally:
                self.motion.resume_loop()
                self.motion.clear_manual_override()

    def start_safe(self):
        """切换到正常站立/行走模式 (FSM 200)：调 SDK Start → 清除蹲下标记。"""
        with self._action_lock:
            _nav_logger.info("[start_safe] 切换到站立模式")
            self.loco.Start()
            time.sleep(3.0)  # 等待站立姿态就绪
            self._is_squatting = False
            _nav_logger.info("[start_safe] 站立模式切换完成")
            return {"status": "success", "message": "已切换到站立模式"}

    def get_fsm_id_safe(self):
        """查询当前 FSM ID。失败抛异常由 service 层捕获。"""
        return int(self.loco.GetFsmId())

    def set_fsm_id_safe(self, fsm_id):
        """切换到指定 FSM 状态。

        - 仅透传 SetFsmId 到底层；到位判定由上层（MotionModeService）通过
          GetFsmId 轮询完成。
        - 用 _action_lock 串行化，避免与 squat / start 系列并发。
        """
        fsm_id = int(fsm_id)
        with self._action_lock:
            _nav_logger.info("[set_fsm_id_safe] SetFsmId(%d)", fsm_id)
            ret = self.loco.SetFsmId(fsm_id)
            ret_int = int(ret) if ret is not None else 0
            status = "success" if ret_int == 0 else "error"
            if ret_int == 0:
                _nav_logger.info("[set_fsm_id_safe] 返回码: %s", ret)
            else:
                _nav_logger.warning("[set_fsm_id_safe] 非零返回码: %s", ret)
            return {
                "status": status,
                "message": f"SetFsmId({fsm_id}) 返回 {ret_int}",
                "return_code": ret_int,
            }

    # FSM ID 801 = 站立稳态 (从蹲下起来后, 由 SDK GetFsmId RPC 7001 返回)
    # FSM ID 1   = Damp (歧义: 可能是蹲下稳态, 也可能是初始开机)
    FSM_ID_STANDING = 801

    @property
    def is_squatting(self):
        """返回当前是否处于蹲下状态。

        以软件标记 ``_is_squatting`` 为主;
        如果硬件 FSM == 801 (确定站立), 则强制修正为 False。
        """
        try:
            fsm_id = self.loco.GetFsmId()
            if fsm_id == self.FSM_ID_STANDING and self._is_squatting:
                _nav_logger.info(
                    "[is_squatting] 硬件 FSM=%s 确认站立, "
                    "修正 _is_squatting=True → False",
                    fsm_id,
                )
                self._is_squatting = False
        except Exception:
            pass
        return self._is_squatting

    def shutdown(self):
        try:
            self.motion.shutdown()
        finally:
            self.sdk_bridge.close()


class MissionNode(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.args = args
        self.ready_state = None
        self.current_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.pose_lock = threading.Lock()
        self.goal_costmap_lock = threading.Lock()
        self.goal_costmap = None
        self.goal_costmap_stamp = 0.0
        self._profile_warn_times = {}
        self._external_spin = False
        self.motion_policy = load_motion_policy_config()
        self.walking_mode = load_walking_mode_config()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.client_cb_group = ReentrantCallbackGroup()

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.client_cb_group,
        )
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
            callback_group=self.client_cb_group,
        )
        parameter_client_cls = AsyncParameterClient or HumbleAsyncParameterClientCompat
        self.parameter_client = parameter_client_cls(self, "controller_server")
        self.velocity_smoother_parameter_client = parameter_client_cls(
            self, "velocity_smoother"
        )
        self.controller_get_parameters_client = self.create_client(
            GetParameters,
            "/controller_server/get_parameters",
            callback_group=self.client_cb_group,
        )
        self.ensure_client = self.create_client(
            Trigger,
            NAV_MANAGER_ENSURE_SERVICE,
            callback_group=self.client_cb_group,
        )
        self.bt_navigator_state_client = self.create_client(
            GetState,
            "/bt_navigator/get_state",
            callback_group=self.client_cb_group,
        )
        self.bt_navigator_param_client = self.create_client(
            GetParameters,
            "/bt_navigator/get_parameters",
            callback_group=self.client_cb_group,
        )
        self.yield_prompt_event_pub = self.create_publisher(
            String, YIELD_PROMPT_EVENT_TOPIC, 10
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self.client_cb_group,
        )
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self.client_cb_group,
        )
        self.ready_sub = self.create_subscription(
            Bool,
            NAV_MANAGER_READY_TOPIC,
            self._ready_callback,
            10,
            callback_group=self.client_cb_group,
        )
        self.goal_costmap_sub = None
        if self.motion_policy.goal_occupancy_enabled:
            self.goal_costmap_sub = self.create_subscription(
                Costmap,
                self.motion_policy.goal_occupancy_costmap_topic,
                self._goal_costmap_callback,
                10,
                callback_group=self.client_cb_group,
            )

    def shutdown_resources(self):
        listener = getattr(self, "tf_listener", None)
        if listener is None:
            return

        executor = getattr(listener, "executor", None)
        if executor is not None:
            try:
                executor.shutdown()
            except Exception as exc:
                self.get_logger().warning(f"关闭 TF listener executor 失败: {exc}")

        thread = getattr(listener, "dedicated_listener_thread", None)
        if thread is not None:
            try:
                thread.join(timeout=2.0)
            except Exception as exc:
                self.get_logger().warning(f"等待 TF listener 线程退出失败: {exc}")

        if hasattr(listener, "tf_sub") or hasattr(listener, "tf_static_sub"):
            try:
                listener.unregister()
            except Exception as exc:
                self.get_logger().warning(f"注销 TF listener 订阅失败: {exc}")

        try:
            listener.__del__ = lambda *a, **kw: None
        except Exception:
            pass

        self.tf_listener = None

    def _ready_callback(self, msg):
        self.ready_state = msg.data

    def _goal_costmap_callback(self, msg):
        with self.goal_costmap_lock:
            self.goal_costmap = msg
            self.goal_costmap_stamp = time.time()

    def set_external_spin(self, enabled):
        self._external_spin = bool(enabled)

    def spin_for(self, timeout_sec):
        end = time.time() + timeout_sec
        if self._external_spin:
            while time.time() < end and rclpy.ok():
                time.sleep(min(0.05, max(end - time.time(), 0.0)))
            return
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(end - time.time(), 0.0)))

    def wait_for_future(self, future, timeout_sec=None):
        deadline = None if timeout_sec is None else time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            self.spin_for(0.05)
            if deadline is not None and time.time() >= deadline:
                raise TimeoutError("等待 future 超时")
        return future.result()

    def wait_for_service(self, client, timeout_sec, label):
        if hasattr(client, "wait_for_service") and client.wait_for_service(timeout_sec=timeout_sec):
            return
        if hasattr(client, "wait_for_services") and client.wait_for_services(timeout_sec=timeout_sec):
            return
        raise RuntimeError(f"未发现 {label}")

    def _wait_for_service_optional(self, client, timeout_sec):
        try:
            self.wait_for_service(client, timeout_sec, "optional")
            return True
        except RuntimeError:
            return False

    @staticmethod
    def _parameter_value_to_text(value):
        if value.type == ParameterType.PARAMETER_BOOL:
            return str(value.bool_value)
        if value.type == ParameterType.PARAMETER_INTEGER:
            return str(value.integer_value)
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return str(value.double_value)
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return str(list(value.byte_array_value))
        if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return str(list(value.bool_array_value))
        if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return str(list(value.integer_array_value))
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return str(list(value.double_array_value))
        if value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return str(list(value.string_array_value))
        return "<unset>"

    def diagnose_goal_rejection(self, bt_path):
        lines = []
        bt_info = inspect_bt_file(bt_path)

        lines.append("导航目标拒绝诊断:")
        lines.append(f"  bt_path={bt_info['path']}")
        lines.append(f"  bt_exists={'yes' if bt_info['exists'] else 'no'}")
        lines.append(f"  bt_resolved={bt_info['resolved']}")
        if bt_info["size"] is not None:
            lines.append(f"  bt_size={bt_info['size']} bytes")
        if bt_info["xml_ok"]:
            lines.append(f"  bt_xml_parse=ok, tags={bt_info['tags']}")
        else:
            lines.append(f"  bt_xml_parse=error, detail={bt_info['xml_error'] or 'unknown'}")

        try:
            nav_ready = self.navigate_client.wait_for_server(timeout_sec=0.2)
            lines.append(f"  navigate_to_pose_server_ready={'yes' if nav_ready else 'no'}")
        except Exception as exc:
            lines.append(f"  navigate_to_pose_server_ready=error, detail={exc}")

        if self._wait_for_service_optional(self.bt_navigator_state_client, 0.5):
            try:
                response = self.wait_for_future(
                    self.bt_navigator_state_client.call_async(GetState.Request()), 2.0
                )
                lines.append(
                    "  bt_navigator_state="
                    f"{response.current_state.label} ({response.current_state.id})"
                )
            except Exception as exc:
                lines.append(f"  bt_navigator_state=error, detail={exc}")
        else:
            lines.append("  bt_navigator_state=service_unavailable")

        plugin_lib_names = []
        if self._wait_for_service_optional(self.bt_navigator_param_client, 0.5):
            try:
                request = GetParameters.Request()
                request.names = [
                    "default_nav_to_pose_bt_xml",
                    "default_nav_through_poses_bt_xml",
                    "plugin_lib_names",
                    "navigators",
                ]
                response = self.wait_for_future(
                    self.bt_navigator_param_client.call_async(request), 2.0
                )
                params = {
                    name: value for name, value in zip(request.names, response.values)
                }
                for name in request.names:
                    value = params[name]
                    text = self._parameter_value_to_text(value)
                    lines.append(f"  bt_navigator_param[{name}]={text}")
                    if name == "plugin_lib_names" and value.type == ParameterType.PARAMETER_STRING_ARRAY:
                        plugin_lib_names = list(value.string_array_value)
            except Exception as exc:
                lines.append(f"  bt_navigator_param=error, detail={exc}")
        else:
            lines.append("  bt_navigator_param=service_unavailable")

        if bt_info["required_plugins"]:
            lines.append(f"  bt_required_plugins={bt_info['required_plugins']}")
            if plugin_lib_names:
                missing = missing_bt_plugins(bt_info["required_plugins"], plugin_lib_names)
                lines.append(
                    f"  bt_missing_plugins={missing if missing else '[]'}"
                )
            else:
                lines.append("  bt_missing_plugins=unknown (plugin_lib_names unavailable)")

        return "\n".join(lines)

    def get_bt_navigator_state(self):
        if not self._wait_for_service_optional(self.bt_navigator_state_client, 0.5):
            return None, "service_unavailable"
        try:
            response = self.wait_for_future(
                self.bt_navigator_state_client.call_async(GetState.Request()), 2.0
            )
            if response is None:
                return None, "query_failed"
            state_id = response.current_state.id
            state_label = response.current_state.label or "unknown"
            return state_id, state_label
        except Exception as exc:
            return None, f"error:{exc}"

    def wait_for_bt_navigator_active(self, timeout_sec):
        deadline = time.time() + timeout_sec
        last_state = "unknown"
        while time.time() < deadline and rclpy.ok():
            state_id, state_label = self.get_bt_navigator_state()
            last_state = state_label
            if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
                return True, state_label
            self.spin_for(0.1)
        return False, last_state

    def _runtime_navigation_ready(self, timeout_sec=1.0):
        nav_action_ready = False
        try:
            nav_action_ready = self.navigate_client.wait_for_server(timeout_sec=0.2)
        except Exception:
            nav_action_ready = False

        active, state_label = self.wait_for_bt_navigator_active(timeout_sec)
        return bool(nav_action_ready and active), state_label

    def lookup_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            raise RuntimeError(f"无法获取 TF 变换: {exc}") from exc

        q = transform.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self.pose_lock:
            self.current_pose["x"] = transform.transform.translation.x
            self.current_pose["y"] = transform.transform.translation.y
            self.current_pose["yaw"] = yaw
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def get_pose_snapshot(self):
        with self.pose_lock:
            return (
                self.current_pose["x"],
                self.current_pose["y"],
                self.current_pose["yaw"],
            )

    def ensure_navigation_stack_ready_with_manager(self):
        self.get_logger().info("等待 navigation_manager 服务...")
        try:
            self.wait_for_service(
                self.ensure_client, NAV_MANAGER_DISCOVERY_TIMEOUT, "navigation_manager ensure_ready"
            )
        except RuntimeError as exc:
            raise RuntimeError(
                "未发现 /navigation_manager/ensure_ready 服务。"
                "请先在另一个终端运行 `bash ./start_navigation_manager.sh`，"
                "或用 `ros2 service list | grep navigation_manager` 检查管理节点是否已启动。"
            ) from exc

        runtime_ready, state_label = self._runtime_navigation_ready(1.0)
        if runtime_ready:
            if self.ready_state is True:
                self.get_logger().info("navigation_manager 已确认底层导航 ready")
            else:
                self.get_logger().warning(
                    "未收到 /navigation_manager/ready 快照，但 Nav2 action 与 "
                    f"bt_navigator 已就绪({state_label})，跳过 ensure_ready service"
                )
            return

        if self.ready_state is True:
            self.get_logger().warning(
                f"navigation_manager 标记 ready，但 bt_navigator 当前仍是 {state_label}，继续调用 ensure_ready"
            )

        request = Trigger.Request()
        try:
            response = self.wait_for_future(self.ensure_client.call_async(request), 60.0)
        except TimeoutError:
            runtime_ready, state_label = self._runtime_navigation_ready(2.0)
            if runtime_ready:
                self.get_logger().warning(
                    "等待 navigation_manager ensure_ready 响应超时，但 Nav2 已就绪"
                    f"({state_label})，继续执行导航"
                )
                return
            raise
        if not response.success:
            raise RuntimeError(
                f"navigation_manager ensure_ready 失败: {response.message or '未知错误'}"
            )

        deadline = time.time() + NAV_MANAGER_READY_TOPIC_TIMEOUT
        while time.time() < deadline and rclpy.ok():
            self.spin_for(0.1)
            if self.ready_state is True:
                active, state_label = self.wait_for_bt_navigator_active(5.0)
                if active:
                    self.get_logger().info("navigation_manager ensure_ready 成功")
                    return
                self.get_logger().warning(
                    f"/navigation_manager/ready=true，但 bt_navigator 仍是 {state_label}"
                )

        active, state_label = self.wait_for_bt_navigator_active(2.0)
        if active:
            self.get_logger().warning(
                "ensure_ready 已成功，虽然 /navigation_manager/ready 信号延迟，但 bt_navigator 已 active，继续执行"
            )
            return

        raise RuntimeError(
            "navigation_manager ensure_ready 已返回成功，但 bt_navigator 仍未进入 active "
            f"(current={state_label})"
        )

    def _log_throttle_profile(self, message, period=10.0):
        now = time.time()
        last = self._profile_warn_times.get(message, 0.0)
        if now - last < period:
            return
        self._profile_warn_times[message] = now
        self.get_logger().warning(message)

    @staticmethod
    def _navigation_profile_lateral_values(policy):
        if policy.global_allow_lateral_motion:
            return NavigationProfileLateralValues(
                max_vel_y=policy.global_max_vel_y,
                min_vel_y=policy.global_min_vel_y,
                vy_samples=policy.global_vy_samples,
                acc_lim_y=policy.global_acc_lim_y,
                decel_lim_y=policy.global_decel_lim_y,
            )
        return NavigationProfileLateralValues(
            max_vel_y=0.0,
            min_vel_y=0.0,
            vy_samples=1,
            acc_lim_y=0.2,
            decel_lim_y=-0.2,
        )

    @staticmethod
    def _navigation_profile_parameter_groups(profile, policy):
        lateral = MissionNode._navigation_profile_lateral_values(policy)
        legacy_parameters = [
            _double_parameter("FollowPath.max_vel_x", profile["max_vel_x"]),
            _double_parameter(
                "FollowPath.max_speed_xy",
                max(profile["max_vel_x"], lateral.max_vel_y),
            ),
            _double_parameter("FollowPath.min_vel_y", lateral.min_vel_y),
            _double_parameter("FollowPath.max_vel_y", lateral.max_vel_y),
            _double_parameter("FollowPath.max_vel_theta", profile["max_vel_theta"]),
            _double_parameter("FollowPath.acc_lim_x", profile["acc_lim_x"]),
            _double_parameter("FollowPath.acc_lim_y", lateral.acc_lim_y),
            _double_parameter("FollowPath.acc_lim_theta", profile["acc_lim_theta"]),
            _double_parameter("FollowPath.decel_lim_y", lateral.decel_lim_y),
            _integer_parameter("FollowPath.vy_samples", lateral.vy_samples),
            _double_parameter("FollowPath.xy_goal_tolerance", 0.3),
            _double_parameter("FollowPath.PathAlign.scale", profile["path_distance_bias"]),
            _double_parameter("FollowPath.PathDist.scale", profile["path_distance_bias"]),
            _double_parameter("FollowPath.GoalAlign.scale", profile["goal_distance_bias"]),
            _double_parameter("FollowPath.GoalDist.scale", profile["goal_distance_bias"]),
            _double_parameter("FollowPath.BaseObstacle.scale", profile["occdist_scale"]),
            _double_parameter(
                "FollowPath.ObstacleFootprint.scale",
                profile["footprint_scale"],
            ),
        ]
        mppi_parameters = [
            _double_parameter("FollowPath.vx_max", profile["max_vel_x"]),
            _double_parameter("FollowPath.vx_min", 0.0),
            _double_parameter("FollowPath.vy_max", lateral.max_vel_y),
            _double_parameter("FollowPath.wz_max", profile["max_vel_theta"]),
        ]
        return {"mppi": mppi_parameters, "legacy": legacy_parameters}, lateral

    @staticmethod
    def _select_profile_parameters(parameter_groups, declared_names):
        if declared_names:
            declared = set(declared_names)
            for controller_kind in ("mppi", "legacy"):
                selected = [
                    parameter
                    for parameter in parameter_groups[controller_kind]
                    if parameter.name in declared
                ]
                if selected:
                    return selected, controller_kind
        return list(parameter_groups["legacy"]), "legacy"

    def _declared_controller_parameters(self, names):
        client = getattr(self, "controller_get_parameters_client", None)
        if client is None or not self._wait_for_service_optional(client, 0.5):
            return None

        request = GetParameters.Request()
        request.names = list(names)
        try:
            response = self.wait_for_future(client.call_async(request), 1.0)
        except Exception as exc:
            self._log_throttle_profile(
                f"[profile] controller_server 参数声明查询失败，使用兼容模式: {exc}"
            )
            return None

        not_set = getattr(ParameterType, "PARAMETER_NOT_SET", 0)
        declared = set()
        for name, value in zip(request.names, getattr(response, "values", [])):
            if getattr(value, "type", not_set) != not_set:
                declared.add(name)
        return declared

    def apply_navigation_profile(self, blend, reason=None):
        profile = build_navigation_profile(blend, walking_mode=self.walking_mode)
        policy = self.motion_policy
        parameter_groups, lateral = self._navigation_profile_parameter_groups(
            profile,
            policy,
        )
        candidate_names = [
            parameter.name
            for parameters in parameter_groups.values()
            for parameter in parameters
        ]
        declared_names = self._declared_controller_parameters(candidate_names)
        parameters, controller_kind = self._select_profile_parameters(
            parameter_groups,
            declared_names,
        )

        self.wait_for_service(self.parameter_client, 5.0, "controller_server 参数服务")
        try:
            result = self.wait_for_future(
                self.parameter_client.set_parameters(parameters), 10.0
            )
        except TimeoutError:
            self.get_logger().warning(
                f"controller_server 参数更新超时，跳过本次档位切换 (blend={profile['blend']:.1f})"
            )
            return
        hard_failures = []
        for parameter, entry in zip(parameters, result.results):
            if entry.successful:
                continue
            reason_text = (entry.reason or "").lower()
            if "not declared" in reason_text or "undeclared" in reason_text:
                self._log_throttle_profile(
                    f"[profile] 跳过未声明参数 {parameter.name}（critic 未加载？请重启 nav2 拉新 YAML）"
                )
                continue
            hard_failures.append(f"{parameter.name}: {entry.reason}")
        if hard_failures:
            raise RuntimeError("; ".join(hard_failures))
        self._apply_velocity_smoother_lateral_policy(
            profile,
            lateral.max_vel_y,
            lateral.min_vel_y,
            lateral.acc_lim_y,
            lateral.decel_lim_y,
        )
        stage = (
            "快速巡航"
            if profile["blend"] <= 0.0
            else "精准调整"
            if profile["blend"] >= 1.0
            else "渐进减速"
        )
        reason_text = f", {reason}" if reason else ""
        if controller_kind == "mppi":
            self.get_logger().info(
                f"导航速度档位: {stage} (controller=MPPI, blend={profile['blend']:.1f}, "
                f"v_x={profile['max_vel_x']:.2f}, v_y={lateral.max_vel_y:.2f}, "
                f"w_z={profile['max_vel_theta']:.2f}{reason_text})"
            )
        else:
            self.get_logger().info(
                f"导航速度档位: {stage} (controller=legacy, blend={profile['blend']:.1f}, "
                f"v_x={profile['max_vel_x']:.2f}, v_y={lateral.max_vel_y:.2f}, "
                f"acc_x={profile['acc_lim_x']:.2f}, "
                f"path_bias={profile['path_distance_bias']:.1f}, occdist={profile['occdist_scale']:.2f}, "
                f"footprint={profile['footprint_scale']:.2f}{reason_text})"
            )

    def _apply_velocity_smoother_lateral_policy(
        self, profile, max_vel_y, min_vel_y, acc_lim_y, decel_lim_y
    ):
        if not self._wait_for_service_optional(
            self.velocity_smoother_parameter_client, 0.5
        ):
            self._log_throttle_profile(
                "[profile] velocity_smoother 参数服务不可用，跳过横向速度平滑参数更新"
            )
            return

        parameters = [
            _double_array_parameter(
                "max_velocity",
                [profile["max_vel_x"], max_vel_y, profile["max_vel_theta"]],
            ),
            _double_array_parameter(
                "min_velocity", [0.0, min_vel_y, -profile["max_vel_theta"]]
            ),
            _double_array_parameter(
                "max_accel",
                [profile["acc_lim_x"], acc_lim_y, profile["acc_lim_theta"]],
            ),
            _double_array_parameter(
                "max_decel",
                [-profile["acc_lim_x"], decel_lim_y, -profile["acc_lim_theta"]],
            ),
        ]
        try:
            result = self.wait_for_future(
                self.velocity_smoother_parameter_client.set_parameters(parameters),
                5.0,
            )
        except TimeoutError:
            self._log_throttle_profile(
                "[profile] velocity_smoother 参数更新超时，跳过本次横向速度策略同步"
            )
            return

        failures = []
        for parameter, entry in zip(parameters, result.results):
            if entry.successful:
                continue
            reason_text = (entry.reason or "").lower()
            if "not declared" in reason_text or "undeclared" in reason_text:
                self._log_throttle_profile(
                    f"[profile] 跳过未声明 velocity_smoother 参数 {parameter.name}"
                )
                continue
            failures.append(f"{parameter.name}: {entry.reason}")
        if failures:
            self._log_throttle_profile(
                "[profile] velocity_smoother 参数更新失败: " + "; ".join(failures)
            )

    def clear_costmaps(self):
        request = ClearEntireCostmap.Request()
        for client, label in (
            (self.clear_local_costmap_client, "local_costmap"),
            (self.clear_global_costmap_client, "global_costmap"),
        ):
            self.wait_for_service(client, 5.0, label)
            self.wait_for_future(client.call_async(request), 5.0)

    def build_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.args.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = quaternion_dict_from_yaw(yaw)
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        return pose

    def compute_path(self, start_pose, goal_pose):
        if not self.path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("compute_path_to_pose action server 不可用，跳过路径预计算")
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        send_future = self.path_client.send_goal_async(goal_msg)
        goal_handle = self.wait_for_future(send_future, 10.0)
        if not goal_handle.accepted:
            self.get_logger().warning("ComputePathToPose 请求被拒绝")
            return None
        result = self.wait_for_future(goal_handle.get_result_async(), 10.0)
        return result.result.path

    def align_with_initial_path(self, robot_controller, current_x, current_y, current_yaw):
        path_heading = None
        heading_source = ""
        deadline = time.time() + PATH_ALIGN_TIMEOUT

        robot_controller.motion.hold_position("path_align_wait")
        try:
            current_x, current_y, current_yaw = self.lookup_current_pose()
        except RuntimeError as exc:
            self.get_logger().warning(f"预旋转前刷新当前位姿失败，使用导航开始时位姿: {exc}")

        while time.time() < deadline and rclpy.ok():
            heading_result = robot_controller.get_recommended_navigation_heading(
                current_x, current_y
            )
            if heading_result is not None:
                path_heading, heading_source = heading_result
                break
            self.spin_for(0.05)

        if path_heading is None:
            self.get_logger().warning("未及时获取导航参考方向，跳过预旋转")
            robot_controller.motion.clear_manual_override()
            return

        try:
            _, _, current_yaw = self.lookup_current_pose()
        except RuntimeError as exc:
            self.get_logger().warning(f"计算预旋转偏差前刷新当前航向失败，使用上次航向: {exc}")

        yaw_diff = normalize_angle(path_heading - current_yaw)
        if abs(yaw_diff) < PATH_ALIGN_YAW_THRESHOLD:
            self.get_logger().info("当前朝向已基本与起始路径平行，跳过预旋转")
            robot_controller.motion.clear_manual_override()
            return

        self.get_logger().info(
            f"起步前先对齐导航方向({heading_source})，当前偏差 {math.degrees(yaw_diff):.1f}°"
        )
        robot_controller.rotate_to_yaw(path_heading)

    def send_navigation_goal(self, goal_pose):
        if not self.navigate_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("navigate_to_pose action server 不可用")
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = str(
            package_share_dir() / "behavior_trees" / DEFAULT_NAV_TO_POSE_BT
        )
        self.get_logger().info(f"发送导航目标时指定 BT: {goal.behavior_tree}")
        send_future = self.navigate_client.send_goal_async(goal)
        goal_handle = self.wait_for_future(send_future, 10.0)
        if not goal_handle.accepted:
            diagnostic = self.diagnose_goal_rejection(goal.behavior_tree)
            self.get_logger().error(diagnostic)
            raise RuntimeError(f"导航目标被拒绝 (BT: {goal.behavior_tree})\n{diagnostic}")
        result_future = goal_handle.get_result_async()
        return goal_handle, result_future

    def cancel_goal(self, goal_handle):
        try:
            self.wait_for_future(goal_handle.cancel_goal_async(), 5.0)
        except Exception:
            pass

    def restart_navigation_goal(
        self, robot_controller, goal_handle, goal_pose, hold_source="forward_replan_hold"
    ):
        robot_controller.motion.hold_position(hold_source)
        self.cancel_goal(goal_handle)

        try:
            self.clear_costmaps()
        except Exception as exc:
            self.get_logger().warning(f"前向重规划前清除代价地图失败: {exc}")

        try:
            current_x, current_y, current_yaw = self.lookup_current_pose()
            start_pose = self.build_pose_stamped(current_x, current_y, current_yaw)
            path = self.compute_path(start_pose, goal_pose)
            if path is not None and path.poses:
                robot_controller.update_global_plan(path)
            else:
                robot_controller.clear_global_plan()
        except Exception as exc:
            self.get_logger().warning(f"前向重规划预计算路径失败: {exc}")
            robot_controller.clear_global_plan()

        robot_controller.motion.begin_navigation_goal(release_stop_latch=False)
        robot_controller.motion.clear_manual_override()
        return self.send_navigation_goal(goal_pose)

    @staticmethod
    def _should_restart_after_close_obstacle_clear(
        motion_policy,
        distance_to_goal,
        stop_distance,
        now,
        last_restart_at,
    ):
        if not motion_policy.close_obstacle_replan_on_clear:
            return False
        goal_margin = max(0.0, motion_policy.close_obstacle_replan_goal_margin)
        if distance_to_goal <= stop_distance + goal_margin:
            return False
        cooldown = max(0.0, motion_policy.close_obstacle_replan_cooldown)
        if now - last_restart_at < cooldown:
            return False
        return True

    @staticmethod
    def _should_trigger_stall_replan(
        motion_snapshot,
        distance_to_goal,
        stop_distance,
        elapsed,
        stalled_for,
    ):
        if elapsed < STALL_REPLAN_TIMEOUT:
            return False, ""

        min_distance = max(stop_distance + 0.2, STALL_REPLAN_MIN_GOAL_DISTANCE)
        if distance_to_goal <= min_distance:
            return False, ""

        if stalled_for < STALL_REPLAN_TIMEOUT:
            return False, ""

        if motion_snapshot["manual_active"] or motion_snapshot["rotate_recovery_active"]:
            return False, ""

        executed_vx, _, executed_wz = motion_snapshot["executed_cmd"]
        if (
            abs(executed_vx) > STALL_EXECUTED_VX_EPS
            or abs(executed_wz) > STALL_EXECUTED_WZ_EPS
        ):
            return False, ""

        planner_age = motion_snapshot["planner_age"]
        if planner_age > 1.5:
            reason = f"planner_silent={planner_age:.1f}s"
        else:
            reason = f"no_progress={stalled_for:.1f}s"
        return True, reason

    @staticmethod
    def _costmap_metadata_value(metadata, primary, fallback=None, default=0):
        if metadata is None:
            return default
        if hasattr(metadata, primary):
            return getattr(metadata, primary)
        if fallback and hasattr(metadata, fallback):
            return getattr(metadata, fallback)
        return default

    @staticmethod
    def _costmap_origin_yaw(origin):
        orientation = getattr(origin, "orientation", None)
        if orientation is None:
            return 0.0
        x = float(getattr(orientation, "x", 0.0) or 0.0)
        y = float(getattr(orientation, "y", 0.0) or 0.0)
        z = float(getattr(orientation, "z", 0.0) or 0.0)
        w = float(getattr(orientation, "w", 1.0) or 1.0)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _goal_costmap_occupied(
        costmap,
        *,
        goal_x,
        goal_y,
        radius,
        cost_threshold,
        min_occupied_cells,
    ):
        metrics = {
            "available": False,
            "occupied_cells": 0,
            "sampled_cells": 0,
            "max_cost": None,
        }
        metadata = getattr(costmap, "metadata", None) or getattr(costmap, "info", None)
        width = int(
            MissionNode._costmap_metadata_value(metadata, "size_x", "width", 0) or 0
        )
        height = int(
            MissionNode._costmap_metadata_value(metadata, "size_y", "height", 0) or 0
        )
        resolution = float(getattr(metadata, "resolution", 0.0) or 0.0)
        data = getattr(costmap, "data", None)
        if width <= 0 or height <= 0 or resolution <= 0.0 or data is None:
            return False, metrics
        expected = width * height
        if len(data) < expected:
            return False, metrics

        origin = getattr(metadata, "origin", None)
        position = getattr(origin, "position", None)
        origin_x = float(getattr(position, "x", 0.0) or 0.0)
        origin_y = float(getattr(position, "y", 0.0) or 0.0)
        origin_yaw = MissionNode._costmap_origin_yaw(origin)
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        dx = float(goal_x) - origin_x
        dy = float(goal_y) - origin_y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        center_x = int(math.floor(local_x / resolution))
        center_y = int(math.floor(local_y / resolution))
        if center_x < 0 or center_x >= width or center_y < 0 or center_y >= height:
            return False, metrics

        metrics["available"] = True
        radius_cells = max(0, int(math.ceil(float(radius) / resolution)))
        threshold = max(0, int(cost_threshold))
        min_cells = max(1, int(min_occupied_cells))
        occupied_cells = 0
        sampled_cells = 0
        max_cost = None

        for cell_y in range(
            max(0, center_y - radius_cells),
            min(height, center_y + radius_cells + 1),
        ):
            for cell_x in range(
                max(0, center_x - radius_cells),
                min(width, center_x + radius_cells + 1),
            ):
                cell_dx = (cell_x - center_x) * resolution
                cell_dy = (cell_y - center_y) * resolution
                if math.hypot(cell_dx, cell_dy) > radius:
                    continue
                sampled_cells += 1
                value = int(data[cell_y * width + cell_x])
                if value < 0 or value == 255:
                    continue
                max_cost = value if max_cost is None else max(max_cost, value)
                if value >= threshold:
                    occupied_cells += 1

        metrics["occupied_cells"] = occupied_cells
        metrics["sampled_cells"] = sampled_cells
        metrics["max_cost"] = max_cost
        return occupied_cells >= min_cells, metrics

    def _goal_occupied_from_costmap(self, waypoint, now, policy):
        with self.goal_costmap_lock:
            costmap = self.goal_costmap
            stamp = self.goal_costmap_stamp
        metrics = {
            "available": False,
            "occupied_cells": 0,
            "sampled_cells": 0,
            "max_cost": None,
            "age": None,
        }
        if costmap is None or stamp <= 0.0:
            return False, metrics
        age = now - stamp
        metrics["age"] = age
        if age > policy.goal_occupancy_costmap_timeout:
            return False, metrics
        occupied, sampled_metrics = self._goal_costmap_occupied(
            costmap,
            goal_x=waypoint["x"],
            goal_y=waypoint["y"],
            radius=policy.goal_occupancy_radius,
            cost_threshold=policy.goal_occupancy_cost_threshold,
            min_occupied_cells=policy.goal_occupancy_min_occupied_cells,
        )
        sampled_metrics["age"] = age
        return occupied, sampled_metrics

    def _publish_yield_prompt_event(
        self,
        *,
        event_type,
        text="请您让一让",
        state="blocked",
        distance_to_goal=None,
        attempt=None,
        metrics=None,
    ):
        try:
            payload = {
                "type": str(event_type),
                "state": str(state),
                "text": str(text),
                "timestamp": time.time(),
                "topic": YIELD_PROMPT_EVENT_TOPIC,
            }
            if distance_to_goal is not None:
                payload["distance_to_goal"] = round(float(distance_to_goal), 3)
            if attempt is not None:
                payload["attempt"] = int(attempt)
            if isinstance(metrics, dict):
                if "occupied_cells" in metrics:
                    payload["occupied_cells"] = int(metrics.get("occupied_cells") or 0)
                if metrics.get("max_cost") is not None:
                    payload["max_cost"] = int(metrics["max_cost"])
            self.yield_prompt_event_pub.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:
            self.get_logger().warning(f"让行提示事件发布失败: {exc}")

    @staticmethod
    def _update_goal_occupancy_wait(
        state,
        policy,
        *,
        now,
        distance_to_goal,
        goal_occupied,
    ):
        if (
            not policy.goal_occupancy_enabled
            or distance_to_goal > policy.goal_occupancy_start_distance
        ):
            if state.active:
                return GoalOccupancyWaitState(), "cleared"
            return GoalOccupancyWaitState(), "inactive"

        if not state.active:
            if goal_occupied:
                return (
                    GoalOccupancyWaitState(active=True, started_at=now),
                    "entered",
                )
            return state, "inactive"

        if not goal_occupied:
            if state.clear_since <= 0.0:
                return (
                    GoalOccupancyWaitState(
                        active=True,
                        started_at=state.started_at,
                        clear_since=now,
                        retry_count=state.retry_count,
                        retrying_until=state.retrying_until,
                        last_retry_at=state.last_retry_at,
                    ),
                    "clearing",
                )
            if now - state.clear_since >= policy.goal_occupancy_clear_duration:
                return GoalOccupancyWaitState(), "cleared"
            return state, "clearing"

        if now - state.started_at >= policy.goal_occupancy_wait_timeout:
            return state, "timeout"

        if state.retrying_until > now:
            if state.clear_since > 0.0:
                return (
                    GoalOccupancyWaitState(
                        active=True,
                        started_at=state.started_at,
                        retry_count=state.retry_count,
                        retrying_until=state.retrying_until,
                        last_retry_at=state.last_retry_at,
                    ),
                    "retrying",
                )
            return state, "retrying"

        if (
            state.retrying_until > 0.0
            and state.retry_count >= policy.goal_occupancy_max_retries
        ):
            return state, "timeout"

        wait_started_at = (
            state.retrying_until if state.retry_count > 0 else state.started_at
        )
        if (
            state.retry_count < policy.goal_occupancy_max_retries
            and now - wait_started_at >= policy.goal_occupancy_retry_hold_duration
        ):
            return (
                GoalOccupancyWaitState(
                    active=True,
                    started_at=state.started_at,
                    retry_count=state.retry_count + 1,
                    retrying_until=now + policy.goal_occupancy_retry_window,
                    last_retry_at=now,
                ),
                "retry",
            )

        if state.clear_since > 0.0:
            return (
                GoalOccupancyWaitState(
                    active=True,
                    started_at=state.started_at,
                    retry_count=state.retry_count,
                    retrying_until=state.retrying_until,
                    last_retry_at=state.last_retry_at,
                ),
                "waiting",
            )
        return state, "waiting"

    @staticmethod
    def _navigation_health_abort_reason(motion_snapshot, sdk_snapshot, *, elapsed=0.0):
        planner_age = float(motion_snapshot.get("planner_age", 0.0))
        if (
            elapsed > INITIAL_PLANNER_CMD_GRACE
            and planner_age > PLANNER_SILENT_ABORT_TIMEOUT
        ):
            return "planner_silent"

        if elapsed <= INITIAL_PLANNER_CMD_GRACE:
            return ""

        active_sample_count = int(
            sdk_snapshot.get("active_loco_latency_sample_count", 0) or 0
        )
        active_p95_ms = float(
            sdk_snapshot.get("active_loco_latency_p95_ms", 0.0) or 0.0
        )
        active_age = sdk_snapshot.get("active_loco_latency_age_sec")
        active_recent = True
        if active_age is not None:
            active_recent = float(active_age) <= SDK_SLOW_ABORT_ACTIVE_SAMPLE_MAX_AGE

        if (
            active_recent
            and active_sample_count >= SDK_SLOW_ABORT_MIN_ACTIVE_SAMPLES
            and active_p95_ms > SDK_SLOW_ABORT_P95_MS
        ):
            return "sdk_slow"

        return ""

    def _finalize_navigation_success(
        self,
        robot_controller,
        waypoint,
        reason,
        goal_handle,
        perform_interaction=True,
        success_message=None,
    ):
        self.get_logger().info(f"{reason}：触发停止流程")
        robot_controller.motion.end_navigation_goal()
        robot_controller.clear_current_navigation_goal()
        self.cancel_goal(goal_handle)
        force_robot_stop(robot_controller, hold=True)

        try:
            _, _, current_yaw = self.lookup_current_pose()
            yaw_diff = normalize_angle(waypoint["yaw"] - current_yaw)
            if waypoint.get("align_final_yaw", True) and abs(yaw_diff) > 0.15:
                robot_controller.rotate_to_yaw(waypoint["yaw"])
        except RuntimeError:
            pass

        if not perform_interaction:
            robot_controller.motion.clear_manual_override()
            return success_message or "已到达目标点"

        robot_controller.motion.hold_position("interaction_hold")
        robot_controller.perform_interaction(
            waypoint.get("say_text", "你好"), waypoint.get("action_id", 25)
        )
        robot_controller.motion.clear_manual_override()
        self.get_logger().info("讲解与动作执行完毕，准备前往下一站")
        if success_message:
            return f"{success_message}，讲解与动作执行完毕"
        return "已到达目标点，讲解与动作执行完毕"

    def _finalize_navigation_failure(
        self,
        robot_controller,
        goal_handle,
        message,
        *,
        speak_text=None,
        canceled=False,
        failure_reason=None,
    ):
        if failure_reason:
            robot_controller.motion.set_navigation_failure_reason(failure_reason)
        robot_controller.motion.end_navigation_goal()
        robot_controller.clear_current_navigation_goal()
        if goal_handle is not None:
            self.cancel_goal(goal_handle)
        force_robot_stop(robot_controller, hold=True)
        if speak_text:
            robot_controller.speak(speak_text)
            time.sleep(3.0)
        robot_controller.motion.clear_manual_override()
        return {
            "status": "canceled" if canceled else "error",
            "message": message,
        }

    def handle_arrival(self, robot_controller, waypoint, reason, goal_handle):
        self._finalize_navigation_success(
            robot_controller,
            waypoint,
            reason,
            goal_handle,
            perform_interaction=True,
        )

    def navigate_to_waypoint(
        self,
        waypoint,
        robot_controller,
        *,
        index=None,
        perform_interaction=True,
        announce_failures=True,
        feedback_cb=None,
        cancel_requested=None,
    ):
        waypoint_name = waypoint.get("name") or f"目标{index or ''}"
        self.get_logger().info("获取机器人在地图中的真实初始位置...")
        current_x, current_y, current_yaw = self.lookup_current_pose()
        self.get_logger().info(f"初始全局位置: ({current_x:.2f}, {current_y:.2f})")

        start_dist = math.hypot(waypoint["x"] - current_x, waypoint["y"] - current_y)
        stop_distance = 0.3 if start_dist < 1.0 else 0.4
        goal_pose = self.build_pose_stamped(waypoint["x"], waypoint["y"], waypoint["yaw"])
        start_pose = self.build_pose_stamped(current_x, current_y, current_yaw)

        if feedback_cb is not None:
            feedback_cb("planning", float(start_dist))

        target_label = f"第{index}个目标" if index is not None else waypoint_name
        self.get_logger().info(
            f"发送{target_label}: ({waypoint['x']}, {waypoint['y']}, "
            f"yaw: {math.degrees(waypoint['yaw']):.1f}°)"
        )

        path = self.compute_path(start_pose, goal_pose)
        if path is not None and path.poses:
            robot_controller.update_global_plan(path)
        else:
            robot_controller.clear_global_plan()

        slowdown_blend = quantize_profile_blend(profile_blend_for_distance(start_dist))
        self.apply_navigation_profile(
            slowdown_blend, reason=f"start_dist={start_dist:.2f}m"
        )
        robot_controller.motion.begin_navigation_goal()
        robot_controller.set_current_navigation_goal(waypoint["x"], waypoint["y"])
        robot_controller.motion.clear_manual_override()

        if feedback_cb is not None:
            feedback_cb("aligning", float(start_dist))

        self.align_with_initial_path(robot_controller, current_x, current_y, current_yaw)

        goal_handle, result_future = self.send_navigation_goal(goal_pose)
        start_time = time.time()
        last_spoke_time = 0.0
        speak_interval = 10.0
        map_cleared = False
        distance_to_goal = start_dist
        forced_replan_attempts = 0
        last_progress_distance = start_dist
        last_progress_time = start_time
        goal_occupancy_state = GoalOccupancyWaitState()
        last_goal_occupancy_spoke_time = 0.0
        last_close_obstacle_replan_time = -float("inf")
        motion_policy = robot_controller.motion.motion_policy

        while rclpy.ok():
            if cancel_requested is not None and cancel_requested():
                return self._finalize_navigation_failure(
                    robot_controller,
                    goal_handle,
                    f"导航到 {waypoint_name} 已取消",
                    canceled=True,
                )

            self.spin_for(0.05)
            try:
                cur_x, cur_y, current_yaw = self.lookup_current_pose()
                distance_to_goal = math.hypot(
                    waypoint["x"] - cur_x, waypoint["y"] - cur_y
                )
            except RuntimeError:
                cur_x, cur_y = current_x, current_y

            if feedback_cb is not None:
                feedback_cb("navigating", float(distance_to_goal))

            now = time.time()
            if last_progress_distance - distance_to_goal >= STALL_REPLAN_DISTANCE_EPS:
                last_progress_distance = distance_to_goal
                last_progress_time = now

            elapsed = time.time() - start_time
            next_blend = quantize_profile_blend(
                profile_blend_for_distance(distance_to_goal)
            )
            if next_blend > slowdown_blend:
                slowdown_blend = next_blend
                self.apply_navigation_profile(
                    slowdown_blend, reason=f"dist={distance_to_goal:.2f}m"
                )

            motion_snapshot = robot_controller.motion.get_command_snapshot()
            goal_occupied = False
            goal_occupancy_metrics = {}
            if distance_to_goal <= motion_policy.goal_occupancy_start_distance:
                goal_occupied, goal_occupancy_metrics = self._goal_occupied_from_costmap(
                    waypoint,
                    now,
                    motion_policy,
                )
            goal_occupancy_state, goal_occupancy_action = (
                self._update_goal_occupancy_wait(
                    goal_occupancy_state,
                    motion_policy,
                    now=now,
                    distance_to_goal=distance_to_goal,
                    goal_occupied=goal_occupied,
                )
            )
            if goal_occupancy_action == "entered":
                robot_controller.motion.hold_position("goal_occupied_wait")
                self._publish_yield_prompt_event(
                    event_type="goal_occupied",
                    distance_to_goal=distance_to_goal,
                    attempt=goal_occupancy_state.retry_count,
                    metrics=goal_occupancy_metrics,
                )
                last_goal_occupancy_spoke_time = now
                self.get_logger().warning(
                    "目标点附近被动态障碍占用，进入等待 "
                    f"(dist={distance_to_goal:.2f}m, "
                    f"cells={goal_occupancy_metrics.get('occupied_cells', 0)}, "
                    f"max_cost={goal_occupancy_metrics.get('max_cost')}, "
                    f"timeout={motion_policy.goal_occupancy_wait_timeout:.1f}s)"
                )
            elif goal_occupancy_action == "cleared":
                robot_controller.motion.clear_manual_override()
                last_progress_distance = distance_to_goal
                last_progress_time = now
                self.get_logger().info("目标点附近障碍已清空，恢复导航")

            if goal_occupancy_action == "retry":
                if feedback_cb is not None:
                    feedback_cb("goal_occupied_retry", float(distance_to_goal))
                self.get_logger().warning(
                    "终点占用重试导航 "
                    f"(attempt={goal_occupancy_state.retry_count}/"
                    f"{motion_policy.goal_occupancy_max_retries}, "
                    f"dist={distance_to_goal:.2f}m, "
                    f"release_window={motion_policy.goal_occupancy_retry_window:.1f}s)"
                )
                try:
                    goal_handle, result_future = self.restart_navigation_goal(
                        robot_controller,
                        goal_handle,
                        goal_pose,
                        hold_source="goal_occupied_retry_hold",
                    )
                    start_time = time.time()
                    last_progress_distance = distance_to_goal
                    last_progress_time = start_time
                    map_cleared = True
                    continue
                except Exception as exc:
                    self.get_logger().error(f"终点占用重试导航失败: {exc}")
                    return self._finalize_navigation_failure(
                        robot_controller,
                        goal_handle,
                        f"终点占用重试导航失败: {exc}",
                    )

            if goal_occupancy_action in ("entered", "waiting", "clearing"):
                if feedback_cb is not None:
                    feedback_cb("goal_occupied_wait", float(distance_to_goal))
                if goal_occupancy_action == "waiting":
                    robot_controller.motion.hold_position("goal_occupied_wait")
                if (
                    goal_occupancy_action == "waiting"
                    and now - last_goal_occupancy_spoke_time
                    >= motion_policy.goal_occupancy_speak_interval
                ):
                    self._publish_yield_prompt_event(
                        event_type="goal_occupied",
                        distance_to_goal=distance_to_goal,
                        attempt=goal_occupancy_state.retry_count,
                        metrics=goal_occupancy_metrics,
                    )
                    last_goal_occupancy_spoke_time = now
                continue

            if goal_occupancy_action == "timeout":
                if feedback_cb is not None:
                    feedback_cb("arrived", 0.0)
                message = self._finalize_navigation_success(
                    robot_controller,
                    waypoint,
                    (
                        "目标点被占用，等待超时后在目标附近完成 "
                        f"(Dist: {distance_to_goal:.2f}m)"
                    ),
                    goal_handle,
                    perform_interaction=perform_interaction,
                    success_message="目标点被占用，多次重试后在目标附近完成",
                )
                return {"status": "success", "message": message}

            if (
                elapsed > 0.8
                and distance_to_goal < stop_distance
                and not motion_snapshot.get("close_obstacle_active", False)
            ):
                if feedback_cb is not None:
                    feedback_cb("arrived", 0.0)
                message = self._finalize_navigation_success(
                    robot_controller,
                    waypoint,
                    f"物理距离达标 (Dist: {distance_to_goal:.2f}m)",
                    goal_handle,
                    perform_interaction=perform_interaction,
                )
                return {"status": "success", "message": message}

            if robot_controller.motion.consume_close_obstacle_clear_event():
                if self._should_restart_after_close_obstacle_clear(
                    motion_policy,
                    distance_to_goal,
                    stop_distance,
                    now,
                    last_close_obstacle_replan_time,
                ):
                    if feedback_cb is not None:
                        feedback_cb("close_obstacle_replan", float(distance_to_goal))
                    self.get_logger().info(
                        "近距离让行清除后重发导航目标 "
                        f"(dist={distance_to_goal:.2f}m, "
                        f"cooldown={motion_policy.close_obstacle_replan_cooldown:.1f}s)"
                    )
                    try:
                        goal_handle, result_future = self.restart_navigation_goal(
                            robot_controller,
                            goal_handle,
                            goal_pose,
                            hold_source="close_obstacle_clear_replan_hold",
                        )
                        start_time = time.time()
                        last_close_obstacle_replan_time = start_time
                        last_progress_distance = distance_to_goal
                        last_progress_time = start_time
                        map_cleared = True
                        continue
                    except Exception as exc:
                        self.get_logger().error(f"近距离让行后重发导航目标失败: {exc}")
                        return self._finalize_navigation_failure(
                            robot_controller,
                            goal_handle,
                            f"近距离让行后重发导航目标失败: {exc}",
                        )
                else:
                    self.get_logger().info(
                        "近距离让行已清除，保持当前导航目标 "
                        f"(dist={distance_to_goal:.2f}m)"
                    )

            if robot_controller.motion.consume_yield_hold_event():
                self.get_logger().info("让行静止触发，主动清除代价地图")
                try:
                    self.clear_costmaps()
                    map_cleared = True
                except Exception as exc:
                    self.get_logger().error(f"让行静止时清除代价地图失败: {exc}")

            if robot_controller.motion.consume_reverse_replan_event():
                if forced_replan_attempts >= FORCED_REPLAN_MAX_ATTEMPTS:
                    failure_reason = "goal_or_path_blocked"
                    self.get_logger().warning(
                        "连续后退触发前向重规划已达上限，停止本轮导航 "
                        f"(reason={failure_reason})"
                    )
                    return self._finalize_navigation_failure(
                        robot_controller,
                        goal_handle,
                        f"导航到 {waypoint_name} 失败，原因: {failure_reason}",
                        speak_text=(
                            "当前路线被阻挡，无法重新规划，即将前往下一位置"
                            if announce_failures
                            else None
                        ),
                        failure_reason=failure_reason,
                    )
                else:
                    forced_replan_attempts += 1
                    self.get_logger().warning(
                        "检测到 Nav2 请求后退，改为前向重规划 "
                        f"(attempt={forced_replan_attempts}/{FORCED_REPLAN_MAX_ATTEMPTS})"
                    )
                    try:
                        goal_handle, result_future = self.restart_navigation_goal(
                            robot_controller,
                            goal_handle,
                            goal_pose,
                            hold_source="reverse_replan_hold",
                        )
                        start_time = time.time()
                        last_progress_distance = distance_to_goal
                        last_progress_time = start_time
                        map_cleared = True
                        continue
                    except Exception as exc:
                        self.get_logger().error(f"前向重规划失败: {exc}")
                        return self._finalize_navigation_failure(
                            robot_controller,
                            goal_handle,
                            f"前向重规划失败: {exc}",
                            speak_text=(
                                "当前路线被阻挡，无法重新规划，即将前往下一位置"
                                if announce_failures
                                else None
                            ),
                        )

            sdk_snapshot = {}
            try:
                sdk_snapshot = robot_controller.loco.GetHealthSnapshot()
            except Exception:
                pass
            health_abort_reason = self._navigation_health_abort_reason(
                motion_snapshot,
                sdk_snapshot,
                elapsed=elapsed,
            )
            if health_abort_reason:
                self.get_logger().warning(
                    "导航系统健康检查失败，停止本轮导航 "
                    f"(reason={health_abort_reason}, "
                    f"planner_age={motion_snapshot.get('planner_age', 0.0):.1f}s, "
                    f"sdk_p95={sdk_snapshot.get('loco_latency_p95_ms', 0.0)}ms)"
                )
                return self._finalize_navigation_failure(
                    robot_controller,
                    goal_handle,
                    f"导航到 {waypoint_name} 失败，原因: {health_abort_reason}",
                    speak_text=(
                        "导航系统暂时不可用，即将停止"
                        if announce_failures
                        else None
                    ),
                    failure_reason=health_abort_reason,
                )

            stalled_for = now - last_progress_time
            should_replan, stall_reason = self._should_trigger_stall_replan(
                motion_snapshot,
                distance_to_goal,
                stop_distance,
                elapsed,
                stalled_for,
            )
            if should_replan:
                if forced_replan_attempts >= FORCED_REPLAN_MAX_ATTEMPTS:
                    failure_reason = (
                        "planner_silent"
                        if stall_reason.startswith("planner_silent")
                        else "goal_or_path_blocked"
                    )
                    self.get_logger().warning(
                        "无进展触发前向重规划已达上限，停止本轮导航 "
                        f"(reason={failure_reason}, detail={stall_reason})"
                    )
                    return self._finalize_navigation_failure(
                        robot_controller,
                        goal_handle,
                        f"导航到 {waypoint_name} 失败，原因: {failure_reason} ({stall_reason})",
                        speak_text=(
                            "当前路线被阻挡，无法重新规划，即将前往下一位置"
                            if announce_failures
                            else None
                        ),
                        failure_reason=failure_reason,
                    )
                else:
                    forced_replan_attempts += 1
                    output_source = motion_snapshot["output_source"]
                    self.get_logger().warning(
                        "检测到导航长时间无进展，改为前向重规划 "
                        f"(reason={stall_reason}, source={output_source}, "
                        f"attempt={forced_replan_attempts}/{FORCED_REPLAN_MAX_ATTEMPTS})"
                    )
                    try:
                        goal_handle, result_future = self.restart_navigation_goal(
                            robot_controller,
                            goal_handle,
                            goal_pose,
                            hold_source="stall_replan_hold",
                        )
                        start_time = time.time()
                        last_progress_distance = distance_to_goal
                        last_progress_time = start_time
                        map_cleared = True
                        continue
                    except Exception as exc:
                        self.get_logger().error(f"无进展前向重规划失败: {exc}")
                        return self._finalize_navigation_failure(
                            robot_controller,
                            goal_handle,
                            f"无进展前向重规划失败: {exc}",
                            speak_text=(
                                "当前路线被阻挡，无法重新规划，即将前往下一位置"
                                if announce_failures
                                else None
                            ),
                        )

            if (
                elapsed > INITIAL_PLANNER_CMD_GRACE
                and robot_controller.motion.time_since_planner_cmd() > 1.5
            ):
                if announce_failures and now - last_spoke_time > speak_interval:
                    self._publish_yield_prompt_event(
                        event_type="planner_blocked",
                        distance_to_goal=distance_to_goal,
                    )
                    last_spoke_time = now
                if not map_cleared:
                    self.get_logger().info("尝试清除代价地图...")
                    try:
                        self.clear_costmaps()
                        map_cleared = True
                    except Exception as exc:
                        self.get_logger().error(f"清除代价地图失败: {exc}")
            else:
                map_cleared = False

            if result_future.done():
                result = result_future.result()
                status = result.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    if feedback_cb is not None:
                        feedback_cb("arrived", 0.0)
                    message = self._finalize_navigation_success(
                        robot_controller,
                        waypoint,
                        "Nav2 判定到达",
                        goal_handle,
                        perform_interaction=perform_interaction,
                    )
                    return {"status": "success", "message": message}
                if status == GoalStatus.STATUS_CANCELED:
                    return self._finalize_navigation_failure(
                        robot_controller,
                        goal_handle,
                        f"导航到 {waypoint_name} 已取消",
                        canceled=True,
                        failure_reason="canceled",
                    )
                failure_reason = (
                    "goal_or_path_blocked"
                    if status == GoalStatus.STATUS_ABORTED
                    else f"nav2_status_{status}"
                )
                return self._finalize_navigation_failure(
                    robot_controller,
                    goal_handle,
                    f"导航到 {waypoint_name} 失败，原因: {failure_reason}，状态码: {status}",
                    speak_text=(
                        "无法到达该目标位置，即将前往下一位置"
                        if announce_failures
                        else None
                    ),
                    failure_reason=failure_reason,
                )

        return self._finalize_navigation_failure(
            robot_controller,
            goal_handle,
            f"导航到 {waypoint_name} 被中断",
            canceled=True,
        )

    def navigate_to_waypoints(self, waypoints, robot_controller):
        for index, waypoint in enumerate(waypoints, start=1):
            result = self.navigate_to_waypoint(
                waypoint,
                robot_controller,
                index=index,
                perform_interaction=True,
                announce_failures=True,
            )
            if result.get("status") == "canceled":
                self.get_logger().warning(f"任务在第{index}个目标被取消: {result['message']}")
                break
            if result.get("status") != "success":
                self.get_logger().warning(f"第{index}个目标执行失败: {result['message']}")

    def run_mission(self, waypoints):
        self.ensure_navigation_stack_ready_with_manager()
        robot_controller = RobotController(self, self.args.net_if)
        try:
            time.sleep(1.0)
            robot_controller.speak("启动成功")
            self.navigate_to_waypoints(waypoints, robot_controller)
        finally:
            robot_controller.shutdown()


def force_robot_stop(robot_controller_instance, hold=False):
    robot_controller_instance.node.get_logger().warning("强制停止...")
    if hold:
        robot_controller_instance.motion.hold_position("force_stop_hold")
    else:
        robot_controller_instance.motion.set_manual_velocity(
            0.0, 0.0, 0.0, timeout=0.3, source="force_stop"
        )
    robot_controller_instance.node.get_logger().info("停止指令已发送")
    time.sleep(0.1)


def parse_args(argv=None, default_route_path=None):
    parser = argparse.ArgumentParser(description="G1 ROS2 waypoint mission runner")
    parser.add_argument("--net-if", default="enP8p1s0")
    parser.add_argument("--route", default=default_route_path or route_file("default.yaml"))
    parser.add_argument("--node-name", default="multi_waypoint_nav")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    return parser.parse_known_args(argv)


def run_mission_from_yaml(default_route_path, argv=None):
    args, ros_args = parse_args(argv=argv, default_route_path=default_route_path)
    resolved_path, route_name, waypoints = load_waypoints_from_yaml(args.route)
    print(f"已加载路线: {route_name} ({resolved_path})", flush=True)

    rclpy.init(args=ros_args)
    node = MissionNode(args)
    try:
        node.run_mission(waypoints)
    finally:
        node.shutdown_resources()
        node.destroy_node()
        rclpy.shutdown()
