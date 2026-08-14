import json
from pathlib import Path
import sys
import threading
import types


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _ensure_module(name):
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    if "." in name:
        parent_name, child_name = name.rsplit(".", 1)
        parent = _ensure_module(parent_name)
        setattr(parent, child_name, module)
    return module


class _String:
    def __init__(self, data=""):
        self.data = data


class _Parameter:
    class Type:
        DOUBLE = "double"
        INTEGER = "integer"
        DOUBLE_ARRAY = "double_array"

    def __init__(self, name, parameter_type, value):
        self.name = name
        self.parameter_type = parameter_type
        self.value = value


def _install_ros_stubs():
    rclpy = _ensure_module("rclpy")
    rclpy.ok = lambda: True
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None

    _ensure_module("action_msgs.msg").GoalStatus = types.SimpleNamespace(
        STATUS_SUCCEEDED=4,
        STATUS_CANCELED=5,
        STATUS_ABORTED=6,
    )
    _ensure_module("geometry_msgs.msg").PoseStamped = object
    _ensure_module("geometry_msgs.msg").Twist = object
    _ensure_module("lifecycle_msgs.msg").State = object
    _ensure_module("lifecycle_msgs.srv").GetState = object
    _ensure_module("nav2_msgs.action").ComputePathToPose = object
    _ensure_module("nav2_msgs.action").NavigateToPose = object
    _ensure_module("nav2_msgs.srv").ClearEntireCostmap = object
    _ensure_module("nav_msgs.msg").Path = object
    _ensure_module("rcl_interfaces.msg").ParameterType = object
    _ensure_module("rcl_interfaces.srv").GetParameters = object
    _ensure_module("rcl_interfaces.srv").SetParameters = object
    _ensure_module("sensor_msgs.msg").LaserScan = object
    _ensure_module("std_msgs.msg").Bool = object
    _ensure_module("std_msgs.msg").String = _String
    _ensure_module("std_srvs.srv").Trigger = object

    action = _ensure_module("rclpy.action")
    action.ActionClient = object
    action.ActionServer = object
    action.CancelResponse = types.SimpleNamespace(ACCEPT=1)
    action.GoalResponse = types.SimpleNamespace(ACCEPT=1)
    _ensure_module("rclpy.callback_groups").ReentrantCallbackGroup = object
    _ensure_module("rclpy.duration").Duration = object
    executors = _ensure_module("rclpy.executors")
    executors.MultiThreadedExecutor = object
    _ensure_module("rclpy.node").Node = object
    _ensure_module("rclpy.parameter").Parameter = _Parameter
    qos = _ensure_module("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = types.SimpleNamespace(
        BEST_EFFORT="best_effort",
        RELIABLE="reliable",
    )

    class _QoSProfile:
        def __init__(self, *args, **kwargs):
            self.args = args
            for key, value in kwargs.items():
                setattr(self, key, value)

    qos.QoSProfile = _QoSProfile

    tf2_ros = _ensure_module("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformException = Exception
    tf2_ros.TransformListener = object

    _ensure_module("g1_base_interfaces.action").NavigateToTarget = object
    srv = _ensure_module("g1_base_interfaces.srv")
    for name in (
        "ExecuteArmAction",
        "ExecuteCustomAction",
        "GetFsmId",
        "MoveRobot",
        "PlayNamedAction",
        "RotateRobot",
        "RunMovementScript",
        "SetFsmId",
        "SquatRobot",
        "StopRobot",
    ):
        setattr(srv, name, object)


_install_ros_stubs()

from g1_base import g1_control_server


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Motion:
    last_sent_source = "planner"
    navigation_failure_reason = ""
    motion_policy = types.SimpleNamespace(
        close_obstacle_trigger_distance=0.60,
        close_obstacle_release_distance=0.75,
    )

    def is_stop_latched(self):
        return False

    def get_command_snapshot(self):
        return {
            "close_obstacle_active": True,
            "close_obstacle_front_min": 0.52,
        }


class _Loco:
    def GetHealthSnapshot(self):
        return {
            "last_loco_latency_ms": 3.0,
            "loco_latency_p95_ms": 8.0,
            "loco_slow_count": 1,
        }


def test_status_includes_close_obstacle_diagnostics():
    server = object.__new__(g1_control_server.G1ControlServer)
    server._status_lock = threading.Lock()
    server._current_activity = "navigating"
    server._current_activity_detail = "unit-test"
    server._activity_start_time = 0.0
    server._status_pub = _Publisher()
    server.robot_controller = types.SimpleNamespace(
        is_squatting=False,
        motion=_Motion(),
        loco=_Loco(),
    )

    server._publish_status()

    payload = json.loads(server._status_pub.messages[-1].data)
    assert payload["close_obstacle_active"] is True
    assert payload["close_obstacle_front_min"] == 0.52
    assert payload["close_obstacle_trigger_distance"] == 0.60
    assert payload["close_obstacle_release_distance"] == 0.75
