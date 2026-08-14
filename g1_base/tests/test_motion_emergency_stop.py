from pathlib import Path
import json
import math
import sys
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


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Vector:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vector()
        self.angular = _Vector()


class _String:
    def __init__(self, data=""):
        self.data = data


class _LaserScan:
    def __init__(
        self,
        ranges,
        *,
        angle_min=-0.5,
        angle_increment=0.5,
        range_min=0.05,
        range_max=5.0,
    ):
        self.ranges = list(ranges)
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max


class _Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Orientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _Pose:
    def __init__(self):
        self.position = _Position()
        self.orientation = _Orientation()


class _CostmapMetaData:
    def __init__(self, *, width, height, resolution, origin=None):
        self.size_x = width
        self.size_y = height
        self.resolution = resolution
        self.origin = origin or _Pose()


class _Costmap:
    def __init__(self, *, width, height, resolution, origin=None, data=None):
        self.metadata = _CostmapMetaData(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
        )
        self.data = list(data or [0] * (width * height))


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
    _ensure_module("geometry_msgs.msg").Twist = _Twist
    _ensure_module("lifecycle_msgs.msg").State = object
    _ensure_module("lifecycle_msgs.srv").GetState = object
    _ensure_module("nav2_msgs.action").ComputePathToPose = object
    _ensure_module("nav2_msgs.action").NavigateToPose = object
    _ensure_module("nav2_msgs.msg").Costmap = _Costmap
    _ensure_module("nav2_msgs.srv").ClearEntireCostmap = object
    _ensure_module("nav_msgs.msg").Path = object
    _ensure_module("rcl_interfaces.msg").ParameterType = object
    _ensure_module("rcl_interfaces.srv").GetParameters = object
    _ensure_module("rcl_interfaces.srv").SetParameters = object
    _ensure_module("sensor_msgs.msg").LaserScan = object
    _ensure_module("std_msgs.msg").Bool = object
    _ensure_module("std_msgs.msg").String = _String
    _ensure_module("std_srvs.srv").Trigger = object

    _ensure_module("rclpy.action").ActionClient = object
    _ensure_module("rclpy.callback_groups").ReentrantCallbackGroup = object
    _ensure_module("rclpy.duration").Duration = object
    _ensure_module("rclpy.node").Node = object
    _ensure_module("rclpy.parameter").Parameter = object
    qos = _ensure_module("rclpy.qos")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")

    class _QoSProfile:
        def __init__(self, *, history=None, depth=None, reliability=None):
            self.history = history
            self.depth = depth
            self.reliability = reliability

    qos.QoSProfile = _QoSProfile
    tf2_ros = _ensure_module("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformException = Exception
    tf2_ros.TransformListener = object


_install_ros_stubs()

from g1_base import nav_core

nav_core.Parameter = _Parameter


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Node:
    def __init__(self):
        self.motion_policy = nav_core.MotionPolicyConfig(lateral_assist_enabled=False)
        self.walking_mode = nav_core.WalkingModeConfig()
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, *_args, **_kwargs):
        pub = _Publisher()
        self.publishers.append(pub)
        return pub

    def create_subscription(self, *args, **kwargs):
        self.subscriptions.append((args, kwargs))
        return object()

    def create_timer(self, *_args, **_kwargs):
        return object()

    def get_logger(self):
        return _Logger()

    def get_pose_snapshot(self):
        return (0.0, 0.0, 0.0)


class _Loco:
    def __init__(self):
        self.moves = []
        self.queued_moves = []
        self.urgent_stops = []
        self.reset_health_calls = 0

    def SetSpeedMode(self, _mode):
        return 0

    def Move(self, vx, vy, wz, source=None):
        self.moves.append((vx, vy, wz, source))

    def QueueMove(self, vx, vy, wz, source=None):
        self.queued_moves.append((vx, vy, wz, source))

    def EmergencyStop(self, reason=""):
        self.urgent_stops.append(reason)

    def ResetHealthSnapshot(self):
        self.reset_health_calls += 1


class _ArmClient:
    def __init__(self):
        self.actions = []

    def ExecuteAction(self, action_id):
        self.actions.append(action_id)
        return 0


class _Motion:
    def __init__(self):
        self.paused_sources = []
        self.resume_calls = 0
        self.clear_calls = 0

    def pause_loop(self, source):
        self.paused_sources.append(source)

    def resume_loop(self):
        self.resume_calls += 1

    def clear_manual_override(self):
        self.clear_calls += 1


def _planner_twist(vx, vy, wz):
    msg = _Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


def _front_scan(front_range):
    return _LaserScan([float("inf"), front_range, float("inf")])


def _front_cluster_scan(front_ranges):
    angle_min = -0.08
    angle_increment = 0.08
    ranges = []
    for index, front_x in enumerate(front_ranges):
        angle = angle_min + angle_increment * index
        ranges.append(front_x / math.cos(angle))
    return _LaserScan(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
    )


def _side_cluster_scan():
    angles = [0.90, 0.94, 0.98]
    front_x = 0.25
    ranges = [front_x / math.cos(angle) for angle in angles]
    return _LaserScan(
        ranges,
        angle_min=angles[0],
        angle_increment=angles[1] - angles[0],
    )


def _angled_front_x_cluster_scan(angles, front_x=0.25):
    ranges = [front_x / math.cos(angle) for angle in angles]
    return _LaserScan(
        ranges,
        angle_min=angles[0],
        angle_increment=angles[1] - angles[0],
    )


def _clear_front_scan():
    return _LaserScan([float("inf"), float("inf"), float("inf")])


def test_close_obstacle_guard_defaults_are_narrow_and_cluster_based():
    policy = nav_core.MotionPolicyConfig()

    assert policy.close_obstacle_trigger_distance == 0.35
    assert policy.close_obstacle_release_distance == 0.75
    assert policy.close_obstacle_front_half_width == 0.30
    assert policy.close_obstacle_front_angle_min == -0.66
    assert policy.close_obstacle_front_angle_max == 0.70
    assert policy.close_obstacle_min_points == 3
    assert policy.close_obstacle_replan_on_clear is True
    assert policy.close_obstacle_replan_cooldown == 3.0
    assert policy.close_obstacle_replan_goal_margin == 0.15


def test_goal_occupancy_defaults_are_conservative_and_bounded():
    policy = nav_core.MotionPolicyConfig()

    assert policy.goal_occupancy_enabled is True
    assert policy.goal_occupancy_start_distance == 1.5
    assert policy.goal_occupancy_wait_timeout == 10.0
    assert policy.goal_occupancy_clear_duration == 1.0
    assert policy.goal_occupancy_speak_interval == 4.0
    assert policy.goal_occupancy_costmap_topic == "/local_costmap/costmap_raw"
    assert policy.goal_occupancy_radius == 0.35
    assert policy.goal_occupancy_cost_threshold == 253
    assert policy.goal_occupancy_min_occupied_cells == 2
    assert policy.goal_occupancy_retry_hold_duration == 6.0
    assert policy.goal_occupancy_retry_window == 5.0
    assert policy.goal_occupancy_max_retries == 2


def test_arm_action_safe_uses_configured_preset_duration(monkeypatch):
    sleeps = []
    original_sleep = nav_core.time.sleep
    controller = object.__new__(nav_core.RobotController)
    controller.arm_client = _ArmClient()
    controller.motion = _Motion()
    controller._action_lock = nav_core.threading.Lock()
    controller._preset_config = {17: {"name": "clap", "duration_sec": 8.0}}

    def fake_sleep(seconds):
        sleeps.append(seconds)
        original_sleep(0.01)

    monkeypatch.setattr(nav_core.time, "sleep", fake_sleep)

    result = controller.execute_arm_action_safe(17)

    assert result["status"] == "success"
    assert sleeps == [8.0]
    assert sorted(controller.arm_client.actions) == [17, 99]
    assert controller.motion.paused_sources == ["arm_action_hold"]
    assert controller.motion.resume_calls == 1
    assert controller.motion.clear_calls == 1


def test_goal_occupancy_wait_times_out_near_goal_with_costmap_occupancy():
    policy = nav_core.MotionPolicyConfig(
        goal_occupancy_retry_hold_duration=20.0,
        goal_occupancy_max_retries=0,
    )
    state = nav_core.GoalOccupancyWaitState()

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=100.0,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "entered"
    assert state.active is True
    assert state.started_at == 100.0

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=109.9,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "waiting"
    assert state.active is True

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=110.1,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "timeout"
    assert state.active is True


def test_goal_occupancy_wait_retries_after_hold_duration():
    policy = nav_core.MotionPolicyConfig(
        goal_occupancy_retry_hold_duration=2.0,
        goal_occupancy_retry_window=5.0,
        goal_occupancy_max_retries=2,
    )
    state = nav_core.GoalOccupancyWaitState()

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=100.0,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "entered"

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=101.9,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "waiting"

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=102.1,
        distance_to_goal=0.6,
        goal_occupied=True,
    )
    assert action == "retry"
    assert state.active is True
    assert state.retry_count == 1
    assert state.retrying_until == 107.1
    assert state.last_retry_at == 102.1


def test_goal_occupancy_retry_window_allows_navigation_even_if_still_occupied():
    policy = nav_core.MotionPolicyConfig(
        goal_occupancy_retry_hold_duration=2.0,
        goal_occupancy_retry_window=5.0,
        goal_occupancy_max_retries=2,
    )
    state = nav_core.GoalOccupancyWaitState(
        active=True,
        started_at=100.0,
        retry_count=1,
        retrying_until=107.0,
        last_retry_at=102.0,
    )

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=104.0,
        distance_to_goal=0.6,
        goal_occupied=True,
    )

    assert action == "retrying"
    assert state.retry_count == 1
    assert state.retrying_until == 107.0


def test_goal_occupancy_times_out_after_max_retries_are_used():
    policy = nav_core.MotionPolicyConfig(
        goal_occupancy_retry_hold_duration=2.0,
        goal_occupancy_retry_window=5.0,
        goal_occupancy_max_retries=2,
    )
    state = nav_core.GoalOccupancyWaitState(
        active=True,
        started_at=100.0,
        retry_count=2,
        retrying_until=107.0,
        last_retry_at=102.0,
    )

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=107.1,
        distance_to_goal=0.6,
        goal_occupied=True,
    )

    assert action == "timeout"
    assert state.active is True


def test_goal_occupancy_wait_clears_after_obstacle_has_been_absent():
    policy = nav_core.MotionPolicyConfig()
    state = nav_core.GoalOccupancyWaitState(active=True, started_at=100.0)

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=101.0,
        distance_to_goal=0.6,
        goal_occupied=False,
    )
    assert action == "clearing"
    assert state.active is True
    assert state.clear_since == 101.0

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=102.1,
        distance_to_goal=0.6,
        goal_occupied=False,
    )
    assert action == "cleared"
    assert state.active is False


def test_goal_occupancy_wait_starts_at_measured_goal_approach_distance():
    policy = nav_core.MotionPolicyConfig()
    state = nav_core.GoalOccupancyWaitState()

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=100.0,
        distance_to_goal=1.2,
        goal_occupied=True,
    )

    assert action == "entered"
    assert state.active is True


def test_goal_occupancy_wait_clears_if_robot_leaves_goal_area():
    policy = nav_core.MotionPolicyConfig()
    state = nav_core.GoalOccupancyWaitState(active=True, started_at=100.0)

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=101.0,
        distance_to_goal=1.6,
        goal_occupied=True,
    )

    assert action == "cleared"
    assert state.active is False


def test_goal_costmap_occupancy_detects_high_cost_cells_at_target():
    data = [0] * 100
    # Target (0.45, 0.45) maps to cell (4, 4) in this grid.
    data[4 * 10 + 4] = 254
    data[4 * 10 + 5] = 253
    grid = _Costmap(width=10, height=10, resolution=0.10, data=data)

    occupied, metrics = nav_core.MissionNode._goal_costmap_occupied(
        grid,
        goal_x=0.45,
        goal_y=0.45,
        radius=0.20,
        cost_threshold=253,
        min_occupied_cells=2,
    )

    assert occupied is True
    assert metrics["occupied_cells"] == 2
    assert metrics["max_cost"] == 254


def test_goal_occupancy_prompt_event_payload_targets_bot_mind_voice_channel():
    publisher = _Publisher()
    fake_node = types.SimpleNamespace(
        yield_prompt_event_pub=publisher,
        get_logger=lambda: _Logger(),
    )

    nav_core.MissionNode._publish_yield_prompt_event(
        fake_node,
        event_type="goal_occupied",
        text="请您让一让",
        distance_to_goal=1.49,
        attempt=1,
        metrics={"occupied_cells": 22, "max_cost": 253},
    )

    assert len(publisher.messages) == 1
    payload = json.loads(publisher.messages[0].data)
    assert payload["type"] == "goal_occupied"
    assert payload["state"] == "blocked"
    assert payload["text"] == "请您让一让"
    assert payload["distance_to_goal"] == 1.49
    assert payload["attempt"] == 1
    assert payload["occupied_cells"] == 22
    assert payload["max_cost"] == 253
    assert payload["topic"] == nav_core.YIELD_PROMPT_EVENT_TOPIC


def test_goal_occupancy_wait_starts_from_costmap_even_without_close_obstacle():
    policy = nav_core.MotionPolicyConfig()
    state = nav_core.GoalOccupancyWaitState()

    state, action = nav_core.MissionNode._update_goal_occupancy_wait(
        state,
        policy,
        now=100.0,
        distance_to_goal=0.6,
        goal_occupied=True,
    )

    assert action == "entered"
    assert state.active is True


def test_close_obstacle_guard_uses_best_effort_scan_qos():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
    )
    loco = _Loco()

    nav_core.MotionController(node, loco)

    scan_subscription = node.subscriptions[-1]
    args, _kwargs = scan_subscription
    assert args[1] == "/scan"
    assert args[3].reliability == nav_core.ReliabilityPolicy.BEST_EFFORT


def test_stop_latch_ignores_later_nonzero_planner_commands_until_explicit_release():
    node = _Node()
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)

    controller._planner_cb(_planner_twist(0.4, 0.0, 0.2))
    controller.emergency_stop_latched("unit_test")
    controller._planner_cb(_planner_twist(0.7, 0.0, 0.5))

    assert controller.is_stop_latched()
    assert loco.urgent_stops == ["unit_test"]
    assert controller._select_command(node.get_pose_snapshot) == (
        0.0,
        0.0,
        0.0,
        "stop_latched",
    )

    controller.clear_manual_override()
    assert controller.is_stop_latched()

    controller.release_stop_latch("new_goal")
    controller._planner_cb(_planner_twist(0.7, 0.0, 0.5))
    vx, _vy, wz, source = controller._select_command(node.get_pose_snapshot)
    assert source == "planner"
    assert vx > 0.0
    assert wz > 0.0


def test_close_obstacle_guard_holds_and_urgently_stops_after_consecutive_front_hits():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
        close_obstacle_trigger_distance=0.40,
        close_obstacle_release_distance=0.55,
        close_obstacle_front_half_width=0.45,
        close_obstacle_min_consecutive_frames=2,
        close_obstacle_clear_duration=1.0,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()

    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._planner_cb(_planner_twist(0.7, 0.0, 0.0))

    assert loco.urgent_stops == ["close_obstacle"]
    assert controller._select_command(node.get_pose_snapshot) == (
        0.0,
        0.0,
        0.0,
        "close_obstacle_hold",
    )


def test_close_obstacle_guard_zero_command_is_not_smoothed():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
        close_obstacle_trigger_distance=0.40,
        close_obstacle_release_distance=0.55,
        close_obstacle_front_half_width=0.45,
        close_obstacle_min_consecutive_frames=2,
        close_obstacle_clear_duration=1.0,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller.last_sent_cmd = (0.6, 0.0, 0.0)

    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._loop()

    assert loco.queued_moves[-1] == (0.0, 0.0, 0.0, "close_obstacle_hold")


def test_goal_occupied_wait_zero_command_is_not_smoothed():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller.last_sent_cmd = (0.6, 0.0, 0.0)

    controller.hold_position("goal_occupied_wait")
    controller._loop()

    assert loco.queued_moves[-1] == (0.0, 0.0, 0.0, "goal_occupied_wait")


def test_close_obstacle_guard_releases_after_clear_duration(monkeypatch):
    now = 1000.0
    monkeypatch.setattr(nav_core.time, "time", lambda: now)
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
        close_obstacle_trigger_distance=0.40,
        close_obstacle_release_distance=0.55,
        close_obstacle_front_half_width=0.45,
        close_obstacle_min_consecutive_frames=2,
        close_obstacle_clear_duration=1.0,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))

    assert controller._select_command(node.get_pose_snapshot)[3] == "close_obstacle_hold"

    now += 0.5
    controller._scan_cb(_clear_front_scan())
    assert controller._select_command(node.get_pose_snapshot)[3] == "close_obstacle_hold"

    now += 1.1
    controller._scan_cb(_clear_front_scan())
    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))
    vx, _vy, _wz, source = controller._select_command(node.get_pose_snapshot)
    assert source == "planner"
    assert vx > 0.0


def test_close_obstacle_guard_release_emits_replan_event_once(monkeypatch):
    now = 1000.0
    monkeypatch.setattr(nav_core.time, "time", lambda: now)
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
        close_obstacle_trigger_distance=0.40,
        close_obstacle_release_distance=0.55,
        close_obstacle_front_half_width=0.45,
        close_obstacle_min_consecutive_frames=2,
        close_obstacle_clear_duration=1.0,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()

    assert controller.consume_close_obstacle_clear_event() is False

    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    controller._scan_cb(_front_cluster_scan([0.32, 0.34, 0.35]))
    assert controller.consume_close_obstacle_clear_event() is False

    now += 1.1
    controller._scan_cb(_clear_front_scan())
    now += 1.1
    controller._scan_cb(_clear_front_scan())

    assert controller.consume_close_obstacle_clear_event() is True
    assert controller.consume_close_obstacle_clear_event() is False


def test_close_obstacle_clear_replan_skips_near_goal_and_cooldown():
    policy = nav_core.MotionPolicyConfig(
        close_obstacle_replan_on_clear=True,
        close_obstacle_replan_cooldown=3.0,
        close_obstacle_replan_goal_margin=0.15,
    )

    assert (
        nav_core.MissionNode._should_restart_after_close_obstacle_clear(
            policy,
            distance_to_goal=0.80,
            stop_distance=0.40,
            now=20.0,
            last_restart_at=10.0,
        )
        is True
    )
    assert (
        nav_core.MissionNode._should_restart_after_close_obstacle_clear(
            policy,
            distance_to_goal=0.50,
            stop_distance=0.40,
            now=20.0,
            last_restart_at=10.0,
        )
        is False
    )
    assert (
        nav_core.MissionNode._should_restart_after_close_obstacle_clear(
            policy,
            distance_to_goal=0.80,
            stop_distance=0.40,
            now=12.0,
            last_restart_at=10.0,
        )
        is False
    )
    assert (
        nav_core.MissionNode._should_restart_after_close_obstacle_clear(
            nav_core.MotionPolicyConfig(close_obstacle_replan_on_clear=False),
            distance_to_goal=0.80,
            stop_distance=0.40,
            now=20.0,
            last_restart_at=10.0,
        )
        is False
    )


def test_close_obstacle_guard_ignores_single_close_scan_hit():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))

    controller._scan_cb(_front_scan(0.20))
    controller._scan_cb(_front_scan(0.20))

    assert loco.urgent_stops == []
    assert controller._select_command(node.get_pose_snapshot)[3] == "planner"


def test_close_obstacle_guard_ignores_side_cluster_outside_front_corridor():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))

    controller._scan_cb(_side_cluster_scan())
    controller._scan_cb(_side_cluster_scan())

    assert loco.urgent_stops == []
    assert controller._select_command(node.get_pose_snapshot)[3] == "planner"


def test_close_obstacle_guard_ignores_points_outside_measured_pillar_window():
    node = _Node()
    node.motion_policy = nav_core.MotionPolicyConfig(
        lateral_assist_enabled=False,
        close_obstacle_guard_enabled=True,
    )
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)
    controller.begin_navigation_goal()
    controller._planner_cb(_planner_twist(0.4, 0.0, 0.0))

    outside_right_pillar_inner_edge = _angled_front_x_cluster_scan(
        [0.74, 0.76, 0.78]
    )
    controller._scan_cb(outside_right_pillar_inner_edge)
    controller._scan_cb(outside_right_pillar_inner_edge)

    assert loco.urgent_stops == []
    assert controller._select_command(node.get_pose_snapshot)[3] == "planner"


def test_navigation_health_abort_reason_distinguishes_system_failures():
    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 5.1},
            {"loco_latency_p95_ms": 0.0},
            elapsed=nav_core.INITIAL_PLANNER_CMD_GRACE + 0.1,
        )
        == "planner_silent"
    )

    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 0.1},
            {
                "loco_latency_p95_ms": 220.0,
                "active_loco_latency_p95_ms": 220.0,
                "active_loco_latency_sample_count": 2,
            },
            elapsed=nav_core.INITIAL_PLANNER_CMD_GRACE + 0.1,
        )
        == ""
    )

    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 0.1},
            {"loco_latency_p95_ms": 120.0},
        )
        == ""
    )


def test_navigation_health_abort_reason_respects_sdk_slow_startup_grace():
    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 0.1},
            {
                "loco_latency_p95_ms": 203.6,
                "active_loco_latency_p95_ms": 203.6,
                "active_loco_latency_sample_count": 10,
            },
            elapsed=1.0,
        )
        == ""
    )


def test_navigation_health_abort_reason_ignores_stale_global_sdk_p95_without_active_planner_samples():
    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": float("inf")},
            {
                "loco_latency_p95_ms": 520.0,
                "loco_latency_sample_count": 30,
                "active_loco_latency_p95_ms": 0.0,
                "active_loco_latency_sample_count": 0,
            },
            elapsed=1.0,
        )
        == ""
    )


def test_navigation_health_abort_reason_requires_sustained_severe_active_planner_latency():
    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 0.1},
            {
                "loco_latency_p95_ms": 520.0,
                "active_loco_latency_p95_ms": 520.0,
                "active_loco_latency_sample_count": 2,
            },
            elapsed=nav_core.INITIAL_PLANNER_CMD_GRACE + 0.1,
        )
        == ""
    )

    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": 0.1},
            {
                "loco_latency_p95_ms": 520.0,
                "active_loco_latency_p95_ms": 520.0,
                "active_loco_latency_sample_count": 5,
            },
            elapsed=nav_core.INITIAL_PLANNER_CMD_GRACE + 0.1,
        )
        == "sdk_slow"
    )


def test_navigation_health_abort_reason_respects_initial_planner_grace():
    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": float("inf")},
            {"loco_latency_p95_ms": 0.0},
            elapsed=1.0,
        )
        == ""
    )

    assert (
        nav_core.MissionNode._navigation_health_abort_reason(
            {"planner_age": float("inf")},
            {"loco_latency_p95_ms": 0.0},
            elapsed=nav_core.INITIAL_PLANNER_CMD_GRACE + 0.1,
        )
        == "planner_silent"
    )


def test_begin_navigation_goal_clears_stale_planner_timestamp():
    node = _Node()
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)

    controller._planner_cb(_planner_twist(0.4, 0.0, 0.1))
    assert controller.get_command_snapshot()["planner_age"] < float("inf")

    controller.begin_navigation_goal()
    snapshot = controller.get_command_snapshot()

    assert snapshot["planner_cmd"] == (0.0, 0.0, 0.0)
    assert snapshot["planner_age"] == float("inf")
    assert loco.reset_health_calls == 1


def test_planner_timeout_zero_move_is_debounced_but_state_and_hard_stop_stay_immediate():
    node = _Node()
    loco = _Loco()
    controller = nav_core.MotionController(node, loco)

    controller.begin_navigation_goal()
    controller._loop()
    controller._loop()

    assert loco.queued_moves == [(0.0, 0.0, 0.0, "planner_timeout")]
    executed_pub = node.publishers[0]
    source_pub = node.publishers[1]
    assert len(executed_pub.messages) == 2
    assert [msg.data for msg in source_pub.messages[-2:]] == [
        "planner_timeout",
        "planner_timeout",
    ]

    controller.emergency_stop_latched("unit_test")
    assert loco.urgent_stops == ["unit_test"]

    controller._loop()
    assert loco.queued_moves[-1] == (0.0, 0.0, 0.0, "stop_latched")


def test_navigation_profile_selects_mppi_parameters_when_declared():
    profile = nav_core.build_navigation_profile(0.0, nav_core.WalkingModeConfig())
    groups, lateral = nav_core.MissionNode._navigation_profile_parameter_groups(
        profile,
        nav_core.MotionPolicyConfig(global_allow_lateral_motion=False),
    )

    parameters, controller_kind = nav_core.MissionNode._select_profile_parameters(
        groups,
        {
            "FollowPath.vx_max",
            "FollowPath.vx_min",
            "FollowPath.vy_max",
            "FollowPath.wz_max",
        },
    )
    names = {parameter.name for parameter in parameters}
    by_name = {parameter.name: parameter for parameter in parameters}

    assert controller_kind == "mppi"
    assert names == {
        "FollowPath.vx_max",
        "FollowPath.vx_min",
        "FollowPath.vy_max",
        "FollowPath.wz_max",
    }
    assert "FollowPath.max_vel_x" not in names
    assert "FollowPath.PathAlign.scale" not in names
    assert by_name["FollowPath.vx_max"].value == profile["max_vel_x"]
    assert by_name["FollowPath.wz_max"].value == profile["max_vel_theta"]
    assert lateral.max_vel_y == 0.0


def test_navigation_profile_keeps_legacy_dwb_parameters_when_declared():
    profile = nav_core.build_navigation_profile(1.0, nav_core.WalkingModeConfig())
    groups, _lateral = nav_core.MissionNode._navigation_profile_parameter_groups(
        profile,
        nav_core.MotionPolicyConfig(global_allow_lateral_motion=False),
    )

    parameters, controller_kind = nav_core.MissionNode._select_profile_parameters(
        groups,
        {
            "FollowPath.max_vel_x",
            "FollowPath.max_speed_xy",
            "FollowPath.PathAlign.scale",
        },
    )
    names = {parameter.name for parameter in parameters}

    assert controller_kind == "legacy"
    assert "FollowPath.max_vel_x" in names
    assert "FollowPath.PathAlign.scale" in names
    assert "FollowPath.vx_max" not in names
