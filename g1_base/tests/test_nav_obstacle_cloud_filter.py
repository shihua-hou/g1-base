import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=0, nanosec=0))


class _FakeNode:
    def __init__(self, name):
        self.name = name
        self._params = {}
        self._logger = _FakeLogger()
        self.cloud_pub = None
        self.timers = []

    def declare_parameter(self, name, default_value):
        self._params[name] = default_value

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def create_subscription(self, *args):
        return SimpleNamespace(args=args)

    def create_publisher(self, *args):
        self.cloud_pub = _FakePublisher()
        return self.cloud_pub

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return _FakeClock()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


class _FakePointField:
    FLOAT32 = 7


class _FakeHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


def _install_ros_fakes(monkeypatch):
    fake_rclpy = SimpleNamespace(
        init=lambda args=None: None,
        spin=lambda node: None,
        ok=lambda: False,
        shutdown=lambda: None,
    )
    fake_node_module = SimpleNamespace(Node=_FakeNode)
    fake_qos_module = SimpleNamespace(
        HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
    )
    fake_sensor_msgs = SimpleNamespace(
        msg=SimpleNamespace(
            Imu=object,
            PointCloud2=object,
            PointField=_FakePointField,
        )
    )
    fake_point_cloud2 = SimpleNamespace(
        read_points=lambda msg, field_names, skip_nans: msg.points,
        create_cloud_xyz32=lambda header, points: SimpleNamespace(
            header=header,
            points=points,
        ),
    )

    modules = {
        "rclpy": fake_rclpy,
        "rclpy.node": fake_node_module,
        "rclpy.qos": fake_qos_module,
        "nav_msgs": SimpleNamespace(msg=SimpleNamespace(Odometry=object)),
        "nav_msgs.msg": SimpleNamespace(Odometry=object),
        "sensor_msgs": fake_sensor_msgs,
        "sensor_msgs.msg": fake_sensor_msgs.msg,
        "sensor_msgs_py": SimpleNamespace(point_cloud2=fake_point_cloud2),
        "sensor_msgs_py.point_cloud2": fake_point_cloud2,
        "std_msgs": SimpleNamespace(msg=SimpleNamespace(Header=_FakeHeader)),
        "std_msgs.msg": SimpleNamespace(Header=_FakeHeader),
    }
    for name, module in modules.items():
        monkeypatch.setitem(__import__("sys").modules, name, module)


def _load_filter_module(monkeypatch):
    _install_ros_fakes(monkeypatch)
    module_path = ROOT / "g1_base" / "nav_obstacle_cloud_filter.py"
    spec = importlib.util.spec_from_file_location(
        "nav_obstacle_cloud_filter_under_test", module_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _odom(x=0.0, y=0.0, z=1.0, q=(0.0, 0.0, 0.0, 1.0)):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
    )
    return SimpleNamespace(pose=SimpleNamespace(pose=pose))


def _imu(accel=(0.0, 0.0, -1.0)):
    return SimpleNamespace(
        linear_acceleration=SimpleNamespace(x=accel[0], y=accel[1], z=accel[2])
    )


def _cloud(points):
    return SimpleNamespace(fields=[], points=points, width=len(points), height=1)


def _make_node(module, **params):
    node = module.NavObstacleCloudFilter()
    node._params.update(
        {
            "ground_min_inliers": 20,
            "ground_fit_min_range": 0.0,
            "ground_fit_max_range": 4.0,
            "voxel_leaf_size": 0.0,
        }
    )
    node._params.update(params)
    return node


def _floor_and_obstacles():
    floor = [
        (ix * 0.2, iy * 0.2, 0.0)
        for ix in range(-10, 11)
        for iy in range(-10, 11)
        if math.hypot(ix * 0.2, iy * 0.2) >= 0.5
    ]
    obstacles = [
        (1.0, 0.0, 0.35),
        (1.1, 0.0, 0.35),
        (1.2, 0.0, 0.35),
    ]
    return floor + obstacles


def test_cloud_callback_publishes_world_frame_after_imu_prior_fit(monkeypatch):
    module = _load_filter_module(monkeypatch)
    node = _make_node(module)
    msg = _cloud(_floor_and_obstacles())

    node._on_odom(_odom())
    node._on_imu(_imu())
    node._on_cloud(msg)
    node.cloud_pub.messages.clear()
    node._on_ground_fit_timer()
    node._on_cloud(msg)

    published = node.cloud_pub.messages[-1]
    assert published.header.frame_id == "world"
    assert len(published.points) > 0


def test_single_ransac_failure_reuses_last_stable_plane(monkeypatch):
    module = _load_filter_module(monkeypatch)
    node = _make_node(module)
    msg = _cloud(_floor_and_obstacles())

    node._on_odom(_odom())
    node._on_imu(_imu())
    node._accept_ground_plane(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))
    node._on_cloud(msg)
    node.cloud_pub.messages.clear()

    node._fit_ground_plane = lambda *args, **kwargs: None
    node._on_ground_fit_timer()
    node._on_cloud(msg)

    published = node.cloud_pub.messages[-1]
    assert node.consecutive_fail_count == 1
    assert len(published.points) > 0


def test_thirty_one_consecutive_ransac_failures_publish_empty(monkeypatch):
    module = _load_filter_module(monkeypatch)
    node = _make_node(module)
    msg = _cloud(_floor_and_obstacles())

    node._on_odom(_odom())
    node._on_imu(_imu())
    node._accept_ground_plane(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))
    node._on_cloud(msg)
    node.cloud_pub.messages.clear()

    node._fit_ground_plane = lambda *args, **kwargs: None
    for _ in range(31):
        node._on_ground_fit_timer()
    node._on_cloud(msg)

    published = node.cloud_pub.messages[-1]
    assert node.consecutive_fail_count == 31
    assert published.points == []


def test_imu_prior_seeds_ground_fit_with_fewer_iterations(monkeypatch):
    module = _load_filter_module(monkeypatch)
    points = np.array(_floor_and_obstacles(), dtype=np.float64)
    odom = _odom()
    odom_xyz = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    range_xy = np.linalg.norm(points[:, :2], axis=1)

    with_prior = _make_node(module)
    with_prior._on_odom(odom)
    with_prior._on_imu(_imu())
    prior_normal = with_prior._gravity_prior_normal(odom)
    prior_plane = with_prior._fit_ground_plane(
        points, range_xy, odom_xyz, prior_normal=prior_normal
    )
    prior_iterations = with_prior.last_ground_fit_iterations

    without_prior = _make_node(module)
    without_prior._fit_ground_plane(points, range_xy, odom_xyz, prior_normal=None)
    no_prior_iterations = without_prior.last_ground_fit_iterations

    assert prior_plane is not None
    assert with_prior.last_ground_fit_used_prior
    assert prior_iterations < no_prior_iterations
