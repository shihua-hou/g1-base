import json
from pathlib import Path
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


def _install_ros_stubs():
    rclpy = _ensure_module("rclpy")
    rclpy.ok = lambda: False
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None

    _ensure_module("geometry_msgs.msg").PoseWithCovarianceStamped = object
    _ensure_module("g1_base_interfaces.srv").Relocalize = object
    _ensure_module("lifecycle_msgs.msg").State = object
    _ensure_module("lifecycle_msgs.srv").GetState = object
    _ensure_module("nav2_msgs.action").NavigateToPose = object
    _ensure_module("nav_msgs.msg").Odometry = object
    _ensure_module("sensor_msgs.msg").PointCloud2 = object
    _ensure_module("std_msgs.msg").Bool = object
    _ensure_module("std_msgs.msg").String = object
    _ensure_module("std_srvs.srv").Trigger = object

    _ensure_module("rclpy.action").ActionClient = object
    _ensure_module("rclpy.callback_groups").ReentrantCallbackGroup = object
    _ensure_module("rclpy.duration").Duration = object
    executors = _ensure_module("rclpy.executors")
    executors.ExternalShutdownException = Exception
    executors.MultiThreadedExecutor = object
    _ensure_module("rclpy.node").Node = object

    qos = _ensure_module("rclpy.qos")
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=object())
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=object())
    qos.QoSPresetProfiles = types.SimpleNamespace(
        SENSOR_DATA=types.SimpleNamespace(value=object())
    )
    qos.QoSProfile = lambda *args, **kwargs: types.SimpleNamespace()

    tf2_ros = _ensure_module("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformException = Exception
    tf2_ros.TransformListener = object


_install_ros_stubs()

from g1_base import navigation_manager


class _Proc:
    def __init__(self, returncode):
        self.returncode = returncode

    def poll(self):
        return self.returncode


def test_script_exit_failure_distinguishes_imu_gate_exit_code():
    failure = navigation_manager.NavigationManager._script_exit_failure(
        _Proc(4), "mapping stack"
    )

    assert failure["reason"] == navigation_manager.IMU_NOT_STEADY_REASON
    assert navigation_manager.IMU_NOT_STEADY_MESSAGE in failure["message"]


def test_script_exit_failure_keeps_generic_nonzero_process_exit():
    failure = navigation_manager.NavigationManager._script_exit_failure(
        _Proc(1), "localization stack"
    )

    assert failure == {
        "reason": "process_exited",
        "message": "localization stack exited with code 1",
    }


def test_script_exit_failure_marks_zero_exit_before_readiness():
    failure = navigation_manager.NavigationManager._script_exit_failure(
        _Proc(0), "mapping stack"
    )

    assert failure == {
        "reason": "process_exited",
        "message": "mapping stack exited before readiness",
    }


def test_manifest_writer_formats_tilt_field(tmp_path):
    manager = object.__new__(navigation_manager.NavigationManager)
    manager._resolve_maps_dir = lambda: tmp_path

    navigation_manager.NavigationManager._write_map_manifest(
        manager,
        "20260516_120000",
        status="failed",
        reason="tilted_world",
        error="tilted",
        tilt=10.123,
    )

    payload = json.loads((tmp_path / navigation_manager.MANIFEST_FILENAME).read_text())
    assert payload["status"] == "failed"
    assert payload["reason"] == "tilted_world"
    assert payload["tilt"] == "10.12°"


def test_generate_2d_map_uses_snapshot_style_world_z_filter(tmp_path, monkeypatch):
    from g1_base import pcd_to_2d_map

    calls = []
    pcd_path = tmp_path / "map.pcd"
    pcd_path.write_bytes(b"pcd")

    def fake_convert_pcd_to_2d_map(**kwargs):
        calls.append(kwargs)
        return "/tmp/out.pgm", "/tmp/out.yaml"

    monkeypatch.setattr(
        pcd_to_2d_map,
        "convert_pcd_to_2d_map",
        fake_convert_pcd_to_2d_map,
    )

    manager = object.__new__(navigation_manager.NavigationManager)
    manager._resolve_maps_dir = lambda: tmp_path / "maps"

    pgm, yaml_f, copied_pcd = navigation_manager.NavigationManager._generate_2d_map(
        manager,
        str(pcd_path),
        "20260516_120000",
    )

    assert (pgm, yaml_f) == ("/tmp/out.pgm", "/tmp/out.yaml")
    assert Path(copied_pcd).name == "20260516_120000_map.pcd"
    assert calls == [
        {
            "pcd_path": copied_pcd,
            "output_dir": str(tmp_path / "maps"),
            "output_name": "20260516_120000_exhibit_2d_map",
            "z_min": pcd_to_2d_map.LEGACY_Z_MIN,
            "z_max": pcd_to_2d_map.LEGACY_Z_MAX,
        }
    ]


def test_generate_and_publish_marks_tilted_world():
    from g1_base.pcd_to_2d_map import TiltedWorldError

    calls = []
    manager = object.__new__(navigation_manager.NavigationManager)
    manager._generate_2d_map = lambda pcd_path, base_name: (_ for _ in ()).throw(
        TiltedWorldError(10.0, 1.5, None)
    )
    manager._write_map_manifest = lambda *args, **kwargs: calls.append(
        (args, kwargs)
    )
    manager.get_logger = lambda: types.SimpleNamespace(error=lambda message: None)

    navigation_manager.NavigationManager._generate_and_publish_2d_map(
        manager, "/tmp/map.pcd", "20260516_120000"
    )

    assert calls == [
        (
            ("20260516_120000",),
            {
                "status": "failed",
                "reason": "tilted_world",
                "error": (
                    "PCD world frame is tilted 10.00° from +Z; "
                    "max allowed is 1.50°. Rebuild the map with the robot "
                    "still during LIO initialization."
                ),
                "tilt": 10.0,
            },
        )
    ]
