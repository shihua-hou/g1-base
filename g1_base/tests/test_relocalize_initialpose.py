from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parent


def test_relocalize_service_interface_is_declared():
    srv_path = ROOT / "g1_base_interfaces" / "srv" / "Relocalize.srv"
    cmake_path = ROOT / "g1_base_interfaces" / "CMakeLists.txt"

    srv_text = srv_path.read_text(encoding="utf-8")
    cmake_text = cmake_path.read_text(encoding="utf-8")

    assert "float64 x" in srv_text
    assert "float64 y" in srv_text
    assert "float64 yaw" in srv_text
    assert "float64 duration_sec" in srv_text
    assert "float64 rate_hz" in srv_text
    assert "srv/Relocalize.srv" in cmake_text


def test_navigation_manager_streams_initialpose_during_bringup():
    text = (ROOT / "g1_base" / "navigation_manager.py").read_text(encoding="utf-8")

    assert '"/navigation_manager/relocalize"' in text
    assert '"/initialpose"' in text
    assert "PoseWithCovarianceStamped" in text
    assert "_start_initial_pose_stream" in text

    start_stream = text.index("initial_pose_stop_event, initial_pose_thread = self._start_initial_pose_stream")
    start_localization = text.index("if not self._start_localization_stack():", start_stream)
    stop_stream = text.index("self._stop_initial_pose_stream", start_localization)

    assert start_stream < start_localization < stop_stream


def test_boothshow_requires_dragged_direction_before_relocalize():
    text = (
        REPO_ROOT / "bot_mind" / "src" / "web" / "boothshow.html"
    ).read_text(encoding="utf-8")

    assert "yaw: null" in text
    assert "Number.isFinite(navMap.selectedPose.yaw)" in text
    assert "请在地图上按下并拖动" in text
