import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _launch_text():
    return (ROOT / "launch" / "navigation.launch.py").read_text(encoding="utf-8")


def test_map_z_offset_default_matches_ground_origin_contract():
    """world 原点已落在地面（odom_robo 平移量所致），map->world 的 z 必须是 0。

    填成雷达高度会把整个代价地图抬高一个雷达高度。
    """
    text = _launch_text()
    height_match = re.search(r'DEFAULT_MAP_Z_OFFSET_M\s*=\s*"([^"]+)"', text)
    assert height_match is not None
    assert float(height_match.group(1)) == 0.0

    arg_match = re.search(
        r'DeclareLaunchArgument\(\s*"map_z_offset",\s*'
        r"default_value=DEFAULT_MAP_Z_OFFSET_M",
        text,
    )
    assert arg_match is not None


def test_navigation_launch_documents_world_cloud_tf_contract():
    text = _launch_text()

    assert "map.z=0 is the floor" in text
    assert "nav_obstacle_cloud_filter publishes frame_id=world" in text
    assert "Do not publish world coordinates with a fake map frame label" in text


def test_navigation_launch_uses_cpp_obstacle_filter_with_python_rollback_args():
    text = _launch_text()

    assert 'DeclareLaunchArgument("obstacle_filter_package", default_value="g1_base_perception")' in text
    assert 'DeclareLaunchArgument("obstacle_filter_executable", default_value="nav_obstacle_cloud_filter")' in text
    assert "package=obstacle_filter_package" in text
    assert "executable=obstacle_filter_executable" in text
    assert '"imu_topic": imu_raw_topic' in text


def test_navigation_launch_fails_when_yaml_image_and_fallback_are_missing():
    text = _launch_text()

    assert 'FALLBACK_MAP_IMAGE = "exhibit_2d_map.pgm"' in text
    assert "raise FileNotFoundError" in text
    assert "fallback_path = yaml_path.parent / FALLBACK_MAP_IMAGE" in text
    assert "raise RuntimeError" in text
    assert "fallback image is also missing" in text
