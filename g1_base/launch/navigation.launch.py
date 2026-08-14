import os
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


FALLBACK_MAP_IMAGE = "exhibit_2d_map.pgm"
DEFAULT_MID360_HEIGHT_M = "1.3"


def _parse_yaml_scalar(value):
    value = value.strip()
    if not value:
        return ""

    quote = value[0]
    if quote in ("'", '"'):
        end = value.find(quote, 1)
        return value[1:end] if end != -1 else value[1:]

    return value.split("#", 1)[0].strip()


def _read_map_image(yaml_path):
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        stripped = line.lstrip()
        key, separator, value = stripped.partition(":")
        if separator and key.strip() == "image":
            return _parse_yaml_scalar(value)
    return ""


def _write_fallback_map_yaml(yaml_path, fallback_image_path):
    output = []
    replaced = False
    for line in yaml_path.read_text(encoding="utf-8").splitlines(keepends=True):
        stripped = line.lstrip()
        key, separator, _ = stripped.partition(":")
        if separator and key.strip() == "image" and not replaced:
            indent = line[: len(line) - len(stripped)]
            newline = "\n" if line.endswith("\n") else ""
            output.append(f"{indent}image: {fallback_image_path}{newline}")
            replaced = True
        else:
            output.append(line)

    if not replaced:
        output.insert(0, f"image: {fallback_image_path}\n")

    fd, tmp_path = tempfile.mkstemp(prefix=f"{yaml_path.stem}_fallback_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        handle.writelines(output)
    return tmp_path


def _resolve_map_file(context, map_file):
    yaml_path = Path(map_file.perform(context)).expanduser()
    messages = []

    if not yaml_path.is_file():
        raise FileNotFoundError(f"map yaml not found: {yaml_path}")

    try:
        image_value = _read_map_image(yaml_path)
    except Exception as exc:
        raise RuntimeError(f"failed to read map yaml image field: {yaml_path}: {exc}") from exc

    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    if image_path.is_file():
        return str(yaml_path), messages

    fallback_path = yaml_path.parent / FALLBACK_MAP_IMAGE
    if not fallback_path.is_file():
        raise RuntimeError(
            "map yaml image is missing and fallback image is also missing: "
            f"yaml={yaml_path}, image={image_path}, fallback={fallback_path}"
        )

    try:
        guarded_yaml = _write_fallback_map_yaml(yaml_path, str(fallback_path))
    except Exception as exc:
        raise RuntimeError(f"failed to prepare fallback map yaml: {yaml_path}: {exc}") from exc

    messages.append(
        "[WARN] map yaml image is missing; using fallback image: "
        f"image={image_path}, fallback={fallback_path}, yaml={guarded_yaml}"
    )
    return guarded_yaml, messages


def generate_launch_description():
    package_share = FindPackageShare("g1_base")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    map_file = LaunchConfiguration("map_file")
    map_z_offset = LaunchConfiguration("map_z_offset")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    obstacle_cloud_topic = LaunchConfiguration("obstacle_cloud_topic")
    obstacle_odom_topic = LaunchConfiguration("obstacle_odom_topic")
    obstacle_cloud_max_range = LaunchConfiguration("obstacle_cloud_max_range")
    obstacle_cloud_min_height = LaunchConfiguration("obstacle_cloud_min_height")
    obstacle_cloud_max_height = LaunchConfiguration("obstacle_cloud_max_height")
    obstacle_filter_package = LaunchConfiguration("obstacle_filter_package")
    obstacle_filter_executable = LaunchConfiguration("obstacle_filter_executable")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    imu_data_topic = LaunchConfiguration("imu_data_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    scan_range_min = LaunchConfiguration("scan_range_min")
    use_sim_time = LaunchConfiguration("use_sim_time")

    def map_server_actions(context):
        resolved_map_file, messages = _resolve_map_file(context, map_file)
        return [
            *[LogInfo(msg=message) for message in messages],
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": resolved_map_file,
                        "frame_id": "map",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "maps", "exhibit_2d_map.yaml"]
                ),
            ),
            DeclareLaunchArgument("map_z_offset", default_value=DEFAULT_MID360_HEIGHT_M),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/lio/cloud_world"),
            DeclareLaunchArgument("obstacle_cloud_topic", default_value="/nav/obstacle_cloud"),
            DeclareLaunchArgument("obstacle_odom_topic", default_value="/lio/robo/odom"),
            DeclareLaunchArgument("obstacle_cloud_max_range", default_value="3.0"),
            DeclareLaunchArgument("obstacle_cloud_min_height", default_value="0.15"),
            DeclareLaunchArgument("obstacle_cloud_max_height", default_value="1.6"),
            DeclareLaunchArgument("obstacle_filter_package", default_value="g1_base_perception"),
            DeclareLaunchArgument("obstacle_filter_executable", default_value="nav_obstacle_cloud_filter"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("imu_data_topic", default_value="/imu/data"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_range_min", default_value="0.05"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=map_server_actions),
            # Task 5 z contract:
            # map.z=0 is the floor, world.z=0 is the MID360 startup height.
            # nav_obstacle_cloud_filter publishes frame_id=world; STVL/Nav2
            # transform that cloud through this map->world static TF, using
            # map_z_offset (MID360 floor height H) as the z translation.
            # Do not publish world coordinates with a fake map frame label.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_world",
                output="screen",
                arguments=["0", "0", map_z_offset, "0", "0", "0", "map", "world"],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick",
                output="screen",
                remappings=[
                    ("imu/data_raw", imu_raw_topic),
                    ("imu/data", imu_data_topic),
                ],
                parameters=[
                    {
                        "use_mag": False,
                        "gain": 0.05,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="g1_base",
                executable="gravity_health",
                name="gravity_health",
                output="screen",
                parameters=[
                    {
                        "imu_topic": imu_data_topic,
                        "odom_topic": obstacle_odom_topic,
                        "warning_angle_deg": 3.0,
                        "warning_duration_sec": 5.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package=obstacle_filter_package,
                executable=obstacle_filter_executable,
                name="nav_obstacle_cloud_filter",
                output="screen",
                parameters=[
                    {
                        "input_cloud_topic": cloud_topic,
                        "odom_topic": obstacle_odom_topic,
                        "imu_topic": imu_raw_topic,
                        "output_cloud_topic": obstacle_cloud_topic,
                        "output_frame": "world",
                        "min_range": 0.45,
                        "max_range": ParameterValue(
                            obstacle_cloud_max_range, value_type=float
                        ),
                        "ground_fit_min_range": 0.7,
                        "ground_fit_max_range": 4.0,
                        "obstacle_min_height": ParameterValue(
                            obstacle_cloud_min_height, value_type=float
                        ),
                        "obstacle_max_height": ParameterValue(
                            obstacle_cloud_max_height, value_type=float
                        ),
                        "voxel_leaf_size": 0.08,
                        "voxel_min_points": 1,
                        "max_odom_age": 0.5,
                    }
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pc_to_laserscan",
                output="screen",
                remappings=[("cloud_in", cloud_topic), ("scan", scan_topic)],
                parameters=[
                    {
                        "target_frame": target_frame,
                        "transform_tolerance": 1.0,
                        "min_height": -1.00,
                        "max_height": 0.50,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0349,
                        "scan_time": 0.2,
                        "range_min": scan_range_min,
                        "range_max": 5.0,
                        "use_inf": True,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [nav2_bringup_share, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params_file,
                    "autostart": "true",
                }.items(),
            ),
        ]
    )
