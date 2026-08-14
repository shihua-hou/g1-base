"""Local global-planner-only Nav2 test stack.

This launch file is intentionally robot-free. It starts only map_server,
planner_server, the planner's embedded global_costmap, a fixed map->base_link
TF, and an optional RViz instance so global planners can be compared on a WSL
development machine without touching pc2.
"""

import os
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


FALLBACK_MAP_IMAGE = "exhibit_2d_map.pgm"


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

    fd, tmp_path = tempfile.mkstemp(prefix=f"{yaml_path.stem}_local_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        handle.writelines(output)
    return tmp_path


def _resolve_map_file(context, map_file):
    yaml_path = Path(map_file.perform(context)).expanduser()
    if not yaml_path.is_file():
        raise FileNotFoundError(f"map yaml not found: {yaml_path}")

    image_value = _read_map_image(yaml_path)
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    if image_path.is_file():
        return str(yaml_path), []

    fallback_path = yaml_path.parent / FALLBACK_MAP_IMAGE
    if not fallback_path.is_file():
        raise RuntimeError(
            "map yaml image is missing and fallback image is also missing: "
            f"yaml={yaml_path}, image={image_path}, fallback={fallback_path}"
        )
    guarded_yaml = _write_fallback_map_yaml(yaml_path, str(fallback_path))
    return guarded_yaml, [
        "[WARN] map yaml image is missing; using local fallback image: "
        f"image={image_path}, fallback={fallback_path}, yaml={guarded_yaml}"
    ]


def generate_launch_description():
    package_share = FindPackageShare("g1_base")

    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")

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
                        "use_sim_time": False,
                    }
                ],
            ),
        ]

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": False}],
    )

    map_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="local_test_map_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_planner_test",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": autostart},
            {"node_names": ["map_server", "planner_server"]},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_local_planner_test",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "maps", "exhibit_2d_map.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "local_planner_test_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "local_planner_test.rviz"]
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            OpaqueFunction(function=map_server_actions),
            map_to_base_link_tf,
            planner_server_node,
            lifecycle_manager,
            rviz_node,
        ]
    )
