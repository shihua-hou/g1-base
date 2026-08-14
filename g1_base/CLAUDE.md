# CLAUDE.md

This file provides guidance for coding agents working in this repository.

## Project Overview

G1 Navigation System for the Unitree G1 robot. The current stack targets Ubuntu 22.04, ROS 2 Humble, Python 3.10, and JetPack 6.2.
注意：本机作为开发的地方，系统是ubuntu25

## Architecture

Two-layer design:

1. Bottom layer: MID360 -> Super-LIO relocalization -> `odom_to_tf` -> Nav2 -> `/cmd_vel`
2. Top layer: `nav_script` -> `nav_core` -> Unitree SDK motion, audio, and arm interactions

`navigation_manager` is the orchestration entrypoint. It publishes:
- `/navigation_manager/ready`
- `/navigation_manager/state`
- `/navigation_manager/detail`

It also serves:
- `/navigation_manager/ensure_ready`
- `/navigation_manager/restart_all`
- `/navigation_manager/stop_all`

## Key Files

- `g1_base/navigation_manager.py` — bottom stack orchestration and health checks
- `g1_base/nav_core.py` — mission execution, Nav2 integration, SDK motion dispatch
- `g1_base/nav_script.py` — CLI entrypoint for waypoint missions
- `g1_base/odom_to_tf.py` — converts relocalization odometry into 2D TF + `/odom_2d`
- `g1_base/cmd_vel_mock.py` — dry-run TF and odometry simulator
- `g1_base/show_robot_pose.py` — terminal pose viewer and waypoint recorder
- `g1_base/publish_waypoints_to_rviz.py` — RViz marker publisher for waypoint routes
- `g1_base/diag_obstacle.py` — runtime diagnostics for point cloud, scan, local costmap, path, and command flow
- `launch/navigation.launch.py` — map server, static TF, pointcloud-to-laserscan, Nav2
- `config/nav2_params.yaml` — Nav2 controller, planner, behavior, and costmap configuration

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Bottom stack:
```bash
bash ./start_navigation_manager.sh
```

Mission execution:
```bash
ros2 run g1_base nav_script --net-if enP8p1s0 --route ./config/routes/waypoint_1.yaml
```

Dry run:
```bash
bash ./start_navigation.sh --dry-run
```

## Notes for Changes

- Prefer editing the `g1_base/` package modules over the thin wrappers in `scripts/`
- Keep route YAML format backward compatible
- Default Unitree network interface is `enP8p1s0`
- If upstream Super-LIO or Livox topic names change, adapt through parameters before changing business logic
