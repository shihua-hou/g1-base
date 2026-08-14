# ROS 2 运行说明

## 架构概览
- `start_navigation_manager.sh`
  - 运行 `ros2 run g1_base navigation_manager --auto-ensure`
  - 负责管理定位链与 Nav2 链路
- `start_pc2_localization.sh`
  - 启动 Super-LIO 与 Livox ROS 2 launch
  - 进程存活由脚本守护，话题 ready 由 `navigation_manager` 订阅判断
- `start_navigation.sh`
  - 启动 `odom_to_tf` 或 `cmd_vel_mock`
  - 启动 `navigation.launch.py`
- `nav_script`
  - 通过 Nav2 `navigate_to_pose` 执行 waypoint 路线
  - 负责到点停稳、播报、动作执行与清图恢复

## 话题与服务

底层状态：
- `/navigation_manager/ready`
- `/navigation_manager/state`
- `/navigation_manager/detail`

管理服务：
- `/navigation_manager/ensure_ready`
- `/navigation_manager/restart_all`
- `/navigation_manager/stop_all`

导航相关：
- `/lio/cloud_world`
- `/lio/robo/odom`
- `/odom_2d`
- `/scan`
- `/cmd_vel`
- `/cmd_vel_executed`
- `/motion_source`
- `navigate_to_pose`

## 构建
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 标准启动顺序

终端 A：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash ./start_navigation_manager.sh
```

终端 B：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base nav_script --net-if enP8p1s0 --route ./config/routes/waypoint_1.yaml
```

## 仅启动底层链路
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /navigation_manager/ensure_ready std_srvs/srv/Trigger "{}"
```

## 干跑测试
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash ./start_navigation.sh --dry-run
```

干跑时：
- 用 `cmd_vel_mock` 提供 `/odom_2d`
- 用 RViz 发送目标点
- 观察 `/cmd_vel` 与 `/cmd_vel_executed`

## 地图与参数
- `config/nav2_params.yaml`：Nav2 主参数文件，控制器使用 DWB
- `config/maps/exhibit_2d_map.yaml`：默认占位地图
- `config/routes/*.yaml`：路线文件

## 排障建议
- `/navigation_manager/ready` 长时间为 `false`
  - 先检查 `/lio/cloud_world`、`/lio/robo/odom` 是否有数据
  - 再检查 `map -> base_link` TF 是否建立
  - 最后检查 `navigate_to_pose` action 是否出现
- `nav_script` 无法初始化 SDK
  - 检查 `--net-if` 是否正确
  - 默认值已经切换为 `enP8p1s0`
- 路线可视化
  - `ros2 run g1_base publish_waypoints_to_rviz --yaml ./config/routes/recorded_waypoints.yaml`
- 位置采点
  - `ros2 run g1_base show_robot_pose --record-yaml ./config/routes/recorded_waypoints.yaml`
