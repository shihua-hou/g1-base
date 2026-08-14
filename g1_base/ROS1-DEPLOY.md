# 部署指南

## 说明
- 文件名保留不变是为了兼容现有仓库入口
- 当前内容已经切换为 ROS 2 Humble 部署流程

## 部署到机器人
```bash
rsync -avz --delete /home/lemon/vscode_projects/g1_base/ unitree@192.168.10.99:/home/unitree/g1_base/
```

## 机器人端准备
```bash
cd ~/g1_base
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动方式

终端 A：
```bash
cd ~/g1_base
source /opt/ros/humble/setup.bash
source install/setup.bash
bash ./start_navigation_manager.sh
```

终端 B：
```bash
cd ~/g1_base
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base nav_script --net-if enP8p1s0 --route ./config/routes/waypoint_1.yaml
```

## 启动链路
```text
navigation_manager
  ├─ start_pc2_localization.sh
  │   ├─ ros2 launch super_lio relocation.launch.py
  │   └─ ros2 launch livox_ros_driver2 msg_MID360_launch.py
  └─ start_navigation.sh
      ├─ ros2 run g1_base odom_to_tf
      └─ ros2 launch g1_base navigation.launch.py
          ├─ nav2_map_server
          ├─ static_transform_publisher
          ├─ pointcloud_to_laserscan
          └─ Nav2 navigation stack
```

## 检查项
- `ros2 topic echo --once /lio/cloud_world`
- `ros2 topic echo --once /lio/robo/odom`
- `ros2 topic echo --once /odom_2d`
- `ros2 action list | grep navigate_to_pose`
- `ros2 topic echo --once /navigation_manager/detail`
