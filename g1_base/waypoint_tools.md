# Waypoint Tools

## 采点

进入记录模式：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base show_robot_pose --record-yaml ./config/routes/recorded_waypoints.yaml
```

按键说明：
- `r`
- 空格
- 回车

以上三种按键都会记录当前 `x / y / yaw`

退出并保存：
- `q`

## 查看当前位置
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base show_robot_pose --once
```

持续输出：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base show_robot_pose --rate 2.0
```

## 路线可视化
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_base publish_waypoints_to_rviz --yaml ./config/routes/recorded_waypoints.yaml
```

建议在 RViz 中添加：
- `MarkerArray`
- `Path`

## 路线文件格式
```yaml
route_name: default
waypoints:
  - x: 1.0
    y: 2.0
    yaw_deg: 90.0
    action_id: 31
    say_text: 到达指定地点
```

## 实机运行建议
- 先启动 `start_navigation_manager.sh`
- 确认 `/navigation_manager/ready` 为 `true`
- 再采点或执行任务
