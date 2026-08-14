# Deployment

## Build
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start Bottom Stack
```bash
bash ./start_navigation_manager.sh
```

## Run Mission
```bash
ros2 run g1_base nav_script --net-if enP8p1s0 --route ./config/routes/waypoint_1.yaml
```

## Dry Run
```bash
bash ./start_navigation.sh --dry-run
```

## Verify
```bash
ros2 topic echo --once /navigation_manager/detail
ros2 action list | grep navigate_to_pose
ros2 run g1_base show_robot_pose --once
```
