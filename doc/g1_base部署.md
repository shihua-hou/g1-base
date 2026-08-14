# g1_base 部署说明

以下命令均在机器人 PC2 上执行：

```bash
ssh unitree@192.168.2.189
cd /home/unitree/g1_base
```

## 1. g1_base 职责

`g1_base` 是 Unitree G1 机器人的 ROS 2 底座服务，主要负责：

- 启动并监控定位链路：Livox MID360、Super-LIO、里程计、TF。
- 启动 Nav2 导航链路，输出 `/cmd_vel`。
- 对外提供导航、急停、移动、转向、手臂动作等 ROS 2 接口。
- 向 `bot_mind` 上报底座状态，由 `bot_mind` 负责业务编排和网页交互。

常用状态：

- `/navigation_manager/ready`：底座是否就绪。
- `/navigation_manager/state`：当前状态。
- `/navigation_manager/detail`：详细排错信息。
- `/g1_control/status`：运动/动作执行状态。

## 2. 编译

确认代码目录存在：

```bash
ls /home/unitree/g1_base/package.xml
ls /home/unitree/g1_base_perception/package.xml
ls /home/unitree/g1_centerline_planner/package.xml
```

编译：

```bash
cd /home/unitree/g1_base
bash ./build.sh
```

编译完成后，加载运行环境：

```bash
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env
ros2 pkg list | grep -E 'g1_base|g1_base_interfaces|g1_base_perception|g1_centerline_planner'
```

## 3. 启动

### 推荐方式：由 bot_mind 托管

现场正常运行时，`bot_mind` 会自动拉起 `g1_base_manager`。重启方式：

```bash
sudo systemctl restart bot-mind
sudo systemctl status bot-mind --no-pager
sudo journalctl -u bot-mind -f
```

### 手工启动 g1_base

仅用于现场联调或排错。终端 A 启动底座：

```bash
cd /home/unitree/g1_base
bash ./start-g1-base-manager.sh --net-if enP8p1s0
```

终端 B 查看底座是否就绪：

```bash
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env
ros2 topic echo --once /navigation_manager/ready
ros2 topic echo --once /navigation_manager/detail
```

终端 C 执行一条路线：

```bash
cd /home/unitree/g1_base
bash ./start_nav_script.sh --net-if enP8p1s0 --route /home/unitree/g1_base/config/routes/waypoint_1.yaml
```

干跑验证 Nav2：

```bash
cd /home/unitree/g1_base
bash ./start_navigation.sh --dry-run
```

## 4. 排错

查看日志：

```bash
sudo journalctl -u bot-mind -f
tail -f /home/unitree/bot_mind/logs/navigation_manager.log
tail -f /home/unitree/bot_mind/logs/g1_base.log
```

检查 ROS 2 环境：

```bash
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
ros2 node list
```

底座一直未就绪：

```bash
ros2 topic echo --once /navigation_manager/detail
ros2 topic hz /lio/cloud_world
ros2 topic hz /lio/robo/odom
ros2 run tf2_ros tf2_echo map base_link
ros2 action list | grep navigate_to_pose
```

机器人不执行运动或动作：

```bash
ip addr show enP8p1s0
ros2 topic echo --once /g1_control/status
```

地图或路线异常：

```bash
ros2 param get /map_server yaml_filename
ros2 run g1_base show_robot_pose --once
ls -l /home/unitree/g1_base/config/routes/
```

常见处理：

- `ready=false`：优先看 `/navigation_manager/detail`，再检查 LIO 点云、里程计、TF 和 Nav2 action。
- SDK 初始化失败：确认 `--net-if enP8p1s0` 与 PC2 实际网卡一致。
- 地图不对：确认 `/map_server` 当前加载的 `yaml_filename`。
- 进程状态异常：先重启 `bot-mind`，再观察日志。