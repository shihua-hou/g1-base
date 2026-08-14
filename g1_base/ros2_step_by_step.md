# G1 Base 操作手册

## 1. 编译

```bash
cd /home/unitree/g1_base
bash build.sh
```

或者手动分步执行：

```bash
# ① 编译 g1_base_interfaces + g1_centerline_planner（CMake 包，必须在 g1_ws 中编译）
cd ~/g1_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select g1_base_interfaces g1_centerline_planner

# ② 编译 g1_base（Python 主包）
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env
colcon build --symlink-install --packages-up-to g1_base
source install/setup.bash
```

> **为什么需要两步？** `g1_base_interfaces` 和 `g1_centerline_planner` 都是
> `ament_cmake` 包，需要在 `~/g1_ws` 中编译；`g1_base` 是 `ament_python`
> 主包，在 `/home/unitree/g1_base` 中编译。`build.sh` 会自动检查
> `~/g1_ws/src/g1_base_interfaces` 和 `~/g1_ws/src/g1_centerline_planner`，
> 如果能从源码位置找到包，会自动补 symlink。
>
> `robot_env.sh` 的 `load_ros_env` 会依次 source ROS、Super-LIO、`g1_base`、
> `g1_ws`（接口包 + 中线规划插件）、CycloneDDS 等环境，并设置 `ROS_DOMAIN_ID=42`。
> `--symlink-install` 可以减少 Python 源码和 `install/` 安装副本不一致导致的旧代码问题。

## 2. 启动导航

```bash
# 终端 A — 启动底层（保持运行）
cd ~/g1_base
source /opt/ros/humble/setup.bash
source install/setup.bash
bash start_navigation_manager.sh

# 终端 B — 启动导航脚本
cd ~/g1_base
source /opt/ros/humble/setup.bash
source install/setup.bash
export G1_SDK_PYTHON_BIN=/home/unitree/miniconda3/envs/py310/bin/python
ros2 run g1_base nav_script --net-if enP8p1s0 --route ./config/routes/waypoint_2.yaml
```

> nav_script 不会自己启动 navigation_manager。
> 如果只运行 nav_script，它会卡在"等待 navigation_manager 服务..."。

## 3. 建图

建图和导航互斥。通过 navigation_manager 服务切换模式。

### 3.1 通过 bot_mind MCP 调用（推荐）

```bash
# 开始建图（停止导航栈，启动 Super-LIO SLAM）
bash scripts/mcp_cli.sh start_mapping

# 人遥控机器人走完地图后，停止建图（自动保存 map.pcd）
bash scripts/mcp_cli.sh stop_mapping

# Super-LIO 原 PCD: /home/unitree/ros2_ws/src/Super-LIO/src/super_lio/map/map.pcd
# g1_base 落盘产物: ~/g1_maps/<base>_map.pcd, <base>_exhibit_2d_map.{pgm,yaml}
# 当前最新地图指针:  ~/g1_maps/current_map.json
```

### 3.2 通过 ros2 service 直接调用

```bash
ros2 service call /navigation_manager/start_mapping std_srvs/srv/Trigger
# 走完地图后
ros2 service call /navigation_manager/stop_mapping std_srvs/srv/Trigger
# 可选：单独触发一次 PCD → 2D 转换
ros2 service call /navigation_manager/generate_2d_map std_srvs/srv/Trigger
```

### 3.3 建图自动化流程（已无需人工后处理）

1. `start_mapping` 时生成 `base_name=YYYYMMDD_HHMMSS`，贯穿全部产物命名
2. 建图过程中每 20s 自动刷新 `<base>_exhibit_2d_map.{pgm,yaml}`（同名覆盖）
3. `stop_mapping` 时自动从 `map.pcd` 重生成最终 2D 图，覆盖快照
4. `~/g1_maps/current_map.json`（`$G1_MAPS_DIR`）是当前最新地图指针，bot_mind 读它
5. 仅保留最近 100 份地图三件套（pgm/yaml/pcd），其余自动清理
6. 重启导航：`ros2 service call /navigation_manager/ensure_ready std_srvs/srv/Trigger`

## 4. 停止所有进程

```bash
pkill -KILL -f 'bt_navigator|controller_server|planner_server|behavior_server|velocity_smoother|smoother_server|waypoint_follower|pc_to_laserscan|map_server|lifecycle_manager_navigation|lifecycle_manager_map|relocation_node|livox_lidar_publisher|odom_to_tf|navigation_manager|multi_waypoint_nav|nav_script' || true
```

## 5. RViz 远程查看（Docker）

```bash
# 准备持久化目录
mkdir -p $HOME/.rviz2_docker/rviz
mkdir -p $HOME/.rviz2_docker/qt
xhost +local:docker

# 启动 RViz 容器
docker run -it --rm \
    --name rviz2 \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -e ROS_LOCALHOST_ONLY=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.rviz2_docker/rviz:/root/.rviz2 \
    -v $HOME/.rviz2_docker/qt:/root/.config/ros.org \
    osrf/ros:humble-desktop \
    bash -c "source /opt/ros/humble/setup.bash && rviz2"
```

> RViz 配置保存在宿主机 `$HOME/.rviz2_docker/` 下。
> 首次调好布局后，在 RViz 里 File → Save Config As，保存到 `/root/.rviz2/default.rviz`。

### 5.0.1 X11 转发

```bash
# 1. SSH 连过去
ssh -Y unitree@192.168.2.155

# 2. source 环境（通过 robot_env.sh 一步搞定）
cd ~/g1_base
source ./config/robot_env.sh
load_ros_env

# 3. 启动 rviz2
rviz2
```


### 5.1 手动进入容器排查版

PC2 上先确认底层导航状态：

```bash
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env

# 如果 ros2 topic/node 命令报 !rclpy.ok()，先停掉卡住的 ros2 CLI daemon。
ros2 daemon stop || true

ros2 node list --no-daemon | grep -E 'navigation_manager|relocation_node|bt_navigator|controller_server|planner_server'
ros2 topic info -v /lio/cloud_world --no-daemon
ros2 topic echo --once --full-length /navigation_manager/detail --no-daemon

# 如果 /lio/cloud_world 没有 publisher，重新拉起底层导航链路。
ros2 service call /navigation_manager/ensure_ready std_srvs/srv/Trigger "{}"
```

```bash
# 第一步：准备持久化目录
mkdir -p $HOME/.rviz2_docker/rviz
mkdir -p $HOME/.rviz2_docker/qt
xhost +local:docker

# 第二步：进入容器
# PC2 使用 ROS_DOMAIN_ID=42；ROS_LOCALHOST_ONLY 必须为 0 才能跨机器发现。
docker run -it --rm \
    --name rviz2 \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -e ROS_LOCALHOST_ONLY=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.rviz2_docker/rviz:/root/.rviz2 \
    -v $HOME/.rviz2_docker/qt:/root/.config/ros.org \
    osrf/ros:humble-desktop \
    bash

# 第三步：容器内先确认能看到底层导航，再启动 RViz
source /opt/ros/humble/setup.bash
ros2 node list --no-daemon | grep -E 'navigation_manager|bt_navigator|controller_server|planner_server'
ros2 topic echo --once /navigation_manager/ready
ros2 topic echo --once /navigation_manager/detail
rviz2
```

> RViz 不直接显示 ROS 2 节点列表；节点关系用 `ros2 node list` / `rqt_graph` 看。
> 如果 RViz 画面为空，先把 `Global Options -> Fixed Frame` 设为 `map` 或 `world`，并把 `/map`、`/global_costmap/costmap` 的 `Durability Policy` 改为 `Transient Local`。
> 当前 `osrf/ros:humble-desktop` 容器默认没有 `rmw_cyclonedds_cpp`、`nav2_msgs` 和 `livox_ros_driver2`，能做基础 RViz 可视化；如果要完全对齐 PC2，需要另外构建带这些包的 RViz 镜像。

### 5.2 本次已验证的 PC2 / MCP 命令

PC2 终端加载 `g1_base` 环境：

```bash
cd /home/unitree/g1_base
source ./config/robot_env.sh
load_ros_env

ros2 pkg prefix g1_base
ros2 pkg executables g1_base | grep show_robot_pose
```

如果 `ros2 topic/node/param` 报 `!rclpy.ok()`，先停掉卡住的 ROS 2 CLI daemon：

```bash
ros2 daemon stop || true
```

确认底层导航与点云状态：

```bash
ros2 node list --no-daemon | grep -E 'navigation_manager|relocation_node|bt_navigator|controller_server|planner_server'
ros2 topic info -v /lio/cloud_world --no-daemon
ros2 topic echo --once --full-length /navigation_manager/detail --no-daemon
ros2 service call /navigation_manager/ensure_ready std_srvs/srv/Trigger "{}"
```

查看当前位置 / 采点：

```bash
ros2 run g1_base show_robot_pose --once
ros2 run g1_base show_robot_pose --record-yaml ./config/routes/gy_recorded_waypoints.yaml
sed -n '1,120p' ./config/routes/gy_recorded_waypoints.yaml
```

确认当前 Nav2 2D 地图文件：

```bash
ros2 param get /map_server yaml_filename
cat /home/unitree/g1_maps/current_map.json
ls -lh /home/unitree/g1_maps/*_exhibit_2d_map.*
```

bot_mind MCP 基础检查：

```bash
cd /home/unitree/bot_mind
bash scripts/mcp_cli.sh list
bash scripts/mcp_cli.sh waypoint
bash scripts/mcp_cli.sh call navigation_manager '{"action":"status"}'
```

如果 `navigation_manager` 存活但状态是 `DEGRADED_NAVIGATION`，并且日志中出现：

```text
package 'g1_navigation' not found
```

说明 Nav2 参数文件还在引用旧 ROS 包名。当前包名是 `g1_base`，行为树 XML 安装在
`g1_base/behavior_trees/` 下。修复 `config/nav2_params.yaml` 后重新构建并重启：

```yaml
default_nav_to_pose_bt_xml: "$(find-pkg-share g1_base)/behavior_trees/navigate_to_pose_forward_only.xml"
default_nav_through_poses_bt_xml: "$(find-pkg-share g1_base)/behavior_trees/navigate_through_poses_forward_only.xml"
```

该错误会让 `navigation.launch.py` 在 Nav2 启动阶段直接退出，`map_server` 不会稳定发布
`/map`，RViz 里看到的 map topic 异常通常是这个后果，而不是地图文件本身先坏了。

如果改了 `/home/unitree/bot_mind/config/waypoints/waypoints.yaml`，重启 bot_mind 让点位重新加载：

```bash
pkill -f bot-mind
cd /home/unitree/bot_mind
nohup bash scripts/start-bot-mind.sh > logs/bot_mind.manual.log 2>&1 &
sleep 5
bash scripts/mcp_cli.sh list | grep navigate_to
bash scripts/mcp_cli.sh waypoint
```

确认场地安全后，再通过 MCP 发真实导航：

```bash
cd /home/unitree/bot_mind
bash scripts/mcp_cli.sh nav 起点
# 或：
bash scripts/mcp_cli.sh call navigate_to '{"waypoint_name":"起点"}'
```

如果 MCP 返回 `未发现 navigation_manager，请先启动底层导航系统`，先看 bot_mind 是否把
`g1_base_manager` 子进程拉起后又崩掉：

```bash
tail -n 120 /home/unitree/bot_mind/logs/navigation_manager.log
```

