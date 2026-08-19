# G1 机器人控制系统

面向 **Unitree G1 人形机器人**的一站式控制系统，覆盖自主导航、上半身示教、网页控制台三大场景。

系统由 **g1_base（导航底座）** + **g1_teach_v2（示教工具）** 两个核心模块组成，通过 Docker 容器化部署在机器人的 PC2 主控（Jetson NX）上，对外提供 `http://<机器人IP>:8081` 的网页控制台。

## 特性

- **自主导航**：Livox MID360 → Super-LIO 重定位 → Nav2 DWB 控制器 → 路线执行
- **示教工具**：连续动作录制、关键姿态快照、脚本编排、固件内置动作、Inspire FTP 灵巧手
- **网页控制台**：平板友好的仪表盘 UI，实时显示位姿/状态/点云/相机，支持地图管理与路线导航
- **DDS 双域架构**：Unitree SDK（Domain 0）与 ROS 2 导航栈（Domain 42）隔离，跨域桥接按需转发
- **容器化部署**：一条 `docker compose up -d` 启动，换版本秒级回滚，数据卷持久化

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC2 (Jetson NX)                          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Docker 容器 (network_mode: host)             │  │
│  │                                                           │  │
│  │  Domain 0 (Unitree SDK)          Domain 42 (ROS 2 Nav)   │  │
│  │  ┌──────────────┐                ┌────────────────────┐  │  │
│  │  │ Livox MID360 │──点云/IMU──→  │ Super-LIO 重定位    │  │  │
│  │  │ (激光雷达)    │                │ (NDT + ICP)        │  │  │
│  │  └──────────────┘                └────────┬───────────┘  │  │
│  │                                           │ /lio/robo/odom│  │
│  │  ┌──────────────┐                ┌────────▼───────────┐  │  │
│  │  │ Unitree SDK  │──里程计/电池─→ │ odom_to_tf          │  │  │
│  │  │ (电机/传感器) │                │ (TF + /odom_2d)     │  │  │
│  │  └──────────────┘                └────────┬───────────┘  │  │
│  │         │ dds_domain_bridge                │              │  │
│  │         │ battery_bridge                   │              │  │
│  │         │                                  ▼              │  │
│  │         │                          ┌───────────────┐      │  │
│  │         │                          │ Nav2 + DWB    │      │  │
│  │         │                          │ /cmd_vel      │      │  │
│  │         │                          └───────┬───────┘      │  │
│  │         │                                  ▼              │  │
│  │         │                          ┌───────────────┐      │  │
│  │         └──────────────────────────│ navigation_   │      │  │
│  │                                    │ manager       │      │  │
│  │                                    └───────┬───────┘      │  │
│  │                                            ▼              │  │
│  │  ┌────────────────┐  ┌──────────────┐  ┌──────────────┐  │  │
│  │  │ g1_web_bridge  │  │ g1_control_  │  │ nav_script   │  │  │
│  │  │ (网页控制台:8081)│  │ server       │  │ (路线执行)   │  │  │
│  │  └────────────────┘  └──────────────┘  └──────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## 硬件要求

| 组件 | 规格 |
|------|------|
| 机器人 | Unitree G1 人形机器人 |
| 主控 | PC2: NVIDIA Jetson NX (aarch64) |
| 操作系统 | Ubuntu 22.04 + JetPack 6.2 |
| 激光雷达 | Livox MID360 (3D LiDAR) |
| 网络接口 | `enP8p1s0`（默认网卡名） |

## 软件依赖

| 依赖 | 版本 |
|------|------|
| ROS 2 | Humble |
| DDS | CycloneDDS |
| Python | 3.10 (bt_env conda) |
| Docker | 24+ |
| Docker Compose | v2 |

> **注意**：必须在 **arm64** 机器上构建镜像（Jetson 本机）。在 x86 机器上 build 的镜像 Jetson 跑不了。

## 快速开始

### 1. 构建镜像

```bash
# 在 Jetson 上
cd /path/to/g1_teach_v2
docker build -t g1-base:$(git rev-parse --short HEAD) .
```

> 首次构建约 15-20 分钟（编译 Livox-SDK2 + Super-LIO + 四个 ROS 2 包）。
> 之后只改业务代码时只需重建最后几层，约 1-2 分钟。

### 2. 启动

```bash
G1_IMAGE_TAG=$(git rev-parse --short HEAD) docker compose up -d
```

可选环境变量：
- `G1_NET_IF`：网卡名（默认 `enP8p1s0`）
- `G1_WEB_PORT`：网页端口（默认 `8081`）
- `G1_DATA_HOST_DIR`：数据卷宿主机路径（默认 `/data/g1`）

### 3. 访问控制台

浏览器打开 `http://<机器人IP>:8081`

### 4. 回滚

```bash
# 秒级回滚到任意历史版本
G1_IMAGE_TAG=8e02547 docker compose up -d
```

### 5. 停止

```bash
docker compose down
```

## 项目结构

```
g1_teach_v2/
├── g1_base/                    # 导航底座（ROS 2 包）
│   ├── g1_base/                # Python 模块
│   │   ├── navigation_manager.py  # 底座编排入口：管理定位链 + Nav2 链
│   │   ├── nav_core.py            # 任务执行：Nav2 集成 + SDK 动作分发
│   │   ├── nav_script.py          # CLI：waypoint 路线执行
│   │   ├── g1_web_bridge.py       # 网页控制台网关（WebSocket + 静态文件）
│   │   ├── g1_control_server.py   # 控制服务（HTTP API）
│   │   ├── odom_to_tf.py          # 重定位里程计 → 2D TF + /odom_2d
│   │   ├── dds_domain_bridge.py   # DDS 跨域桥接（Domain 0 → 42）
│   │   ├── battery_bridge.py      # 电量桥接（Domain 0 → 42）
│   │   ├── cmd_vel_mock.py        # 干跑模拟：提供 TF 和里程计
│   │   ├── show_robot_pose.py     # 终端位姿查看 + 航点录制
│   │   ├── map_edit.py            # 地图编辑
│   │   ├── mapping_snapshotter.py # 建图快照
│   │   └── ...                    # 诊断/工具模块
│   ├── g1_base_interfaces/        # 自定义 ROS 2 srv/action 定义
│   ├── teach/                     # 旧版示教模块（g1_teach_v2 的前身）
│   ├── webapp/                    # 网页控制台前端（纯 JS，无框架）
│   │   ├── app.js                 # 主逻辑 + 路由 + 页面渲染
│   │   ├── style.css              # 仪表盘主题（日/夜自适应）
│   │   ├── robot3d.js             # Three.js 3D 机器人渲染
│   │   └── mapcloud.js            # 点云可视化
│   ├── config/                    # 配置文件
│   │   ├── nav2_params.yaml       # Nav2 主参数（DWB 控制器）
│   │   ├── maps/                  # 2D 栅格地图
│   │   ├── routes/                # 路线文件
│   │   ├── high_level_actions/    # 高级动作定义
│   │   └── walking_mode.yaml      # 行走模式
│   ├── launch/
│   │   └── navigation.launch.py   # Nav2 启动文件
│   ├── scripts/                   # 薄封装脚本
│   └── start_*.sh                 # 各组件启动脚本
│
├── g1_teach_v2/                # 示教工具（Python 包）
│   ├── cli.py                     # CLI 入口（argparse）
│   ├── record_modes.py            # 多种录制策略
│   ├── motion_io.py               # 动作录制/回放
│   ├── snapshot_io.py             # 快照捕获/移动
│   ├── script_runner.py           # 脚本执行引擎
│   ├── robot_io.py                # Unitree SDK 通信封装
│   ├── joints.py                  # 关节名/ID/分组
│   ├── profiles.py                # PD 增益配置
│   ├── hand_adapters.py           # Inspire FTP 灵巧手适配器
│   ├── movement/
│   │   ├── motions/               # 录制轨迹 (.jsonl)
│   │   ├── snapshots/             # 关键姿态 (.json)
│   │   └── scripts/               # 编排脚本 (.json)
│   └── music/                     # 背景音乐
│
├── g1_base_perception/         # C++ 感知节点
│   └── nav_obstacle_cloud_filter  # 导航障碍物点云滤波
│
├── g1_centerline_planner/      # C++ 局部规划器插件
│
├── third_party/                # 第三方依赖
│   ├── Livox-SDK2/             # Livox 雷达驱动
│   ├── Super-LIO/              # 3D LiDAR-inertial SLAM
│   └── livox_ros_driver2/      # Livox ROS 2 驱动
│
├── docker/                     # 部署配置
│   ├── start_g1_base.sh        # 容器主进程（四件套 + 网页网关）
│   ├── rosenv                  # docker exec ROS 环境包装
│   └── lio/                    # Super-LIO 配置
│       ├── MID360_config.json  # MID360 网络配置
│       ├── livox_360.yaml      # 建图参数
│       └── relocation_360.yaml # 重定位参数
│
├── Dockerfile                  # arm64 镜像构建
├── docker-compose.yml          # 容器编排
└── scripts/                    # 构建脚本
```

## 核心模块

### g1_base — 导航底座

#### 进程架构

容器启动时 `start_g1_base.sh` 拉起四件套：

| 进程 | 角色 | 挂了会怎样 |
|------|------|-----------|
| `navigation_manager` | 底座编排入口，管理定位链 + Nav2 | 整体退出，docker restart |
| `dds_domain_bridge` | DDS 跨域桥接（点云/IMU/里程计） | 整体退出，docker restart |
| `g1_control_server` | HTTP 控制服务 | 整体退出，docker restart |
| `g1_web_bridge` | 网页控制台网关 | 自动重启，不影响导航 |
| `battery_bridge` | 电量跨域桥接 | 忽略，网页回退"未接入" |

#### DDS 双域架构

G1 的 Unitree SDK 使用 DDS Domain 0，而 ROS 2 导航栈使用 Domain 42。两个域完全隔离，避免话题冲突。跨域数据通过专门的桥接节点转发：

- **dds_domain_bridge**：fork 子进程在 Domain 0 订阅，父进程在 Domain 42 发布。桥接三个话题：
  - `/utlidar/cloud_livox_mid360` → `/lio/cloud_world`（点云）
  - `/utlidar/imu_livox_mid360` → `/livox/imu`（IMU）
  - `/dog_odom` → `/lio/robo/odom`（里程计）

- **battery_bridge**：同样的 fork 架构。Domain 0 的 `/lf/battery_alarm`（std_msgs/String, JSON）→ Domain 42 的 `/battery_state`（sensor_msgs/BatteryState）。电量从 13 节电芯电压反算 SOC。

#### 重定位

Super-LIO 重定位流程：
1. 加载预存 PCD 全局地图
2. 订阅 `/initialpose` 获取初始位姿猜测（默认 [0,0,0,0,0,0]）
3. NDT + ICP scan-to-map 匹配
4. 收敛后发布 `/lio/robo/odom` + `/lio/cloud_world`

> **注意**：这是"基于初值的局部精匹配"，不是任意位置开机就能定位的全局重定位。机器人需要站在建图起点附近。

#### 话题与服务

状态发布：
- `/navigation_manager/ready` — 底座就绪状态
- `/navigation_manager/state` — 当前状态机状态
- `/navigation_manager/detail` — 详细诊断信息

管理服务：
- `/navigation_manager/ensure_ready` — 确保就绪
- `/navigation_manager/restart_all` — 重启全部
- `/navigation_manager/stop_all` — 停止全部

导航链路：
- `/lio/cloud_world` → `/odom_2d` → `/cmd_vel` → Unitree SDK

### g1_teach_v2 — 示教工具

命令行工具包，支持连续录制 / 回放 / 脚本编排。详见 [g1_teach_v2/README.zh.md](./g1_teach_v2/README.zh.md)。

#### 工作流

```
录制动作 (record-motion)
    ↓
捕获快照 (capture-snapshot)
    ↓
分别验证 (play-motion / goto-snapshot)
    ↓
组合成脚本 (scripts/*.json)
    ↓
执行脚本 (run-script)
```

#### 支持的步骤类型

| 类型 | 说明 |
|------|------|
| `snapshot` | 平滑移动到关键姿态 |
| `motion` | 回放连续动作轨迹 |
| `hold` | 保持当前姿态指定时长 |
| `hand` | Inspire FTP 灵巧手指令（预设或目标值） |
| `audio` | 播放 WAV/MP3 音频（异步或同步） |
| `script` | 嵌套调用另一个脚本 |

#### 固件内置动作

```bash
python -m g1_teach_v2 arm-action --list          # 查询可用动作
python -m g1_teach_v2 arm-action --name clap      # 按名称执行
python -m g1_teach_v2 arm-action --custom my_wave  # 执行 APP 示教动作
```

### g1_base_perception — 感知

C++ 实现的 `nav_obstacle_cloud_filter` 节点，对点云做导航相关的滤波处理。

### g1_centerline_planner — 路径规划

C++ 实现的 Nav2 局部规划器插件，基于中心线规划策略。

## 容器内调试

### rosenv 包装器

`docker exec` 不走容器的 entrypoint，ROS 环境是空的。用 `rosenv` 包装器解决：

```bash
# 查看话题列表
docker exec g1-base rosenv ros2 topic list

# 查看节点
docker exec g1-base rosenv ros2 node list

# 订阅状态
docker exec g1-base rosenv ros2 topic echo /navigation_manager/detail
```

`rosenv` 从 `/proc/1/environ` 拷贝运行期环境变量（ROS_DOMAIN_ID, CYCLONEDDS_URI），然后 source 三层 setup.bash。

### 在 Domain 0 上调试

Unitree SDK 话题在 Domain 0 上，需要手动设置环境：

```bash
docker exec -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp g1-base bash -c \
  'source /opt/ros/humble/setup.bash && ros2 topic list'
```

> **注意**：容器内默认 python 是 miniconda 3.14，rclpy 需要 Python 3.10。运行自定义脚本时需要过滤 PATH：
> ```bash
> export PATH=$(echo $PATH | tr ":" "\n" | grep -v miniconda | paste -sd: -)
> ```

## 开发与调试

### 快速预览前端改动

webapp 是纯静态文件，可以 `docker cp` 覆盖到容器里立即生效，不用重建镜像：

```bash
docker cp g1_base/webapp/app.js    g1-base:/root/g1_ws/install/g1_base/share/g1_base/webapp/app.js
docker cp g1_base/webapp/style.css g1-base:/root/g1_ws/install/g1_base/share/g1_base/webapp/style.css
```

刷新浏览器即可看到效果。

### 重新构建镜像

```bash
# 业务代码改动（只重建最后几层，~1-2 分钟）
docker build -t g1-base:$(git rev-parse --short HEAD) .

# 重启
docker compose down
G1_IMAGE_TAG=$(git rev-parse --short HEAD) docker compose up -d
```

### 常用诊断命令

```bash
# 容器日志
docker compose logs -f

# 查看进程
docker exec g1-base ps aux

# 查看电量桥接日志
docker exec g1-base cat /tmp/battery_bridge.log

# 查看 DDS 桥接日志
docker exec g1-base cat /tmp/bridge.log

# 验证电量话题
docker exec g1-base rosenv ros2 topic echo --once /battery_state

# 检查位姿
docker exec g1-base rosenv ros2 run g1_base show_robot_pose --once
```

## 数据持久化

容器通过 Docker volumes 持久化以下数据：

| 容器路径 | 宿主机路径 | 内容 |
|----------|-----------|------|
| `/data` | `/data/g1` | 地图、路线、示教动作 |
| `/root/logs` | `/data/g1_logs` | 运行日志 |

容器重建不会丢失现场建的图和路线。Super-LIO 的存图目录通过软链指向数据卷。

容器首次启动时，会把包内自带的出厂 routes / maps / movement 数据"播种"到空的数据卷（不覆盖已有文件）。

## 常见问题

### `ros2: command not found`

ros2 装在容器里，宿主机没有。使用 `rosenv` 包装器：
```bash
docker exec g1-base rosenv ros2 topic list
```

### 重定位位姿跳变

通常是多个重复的 `odom_to_tf` 或 `relocation_node` 进程在打架。干净重启：
```bash
docker compose down && docker compose up -d
```

### 网页显示"导航栈未就绪"

检查话题链路：
```bash
docker exec g1-base rosenv ros2 topic echo --once /navigation_manager/detail
```
确认 `/lio/cloud_world` 和 `/lio/robo/odom` 有数据，`map → base_link` TF 已建立。

### 电量显示"未接入"

1. 确认 battery_bridge 在跑：`docker exec g1-base cat /tmp/battery_bridge.log`
2. 确认 Domain 0 上有电池数据：
   ```bash
   docker exec -e ROS_DOMAIN_ID=0 g1-base bash -c \
     'source /opt/ros/humble/setup.bash && ros2 topic hz /lf/battery_alarm'
   ```
3. 确认 Domain 42 上有 BatteryState：
   ```bash
   docker exec g1-base rosenv ros2 topic echo --once /battery_state
   ```

### Python ModuleNotFoundError: rclpy

容器默认 python 是 miniconda 3.14，rclpy 需要 Python 3.10。过滤 PATH：
```bash
export PATH=$(echo $PATH | tr ":" "\n" | grep -v miniconda | paste -sd: -)
```

## 相关文档

| 文档 | 说明 |
|------|------|
| [g1_base/CLAUDE.md](./g1_base/CLAUDE.md) | 导航底座的开发指南 |
| [g1_base/DEPLOY.md](./g1_base/DEPLOY.md) | 底座部署与运行说明 |
| [g1_base/new_readme.md](./g1_base/new_readme.md) | 导航栈话题与服务说明 |
| [g1_base/PARAMETERS.md](./g1_base/PARAMETERS.md) | 参数文件说明 |
| [g1_teach_v2/README.zh.md](./g1_teach_v2/README.zh.md) | 示教工具完整手册 |
| [g1_teach_v2/STRUCTURE.zh.md](./g1_teach_v2/STRUCTURE.zh.md) | 示教工具代码结构 |
| [docker/lio/relocation_360.yaml](./docker/lio/relocation_360.yaml) | 重定位参数 |
| [docker/lio/livox_360.yaml](./docker/lio/livox_360.yaml) | 建图参数 |
