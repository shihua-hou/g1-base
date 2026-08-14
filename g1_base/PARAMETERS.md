# G1 Navigation 可调参数总表

## 0. 说明

- 本文基于当前仓库代码整理，覆盖当前项目里“对行为有影响、且你实际能改”的参数。
- 覆盖范围：命令行参数、ROS 2 launch 参数、ROS 节点参数、环境变量、路线 YAML、`config/nav2_params.yaml`、以及少量常用硬编码调优项。
- 不覆盖：ROS 2 通用 `--ros-args` 的全部官方选项、第三方依赖包内部未在本仓库显式配置的参数、纯内部状态枚举和日志字符串。
- 调参优先级建议：
  1. 临时试验：命令行参数
  2. 本次会话：`export XXX=...`
  3. 长期默认值：改 `config/*.yaml`
  4. 深度调优：改 `launch/navigation.launch.py` 或 `g1_base/*.py`

所有使用 `parse_known_args()` 的脚本，都支持标准 ROS 2 参数透传。例如：

```bash
ros2 run g1_base odom_to_tf --ros-args -p input_odom_topic:=/my_odom
```

---

## 1. 通用运行环境变量

来源：`config/robot_env.sh`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `G1_BASE_ROOT` | 仓库根目录 | 项目根路径 | 现有目录；建议保持为仓库根目录 |
| `G1_MAPS_DIR` | `$G1_USER_HOME/g1_maps` | 建图产物目录，包含 `current_map.json`、PGM、YAML、PCD | 现有可写目录；bot_mind 和 g1_base 必须一致；建议放在代码目录之外 |
| `G1_USER_HOME` | `$HOME` | 用户主目录 | 现有目录 |
| `LIO_WORKSPACE_ROOT` | `$G1_USER_HOME/ros2_ws` | Super-LIO/Livox 工作空间根目录 | 现有 ROS 2 工作空间目录 |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` | ROS 2 环境脚本 | 现有 `setup.bash` 文件 |
| `LIO_SETUP` | `$LIO_WORKSPACE_ROOT/install/setup.bash` | LIO 工作空间环境脚本 | 现有 `setup.bash` 文件 |
| `WORKSPACE_SETUP` | `$G1_BASE_ROOT/install/setup.bash` | 本项目工作空间环境脚本 | 现有 `setup.bash` 文件 |
| `CYCLONEDDS_HOME` | `$G1_USER_HOME/cyclonedds/install` | CycloneDDS 安装目录 | 现有目录；若目录下有 `lib` 会自动加入 `LD_LIBRARY_PATH` |
| `ROS_DOMAIN_ID` | `42` | ROS 2 域 ID，所有相关进程必须一致 | 整数；常用 `0-232`；同一网络内避免冲突 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 中间件实现 | 通常保持 `rmw_cyclonedds_cpp` |
| `ROS_LOCALHOST_ONLY` | `0` | 是否只允许本机通信 | `0` 或 `1`；实机分布式一般用 `0` |
| `G1_DDS_INTERFACES` | `enP8p1s0,wlp3s0` | CycloneDDS 绑定的网卡列表；`robot_env.sh` 会自动忽略不存在的网卡（指定的网卡若不存在直接跳过，不再做 `en*`/`wl*` 同前缀回退，以免把 WiFi 之类无关网卡静默拉进 DDS） | 非空网卡名；多个网卡可逗号分隔，如 `enP8p1s0,wlan0`；建议只列实际承载 ROS 流量的网卡 |
| `CYCLONEDDS_URI` | 自动根据 `G1_DDS_INTERFACES` 生成（`AllowMulticast=spdp`，仅保留参与者发现多播） | 手动覆盖 CycloneDDS XML 配置 | 合法 XML 字符串；只有在你明确要覆盖自动网卡选择或多播策略时再手动设置 |

---

## 2. 定位链启动参数

来源：`start_pc2_localization.sh`

### 2.1 环境变量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `SUPER_LIO_PACKAGE` | `super_lio` | Super-LIO 包名 | 已安装 ROS 包名 |
| `SUPER_LIO_LAUNCH` | `relocation.py` | Super-LIO 启动文件 | 包内存在的 launch 文件 |
| `LIVOX_PACKAGE` | `livox_ros_driver2` | Livox 驱动包名 | 已安装 ROS 包名 |
| `LIVOX_LAUNCH` | `msg_MID360_launch.py` | Livox 启动文件 | 包内存在的 launch 文件 |
| `RVIZ_FLAG` | `false` | 是否让 Super-LIO launch 自带 RViz | `true/false`；无图形环境建议 `false` |

`start_pc2_localization.sh` 只负责拉起/清理进程，不再用 `ros2 topic echo`
等待话题。点云和里程计 ready 判断由 `navigation_manager` 通过 rclpy 订阅完成。
兼容环境变量见第 6 节。

### 2.2 调参示例

```bash
export SUPER_LIO_LAUNCH=your_relocalization.launch.py
bash ./start_pc2_localization.sh
```

---

## 3. Nav2 启动包装参数

来源：`start_navigation.sh`

### 3.1 环境变量

这些环境变量只是对 `launch/navigation.launch.py` 的一层包装。

| 参数 | 默认值 | 对应 launch 参数 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- | --- |
| `MAP_FILE` | `config/maps/exhibit_2d_map.yaml` | `map_file` | 地图 YAML 路径 | 现有地图 YAML；实机应换成真实地图 |
| `NAV2_PARAMS_FILE` | `config/nav2_params.yaml` | `nav2_params_file` | Nav2 主参数文件 | 现有 YAML；建议基于当前文件改副本 |
| `POINTCLOUD_TOPIC` | `/lio/cloud_world` | `cloud_topic` | 点云输入话题 | 合法 ROS topic |
| `SCAN_TOPIC` | `/scan` | `scan_topic` | 激光扫描话题 | 合法 ROS topic；需与 costmap 配置一致 |
| `TARGET_FRAME` | `base_link` | `target_frame` | 点云转激光时的目标坐标系 | 常用 `base_link`；需能从点云 frame 变换到该 frame |

### 3.2 脚本参数

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `--dry-run` | 关闭 | 不接真实机器人，改用 `cmd_vel_mock` 做干跑 | 仅调算法/看 `/cmd_vel` 时开启 |

### 3.3 调参示例

```bash
export MAP_FILE=/data/maps/real_map.yaml
export NAV2_PARAMS_FILE=$PWD/config/nav2_params.yaml
export SCAN_TOPIC=/my_scan
bash ./start_navigation.sh
```

---

## 4. Launch 参数

来源：`launch/navigation.launch.py`

这些参数可以直接用 `ros2 launch g1_base navigation.launch.py xxx:=...` 传入。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `map_file` | `config/maps/exhibit_2d_map.yaml` | 地图 YAML | 现有地图 YAML |
| `map_z_offset` | `1.3` | 任务 5 后语义：MID360 雷达离地高度 H，单位米；约定 `map.z=0` 为地面、`world.z=0` 为雷达启动高度 | G1 静立时 MID360 到地面的距离；现场复测后按实测值更新 |
| `nav2_params_file` | `config/nav2_params.yaml` | Nav2 主配置文件 | 现有 YAML |
| `cloud_topic` | `/lio/cloud_world` | 原始点云输入话题（喂给障碍过滤器 + pointcloud_to_laserscan） | 合法 ROS topic |
| `obstacle_cloud_topic` | `/nav/obstacle_cloud` | `nav_obstacle_cloud_filter` 输出、给 local_costmap voxel_layer 用的过滤后点云 | 合法 ROS topic |
| `obstacle_odom_topic` | `/lio/robo/odom` | 障碍过滤器/`gravity_health` 使用的里程计 | 合法 ROS topic |
| `obstacle_cloud_max_range` | `3.0` | 障碍过滤器接受的最远点云距离，米；同时也是写进 voxel_layer `obstacle_range` 的上界 | 正数；建议 `1-10` |
| `obstacle_cloud_min_height` | `0.15` | 障碍过滤器接受的最低高度（`world` 坐标，米） | 常见 `0.0-0.5` |
| `obstacle_cloud_max_height` | `1.6` | 障碍过滤器接受的最高高度（`world` 坐标，米） | 常见 `0.5-2.5` |
| `obstacle_filter_package` | `g1_base_perception` | 启动障碍过滤器节点的 ROS 包 | 已安装包名 |
| `obstacle_filter_executable` | `nav_obstacle_cloud_filter` | 障碍过滤器可执行 | 包内存在的可执行 |
| `imu_raw_topic` | `/livox/imu` | Madgwick 输入 IMU 话题 | Livox IMU 原始话题 |
| `imu_data_topic` | `/imu/data` | Madgwick 输出姿态话题，供 `gravity_health` 监控使用 | 合法 ROS topic |
| `scan_topic` | `/scan` | `pointcloud_to_laserscan` 输出的激光；目前主要给 `lateral_assist`/`close_obstacle_guard` 用，**不**是 local_costmap 的障碍源 | 合法 ROS topic |
| `scan_range_min` | `0.05` | pointcloud_to_laserscan 的 `range_min`，米 | `0.05-1.0` |
| `target_frame` | `base_link` | 点云转激光的目标坐标系 | 常用 `base_link` |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false`；接仿真时开，实机通常关 |

---

## 5. 任务执行入口参数

来源：`start_nav_script.sh`、`g1_base/nav_core.py`

### 5.1 `start_nav_script.sh` 环境变量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `DEFAULT_NET_IF` | `enP8p1s0` | 未显式传参时默认使用的宇树 SDK 网卡 | 现有网卡名 |
| `DEFAULT_ROUTE` | `config/routes/waypoint_1.yaml` | 未显式传参时默认路线文件 | 现有 YAML |
| `G1_NAV_PYTHON_BIN` | 自动探测 `python/python3` | 主程序 Python 解释器 | 可执行解释器路径；需能导入 `rclpy` |
| `G1_SDK_PYTHON_BIN` | 自动探测 | Unitree SDK 子进程解释器 | 可执行解释器路径；需能导入 `unitree_sdk2py` |

### 5.2 `nav_script` / `nav_core` 命令行参数

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `--net-if` | `enP8p1s0` | 宇树 SDK 使用的网卡 | 现有网卡名；必须能访问机器人 |
| `--route` | `config/routes/default.yaml` 或脚本给的默认路线 | 路线文件路径 | 现有 YAML；支持相对路径或绝对路径 |
| `--node-name` | `multi_waypoint_nav` | ROS 节点名 | 合法 ROS 节点名 |
| `--map-frame` | `map` | 导航目标所在地图坐标系 | 通常 `map` |
| `--base-frame` | `base_link` | 机器人底盘坐标系 | 通常 `base_link` |

### 5.3 任务执行相关环境变量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `G1_SDK_DOMAIN_ID` | `0` | Unitree SDK 自己使用的 domain id | 整数；建议 `0-232`；与机器人 SDK 配置保持一致 |
| `G1_DISABLE_AUDIO` | 关闭 | 禁用语音播报 | `0/1` 或 `true/false` |
| `G1_DISABLE_ARM` | 关闭 | 禁用手臂动作 | `0/1` 或 `true/false` |

### 5.4 调参示例

```bash
export G1_SDK_PYTHON_BIN=/home/unitree/miniconda3/envs/py310/bin/python
ros2 run g1_base nav_script \
  --net-if enP8p1s0 \
  --route ./config/routes/recorded_waypoints.yaml
```

---

## 6. `navigation_manager` 参数

来源：`g1_base/navigation_manager.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `--auto-ensure` | 关闭；但 `start_navigation_manager.sh` 会默认加上 | 启动后自动拉起定位和导航链路 | 一般保持开启 |
| `--node-name` | `navigation_manager` | 管理节点名 | 合法 ROS 节点名 |
| `--monitor-hz` | `2.0` | 状态轮询频率 | 正数；建议 `1-10`。当前代码把小于 `1` 的值按 `1` 处理 |
| `--freshness-window` | `3.0` | 话题“新鲜度”判定窗口，单位秒 | 正数；建议 `1-10`，且最好大于 `1 / monitor-hz` |
| `--localization-timeout` | `60.0` | 等待定位链 ready 的超时，单位秒 | 正数；建议 `10-180` |
| `--navigation-timeout` | `60.0` | 等待 Nav2 ready 的超时，单位秒 | 正数；建议 `10-180` |
| `--max-bringup-attempts` | `2` | 拉起失败后的最大重试次数 | 整数；建议 `1-5` |
| `--pointcloud-topic` | `/lio/cloud_world` | 点云新鲜度监控话题 | 合法 ROS topic |
| `--relocal-odom-topic` | `/lio/robo/odom` | 重定位里程计监控话题 | 合法 ROS topic |
| `--odom-topic` | `/odom_2d` | 2D 里程计监控话题 | 合法 ROS topic |
| `--map-frame` | `map` | 检查 TF 时使用的地图坐标系 | 通常 `map` |
| `--base-frame` | `base_link` | 检查 TF 时使用的底盘坐标系 | 通常 `base_link` |

### 6.1 兼容环境变量

`start_navigation_manager.sh` 和 `start-g1-base-manager.sh` 会把下列历史环境变量转换为
`navigation_manager` 参数，避免 shell 脚本依赖 ROS2 daemon 做话题检查。

| 环境变量 | 转换为 | 说明 |
| --- | --- | --- |
| `READY_TIMEOUT` | `--localization-timeout` | 等待定位/建图 LIO 话题 fresh 的超时时间 |
| `POINTCLOUD_TOPIC` | `--pointcloud-topic` | 点云新鲜度监控话题 |
| `RELOCATION_TOPIC` | `--relocal-odom-topic` | 重定位/建图里程计监控话题 |

### 6.2 建图模式

`navigation_manager` 支持在导航模式和建图模式之间切换。建图模式会停止重定位 + Nav2，启动 Super-LIO 的 SLAM 建图（`Livox_mid360.py`），人工遥控机器人走完场地后停止建图，Super-LIO 自动保存 `map.pcd`。

| ROS2 服务 | 类型 | 意义 |
| --- | --- | --- |
| `/navigation_manager/start_mapping` | `std_srvs/Trigger` | 停止当前导航栈，切换到建图模式 |
| `/navigation_manager/stop_mapping` | `std_srvs/Trigger` | 停止建图（发 SIGINT 保存地图），回到 STOPPED |

使用流程：

```bash
# 开始建图
ros2 service call /navigation_manager/start_mapping std_srvs/srv/Trigger

# 人遥控机器人走完地图...

# 停止建图（Super-LIO 自动保存 map.pcd）
ros2 service call /navigation_manager/stop_mapping std_srvs/srv/Trigger

# 处理完地图后，重新启动导航
ros2 service call /navigation_manager/ensure_ready std_srvs/srv/Trigger
```

注意：建图模式下 `ensure_ready` 和 `restart_all` 会被拒绝，需先调用 `stop_mapping`。`stop_all` 在任何模式下都可以使用。

---

## 7. 调试工具参数

### 7.1 `show_robot_pose`

来源：`g1_base/show_robot_pose.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `--frame` | `map` | 目标参考坐标系 | 通常 `map` |
| `--base-frame` | `base_link` | 机器人底盘坐标系 | 通常 `base_link` |
| `--odom-topic` | `/odom_2d` | 2D 里程计话题 | 合法 ROS topic |
| `--relocal-odom-topic` | `/lio/robo/odom` | 重定位里程计话题 | 合法 ROS topic |
| `--rate` | `2.0` | 输出刷新频率 | 正数；建议 `0.2-20`。代码内部把小于 `0.2` 的值按 `0.2` 处理 |
| `--once` | 关闭 | 只输出一次位姿后退出 | 布尔开关 |
| `--timeout` | `5.0` | `--once` 或采点模式等待首个位姿的超时，单位秒 | 正数；建议 `1-30` |
| `--record-yaml [PATH]` | 关闭；不给 PATH 时写入 `config/routes/recorded_waypoints.yaml` | 进入采点记录模式并保存 YAML | 现有或可创建的 YAML 路径 |
| `--route-name` | `default` | 采点保存时写入 YAML 的 `route_name` | 任意非空短字符串 |
| `--action-id` | `31` | 采点保存时每个 waypoint 默认动作 ID | 整数；建议填机器人动作库里存在的动作编号 |
| `--say-text` | `到达指定地点` | 采点保存时每个 waypoint 默认播报文本 | 任意字符串 |
| `--precision` | `3` | 保存坐标时的小数位数 | 整数；建议 `0-6` |

### 7.2 `publish_waypoints_to_rviz`

来源：`g1_base/publish_waypoints_to_rviz.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `--yaml` | `config/routes/recorded_waypoints.yaml` | 路线文件路径 | 现有 YAML |
| `--frame` | `map` | 发布 Marker/Path 时使用的坐标系 | 通常 `map` |
| `--marker-topic` | `/waypoint_markers` | MarkerArray 话题 | 合法 ROS topic |
| `--path-topic` | `/waypoint_path` | Path 话题 | 合法 ROS topic |
| `--period` | `2.0` | 重发周期，单位秒 | 正数；建议 `0.5-10`。代码内部把小于 `0.5` 的值按 `0.5` 处理 |

---

## 8. ROS 节点参数

这些参数一般用 `--ros-args -p name:=value` 传入。

### 8.1 `odom_to_tf`

来源：`g1_base/odom_to_tf.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `input_odom_topic` | `/lio/robo/odom` | 输入 3D 里程计话题 | 合法 ROS topic |
| `output_odom_topic` | `/odom_2d` | 输出 2D 里程计话题 | 合法 ROS topic |
| `parent_frame` | `world` | TF 父坐标系 | 常用 `world` |
| `child_frame` | `base_link` | TF 子坐标系 | 常用 `base_link` |

示例：

```bash
ros2 run g1_base odom_to_tf --ros-args \
  -p input_odom_topic:=/lio/robo/odom \
  -p output_odom_topic:=/odom_2d
```

### 8.2 `cmd_vel_mock`

来源：`g1_base/cmd_vel_mock.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `cmd_vel_topic` | `/cmd_vel` | 订阅的速度指令话题 | 合法 ROS topic |
| `odom_topic` | `/odom_2d` | 发布的模拟里程计话题 | 合法 ROS topic |
| `parent_frame` | `world` | 模拟 TF 父坐标系 | 常用 `world` |
| `child_frame` | `base_link` | 模拟 TF 子坐标系 | 常用 `base_link` |
| `dt` | `0.02` | 积分周期，单位秒 | 正数；建议 `0.005-0.1` |

### 8.3 `diag_obstacle`

来源：`g1_base/diag_obstacle.py`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `cloud_topic` | `/lio/cloud_world` | 点云监控话题 | 合法 ROS topic |
| `scan_topic` | `/scan` | 激光监控话题 | 合法 ROS topic |
| `costmap_topic` | `/local_costmap/costmap` | 局部代价地图话题 | 合法 ROS topic |
| `plan_topic` | `/plan` | 规划路径话题 | 合法 ROS topic |
| `planner_cmd_topic` | `/cmd_vel` | 规划器输出速度话题 | 合法 ROS topic |
| `executed_cmd_topic` | `/cmd_vel_executed` | 实际下发速度话题 | 合法 ROS topic |
| `motion_source_topic` | `/motion_source` | 当前控制源话题 | 合法 ROS topic |

---

## 9. 路线 YAML 参数

来源：`config/routes/*.yaml`、`g1_base/common.py`

### 9.1 顶层字段

| 字段 | 必填 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `route_name` | 否 | 路线名 | 任意非空字符串；未填时回退到文件名 |
| `waypoints` | 是 | waypoint 列表 | 非空列表 |

### 9.2 waypoint 字段

| 字段 | 必填 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `x` | 是 | 地图坐标 X，单位米 | 有限浮点数；应落在地图有效区域 |
| `y` | 是 | 地图坐标 Y，单位米 | 有限浮点数；应落在地图有效区域 |
| `yaw` | 与 `yaw_deg` 二选一 | 朝向，单位弧度 | 一般 `-pi ~ pi` 或等价角度 |
| `yaw_deg` | 与 `yaw` 二选一 | 朝向，单位度 | 一般 `-180 ~ 180` 或等价角度 |
| `action_id` | 否 | 到点后的动作 ID | 整数；缺省为 `25` |
| `say_text` | 否 | 到点后的播报文本 | 任意字符串；缺省为 `你好` |
| `index` | 否 | 仅用于记录/显示 | 可保留；运行时会重新按顺序编号，不依赖输入值 |

### 9.3 示例

```yaml
route_name: "default"
waypoints:
  - x: 9.862
    y: -5.135
    yaw_deg: 51.815
    action_id: 31
    say_text: "企业数据治理体系"
```

### 9.4 额外说明

- 路线加载支持相对路径和绝对路径。
- 相对路径解析顺序：当前工作目录 -> 仓库根目录 -> package share 目录。
- 顶层也兼容“直接给列表”的旧格式，但推荐统一使用 `route_name + waypoints` 的结构。

---

## 10. Nav2 参数文件

来源：`config/nav2_params.yaml`

这一节只列当前仓库里已经显式写出的参数。注意：如果你是通过 `nav_script` 执行任务，其中一部分 `FollowPath.*` 参数会在运行时被二次修改，见第 11 节。

### 10.1 `bt_navigator`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `global_frame` | `map` | 全局地图坐标系 | 通常 `map` |
| `robot_base_frame` | `base_link` | 机器人底盘坐标系 | 通常 `base_link` |
| `odom_topic` | `/odom_2d` | 里程计话题 | 合法 ROS topic |
| `bt_loop_duration` | `10` | BT 循环周期，单位 ms | 正数；建议 `5-100` |
| `default_server_timeout` | `20` | 默认 action/service 超时，单位秒 | 正数；建议 `5-60` |
| `wait_for_service_timeout` | `1000` | 等待服务超时 | 正数；通常保持现值 |
| `navigators` | `["navigate_to_pose", "navigate_through_poses"]` | 启用的导航器列表 | 通常保持现值 |
| `navigate_to_pose.plugin` | `nav2_bt_navigator::NavigateToPoseNavigator` | 单点导航器插件 | 除非更换架构，否则保持现值 |
| `navigate_through_poses.plugin` | `nav2_bt_navigator::NavigateThroughPosesNavigator` | 多点穿越导航器插件 | 同上 |
| `default_nav_to_pose_bt_xml` | `$(find-pkg-share g1_base)/behavior_trees/navigate_to_pose_forward_only.xml` | 自定义 BT，禁用旋转/后退恢复行为以适配人形步态 | 路径需指向已编译进 share 目录的 BT XML |
| `default_nav_through_poses_bt_xml` | `$(find-pkg-share g1_base)/behavior_trees/navigate_through_poses_forward_only.xml` | 多点穿越对应的自定义 BT | 同上 |

### 10.2 `controller_server`

> 当前控制器是 **MPPI**（`nav2_mppi_controller::MPPIController`），不再是 DWB。下表只列当前 YAML 里显式写出的参数；MPPI critic 子项见 `config/nav2_params.yaml` `FollowPath.*Critic.*`，调权重前请先看官方文档。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `controller_frequency` | `8.0` | 局部控制器频率，Hz | 正数；建议 `5-20`（MPPI 单步算力比 DWB 重，过高会跑不满） |
| `min_x_velocity_threshold` | `0.01` | 线速度死区阈值 | `0-0.1` |
| `min_y_velocity_threshold` | `0.0` | 横移速度死区阈值 | `0-0.1` |
| `min_theta_velocity_threshold` | `0.01` | 角速度死区阈值 | `0-0.2` |
| `failure_tolerance` | `0.3` | 允许短时失败容忍度 | `0-1` |
| `progress_checker_plugins` | `["progress_checker"]` | 进度检查器列表 | 通常保持现值 |
| `goal_checker_plugins` | `["goal_checker"]` | 目标检查器列表 | 通常保持现值 |
| `controller_plugins` | `["FollowPath"]` | 控制器插件列表 | 通常保持现值 |
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker` | 进度检查器插件 | 通常保持现值 |
| `progress_checker.required_movement_radius` | `0.10` | 认为"有前进"的最小位移，米 | `0.03-0.3` |
| `progress_checker.movement_time_allowance` | `4.0` | 在该时长内没明显移动则判失败，秒 | `2-20` |
| `goal_checker.plugin` | `nav2_controller::SimpleGoalChecker` | 目标检查器插件 | 通常保持现值 |
| `goal_checker.stateful` | `true` | 是否使用有状态目标判断 | `true/false`；一般保持 `true` |
| `goal_checker.xy_goal_tolerance` | `0.30` | 到点位置容差，米 | `0.05-0.5` |
| `goal_checker.yaw_goal_tolerance` | `0.30` | 到点朝向容差，弧度 | `0.05-0.8` |
| `FollowPath.plugin` | `nav2_mppi_controller::MPPIController` | MPPI 控制器插件 | 除非更换控制器，否则保持现值 |
| `FollowPath.time_steps` | `24` | 滚动优化步数 | `15-40`；步数 × `model_dt` ≈ 前瞻时长 |
| `FollowPath.model_dt` | `0.125` | 单步积分步长，秒 | `0.05-0.2` |
| `FollowPath.batch_size` | `550` | 每周期采样轨迹数 | `200-2000`；越大越稳但越吃 CPU |
| `FollowPath.iteration_count` | `1` | 每周期 MPPI 迭代次数 | 通常 `1` |
| `FollowPath.prune_distance` | `2.4` | 路径裁剪长度，米 | `1-5` |
| `FollowPath.transform_tolerance` | `1.0` | TF 容忍时长，秒 | `0.05-2.0` |
| `FollowPath.vx_std` / `vy_std` / `wz_std` | `0.32 / 0.0 / 0.35` | 高斯采样标准差 | 调小更稳，调大探索性更强 |
| `FollowPath.vx_max` / `vx_min` | `0.8 / 0.0` | 线速度上下限；运行时被第 11 节按距离动态覆盖 | 与底盘能力一致 |
| `FollowPath.vy_max` | `0.0` | 横移速度上限；差速底盘保持 `0` | `0` |
| `FollowPath.wz_max` | `1.0` | 角速度上限；运行时被第 11 节覆盖 | `0.2-2.5` |
| `FollowPath.ax_max` / `ax_min` | `2.0 / -2.0` | 线加速度区间 | 与底盘能力一致 |
| `FollowPath.ay_max` / `ay_min` | `0.0 / 0.0` | 横移加速度区间；差速保持 `0` | `0` |
| `FollowPath.az_max` | `1.0` | 角加速度上限 | `0.5-3.0` |
| `FollowPath.temperature` | `0.3` | softmax 温度，影响采样集中度 | `0.1-1.0` |
| `FollowPath.gamma` | `0.015` | 控制成本权重 | `0.001-0.1` |
| `FollowPath.motion_model` | `"DiffDrive"` | 运动模型 | `DiffDrive`（其它如 `Omni`/`Ackermann`） |
| `FollowPath.reset_period` | `1.0` | 多久没收到新路径就重置 MPPI，秒 | `0.5-3.0` |
| `FollowPath.critics` | 9 个 critic（ConstraintCritic / CostCritic / GoalCritic / GoalAngleCritic / PathAlignCritic / PathFollowCritic / PathAngleCritic / PreferForwardCritic / VelocityDeadbandCritic） | 参与打分的 critic 列表 | 改动需同时改对应子节 |
| `FollowPath.CostCritic.cost_weight` | `3.81` | 障碍代价权重；过低易贴障，过高易停 | `1-10` |
| `FollowPath.GoalCritic.cost_weight` | `5.0` | 终点距离权重 | `1-20` |
| `FollowPath.PathFollowCritic.cost_weight` | `14.0` | 沿路径推进权重；当前明显大于其它项 | `1-30` |
| `FollowPath.PathAlignCritic.cost_weight` | `4.0` | 贴近路径权重；运行时被第 11 节覆盖 | `1-30` |
| `FollowPath.VelocityDeadbandCritic.deadband_velocities` | `[0.25, 0.0, 0.05]` | `[vx, vy, wz]` 死区；低于该值给惩罚以避免抖动 | 与 `MIN_VX_THRESHOLD` 协同 |

### 10.3 `planner_server`

> 当前全局规划器是仓库内自研的 **`g1_centerline_planner/CenterlinePlanner`**，不再是 navfn。它在标准代价地图搜索上加了"贴中线 + 远墙"的代价整形，调参主要是平衡 length / wall / center / costmap 这几项权重。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `expected_planner_frequency` | `1.0` | 全局规划期望频率，Hz | 正数；建议 `0.5-10` |
| `planner_plugins` | `["GridBased"]` | 规划器列表 | 通常保持现值 |
| `GridBased.plugin` | `g1_centerline_planner/CenterlinePlanner` | 全局规划器插件 | 除非更换算法，否则保持现值 |
| `GridBased.allow_unknown` | `false` | 是否允许经过未知区域 | 已知地图建议 `false` |
| `GridBased.use_final_approach_orientation` | `false` | 终点是否强制对齐最后一段路径朝向 | `true/false` |
| `GridBased.min_clearance` | `0.38` | 路径离障碍最小净距，米；低于该值代价急升 | 略大于机器人半径 |
| `GridBased.target_clearance` | `0.90` | 目标净距，米；超过即按中线走 | `0.5-1.5` |
| `GridBased.length_weight` | `1.0` | 路径长度权重 | `0.5-3.0` |
| `GridBased.wall_weight` | `8.0` | 远墙惩罚权重，越大越远离墙 | `1-20` |
| `GridBased.center_weight` | `4.0` | 走中线奖励权重 | `1-15` |
| `GridBased.center_clearance_cap` | `2.0` | 中线代价考虑的最大净距 | `1-4` |
| `GridBased.costmap_weight` | `2.0` | 原 costmap 代价的权重 | `0.5-5` |
| `GridBased.shortcut_min_clearance` | `0.45` | 抄近道允许的最低净距 | `>= min_clearance` |
| `GridBased.shortcut_cost_tolerance` | `1.02` | 抄近道允许的代价上限比 | `1.0-1.2` |
| `GridBased.endpoint_snap_radius` | `1.20` | 起终点附近的吸附搜索半径 | `0.5-2.0` |
| `GridBased.endpoint_min_clearance` | `0.18` | 起终点允许的最小净距（窄于 min_clearance 以救活近障碍发点的任务） | 略小于 `min_clearance` |
| `GridBased.fallback_min_clearance` | `0.25` | 全图找不到满足 min_clearance 的路径时的兜底净距 | `0.15-0.35` |

### 10.4 `behavior_server`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `global_frame` | `map` | 全局坐标系 | 通常 `map` |
| `robot_base_frame` | `base_link` | 底盘坐标系 | 通常 `base_link` |
| `costmap_topic` | `local_costmap/costmap_raw` | 行为服务器使用的 costmap 话题 | 合法 ROS topic |
| `footprint_topic` | `local_costmap/published_footprint` | 足迹话题 | 合法 ROS topic |
| `cycle_frequency` | `10.0` | 行为循环频率 | 正数；建议 `2-20` |
| `behavior_plugins` | `["spin"]` | 行为插件列表；当前为了适配人形步态，**已移除 backup/wait** | 通常保持现值 |
| `spin.plugin` | `nav2_behaviors/Spin` | 原地转向行为 | 通常保持现值 |

### 10.5 `waypoint_follower`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `stop_on_failure` | `false` | waypoint 失败后是否停止整个流程 | `true/false` |
| `loop_rate` | `20` | 跟随器循环频率 | 正数；建议 `5-50` |

### 10.6 `velocity_smoother`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `smoothing_frequency` | `20.0` | 速度平滑器频率 | 正数；建议 `10-100` |
| `scale_velocities` | `false` | 是否按比例缩放速度 | `true/false` |
| `feedback` | `OPEN_LOOP` | 平滑反馈模式 | 常见 `OPEN_LOOP/CLOSED_LOOP`；当前保持现值 |
| `max_velocity` | `[0.8, 0.0, 1.0]` | 最大速度 `[vx, vy, wz]` | 与底盘能力一致；差速底盘 `vy` 通常为 `0` |
| `min_velocity` | `[0.0, 0.0, -1.0]` | 最小速度 `[vx, vy, wz]` | 常见为前进不小于 `0`，角速度允许负值 |
| `max_accel` | `[2.0, 0.0, 1.0]` | 最大加速度 `[vx, vy, wz]` | 正数；与底盘能力一致 |
| `max_decel` | `[-2.0, 0.0, -1.0]` | 最大减速度 `[vx, vy, wz]` | 负数；与底盘能力一致 |
| `odom_topic` | `/odom_2d` | 反馈里程计话题 | 合法 ROS topic |
| `odom_duration` | `0.1` | 里程计时间窗口 | 正数；建议 `0.02-1.0` |
| `deadband_velocity` | `[0.0, 0.0, 0.0]` | 死区速度 | 非负；通常保持很小 |
| `velocity_timeout` | `1.0` | 多久没新指令就清空速度，秒 | 正数；建议 `0.1-3.0` |

### 10.7 `global_costmap`

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `global_frame` | `map` | 全局地图坐标系 | 通常 `map` |
| `robot_base_frame` | `base_link` | 底盘坐标系 | 通常 `base_link` |
| `update_frequency` | `1.0` | 更新频率，Hz | 正数；建议 `0.5-10` |
| `publish_frequency` | `0.5` | 发布频率，Hz | 正数；建议 `0.2-10` |
| `resolution` | `0.08` | 地图分辨率，米/格 | `0.02-0.2` |
| `robot_radius` | `0.30` | 机器人半径，米 | 按真实机体外接圆测量；常见 `0.2-0.6` |
| `transform_tolerance` | `1.0` | TF 容忍时长，秒 | `0.05-2.0` |
| `rolling_window` | `false` | 是否滚动窗口 | 全局地图通常 `false` |
| `track_unknown_space` | `true` | 是否跟踪未知区域 | `true/false` |
| `plugins` | `["static_layer", "inflation_layer"]` | costmap 层列表 | 通常保持现值 |
| `static_layer.plugin` | `nav2_costmap_2d::StaticLayer` | 静态层插件 | 通常保持现值 |
| `static_layer.map_subscribe_transient_local` | `true` | 是否用 transient_local 订阅地图 | 一般保持 `true` |
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | 膨胀层插件 | 通常保持现值 |
| `inflation_layer.inflation_radius` | `0.65` | 膨胀半径，米 | 一般略大于机器人半径；当前比局部 costmap 大很多以鼓励远离障碍走中线 |
| `inflation_layer.cost_scaling_factor` | `2.0` | 膨胀代价衰减系数 | 正数；建议 `1-20`（值小代价区扩散更平缓） |
| `always_send_full_costmap` | `true` | 是否总是发全量 costmap | `true/false` |

### 10.8 `local_costmap`

> 局部 costmap 的障碍源已经从 LaserScan 换成 **SpatioTemporalVoxelLayer + PointCloud2**（订阅 `/nav/obstacle_cloud`，由 `nav_obstacle_cloud_filter` 输出）。`/scan` 现在主要供 `lateral_assist` / `close_obstacle_guard` 使用，**不再**喂 local_costmap。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间 | `true/false` |
| `global_frame` | `map` | 局部 costmap 的全局参考系 | 当前项目保持 `map` |
| `robot_base_frame` | `base_link` | 底盘坐标系 | 通常 `base_link` |
| `update_frequency` | `8.0` | 更新频率，Hz | 正数；建议 `2-20`（已和 MPPI `controller_frequency` 对齐） |
| `publish_frequency` | `5.0` | 发布频率，Hz | 正数；建议 `1-20` |
| `resolution` | `0.08` | 分辨率，米/格 | `0.02-0.2` |
| `width` | `7` | 局部窗口宽度，米 | 正数；建议 `4-12`（要覆盖 MPPI 前瞻 `time_steps × model_dt × vx_max`） |
| `height` | `7` | 局部窗口高度，米 | 正数；建议同上 |
| `rolling_window` | `true` | 是否滚动窗口 | 局部地图一般 `true` |
| `robot_radius` | `0.30` | 机器人半径，米 | 与真实机体一致 |
| `transform_tolerance` | `1.0` | TF 容忍时长，秒 | `0.05-2.0` |
| `footprint_padding` | `0.05` | 足迹外扩，米 | `0-0.2` |
| `plugins` | `["static_layer", "voxel_layer", "inflation_layer"]` | 层列表 | 通常保持现值 |
| `static_layer.plugin` | `nav2_costmap_2d::StaticLayer` | 静态层插件 | 通常保持现值 |
| `static_layer.map_subscribe_transient_local` | `true` | 静态地图订阅方式 | 一般保持 `true` |
| `voxel_layer.plugin` | `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer` | STVL 体素层插件 | 通常保持现值 |
| `voxel_layer.voxel_decay` | `4.0` | 体素衰减时间，秒；过短会让障碍闪烁、过长会留尾迹 | `1-15` |
| `voxel_layer.decay_model` | `0` | 衰减模型（`0` 线性 / `1` 指数 / `2` 永久） | 一般 `0` |
| `voxel_layer.voxel_size` | `0.08` | 体素边长，米 | `0.04-0.15`；与 costmap `resolution` 一致较好 |
| `voxel_layer.track_unknown_space` | `false` | 是否跟踪未知空间 | 局部一般 `false` |
| `voxel_layer.z_resolution` | `0.08` | 垂直方向分辨率，米 | 与 `voxel_size` 一致 |
| `voxel_layer.z_voxels` | `24` | 垂直方向体素数；`z_voxels × z_resolution` 决定可见高度（当前 ≈ 1.92m） | 建议 `> (obstacle_max_height - 0)/z_resolution` |
| `voxel_layer.publish_voxel_map` | `true` | 是否发体素 marker，供 RViz 可视化 | 调试时 `true`，发布生产可关 |
| `voxel_layer.combination_method` | `1` | 多 source 合并方式（`0` overwrite / `1` max） | 一般 `1` |
| `voxel_layer.mapping_mode` | `false` | 建图模式开关 | 导航 `false` |
| `voxel_layer.observation_sources` | `obstacle_cloud` | 障碍源名称（一行字符串，不是数组） | 与下方子节名一致 |
| `voxel_layer.obstacle_cloud.topic` | `/nav/obstacle_cloud` | 障碍点云话题；由 `nav_obstacle_cloud_filter` 发出 | 合法 ROS topic |
| `voxel_layer.obstacle_cloud.data_type` | `PointCloud2` | 数据类型 | 保持 `PointCloud2` |
| `voxel_layer.obstacle_cloud.marking` | `true` | 是否标记障碍 | `true/false` |
| `voxel_layer.obstacle_cloud.clearing` | `false` | 是否做 raytrace 清障；当前依赖 STVL 衰减而非 raytrace 清除 | `false` 时 `voxel_decay` 必须有效 |
| `voxel_layer.obstacle_cloud.obstacle_range` | `3.0` | 障碍写入最远距离，米 | 与 launch `obstacle_cloud_max_range` 同步 |
| `voxel_layer.obstacle_cloud.min_obstacle_height` | `0.15` | 最低障碍高度（`world` 坐标，米） | 与 launch `obstacle_cloud_min_height` 同步 |
| `voxel_layer.obstacle_cloud.max_obstacle_height` | `1.6` | 最高障碍高度（`world` 坐标，米） | 与 launch `obstacle_cloud_max_height` 同步 |
| `voxel_layer.obstacle_cloud.voxel_min_points` | `1` | 一个体素中最少多少点才算障碍 | `1-5`；过滤稀疏噪点可调到 `2` |
| `voxel_layer.obstacle_cloud.filter` | `voxel` | 入层前的额外滤波 | 常用 `voxel` |
| `voxel_layer.obstacle_cloud.clear_after_reading` | `true` | 读完点云就丢；适合高频订阅 | `true/false` |
| `voxel_layer.obstacle_cloud.sensor_frame` | `base_link` | 传感器坐标系 | 与点云 frame 一致 |
| `inflation_layer.plugin` | `nav2_costmap_2d::InflationLayer` | 膨胀层插件 | 通常保持现值 |
| `inflation_layer.inflation_radius` | `0.40` | 膨胀半径，米 | 略大于机器人半径 |
| `inflation_layer.cost_scaling_factor` | `3.0` | 膨胀衰减系数 | 正数；建议 `1-20` |
| `always_send_full_costmap` | `true` | 是否总是发全量 costmap | `true/false` |

### 10.9 其它节点

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `local_costmap_client.use_sim_time` | `false` | 仿真时间开关 | `true/false` |
| `global_costmap_client.use_sim_time` | `false` | 仿真时间开关 | `true/false` |
| `map_server.use_sim_time` | `false` | 仿真时间开关 | `true/false` |
| `map_server.frame_id` | `map` | 地图 frame | 通常 `map` |

---

## 11. `nav_core.py` 运行时二次调参与硬编码策略

来源：`g1_base/g1_base/nav_core.py`（`scripts/nav_core.py` 是装入 entry_points 的薄壳）

这一节是"会直接影响实机行为"的运行时策略。速度档位仍由代码按距离动态计算，但**默认值来自 `config/walking_mode.yaml`**（按 `walking_mode: locked_waist | unlocked_waist` 切档），表中的数字是 `nav_core.py` 里的兜底常量；前方近障碍横移辅助由 `config/motion_policy.yaml` 配置。

### 11.1 运行时会覆盖的 `FollowPath.*`

执行 `nav_script` 时，会根据与目标点的距离按 `blend ∈ [0, 1]` 在 fast / slow 之间线性插值，下发到当前 MPPI 控制器：

| 参数 | 快速档 (`nav_core.py` 兜底) | 精准档 (`nav_core.py` 兜底) | 当前 `unlocked_waist` 配置 | 意义 |
| --- | --- | --- | --- | --- |
| `FollowPath.max_vel_x` / `vx_max` / `max_speed_xy` | `1.04` | `0.46` | `1.95` / `0.98` | 最大前进速度 |
| `FollowPath.max_vel_theta` / `wz_max` | `1.2` | `0.8` | `1.5` / `1.0` | 最大角速度 |
| `FollowPath.acc_lim_x` | `2.0` | `0.5` | `2.5` / `0.8` | 线加速度上限 |
| `FollowPath.acc_lim_theta` | `1.0` | `0.8` | `1.2` / `1.0` | 角加速度上限 |
| `FollowPath.PathAlign.scale` | `18.0` | `18.0` | 同左 | 贴路径权重 |
| `FollowPath.PathDist.scale` | `18.0` | `18.0` | 同左 | 路径距离权重 |
| `FollowPath.GoalAlign.scale` | `16.0` | `22.0` | 同左 | 目标朝向权重 |
| `FollowPath.GoalDist.scale` | `16.0` | `22.0` | 同左 | 目标距离权重 |
| `FollowPath.BaseObstacle.scale` | `10.0` | `10.0` | 同左 | 障碍物权重（注：MPPI 走 `CostCritic.cost_weight` 路径，这里映射的是兼容名） |
| `FollowPath.<footprint>.scale` | `14.0` | `14.0` | 同左 | footprint 相关权重 |
| `FollowPath.xy_goal_tolerance` | `0.3` | `0.3` | 同左 | 位置容差，代码固定写回 `0.3` |

> 切换 `walking_mode` 后，对应分组下的 `fast_*` / `slow_*` 会覆盖 `nav_core.py` 的兜底常量；其余权重项（PathAlign/GoalAlign/BaseObstacle/footprint）目前仅由兜底常量决定，未暴露到 YAML。

距离分档相关参数：

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `SLOWDOWN_START_DISTANCE` | `2.0` | 从多远开始进入减速档 | 正数；建议 `0.5-5.0` |
| `SLOWDOWN_FULL_PROFILE_DISTANCE` | `0.6` | 距离目标多近时切到完整慢速档 | 正数；且应小于 `SLOWDOWN_START_DISTANCE` |
| `SLOWDOWN_PROFILE_STEP` | `0.1` | 档位量化步长 | `0.05-0.5` |

### 11.2 运动控制相关常量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `SPEED_MODE` | `1` | SDK 速度模式（也可由 `walking_mode.yaml` 的 `speed_mode` 覆盖） | 整数；通常保持 `1`，除非明确知道宇树速度档含义 |
| `MIN_VX_THRESHOLD` | `0.25` | 前向速度最小钳制，低于该值会被上拉，避免走路落不了脚 | `0-1.0` |
| `MIN_VX_BYPASS_WZ` | `0.25` | 当指令角速度 ≥ 该值时跳过 `MIN_VX_THRESHOLD` 钳制，让原地转弯不被前向上拉拖出来 | `0.1-0.5` |
| `PLANNER_TIMEOUT` | `0.5` | 多久没收到规划速度就视为超时，秒 | `0.1-2.0` |
| `CONTROL_LOOP_HZ` | `20.0` | 底层控制循环频率 | 正数；建议 `20-200` |
| `YIELD_HOLD_TIMEOUT` | `0.7` | 让行静止保持时长，秒 | `0.1-3.0` |
| `MAX_CMD_STEP_VX` | `0.10` | 单周期线速度平滑步长 | 正数；建议 `0.01-0.2` |
| `MAX_CMD_STEP_VY` | `0.05` | 单周期横移速度平滑步长 | 正数；差速底盘一般很小 |
| `MAX_CMD_STEP_WZ` | `0.15` | 单周期角速度平滑步长 | 正数；建议 `0.01-0.3` |

### 11.3 前方近障碍横移辅助

`MotionController` 默认仍会把自动导航的 `vy` 清零；只有 `/scan` 中机器人正前方走廊检测到近障碍时，才临时注入横移速度。这样可以帮助绕开近前方障碍，同时避免全程横着走。

这些参数在 `config/motion_policy.yaml`：

如需临时测试另一份配置，可在启动前设置 `G1_MOTION_POLICY_FILE=/path/to/motion_policy.yaml`。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `motion.global_allow_lateral_motion` | `false` | 全局允许横向移动。`false` 时仅近障碍辅助横移；`true` 时 Nav2 可全程输出 `vy` | 一般保持 `false`，只在需要全向绕行时打开 |
| `lateral_assist.enabled` | `false` | 是否启用前方近障碍横移辅助 | 当前默认关闭，开启前确认 vy 安全 |
| `lateral_assist.scan_topic` | `/scan` | 用于检测前方障碍的 LaserScan | 与 launch 输出一致即可 |
| `lateral_assist.trigger_distance` | `2.50` | 正前方多少米内有障碍时触发横移辅助 | `0.5-3.0` |
| `lateral_assist.clear_distance` | `2.70` | 已触发后，障碍远到多少米外才释放，形成轻微滞回 | 应大于触发距离 |
| `lateral_assist.front_half_width` | `0.45` | 认为属于"路径前方"的半宽，米 | 通常略大于机器人半径 |
| `lateral_assist.side_lookahead` | `2.80` | 评估横移方向时该侧前向扫多远 | `1.5-4.0` |
| `lateral_assist.side_min_clearance` | `0.55` | 选择横移方向时，该侧至少需要的近侧空间 | 应大于机器人半径 |
| `lateral_assist.side_band_min` | `0.35` | 侧向带通最小宽度，米 | `0.2-0.6` |
| `lateral_assist.side_band_max` | `1.20` | 侧向带通最大宽度，米 | `0.8-2.0` |
| `lateral_assist.vy` | `0.22` | 横移辅助下发的横向速度，m/s | `0.05-0.4`；太大会显得突兀 |
| `lateral_assist.max_forward_vx` | `0.15` | 横移时允许保留的最大前向速度，m/s | `0-0.3` |
| `lateral_assist.scan_timeout` | `0.60` | 多久没收到 scan 就关闭辅助，秒 | `0.2-2.0` |
| `close_obstacle_guard.enabled` | `true` | 极近距离障碍急停守门 | 一般保持开启 |
| `close_obstacle_guard.scan_topic` | `/scan` | LaserScan 话题 | 同 lateral_assist |
| `close_obstacle_guard.min_range` | `0.05` | 视为有效回波的最近距离 | `0.03-0.2` |
| `close_obstacle_guard.trigger_distance` | `0.35` | 进入急停的距离阈值，米 | `0.2-0.6` |
| `close_obstacle_guard.release_distance` | `0.75` | 解除急停的距离阈值（滞回），米 | 应大于 trigger |
| `close_obstacle_guard.front_half_width` | `0.30` | 仅在该半宽内的回波算"正前方"，米 | `0.2-0.5` |
| `close_obstacle_guard.front_angle_min` / `front_angle_max` | `-0.66` / `0.70` | 触发扇区的角度区间，弧度（已避开 Livox 立柱阴影区） | 现场标定后改；不要让阴影角进入区间 |
| `close_obstacle_guard.min_points` | `3` | 触发需要的最少回波点数 | `2-10` |
| `close_obstacle_guard.trigger_percentile` | `0.10` | 用距离的低分位数判定，抗稀疏噪点 | `0.05-0.3` |
| `close_obstacle_guard.min_consecutive_frames` | `2` | 连续多少帧满足触发条件才生效 | `1-5` |
| `close_obstacle_guard.clear_duration` | `1.00` | 解除条件保持多久才真正释放，秒 | `0.5-3.0` |
| `goal_occupancy.enabled` | `true` | 终点被人/物占用时的等待/绕行逻辑 | 一般保持开启 |
| `goal_occupancy.start_distance` | `1.50` | 距终点多近开始检测占用，米 | `0.5-3.0` |
| `goal_occupancy.wait_timeout` | `10.00` | 占用等待超时，秒 | `5-60` |
| `goal_occupancy.clear_duration` | `1.00` | 解除占用需要保持的时长，秒 | `0.5-3.0` |
| `goal_occupancy.speak_interval` | `4.00` | 等待时语音播报间隔，秒 | `2-15` |
| `global_lateral_motion.max_vel_y` | `0.0` | 全局横移打开时的最大左向速度，m/s（当前默认关掉横移） | 启用横移时 `0.05-0.4` |
| `global_lateral_motion.min_vel_y` | `0.0` | 全局横移打开时的最大右向速度，m/s | 启用横移时 `-0.4~-0.05` |
| `global_lateral_motion.vy_samples` | `1` | 全局横移打开时 MPPI 的横向采样数 | 启用横移时 `3-9` |
| `global_lateral_motion.acc_lim_y` | `0.0` | 横移加速度上限 | 启用横移时与底盘能力一致 |
| `global_lateral_motion.decel_lim_y` | `0.0` | 横移减速度上限 | 启用横移时为负值 |

### 11.4 旋转恢复相关常量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `ROTATE_RECOVERY_TRIGGER_REQUESTS` | `2` | 连续触发多少次后进入旋转恢复 | 整数；建议 `1-10` |
| `ROTATE_RECOVERY_TIMEOUT` | `2.0` | 单次旋转恢复超时，秒 | `0.5-10` |
| `ROTATE_RECOVERY_COOLDOWN` | `1.0` | 两次恢复之间冷却时间，秒 | `0-10` |
| `ROTATE_RECOVERY_MAX_RETRIES` | `2` | 最大重试次数 | 整数；建议 `0-10` |
| `ROTATE_RECOVERY_YAW_THRESHOLD` | `15 deg` | 触发恢复所需的航向误差阈值 | 建议 `5-45 deg` |
| `ROTATE_RECOVERY_MIN_WZ` | `0.2` | 恢复时最小角速度 | `0.05-1.0` |
| `ROTATE_RECOVERY_MAX_WZ` | `1.0` | 恢复时最大角速度 | `0.2-3.0` |
| `ROTATE_RECOVERY_GAIN` | `2.5` | 恢复转向比例增益 | `0.5-10` |
| `ROTATE_RECOVERY_FALLBACK_YAW` | `30 deg` | 无参考方向时的兜底旋转角 | 建议 `10-90 deg` |
| `ROTATE_RECOVERY_MIN_PLANNER_WZ` | `0.15` | 认为规划器已经在主动转向的最小角速度 | `0-1.0` |

### 11.5 起步对齐相关常量

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `PATH_ALIGN_MIN_POINTS` | `2` | 起步前对齐方向所需的最少路径点数 | 整数；建议 `2-10` |
| `PATH_ALIGN_MIN_SEGMENT` | `0.15` | 认为路径段有效的最小长度，米 | `0.05-1.0` |
| `PATH_ALIGN_TIMEOUT` | `4.0` | 等待获取参考方向的超时，秒 | `0.5-10` |
| `PATH_ALIGN_YAW_THRESHOLD` | `15 deg` | 若当前航向误差小于该值则跳过预旋转 | 建议 `5-45 deg` |

---

## 12. 点云转激光的硬编码调优项

来源：`launch/navigation.launch.py` 里的 `pointcloud_to_laserscan_node`

这些参数目前没有单独暴露成 launch 参数，需要改 launch 文件。它们对“避障好不好使”非常关键。

| 参数 | 当前值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `transform_tolerance` | `1.0` | TF 容忍时长，秒 | `0.05-2.0` |
| `min_height` | `-1.00` | 参与投影的最低点云高度，米；注意 `base_link.z=0` 实际位于 LIO 初始化的身体中心高度（约地面上方 1.2m），不是地面。这里 `-1.00` 对应地面上方约 0.2m 起 | 常见 `-1.5 ~ 0.5`；这是你排查"扫不到障碍物/扫到地面"的关键参数之一 |
| `max_height` | `0.50` | 参与投影的最高点云高度，米；对应 `base_link` 原点上方 0.5m，约地面 1.7m，可覆盖成年人胸肩 | 常见 `0.0-2.0`；与机器人安装高度和展台/墙面几何有关 |
| `angle_min` | `-3.14159` | 激光最小角度 | 通常 `-pi ~ 0` |
| `angle_max` | `3.14159` | 激光最大角度 | 通常 `0 ~ pi`，当前是 360 度 |
| `angle_increment` | `0.0349` | 角分辨率，弧度；约 2°。MID360 非重复扫描在 100-200ms 窗口内并不会均匀覆盖所有方位角，过细的 bin 会导致大量空 bin，配合 `inf_is_valid` 反而放大漏检风险 | 正数；常见 `0.004-0.05`。MID360 建议 `0.0175-0.0349`（1°-2°） |
| `scan_time` | `0.2` | 一帧激光扫描对应周期，秒；拉长聚合窗口可降低空 bin 比例，代价是 scan 发布频率下降（此处从 10Hz 降到 5Hz）和延迟上升 | 正数；建议 `0.05-0.3`。G1 步速 ≤0.5m/s 时 `0.2` 可接受 |
| `range_min` | 来自 launch 参数 `scan_range_min`（默认 `0.05`） | 最近可用距离，米 | 正数；通常 `0.05-1.0` |
| `range_max` | `5.0` | 最远可用距离，米 | 正数；通常 `1-30` |
| `use_inf` | `true` | 无回波时是否输出 `inf`；必须与 `obstacle_layer.scan.inf_is_valid` 配合才能让空 bin 触发 raytrace 清除 | `true/false`；建议保持 `true` |

---

## 13. `diag_obstacle.py` 采样阈值

来源：`g1_base/diag_obstacle.py`

这些参数影响诊断输出口径，不直接控制导航。

| 参数 | 默认值 | 意义 | 合理范围 / 建议 |
| --- | --- | --- | --- |
| `NEAR_THRESHOLD` | `3.0` | 认为“近障碍”的距离阈值，米 | 正数；建议 `0.5-10` |
| `DANGER_THRESHOLD` | `1.5` | 认为“危险障碍”的距离阈值，米 | 正数；建议小于 `NEAR_THRESHOLD` |
| `FRONT_HALF_ANGLE` | `45.0 deg` | 正前方窄扇区半角 | `10-90 deg` |
| `FRONT_WIDE_ANGLE` | `90.0 deg` | 正前方宽扇区半角 | `30-180 deg` |
| `SAMPLE_INTERVAL` | `0.2` | 诊断采样周期，秒 | 正数；建议 `0.05-2.0` |

---

## 13.5 MID360 + LaserScan 路径的已知陷阱与调参联动

> 当前 local_costmap 已切换到 SpatioTemporalVoxelLayer（消费 `/nav/obstacle_cloud`），下面 1/2/3/4 主要影响的是 `/scan` 的消费方——`lateral_assist` / `close_obstacle_guard` / `diag_obstacle`。如果你只用 voxel_layer 走点云直通，第 12 节的几条对 costmap 已无直接影响。

1. **`base_link.z=0` 不是地面**：在当前项目，`world`（LIO 原点）≈ 机器人启动时的 IMU/身体中心高度，约地面上方 1.2m。
   - `min_height / max_height` 是在 `target_frame=base_link` 下生效的，所以 `-1.00 / +0.50` 实际对应地面 0.2m-1.7m 这一条立体切片。
   - 如果改 `target_frame`，必须同步重新标定这两个值。
   - **注**：voxel_layer 消费的 `/nav/obstacle_cloud` 由 `nav_obstacle_cloud_filter` 输出，它已经在 `world` 坐标系里按 `obstacle_min_height`/`obstacle_max_height` 做高度过滤，和这里的 LaserScan 路径**互不影响**。

2. **高度窗口过窄 → 前向扇区大量 `inf`**：在展台环境实测，过窄的高度窗口会让正前方射线大部分打到地面（z 超下限），被丢弃；而背后射线打到展台立面被保留，造成 /scan 的 front/back 命中比严重失衡（观测到 0.16）。窗口拉宽后能恢复到 0.7 附近。

3. **`angle_increment` + `scan_time` 决定空 bin 率**：MID360 是非重复扫描，`1° × 100ms` 组合下每个 bin 期望命中点少且方差大；`2° × 200ms` 组合下期望命中翻 4 倍，空 bin 显著减少。

4. **`use_inf` / `inf_is_valid` 历史背景**：之前 obstacle_layer 走 /scan 时这是清旧 mark 的关键；切到 STVL 后，清障靠 `voxel_decay` 自然衰减，对 `/scan` `inf` 行为不再敏感。

5. **STVL 的"清不掉"主要看 `voxel_decay` 和 `clearing`**：当前 `obstacle_cloud.clearing=false` + `voxel_decay=4.0`，意味着障碍靠 4 秒线性衰减"自然消失"。如果遇到"障碍走开了 costmap 还红"的现象，先把 `voxel_decay` 改小一点；如果遇到"瞬时噪点变长期障碍"，把 `voxel_min_points` 或 `mark_threshold` 拉高。

---

## 14. 实际调参顺序建议

如果你现在是为了把系统跑稳，建议优先动下面这些参数：

1. `map_z_offset`
2. launch 参数 `obstacle_cloud_max_range / obstacle_cloud_min_height / obstacle_cloud_max_height`（控制喂给 voxel_layer 的高度切片和最远距离）
3. `local_costmap.voxel_layer.obstacle_cloud.topic / voxel_decay / voxel_min_points`
4. `global_costmap / local_costmap` 的 `robot_radius / inflation_radius / cost_scaling_factor`
5. MPPI: `FollowPath.vx_max / wz_max / CostCritic.cost_weight / PathFollowCritic.cost_weight`
6. `config/walking_mode.yaml` 里的 `fast_max_vel_x / slow_max_vel_x`（实际跑的速度档来源）
7. `SLOWDOWN_START_DISTANCE / SLOWDOWN_FULL_PROFILE_DISTANCE`
8. 路线 YAML 里的 `x / y / yaw_deg / action_id / say_text`
9. 如果 `lateral_assist` / `close_obstacle_guard` 失常，再回头看 `launch/navigation.launch.py` 里 pointcloud_to_laserscan 的 `min_height / max_height / angle_increment / scan_time`

如果你希望，我下一步可以继续补一版“问题到参数”的对照表，比如：

- 机器人不避障时先看哪些参数
- 到点过冲时先调哪些参数
- 原地转圈或抖动时先调哪些参数
- 贴墙太近或太保守时先调哪些参数
