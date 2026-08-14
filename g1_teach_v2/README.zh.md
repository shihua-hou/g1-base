# g1_teach_v2

[English](./README.md)

`g1_teach_v2` 是用于 Unitree G1 的上半身示教命令行工具包。
它保留了 `g1_teach` 的连续录制 / 回放工作流，并添加了以下功能：

- `snapshot`: 保存关键姿势
- `script`: 组合 `snapshot / motion / hold / script / hand / audio`
- `arm-action`: 执行 Unitree G1 固件内置动作和 APP 示教动作
- `dry-run`: 在不接触机器人的情况下验证命令和脚本
- `run-script --hand-backend inspire_ftp_right`: 通过外部 DDS 桥接控制 Inspire FTP 右手灵巧手

默认网络接口是 `enP8p1s0`。
当你的机器人使用不同的网卡时，可以通过 `--iface <name>` 进行覆盖。

## 工作目录

以下所有命令都假定你在 `g1_teach_v2` 的父目录中运行，而不是在 `g1_teach_v2` 本身内部运行。

例如：

```bash
cd /path/to/G1_Control
```

这意味着：

- 项目目录为 `g1_teach_v2/`
- 动作路径写为 `g1_teach_v2/movement/motions/...`
- 快照路径写为 `g1_teach_v2/movement/snapshots/...`
- 脚本路径写为 `g1_teach_v2/movement/scripts/...`

## 目录结构

- `g1_teach_v2/movement/motions/`: 连续运动轨迹
- `g1_teach_v2/movement/snapshots/`: 关键姿势文件
- `g1_teach_v2/movement/scripts/`: 仅手臂以及手臂+手部的脚本
- `python -m g1_teach_v2 ...`: 统一的 CLI 入口

## 推荐工作流

如果你是从头开始，这是最快的路径：

1. 录制一个 `motion`（动作）
2. 捕获几个 `snapshot`（快照姿势）
3. 分别使用 `play-motion` 和 `goto-snapshot` 进行验证
4. 将它们组合成一个 `script`（脚本）
5. 如果需要，在 `run-script` 中添加 Inspire FTP 灵巧手后端

## 1. 录制动作 (Record a Motion)

使用此命令将一段连续的上半身运动录制到 `.jsonl` 文件中。

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/my_demo_01.jsonl
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--mode`
  必填。录制模式。
  可选项：
  - `raw`
    仅采样状态，不主动控制手臂。
  - `teach_upper`
    连续示教腰部 + 双臂。
  - `right_hold_left`
    保持左臂稳定，并在右臂上启用轻松拖拽。
  - `lock_forearm`
    在锁定手腕 3 轴运动的同时进行录制。
- `--out`
  必填。输出动作文件路径。建议使用 `.jsonl` 扩展名。
- `--group`
  关节组。主要用于 `raw` 模式。
  可选项：
  - `left`
  - `right`
  - `both`
  - `upper`
- `--auto-hold`
  实验性。在示教模式下，当手臂静止后自动冻结当前姿势，再次移动时恢复拖拽模式。
- `--control_dt`
  可选。临时覆盖录制循环的时间步长（秒）。
  也接受别名 `--control-dt`。
  - 较小值：频率较高
  - 较大值：频率较低
  示例：
  - `0.01` 约为 `100 Hz`
  - `0.02` 约为 `50 Hz`
  - `0.10` 约为 `10 Hz`

### 录制时播放背景音乐

`record-motion` 可以在录制过程中播放 WAV 或 MP3 背景音乐。默认情况下，`--music` 会在真正写入动作时间轴的 `t=0` 开始播放，也就是 hold/blend 准备阶段结束后。录制打鼓、舞蹈这类需要跟音乐对齐的动作时很有用。

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/haorizi_new.jsonl --music g1_teach_v2/music/haorizi.wav --music-backend g1
```

音乐参数：

- `--music` / `--audio`
  录制时播放的 WAV 或 MP3 文件。MP3 需要系统里有 `ffmpeg`，或者设置 `G1_TEACH_FFMPEG` 指向 `ffmpeg` 可执行文件。
- `--music-backend`
  `auto`、`g1`、`system` 或 `none`。机器人上建议用 `g1`，本机扬声器调试用 `system`。
- `--music-start`
  `recording` 表示在录制动作的 `t=0` 开始；`command` 表示命令启动后立刻开始。
- `--music-delay`
  相对 `--music-start` 的偏移秒数。正数表示晚一点播放，负数表示在准备阶段内提前播放。
- `--music-stream-name`
  G1 音频流名称，默认 `music`。
- `--music-volume`
  G1 背景音乐音量，范围 `0-100`，默认 `100`。

### 当前内置录制行为

- `teach_upper`
  默认锁定两端的 `WristPitch + WristYaw` 对。
- `right_hold_left`
  默认锁定活动的右侧 `WristPitch + WristYaw`。
- 锁定方法
  将手腕目标固定在录制开始时测量的姿势。
- 锁定增益
  默认 `kp = 60.0`, `kd = 1.5`。

### 常见示例

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/explain_01.jsonl
python -m g1_teach_v2 record-motion --mode right_hold_left --out g1_teach_v2/movement/motions/right_demo_01.jsonl
python -m g1_teach_v2 record-motion --mode raw --group left --out g1_teach_v2/movement/motions/left_raw_01.jsonl
python -m g1_teach_v2 record-motion --mode teach_upper --auto-hold --out g1_teach_v2/movement/motions/explain_auto_hold_01.jsonl
python -m g1_teach_v2 record-motion --mode teach_upper --control_dt 0.02 --out g1_teach_v2/movement/motions/explain_50hz.jsonl
```

## 2. 捕获快照 (Capture a Snapshot)

使用此命令保存一个关键姿势。

`capture-snapshot` 现在是交互式的。它不再仅仅读取当前姿势。
流程如下：

1. 稍微保持当前姿势
2. 软化进入轻松拖拽状态
3. 手动引导手臂到达目标姿势
4. 按 `ENTER` 键保存
5. 释放 `arm_sdk` 并退出

```bash
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/my_pose_01.json
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--out`
  必填。输出快照路径。建议使用 `.json` 扩展名。
- `--group`
  要保存的关节组。
  可选项：
  - `left`
  - `right`
  - `both`
  - `upper`

### 常见示例

```bash
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/safe_default.json
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/explain_ready.json
python -m g1_teach_v2 capture-snapshot --group right --out g1_teach_v2/movement/snapshots/right_pose_01.json
```

### 安全提示

- 此命令会主动接管 `arm_sdk`
- 在使用前清除机器人周围的障碍物
- 当前版本不再保留单独的只读快照模式

## 3. 回放动作 (Replay One Motion)

使用此命令回放一段 `.jsonl` 连续轨迹。

```bash
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/demo_wave.jsonl
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--path`
  必填。动作文件路径。
- `--speed`
  回放速度乘数。默认：`1.0`。
  - `> 1.0`：更快
  - `< 1.0`：更慢
- `--kp`
  `--profile` 的别名。
- `--profile`
  回放配置名称。默认：`upper_playback`。
  当前主要选项：
  - `upper_playback`
- `--takeover-tau-ff`
  默认启用。在第一次 `arm_sdk` 切换期间，使用实时 `tau_est` 填充命令，然后淡出该支持，这有助于在从实时机器人模式进入回放时减少明显的下垂。
- `--no-takeover-tau-ff`
  禁用接管支持路径，用于 A/B 对比或调试。
- `--debug-takeover`
  在第一次接管窗口附近打印额外的 `q / dq / tau_est / tau_ff` 诊断信息。
- `--dry-run`
  验证文件并打印计划，而不连接到机器人。

### 常见示例

```bash
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/explain_01.jsonl --dry-run
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/explain_01.jsonl --no-takeover-tau-ff
```

## 4. 移动至快照 (Move to a Snapshot)

使用此命令将机器人平滑移动到保存的快照。

```bash
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--path`
  必填。快照文件路径。
- `--duration`
  过渡持续时间（秒）。默认：`1.5`。
- `--profile`
  回放配置名称。默认：`upper_hold`。
  当前主要选项：
  - `upper_hold`
- `--takeover-tau-ff`
  默认启用。在第一次 `arm_sdk` 接管期间保持实时 `tau_est` 支持，并在早期快照过渡期间将其淡出。
- `--no-takeover-tau-ff`
  禁用接管支持路径，回退到纯位置 PD 接管。
- `--debug-takeover`
  为第一次 `goto-snapshot` 切换打印接管诊断信息。
- `--dry-run`
  打印计划，而不连接到机器人。

### 常见示例

```bash
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/explain_ready.json --duration 2.5
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --dry-run
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --debug-takeover
```

## 5. 运行脚本 (Run a Script)

使用此命令执行一个脚本序列。

支持的步骤类型：

- `snapshot`
- `motion`
- `hold`
- `script`
- `hand`
- `audio`

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--path`
  必填。脚本文件路径。
- `--profile`
  覆盖脚本中定义的手臂回放配置。默认：`upper_playback`。
- `--hand-backend`
  可选的手部后端。默认：`none`。
  可选项：
  - `none`
  - `inspire_ftp_right`
- `--audio-backend`
  `audio` 脚本步骤使用的音频后端。可选项：`auto`、`g1`、`system`、`none`。
- `--audio-volume`
  `audio` 脚本步骤使用的 G1 音量，范围 `0-100`，默认 `100`。
- `--takeover-tau-ff`
  默认启用。将相同的首次切换支持逻辑应用于脚本中的第一个 `snapshot` 或 `motion` 步骤。
- `--no-takeover-tau-ff`
  禁用脚本回放的首次切换支持逻辑。
- `--debug-takeover`
  为脚本中的第一个 `snapshot` 或 `motion` 步骤打印接管诊断信息。
- `--dry-run`
  打印执行计划，而不连接到机器人。

### 常见示例

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/speech_suit_01.json --debug-takeover
```

## 6. 固件内置与示教动作 (arm-action)

此模块提供了一个独立的 CLI 来触发 Unitree G1 的固件预设 RPC 手臂动作服务。它与上面描述的关节级控制是互斥的（即，不能混合到 `script` 序列中）。

```bash
# 打印本地内置动作表（无需机器人）
python -m g1_teach_v2 arm-action --local-list

# 查询机器人上的可用动作（包括示教动作）
python -m g1_teach_v2 arm-action --iface enP8p1s0 --list
```

### 参数

- `--iface`
  机器人网络接口。默认：`enP8p1s0`。
- `--id` / `--name`
  执行一个内置动作（例如，17 或 `clap`）。
- `--custom`
  执行一个 APP 记录的示教动作（区分大小写）。
- `--stop`
  停止当前正在运行的自定义示教动作。
- `--auto-release`
  动作完成后自动恢复初始手臂姿势。
- `--hold-time`
  自动释放前的等待时间（秒）。默认：2.0。

### 常见示例

```bash
# 执行内置动作（通过 ID）
python -m g1_teach_v2 arm-action --id 17

# 执行内置动作（通过名称），然后自动释放
python -m g1_teach_v2 arm-action --name clap --auto-release --hold-time 3

# 执行 APP 示教动作，等待 5 秒
python -m g1_teach_v2 arm-action --custom my_wave --wait 5
```

## 7. Inspire FTP 灵巧手集成

第一次灵巧手集成仅通过 DDS：

- `g1_teach_v2` 不直接与 Modbus / 485 通信
- 首先从 `inspire_hand_ws` 启动桥接驱动程序
- 然后让 `g1_teach_v2` 通过 DDS 控制右手

推荐的运行顺序：

```bash
python inspire_hand_ws/inspire_hand_sdk/example/Headless_driver_485_r.py
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right
```

### 当前灵巧手功能边界

- 仅右手
- 无触觉集成
- 无手部动作录制
- 灵巧手支持目前仅在 `run-script` 中存在

### 串口注意事项

当前的桥接示例使用 `/dev/ttyUSB1`。
如果您的机器人使用不同的串口，请首先更新桥接脚本。

## 8. `hand` 步骤语法

第一版中支持两种形式。

### 形式 1: 预设 (Preset)

```json
{"type": "hand", "preset": "count_2", "duration": 0.35}
```

### 形式 2: 目标 (Targets)

```json
{"type": "hand", "targets": {"index": 0.0, "middle": 0.0}, "duration": 0.4}
```

### 支持的键

- `pinky` (小指)
- `ring` (无名指)
- `middle` (中指)
- `index` (食指)
- `thumb_bend` (拇指弯曲)
- `thumb_rotation` (拇指旋转)

所有值都标准化为 `0.0 ~ 1.0`。

### 内置预设

- `open_all` (全开)
- `close_all` (全关)
- `relaxed` (放松)
- `count_1` (计数 1)
- `count_2` (计数 2)
- `count_3` (计数 3)

`count_3` 固定为：
- 食指打开
- 中指打开
- 无名指打开
- 小指闭合
- 拇指弯曲
- `thumb_rotation` 保持在内置中立值

## 9. 计数手势演示

代码库中已包含一个最简单的示例：

- [demo_hand_count.json](/c:/workSoftWare/code/G1_Control/g1_teach_v2/movement/scripts/demo_hand_count.json)

流程：

- 移动到 `explain_ready`
- 显示 `count_1`
- 保持 (hold)
- 显示 `count_2`
- 保持 (hold)
- 显示 `count_3`

试运行 (Dry-run)：

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right --dry-run
```

## 10. 试运行 (Dry Run)

这些命令无需连接机器人即可工作：

```bash
python -m g1_teach_v2 play-motion --path g1_teach/recordings/neautral_upper_01.jsonl --dry-run
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right --dry-run
```

## 11. 动作兼容性

`play-motion` 保持与现有 `g1_teach` `.jsonl` 文件的兼容性。
每一行必须至少包含：

- `t`
- `group`
- `joints`
- `q`

允许在回放期间忽略 `dq` 和 `tau_est` 等额外字段。

## 12. 编写脚本

本节是编写您自己的 `scripts/*.json` 的快速参考。

### 基本结构

脚本文件是一个 JSON 对象，具有以下基本结构：

```json
{
  "kind": "script",
  "version": 1,
  "defaults": {
    "profile": "upper_playback",
    "snapshot_duration": 1.5,
    "motion_speed": 1.0,
    "hand_duration": 0.4
  },
  "steps": []
}
```

### 顶层字段

- `kind`
  必须为 `"script"`。
- `version`
  目前必须为 `1`。
- `defaults`
  可选的脚本范围默认参数。
- `steps`
  必填。按顺序执行的非空列表。

### `defaults` 中支持的字段

- `profile`
  默认的手臂回放配置。通常是 `upper_playback`。
- `snapshot_duration`
  `snapshot` 步骤的默认过渡时间（秒）。
- `motion_speed`
  `motion` 步骤的默认回放速度。
- `hand_duration`
  `hand` 步骤的默认持续时间。

### 支持的步骤类型

#### 1. `snapshot`

移动到快照文件。

```json
{
  "type": "snapshot",
  "path": "snapshots/explain_ready.json",
  "duration": 2.0
}
```

#### 2. `motion`

播放动作轨迹。

```json
{
  "type": "motion",
  "path": "motions/demo_wave.jsonl",
  "speed": 1.2
}
```

#### 3. `hold`

保持在原位。

```json
{
  "type": "hold",
  "duration": 1.0
}
```

#### 4. `hand`

发送手部目标。

```json
{
  "type": "hand",
  "preset": "count_2",
  "duration": 0.35
}
```

#### 5. `script` (嵌套调用)

以内联方式运行另一个脚本文件。

```json
{
  "type": "script",
  "path": "scripts/sub_routine_01.json"
}
```

#### 6. `audio`

在脚本中播放 WAV 或 MP3 音频。`async` 默认为 `true` 时，音频会在后台播放，后续 `motion` 可以立即开始。

```json
{"type": "audio", "path": "../music/haorizi.wav", "delay": 5.5, "async": true, "stream_name": "music", "volume": 100}
```

字段：

- `type`
  必须为 `"audio"`。
- `path`
  必填。WAV 或 MP3 文件路径。MP3 需要系统里有 `ffmpeg`，或者设置 `G1_TEACH_FFMPEG` 指向 `ffmpeg` 可执行文件。
- `delay`
  可选。开始播放前等待的秒数。
- `async`
  可选。默认 `true`；设为 `false` 时会阻塞脚本直到播放完成。
- `backend`
  可选。覆盖此步骤使用的 `--audio-backend`，可选项为 `auto`、`g1`、`system`、`none`。
- `stream_name`
  可选。G1 音频流名称，默认 `music`。
- `volume` / `audio_volume`
  可选。G1 音量，范围 `0-100`，默认 `100`。只影响 `g1` / `auto` 中成功走 G1 的播放；`system` 后端受电脑系统音量控制。
- `repeat`
  可选。正整数重复次数。

### 包含所有步骤的完整示例

```json
{
  "kind": "script",
  "version": 1,
  "steps": [
    {"type": "snapshot", "path": "snapshots/explain_ready.json", "duration": 1.5},
    {"type": "motion", "path": "motions/demo_wave.jsonl", "speed": 1.0},
    {"type": "hold", "duration": 0.8},
    {"type": "hand", "preset": "count_1", "duration": 0.35},
    {"type": "hold", "duration": 0.8},
    {"type": "snapshot", "path": "snapshots/safe_default.json", "duration": 1.5}
  ]
}
```

### 推荐风格

- 保持每个动作文件专注于一个清晰的含义
- 保持动作简短，通常为 1 到 3 秒
- 在强语义动作之后添加 `hold`（保持）
- 在段落之间（如果有用）返回 `safe_default` 或 `explain_ready`
- 在机器人上运行新脚本之前首先使用 `--dry-run`

## 13. 从外部程序调用

CLI 可以通过三种方式由任何外部系统驱动。

### 方法 1：Shell 子进程 (任何语言)

作为子进程启动 `python -m g1_teach_v2` 并等待其退出。
检查返回码：`0` = 成功，非零 = 失败。

```bash
# Shell 调用示例
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json
```

Python 示例：

```python
import subprocess, sys

result = subprocess.run(
    [sys.executable, "-m", "g1_teach_v2",
     "run-script", "--path", "g1_teach_v2/movement/scripts/demo_hand_count.json"],
    check=True,   # 非零退出时引发 CalledProcessError
)
```

这种方法适用于 Node.js、C++、ROS 节点或任何可以生成子进程的语言。

### 方法 2：`main(argv)` — 同一 Python 进程

`cli.main()` 接受显式的 `argv` 列表，因此可以像命令行一样调用它，而无需生成新进程。

```python
from g1_teach_v2.cli import main

# 等同于: python -m g1_teach_v2 run-script --path ...
exit_code = main(["run-script", "--path", "g1_teach_v2/movement/scripts/demo_hand_count.json"])
```

支持的子命令字符串与 CLI 完全匹配：
- `"record-motion"`
- `"play-motion"`
- `"capture-snapshot"`
- `"goto-snapshot"`
- `"run-script"`

### 方法 3：直接导入底层函数 (推荐)

完全绕过 CLI 层并调用核心函数。
这为您提供了最多的控制权，并避免了 `argparse` 开销。

**运行脚本:**

```python
from g1_teach_v2.script_runner import run_script
from g1_teach_v2.robot_io import RobotSession

session = RobotSession(iface="enP8p1s0", enable_pub=True)
run_script(session, "g1_teach_v2/movement/scripts/demo_hand_count.json")
```

**播放动作:**

```python
from g1_teach_v2.motion_io import load_motion, play_motion
from g1_teach_v2.robot_io import RobotSession

trajectory = load_motion("g1_teach_v2/movement/motions/demo_wave.jsonl")
session = RobotSession(iface="enP8p1s0", enable_pub=True)
play_motion(session, trajectory, profile_name="upper_playback", speed=1.0, release=True)
```

**移动至快照:**

```python
from g1_teach_v2.snapshot_io import load_snapshot, goto_snapshot
from g1_teach_v2.robot_io import RobotSession

snapshot = load_snapshot("g1_teach_v2/movement/snapshots/default.json")
session = RobotSession(iface="enP8p1s0", enable_pub=True)
goto_snapshot(session, snapshot, duration=1.5, profile_name="upper_hold", release=True)
```

### 比较

| 方法 | 最适合 | 入口点 |
|---|---|---|
| Shell 子进程 | 跨语言调用者、ROS、外部调度程序 | `python -m g1_teach_v2 <subcommand>` |
| `main(argv)` | 简单的 Python 集成、同一进程脚本 | `cli.main([...])` |
| 直接导入函数 | 需要细粒度控制的 Python 调用者 | `run_script()` / `play_motion()` / `goto_snapshot()` |

## 14. 命令备忘录

以下是 `g1_teach_v2` 中所有核心命令及其参数的快速参考。

### 1. 录制动作 (record-motion)
**命令模板:**
```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out <output.jsonl> [--iface <iface>] [--group <group>] [--auto-hold] [--control_dt <dt>] [--music <audio>] [--music-backend g1] [--music-volume 100]
```
**参数:**
- `--mode`: 录制模式。`teach_upper`（双臂）、`right_hold_left`（锁定左臂，移动右臂）、`lock_forearm`（锁定手腕）、`raw`（录制状态而不接管）。
- `--out`: 输出文件路径，建议使用 `.jsonl` 后缀。
- `--iface`: (可选) 机器人网络接口，默认 `enP8p1s0`。
- `--group`: (可选) 要保存的关节组 (`left`, `right`, `both`, `upper`)。主要用于 `raw` 或单臂模式。
- `--auto-hold`: (可选) 当手臂静止时自动冻结姿势，再次拖动以恢复。
- `--control_dt`: (可选) 覆盖控制循环时间步长（例如，`0.02` 为 50Hz）。
- `--music`: (可选) 录制时播放 WAV 或 MP3 文件。默认在录制动作 `t=0` 开始。MP3 需要 `ffmpeg`。
- `--music-backend`: (可选) 录制背景音乐的音频后端：`auto`、`g1`、`system` 或 `none`。
- `--music-start`: (可选) 选择在 `recording`（动作 `t=0`）或 `command` 启动时开始播放。
- `--music-delay`: (可选) 音乐开始时间的秒级偏移。
- `--music-volume`: (可选) G1 背景音乐音量，范围 `0-100`，默认 `100`。

### 2. 捕获快照 (capture-snapshot)
**命令模板:**
```bash
python -m g1_teach_v2 capture-snapshot --out <output.json> [--mode soft_teach] [--group <group>] [--settle-time <time>]
```
**参数:**
- `--out`: 输出文件路径，建议使用 `.json` 后缀。
- `--mode`: (可选) `soft_teach`（手动拖动到姿势，默认）或 `current_state`（保存当前活动的姿势而不拖动）。
- `--group`: (可选) 要保存的关节组 (`left`, `right`, `both`, `upper`)。
- `--settle-time`: (可选) 捕获前的等待时间，仅在 `current_state` 模式下有效。

### 3. 回放动作 (play-motion)
**命令模板:**
```bash
python -m g1_teach_v2 play-motion --path <input.jsonl> [--speed 1.0] [--profile upper_playback] [--no-takeover-tau-ff] [--dry-run]
```
**参数:**
- `--path`: 要回放的轨迹文件路径。
- `--speed`: (可选) 回放速度乘数，`1.0` 为正常速度。
- `--profile`: (可选) 要加载的控制器配置，默认 `upper_playback`。
- `--no-takeover-tau-ff`: (可选) 禁用使用前馈扭矩的平滑接管。
- `--dry-run`: (可选) 打印执行计划而不连接到机器人。

### 4. 移动至快照 (goto-snapshot)
**命令模板:**
```bash
python -m g1_teach_v2 goto-snapshot --path <input.json> [--duration 1.5] [--profile upper_hold] [--no-takeover-tau-ff]
```
**参数:**
- `--path`: 目标快照文件路径。
- `--duration`: (可选) 过渡持续时间（秒），默认 `1.5`。
- `--profile`: (可选) 要加载的控制器配置，默认 `upper_hold`。
- `--no-takeover-tau-ff`: (可选) 禁用使用前馈扭矩的平滑接管。

### 5. 运行脚本 (run-script)
**命令模板:**
```bash
python -m g1_teach_v2 run-script --path <script.json> [--hand-backend inspire_ftp_right] [--audio-backend g1] [--audio-volume 100] [--profile upper_playback] [--dry-run]
```
**参数:**
- `--path`: 脚本文件路径。
- `--hand-backend`: (可选) 为 `hand` 步骤启用 Inspire FTP 右手后端。
- `--audio-backend`: (可选) `audio` 步骤使用的音频后端：`auto`、`g1`、`system` 或 `none`。
- `--audio-volume`: (可选) G1 音量，范围 `0-100`，默认 `100`。
- `--profile`: (可选) 覆盖用于脚本内步骤的默认配置。
- `--dry-run`: (可选) 打印完整的执行流程而不连接到机器人。

### 6. 内置与示教动作 (arm-action)
**列出可用动作:**
```bash
python -m g1_teach_v2 arm-action --list [--iface <iface>]
# 或者查看本地硬编码表：
python -m g1_teach_v2 arm-action --local-list
```

**执行内置动作:**
```bash
python -m g1_teach_v2 arm-action --id <action_id> [--auto-release] [--hold-time <time>]
# 或者按名称：
python -m g1_teach_v2 arm-action --name <action_name> [--auto-release]
```
**参数:**
- `--id`: 固件内置动作 ID (例如，`17`)。
- `--name`: 固件内置动作名称别名 (例如，`clap`)。
- `--auto-release`: (可选) 动作完成后自动恢复初始手臂姿势。
- `--hold-time`: (可选) 触发自动释放前的等待时间，默认 `2.0` 秒。

**执行/停止 APP 示教动作:**
```bash
python -m g1_teach_v2 arm-action --custom <action_name> [--wait <time>]
# 强制停止当前运行的示教动作：
python -m g1_teach_v2 arm-action --stop
```
**参数:**
- `--custom`: 执行您通过 APP 录制的示教动作 (区分大小写)。
- `--wait`: (可选) 示教动作是非阻塞的；这会强制命令在退出之前等待指定的秒数。
- `--stop`: 立即停止当前正在执行的 APP 示教动作。
