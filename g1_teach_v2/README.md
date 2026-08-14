# g1_teach_v2

[中文说明](./README.zh.md)

`g1_teach_v2` is a CLI-first upper-body teaching toolkit for Unitree G1.
It keeps the continuous record / replay workflow from `g1_teach`, and adds:

- `snapshot`: save key poses
- `script`: compose `snapshot / motion / hold / script / hand / audio`
- `arm-action`: execute Unitree G1 firmware built-in and APP-taught actions
- `dry-run`: validate commands and scripts without touching the robot
- `run-script --hand-backend inspire_ftp_right`: control an Inspire FTP right hand through an external DDS bridge

The default network interface is `enP8p1s0`.
Override it with `--iface <name>` when your robot uses a different NIC.

## Working Directory

All commands below assume you run them from the parent directory of `g1_teach_v2`, not from inside `g1_teach_v2` itself.

For example:

```bash
cd /path/to/G1_Control
```

That means:

- the project directory is `g1_teach_v2/`
- motion paths are written as `g1_teach_v2/movement/motions/...`
- snapshot paths are written as `g1_teach_v2/movement/snapshots/...`
- script paths are written as `g1_teach_v2/movement/scripts/...`

## Layout

- `g1_teach_v2/movement/motions/`: continuous motion trajectories
- `g1_teach_v2/movement/snapshots/`: key-pose files
- `g1_teach_v2/movement/scripts/`: arm-only and arm+hand scripts
- `python -m g1_teach_v2 ...`: unified CLI entry

## Recommended Workflow

If you are starting from scratch, this is the shortest path:

1. Record one `motion`
2. Capture a few `snapshot` poses
3. Validate them separately with `play-motion` and `goto-snapshot`
4. Compose them into one `script`
5. If needed, add the Inspire FTP hand backend to `run-script`

## 1. Record a Motion

Use this to record one continuous upper-body motion into a `.jsonl` file.

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/my_demo_01.jsonl
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--mode`
  Required. Recording mode.
  Choices:
  - `raw`
    Sample state only and do not actively control the arms.
  - `teach_upper`
    Continuous teaching for waist + both arms.
  - `right_hold_left`
    Keep the left arm stable and enable easy-drag on the right arm.
  - `lock_forearm`
    Record while locking wrist 3-axis motion.
- `--out`
  Required. Output motion file path. `.jsonl` is recommended.
- `--group`
  Joint group. Mainly useful for `raw`.
  Choices:
  - `left`
  - `right`
  - `both`
  - `upper`
- `--auto-hold`
  Experimental. In teach modes, freeze the current pose after the arm settles,
  then return to drag mode when moved again.
- `--control_dt`
  Optional. Temporarily override the record-loop timestep in seconds.
  `--control-dt` is accepted as an alias.
  - smaller: higher frequency
  - larger: lower frequency
  Examples:
  - `0.01` is about `100 Hz`
  - `0.02` is about `50 Hz`
  - `0.10` is about `10 Hz`

### Background Music While Recording

`record-motion` can play a WAV or MP3 file while you record. By default, `--music` starts when the recorded motion timeline starts (`t=0`), after the hold/blend preparation phase. This is useful when recording drum or dance motions against a fixed song.

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/haorizi_new.jsonl --music g1_teach_v2/music/haorizi.wav --music-backend g1
```

Music options:

- `--music` / `--audio`
  WAV or MP3 file to play during recording. MP3 requires `ffmpeg` on PATH, or `G1_TEACH_FFMPEG` pointing at the ffmpeg executable.
- `--music-backend`
  `auto`, `g1`, `system`, or `none`. Use `g1` on the robot and `system` for local speaker tests.
- `--music-start`
  `recording` starts at recorded motion `t=0`; `command` starts as soon as the record command begins.
- `--music-delay`
  Offset in seconds from `--music-start`. Positive starts later; negative starts earlier when still inside the preparation window.
- `--music-stream-name`
  G1 audio stream name. Default: `music`.
- `--music-volume`
  G1 background music volume, 0-100. Default: `100`.

### Current built-in recording behavior

- `teach_upper`
  Locks both `WristPitch + WristYaw` pairs by default
- `right_hold_left`
  Locks the active right-side `WristPitch + WristYaw` by default
- lock method
  Fix the wrist targets to the pose measured at recording start
- lock gains
  Default `kp = 60.0`, `kd = 1.5`

### Common examples

```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out g1_teach_v2/movement/motions/explain_01.jsonl
python -m g1_teach_v2 record-motion --mode right_hold_left --out g1_teach_v2/movement/motions/right_demo_01.jsonl
python -m g1_teach_v2 record-motion --mode raw --group left --out g1_teach_v2/movement/motions/left_raw_01.jsonl
python -m g1_teach_v2 record-motion --mode teach_upper --auto-hold --out g1_teach_v2/movement/motions/explain_auto_hold_01.jsonl
python -m g1_teach_v2 record-motion --mode teach_upper --control_dt 0.02 --out g1_teach_v2/movement/motions/explain_50hz.jsonl
```

## 2. Capture a Snapshot

Use this to save one key pose.

`capture-snapshot` is now interactive. It no longer just reads the current pose.
The flow is:

1. briefly hold the current pose
2. soften into easy-drag
3. hand-guide the arm to the target pose
4. press `ENTER` to save
5. release `arm_sdk` and exit

```bash
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/my_pose_01.json
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--out`
  Required. Output snapshot path. `.json` is recommended.
- `--group`
  Joint group to save.
  Choices:
  - `left`
  - `right`
  - `both`
  - `upper`

### Common examples

```bash
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/safe_default.json
python -m g1_teach_v2 capture-snapshot --out g1_teach_v2/movement/snapshots/explain_ready.json
python -m g1_teach_v2 capture-snapshot --group right --out g1_teach_v2/movement/snapshots/right_pose_01.json
```

### Safety note

- This command actively takes over `arm_sdk`
- Clear obstacles around the robot before using it
- The current version no longer keeps a separate read-only snapshot mode

## 3. Replay One Motion

Use this to replay one `.jsonl` continuous trajectory.

```bash
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/demo_wave.jsonl
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--path`
  Required. Motion file path.
- `--speed`
  Playback speed multiplier. Default: `1.0`.
  - `> 1.0`: faster
  - `< 1.0`: slower
- `--kp`
  Alias of `--profile`.
- `--profile`
  Playback profile name. Default: `upper_playback`.
  Current main option:
  - `upper_playback`
- `--takeover-tau-ff`
  Enabled by default. During the first `arm_sdk` handoff, seed the command with
  live `tau_est` and then fade that support out, which helps reduce visible sag
  when entering playback from a live robot mode.
- `--no-takeover-tau-ff`
  Disable the takeover support path for A/B comparison or debugging.
- `--debug-takeover`
  Print extra `q / dq / tau_est / tau_ff` diagnostics around the first
  takeover window.
- `--dry-run`
  Validate the file and print the plan without connecting to the robot.

### Common examples

```bash
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/demo_wave.jsonl
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/explain_01.jsonl --speed 0.8
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/explain_01.jsonl --dry-run
python -m g1_teach_v2 play-motion --path g1_teach_v2/movement/motions/explain_01.jsonl --no-takeover-tau-ff
```

## 4. Move to a Snapshot

Use this to move the robot smoothly to a saved snapshot.

```bash
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--path`
  Required. Snapshot file path.
- `--duration`
  Transition duration in seconds. Default: `1.5`.
- `--profile`
  Playback profile name. Default: `upper_hold`.
  Current main option:
  - `upper_hold`
- `--takeover-tau-ff`
  Enabled by default. Keep live `tau_est` support during the first
  `arm_sdk` takeover and fade it out during the early snapshot transition.
- `--no-takeover-tau-ff`
  Disable the takeover support path and fall back to pure position-PD takeover.
- `--debug-takeover`
  Print takeover diagnostics for the first `goto-snapshot` handoff.
- `--dry-run`
  Print the plan without connecting to the robot.

### Common examples

```bash
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/explain_ready.json --duration 2.5
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --dry-run
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --debug-takeover
```

## 5. Run a Script

Use this to execute one scripted sequence.

Supported step types:

- `snapshot`
- `motion`
- `hold`
- `script`
- `hand`
- `audio`

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--path`
  Required. Script file path.
- `--profile`
  Override the arm playback profile defined in the script. Default: `upper_playback`.
- `--hand-backend`
  Optional hand backend. Default: `none`.
  Choices:
  - `none`
  - `inspire_ftp_right`
- `--audio-backend`
  Audio backend for `audio` script steps. Choices: `auto`, `g1`, `system`, `none`.
- `--audio-volume`
  G1 volume for `audio` script steps, 0-100. Default: `100`.
- `--takeover-tau-ff`
  Enabled by default. Apply the same first-handoff support logic to the first
  `snapshot` or `motion` step in the script.
- `--no-takeover-tau-ff`
  Disable the first-handoff support logic for script playback.
- `--debug-takeover`
  Print takeover diagnostics for the first `snapshot` or `motion` step in the
  script.
- `--dry-run`
  Print the execution plan without connecting to the robot.

### Common examples

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/speech_suit_01.json --debug-takeover
```

## 6. Firmware Built-in & Teach Actions (arm-action)

This module provides an independent CLI to trigger Unitree G1's firmware-preset RPC arm action service. It is mutually exclusive with the joint-level control described above (i.e., it cannot be mixed into `script` sequences).

```bash
# Print local built-in action table (no robot needed)
python -m g1_teach_v2 arm-action --local-list

# Query available actions from the robot (including teach actions)
python -m g1_teach_v2 arm-action --iface enP8p1s0 --list
```

### Parameters

- `--iface`
  Robot network interface. Default: `enP8p1s0`.
- `--id` / `--name`
  Execute a built-in action (e.g., 17 or `clap`).
- `--custom`
  Execute an APP-recorded teach action (case-sensitive).
- `--stop`
  Stop the currently running custom teach action.
- `--auto-release`
  Automatically restore initial arm pose after the action completes.
- `--hold-time`
  Wait time in seconds before auto-release. Default: 2.0.

### Common examples

```bash
# Execute built-in action (by ID)
python -m g1_teach_v2 arm-action --id 17

# Execute built-in action (by name), then auto-release
python -m g1_teach_v2 arm-action --name clap --auto-release --hold-time 3

# Execute APP-taught action, wait 5 seconds
python -m g1_teach_v2 arm-action --custom my_wave --wait 5
```

## 7. Inspire FTP Hand Integration

The first hand integration is DDS-only:

- `g1_teach_v2` does not talk to Modbus / 485 directly
- start the bridge driver from `inspire_hand_ws` first
- then let `g1_teach_v2` control the right hand through DDS

Recommended runtime order:

```bash
python inspire_hand_ws/inspire_hand_sdk/example/Headless_driver_485_r.py
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right
```

### Current hand feature boundaries

- right hand only
- no touch integration
- no hand recording
- hand support currently exists only in `run-script`

### Serial-port note

The current bridge example uses `/dev/ttyUSB1`.
If your robot uses a different serial port, update the bridge script first.

## 8. `hand` Step Syntax

Two forms are supported in v1.

### Form 1: Preset

```json
{"type": "hand", "preset": "count_2", "duration": 0.35}
```

### Form 2: Targets

```json
{"type": "hand", "targets": {"index": 0.0, "middle": 0.0}, "duration": 0.4}
```

### Supported keys

- `pinky`
- `ring`
- `middle`
- `index`
- `thumb_bend`
- `thumb_rotation`

All values are normalized into `0.0 ~ 1.0`.

### Built-in presets

- `open_all`
- `close_all`
- `relaxed`
- `count_1`
- `count_2`
- `count_3`

`count_3` is fixed as:

- index open
- middle open
- ring open
- pinky closed
- thumb bent
- `thumb_rotation` stays at the built-in neutral value

## 9. Counting Gesture Demo

The repo already includes a minimal example:

- [demo_hand_count.json](/c:/workSoftWare/code/G1_Control/g1_teach_v2/movement/scripts/demo_hand_count.json)

Flow:

- move to `explain_ready`
- show `count_1`
- hold
- show `count_2`
- hold
- show `count_3`

Dry-run:

```bash
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right --dry-run
```

## 10. Dry Run

These commands work without connecting to the robot:

```bash
python -m g1_teach_v2 play-motion --path g1_teach/recordings/neautral_upper_01.jsonl --dry-run
python -m g1_teach_v2 goto-snapshot --path g1_teach_v2/movement/snapshots/default.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo.json --dry-run
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json --hand-backend inspire_ftp_right --dry-run
```

## 11. Motion Compatibility

`play-motion` remains compatible with existing `g1_teach` `.jsonl` files.
Each row must contain at least:

- `t`
- `group`
- `joints`
- `q`

Extra fields such as `dq` and `tau_est` are allowed and ignored during playback.

## 12. Writing Scripts

This section is the quick reference for authoring your own `scripts/*.json`.

### Base structure

A script file is a JSON object with this outer shape:

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

### Top-level fields

- `kind`
  Must be `"script"`.
- `version`
  Must currently be `1`.
- `defaults`
  Optional script-wide default parameters.
- `steps`
  Required. A non-empty list executed in order.

### Supported fields in `defaults`

- `profile`
  Default arm playback profile. Usually `upper_playback`.
- `snapshot_duration`
  Default transition time in seconds for `snapshot` steps.
- `motion_speed`
  Default playback speed for `motion` steps.
- `hand_duration`
  Default duration for `hand` steps.

### Supported step types

#### 1. `snapshot`

Move to a snapshot file.

```json
{"type": "snapshot", "path": "snapshots/explain_ready.json", "duration": 1.5}
```

Fields:

- `type`
  Must be `"snapshot"`.
- `path`
  Required. Snapshot file path.
- `duration`
  Optional. Overrides the default transition time.
- `repeat`
  Optional. Positive integer repeat count.

#### 2. `motion`

Play a motion file.

```json
{"type": "motion", "path": "motions/explain_01.jsonl", "speed": 1.0}
```

Fields:

- `type`
  Must be `"motion"`.
- `path`
  Required. Motion file path.
- `speed`
  Optional. Overrides the default playback speed.
- `repeat`
  Optional. Positive integer repeat count.

#### 3. `hold`

Pause for a fixed time.

```json
{"type": "hold", "duration": 0.8}
```

Fields:

- `type`
  Must be `"hold"`.
- `duration`
  Required. Hold time in seconds.
- `repeat`
  Optional. Positive integer repeat count.

#### 4. `script`

Run another nested script.

```json
{"type": "script", "path": "scripts/wave_then_hold.json", "repeat": 2}
```

Fields:

- `type`
  Must be `"script"`.
- `path`
  Required. Another script file path.
- `repeat`
  Optional. Positive integer repeat count.

#### 5. `hand`

Execute a hand action. This only runs for real when `--hand-backend inspire_ftp_right` is active.

Form A, by preset:

```json
{"type": "hand", "preset": "count_1", "duration": 0.35}
```

Form B, by targets:

```json
{"type": "hand", "targets": {"index": 0.0, "middle": 0.0}, "duration": 0.4}
```

Fields:

- `type`
  Must be `"hand"`.
- `preset`
  Optional. Name of a built-in hand pose preset.
- `targets`
  Optional. Per-channel normalized target values.
- `duration`
  Optional. Duration for the hand move.
- `repeat`
  Optional. Positive integer repeat count.

Notes:

- `preset` and `targets` are mutually exclusive
- `targets` values must stay within `0.0 ~ 1.0`
- if a script contains `hand` steps but no hand backend is enabled, execution fails fast

#### 6. `audio`

Play a WAV or MP3 file during a script. Async audio starts in the background, so the next motion step can begin immediately.

```json
{"type": "audio", "path": "../music/haorizi.wav", "delay": 5.5, "async": true, "stream_name": "music", "volume": 100}
```

Fields:

- `type`
  Must be `"audio"`.
- `path`
  Required. WAV or MP3 file path. MP3 requires `ffmpeg` on PATH, or `G1_TEACH_FFMPEG` pointing at the ffmpeg executable.
- `delay`
  Optional. Seconds to wait before playback starts.
- `async`
  Optional. Default: `true`. Use `false` for blocking playback.
- `backend`
  Optional. Overrides `--audio-backend` for this step. Choices: `auto`, `g1`, `system`, `none`.
- `stream_name`
  Optional. G1 audio stream name. Default: music.
- `repeat`
  Optional. Positive integer repeat count.

### Path resolution rule

Every `path` is resolved relative to the directory of the current script file, not relative to your shell working directory.

For example, if the current file is:

`g1_teach_v2/movement/scripts/my_talk.json`

Then:

- `snapshots/explain_ready.json`
  resolves to `g1_teach_v2/movement/snapshots/explain_ready.json`
- `motions/explain_01.jsonl`
  resolves to `g1_teach_v2/movement/motions/explain_01.jsonl`

### Full example

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

### Recommended style

- keep each motion file focused on one clear meaning
- keep motions short, usually 1 to 3 seconds
- add a `hold` after strong semantic actions
- return to `safe_default` or `explain_ready` between segments when useful
- use `--dry-run` first before running a new script on the robot

## 13. Calling from External Programs

The CLI can be driven by any external system in three ways.

### Method 1: Shell subprocess (any language)

Start `python -m g1_teach_v2` as a child process and wait for it to exit.
Check the return code: `0` = success, non-zero = failure.

```bash
# Example shell call
python -m g1_teach_v2 run-script --path g1_teach_v2/movement/scripts/demo_hand_count.json
```

Python example:

```python
import subprocess, sys

result = subprocess.run(
    [sys.executable, "-m", "g1_teach_v2",
     "run-script", "--path", "g1_teach_v2/movement/scripts/demo_hand_count.json"],
    check=True,   # raises CalledProcessError on non-zero exit
)
```

This approach works from Node.js, C++, ROS nodes, or any other language that can spawn a subprocess.

### Method 2: `main(argv)` �?same Python process

`cli.main()` accepts an explicit `argv` list so it can be called just like the
command line, but without spawning a new process.

```python
from g1_teach_v2.cli import main

# Equivalent to: python -m g1_teach_v2 run-script --path ...
exit_code = main(["run-script", "--path", "g1_teach_v2/movement/scripts/demo_hand_count.json"])
```

Supported subcommand strings match the CLI exactly:
- `"record-motion"`
- `"play-motion"`
- `"capture-snapshot"`
- `"goto-snapshot"`
- `"run-script"`

### Method 3: Import the underlying function directly (recommended)

Bypass the CLI layer entirely and call the core functions.
This gives you the most control and avoids `argparse` overhead.

**Run a script:**

```python
from g1_teach_v2.script_runner import run_script
from g1_teach_v2.robot_io import RobotSession

session = RobotSession(iface="enP8p1s0", enable_pub=True)
run_script(session, "g1_teach_v2/movement/scripts/demo_hand_count.json")
```

**Play a motion:**

```python
from g1_teach_v2.motion_io import load_motion, play_motion
from g1_teach_v2.robot_io import RobotSession

trajectory = load_motion("g1_teach_v2/movement/motions/demo_wave.jsonl")
session = RobotSession(iface="enP8p1s0", enable_pub=True)
play_motion(session, trajectory, profile_name="upper_playback", speed=1.0, release=True)
```

**Move to a snapshot:**

```python
from g1_teach_v2.snapshot_io import load_snapshot, goto_snapshot
from g1_teach_v2.robot_io import RobotSession

snapshot = load_snapshot("g1_teach_v2/movement/snapshots/default.json")
session = RobotSession(iface="enP8p1s0", enable_pub=True)
goto_snapshot(session, snapshot, duration=1.5, profile_name="upper_hold", release=True)
```

### Comparison

| Method | Best for | Entry point |
|---|---|---|
| Shell subprocess | Cross-language callers, ROS, external schedulers | `python -m g1_teach_v2 <subcommand>` |
| `main(argv)` | Simple Python integration, same-process scripting | `cli.main([...])` |
| Direct function import | Python callers that need fine-grained control | `run_script()` / `play_motion()` / `goto_snapshot()` |

## 14. Command Cheat Sheet

Here is a quick reference for all core commands and their parameters in `g1_teach_v2`.

### 1. Record a Motion (record-motion)
**Template Command:**
```bash
python -m g1_teach_v2 record-motion --mode teach_upper --out <output.jsonl> [--iface <iface>] [--group <group>] [--auto-hold] [--control_dt <dt>] [--music <audio>] [--music-backend g1] [--music-volume 100]
```
**Parameters:**
- `--mode`: Recording mode. `teach_upper` (both arms), `right_hold_left` (lock left, move right), `lock_forearm` (lock wrists), `raw` (record state without takeover).
- `--out`: Output file path, `.jsonl` suffix is recommended.
- `--iface`: (Optional) Robot network interface, default `enP8p1s0`.
- `--group`: (Optional) Joint group to save (`left`, `right`, `both`, `upper`). Mainly used for `raw` or single-arm modes.
- `--auto-hold`: (Optional) Freeze the pose automatically when the arm is kept still, drag again to resume.
- `--control_dt`: (Optional) Override the loop timestep (e.g., `0.02` for 50Hz).
- `--music`: (Optional) Play a WAV or MP3 file while recording. Default start is recorded motion `t=0`. MP3 requires `ffmpeg`.
- `--music-backend`: (Optional) Audio backend for recording music: `auto`, `g1`, `system`, or `none`.
- `--music-start`: (Optional) Start music at `recording` (`t=0`) or `command` start.
- --music-delay: (Optional) Offset music start in seconds.

### 2. Capture a Snapshot (capture-snapshot)
**Template Command:**
```bash
python -m g1_teach_v2 capture-snapshot --out <output.json> [--mode soft_teach] [--group <group>] [--settle-time <time>]
```
**Parameters:**
- `--out`: Output file path, `.json` suffix is recommended.
- `--mode`: (Optional) `soft_teach` (hand-drag to pose, default) or `current_state` (save current active pose without dragging).
- `--group`: (Optional) Joint group to save (`left`, `right`, `both`, `upper`).
- `--settle-time`: (Optional) Wait time before capturing, only effective in `current_state` mode.

### 3. Replay a Motion (play-motion)
**Template Command:**
```bash
python -m g1_teach_v2 play-motion --path <input.jsonl> [--speed 1.0] [--profile upper_playback] [--no-takeover-tau-ff] [--dry-run]
```
**Parameters:**
- `--path`: Trajectory file path to replay.
- `--speed`: (Optional) Playback speed multiplier, `1.0` is normal speed.
- `--profile`: (Optional) Controller profile to load, default `upper_playback`.
- `--no-takeover-tau-ff`: (Optional) Disable smooth takeover using feed-forward torque.
- `--dry-run`: (Optional) Print execution plan without connecting to the robot.

### 4. Move to a Snapshot (goto-snapshot)
**Template Command:**
```bash
python -m g1_teach_v2 goto-snapshot --path <input.json> [--duration 1.5] [--profile upper_hold] [--no-takeover-tau-ff]
```
**Parameters:**
- `--path`: Target snapshot file path.
- `--duration`: (Optional) Transition duration in seconds, default `1.5`.
- `--profile`: (Optional) Controller profile to load, default `upper_hold`.
- `--no-takeover-tau-ff`: (Optional) Disable smooth takeover using feed-forward torque.

### 5. Run a Script (run-script)
**Template Command:**
```bash
python -m g1_teach_v2 run-script --path <script.json> [--hand-backend inspire_ftp_right] [--audio-backend g1] [--audio-volume 100] [--profile upper_playback] [--dry-run]
```
**Parameters:**
- `--path`: Script file path.
- `--hand-backend`: (Optional) Enable the Inspire FTP right hand backend for `hand` steps.
- `--audio-backend`: (Optional) Audio backend for `audio` steps: `auto`, `g1`, `system`, or `none`.
- `--audio-volume`: (Optional) G1 volume, 0-100. Default: `100`.
- `--profile`: (Optional) Override the default profile used for steps inside the script.
- `--dry-run`: (Optional) Print the full execution flow without running it on the robot.

### 6. Built-in & Teach Actions (arm-action)
**List Available Actions:**
```bash
python -m g1_teach_v2 arm-action --list [--iface <iface>]
# Or view the local hardcoded table:
python -m g1_teach_v2 arm-action --local-list
```

**Execute Built-in Action:**
```bash
python -m g1_teach_v2 arm-action --id <action_id> [--auto-release] [--hold-time <time>]
# Or by name:
python -m g1_teach_v2 arm-action --name <action_name> [--auto-release]
```
**Parameters:**
- `--id`: The firmware built-in action ID (e.g., `17`).
- `--name`: The built-in action name alias (e.g., `clap`).
- `--auto-release`: (Optional) Automatically restore the initial arm pose after the action completes.
- `--hold-time`: (Optional) Wait time before auto-release triggers, default `2.0` seconds.

**Execute/Stop APP Teach Action:**
```bash
python -m g1_teach_v2 arm-action --custom <action_name> [--wait <time>]
# Force stop the current running teach action:
python -m g1_teach_v2 arm-action --stop
```
**Parameters:**
- `--custom`: Execute a teach action you recorded via the APP (case-sensitive).
- `--wait`: (Optional) Teach actions are non-blocking; this forces the command to wait for the specified seconds before exiting.
- `--stop`: Stop the currently executing APP teach action immediately.
