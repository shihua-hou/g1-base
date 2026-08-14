#!/usr/bin/env python3
"""把示教录制的 .jsonl 轨迹打进网页 3D 预览用的 g1_motions.json。

webapp/assets/g1_motions.json 是 robot3d.js 用来驱动模型的动作包，原本没有
生成脚本，动作换了就只能手改。这个脚本补上这一环：

    # 列出动作包里现有的动作
    python scripts/pack_web_motion.py --list

    # 加/换一个动作（key 已存在就覆盖）
    python scripts/pack_web_motion.py add \\
        --key speech_05 --label "讲解手势" \\
        --src config/movement/motions/speech_05.jsonl

    # 删掉一个
    python scripts/pack_web_motion.py remove --key wave_copy

源 .jsonl 每行形如
    {"t": 0.0096, "group": "upper", "joints": [12,15,...,28], "q": [...]}
其中 q 与 JOINT_NAMES 一一对应（顺序即 joints 里的关节 ID 顺序）。
录制是 65Hz 左右，网页预览用不上那么密，默认抽到 22Hz。
"""

import argparse
import json
import os
import sys

# q 数组的顺序 = 上肢 15 个关节，和 URDF 里的关节名一一对应
JOINT_NAMES = [
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

DEFAULT_PACK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "webapp", "assets", "g1_motions.json")


def load_pack(path):
    with open(path, "r", encoding="utf-8") as fp:
        return json.load(fp)


def save_pack(path, pack):
    # 紧凑写法：这个文件要走网络，别塞无谓的空格
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        json.dump(pack, fp, ensure_ascii=False, separators=(",", ":"))


def read_jsonl(src):
    rows = []
    with open(src, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "q" in obj and "t" in obj:
                rows.append(obj)
    if len(rows) < 5:
        raise SystemExit(f"{src} 不是关节轨迹（可能是 SDK 内置动作的占位文件）")
    if len(rows[0]["q"]) != len(JOINT_NAMES):
        raise SystemExit(f"{src} 每帧 {len(rows[0]['q'])} 个关节，期望 {len(JOINT_NAMES)} 个")
    return rows


def resample(rows, hz):
    """按固定时间间隔抽稀，首尾帧一定保留。"""
    t0 = rows[0]["t"]
    step = 1.0 / hz
    out = []
    next_t = 0.0
    for row in rows:
        rel = row["t"] - t0
        if not out or rel >= next_t:
            out.append((round(rel, 3), [round(v, 4) for v in row["q"]]))
            next_t = rel + step
    last_rel = round(rows[-1]["t"] - t0, 3)
    if out[-1][0] != last_rel:
        out.append((last_rel, [round(v, 4) for v in rows[-1]["q"]]))
    return out


def cmd_list(args):
    pack = load_pack(args.pack)
    print(f"{'key':<22}{'label':<14}{'时长':>8}{'帧数':>8}")
    for key, m in pack.items():
        print(f"{key:<22}{m.get('label', ''):<14}{m.get('duration', 0):>7.1f}s{len(m.get('frames', [])):>8}")


def cmd_add(args):
    pack = load_pack(args.pack)
    rows = read_jsonl(args.src)
    samples = resample(rows, args.hz)
    times = [t for t, _ in samples]
    frames = [q for _, q in samples]
    pack[args.key] = {
        "jointNames": JOINT_NAMES,
        "times": times,
        "frames": frames,
        "duration": round(times[-1], 3),
        "label": args.label,
    }
    save_pack(args.pack, pack)
    print(f"已写入 {args.key}（{args.label}）：{times[-1]:.1f}s / {len(frames)} 帧 / {len(frames)/max(times[-1],0.001):.1f}Hz")
    print(f"动作包现有 {len(pack)} 个动作，{os.path.getsize(args.pack)/1024:.0f} KB")


def cmd_remove(args):
    pack = load_pack(args.pack)
    if args.key not in pack:
        raise SystemExit(f"动作包里没有 {args.key}")
    pack.pop(args.key)
    save_pack(args.pack, pack)
    print(f"已删除 {args.key}，现有 {len(pack)} 个动作")


def main(argv=None):
    parser = argparse.ArgumentParser(description="维护网页 3D 预览的动作包")
    parser.add_argument("--pack", default=os.path.normpath(DEFAULT_PACK), help="g1_motions.json 路径")
    sub = parser.add_subparsers(dest="cmd")

    p_list = sub.add_parser("list", help="列出动作包内容")
    p_list.set_defaults(func=cmd_list)

    p_add = sub.add_parser("add", help="加入/覆盖一个动作")
    p_add.add_argument("--key", required=True)
    p_add.add_argument("--label", required=True, help="界面上显示的中文名")
    p_add.add_argument("--src", required=True, help="示教录制的 .jsonl")
    p_add.add_argument("--hz", type=float, default=22.0, help="抽稀后的帧率，默认 22")
    p_add.set_defaults(func=cmd_add)

    p_rm = sub.add_parser("remove", help="删除一个动作")
    p_rm.add_argument("--key", required=True)
    p_rm.set_defaults(func=cmd_remove)

    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 1
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
