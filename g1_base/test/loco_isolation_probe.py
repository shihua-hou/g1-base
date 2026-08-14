#!/usr/bin/env python3
"""
Isolate which Unitree SDK component causes LocoClient.Move() to stall ~200ms
inside the g1_base worker.

Run on PC2 ONLY, after stopping bot-mind / g1_base / unitree_sdk_bridge:

    sudo systemctl stop bot-mind
    pkill -f g1_base.unitree_sdk_bridge || true
    pkill -f g1_base_manager || true
    pkill -f g1_control_server || true

    source /home/unitree/g1_base/robot_env.sh   # or your env script
    python3 test/loco_isolation_probe.py --net-if enP8p1s0

Each stage adds one extra SDK object on top of the previous one and replays
the same 20Hz Move(0,0,0) burst that nav_core's MotionController does. The
stage where p99 jumps from ~2ms to ~200ms is the culprit.

Stages:
    1. loco_only          : ChannelFactoryInitialize + LocoClient
    2. + arm_client       : also create G1ArmActionClient
    3. + lowstate_sub     : also subscribe rt/lowstate at 500Hz
    4. + arm_sdk_pub      : also create rt/arm_sdk publisher (full g1_base shape)
    5. close_then_burst   : Close() the lowstate sub and burst again — verifies
                            that ChannelSubscriber.Close() actually frees the
                            DDS reader and Move() returns to ~2ms. Required to
                            validate the on-demand-subscribe fix.

Read-only: never sends non-zero velocity; only Move(0,0,0).
"""

import argparse
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Iterable, Optional


DEFAULT_SDK_PATHS = (
    "/home/unitree/unitree_sdk2_python",
    "/home/lemon/Super-LIO/thirdparty/unitree_sdk2_python",
)

CONFLICT_KEYWORDS = (
    "g1_base.unitree_sdk_bridge",
    "g1_base.g1_base_manager",
    "g1_control_server",
    "test_pc1_latency.py",
    "g1_loco_client_example",
    "loco_ack_probe.py",
)


def add_sdk_path(explicit_path: Optional[str]) -> None:
    candidates: Iterable[str]
    if explicit_path:
        candidates = (explicit_path,)
    else:
        env_path = os.environ.get("UNITREE_SDK2_PYTHON", "").strip()
        candidates = (env_path, *DEFAULT_SDK_PATHS) if env_path else DEFAULT_SDK_PATHS

    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        if path.is_dir():
            sys.path.insert(0, str(path))
            return


def find_conflicting_processes() -> list[str]:
    try:
        output = subprocess.check_output(
            ["ps", "-eo", "pid=,ppid=,args="], text=True
        )
    except Exception:
        return []
    me = os.getpid()
    out = []
    for raw in output.splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 2)
        try:
            pid = int(parts[0])
        except (ValueError, IndexError):
            continue
        if pid == me:
            continue
        if any(k in line for k in CONFLICT_KEYWORDS):
            out.append(line)
    return out


def burst_move(loco, count: int, hz: float) -> list[float]:
    """Replay nav_core's _loop: call Move(0,0,0) at hz, return per-call ms."""
    period = 1.0 / hz
    timings: list[float] = []
    next_tick = time.perf_counter()
    for _ in range(count):
        now = time.perf_counter()
        if now < next_tick:
            time.sleep(next_tick - now)
        t0 = time.perf_counter()
        try:
            loco.Move(0.0, 0.0, 0.0)
        except Exception as exc:
            print(f"  Move() raised: {exc!r}", flush=True)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        timings.append(elapsed_ms)
        next_tick += period
    return timings


def summary(label: str, timings: list[float]) -> None:
    if not timings:
        print(f"[{label}] no samples")
        return
    timings_sorted = sorted(timings)
    p50 = timings_sorted[len(timings_sorted) // 2]
    p95 = timings_sorted[int(len(timings_sorted) * 0.95)]
    p99 = timings_sorted[int(len(timings_sorted) * 0.99)]
    over_150 = sum(1 for x in timings if x >= 150.0)
    over_50 = sum(1 for x in timings if x >= 50.0)
    print(
        f"[{label}] n={len(timings)} "
        f"mean={statistics.mean(timings):6.1f}ms "
        f"p50={p50:6.1f}ms p95={p95:6.1f}ms p99={p99:6.1f}ms "
        f"max={max(timings):6.1f}ms "
        f">=50ms: {over_50}/{len(timings)} "
        f">=150ms: {over_150}/{len(timings)}"
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Isolate g1_base loco_move slow root cause")
    parser.add_argument("--net-if", default="enP8p1s0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--sdk-path", default=None)
    parser.add_argument("--burst-count", type=int, default=200,
                        help="Move(0,0,0) calls per stage")
    parser.add_argument("--burst-hz", type=float, default=20.0,
                        help="Match nav_core CONTROL_LOOP_HZ")
    parser.add_argument("--timeout", type=float, default=0.2,
                        help="LocoClient.SetTimeout — keep at 0.2 to match prod")
    parser.add_argument("--settle", type=float, default=0.5,
                        help="Sleep between stages so DDS state can stabilize")
    parser.add_argument("--allow-concurrent", action="store_true")
    parser.add_argument("--skip-stages", default="",
                        help="Comma-separated stage names to skip "
                             "(loco_only,arm_client,lowstate_sub,arm_sdk_pub,close_then_burst)")
    args = parser.parse_args(argv)

    conflicts = find_conflicting_processes()
    if conflicts and not args.allow_concurrent:
        print("Refusing to probe while possible g1_base/SDK clients are active:")
        for line in conflicts:
            print(f"  {line}")
        print("Stop g1_base / bot-mind first, or pass --allow-concurrent.")
        return 3

    add_sdk_path(args.sdk_path)
    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
            ChannelPublisher,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    except Exception as exc:
        print(f"Failed to import unitree_sdk2py: {exc}", file=sys.stderr)
        return 2

    try:
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    except ImportError:
        G1ArmActionClient = None
        print("warn: G1ArmActionClient not importable — 'arm_client' stage will be skipped")

    skip = {s.strip() for s in args.skip_stages.split(",") if s.strip()}

    print("=== loco isolation probe ===")
    print(f"net_if={args.net_if} domain={args.domain} timeout={args.timeout}s "
          f"burst={args.burst_count}@{args.burst_hz}Hz")

    ChannelFactoryInitialize(args.domain, args.net_if)

    print("\n--- creating LocoClient ---")
    loco = LocoClient()
    loco.SetTimeout(args.timeout)
    loco.Init()
    print("LocoClient ready")

    # --- stage 1: loco only ---
    if "loco_only" not in skip:
        time.sleep(args.settle)
        print("\n[stage 1] loco only — burst Move(0,0,0)")
        summary("loco_only", burst_move(loco, args.burst_count, args.burst_hz))

    # --- stage 2: + arm client ---
    arm = None
    if "arm_client" not in skip and G1ArmActionClient is not None:
        time.sleep(args.settle)
        print("\n[stage 2] + G1ArmActionClient.Init()")
        arm = G1ArmActionClient()
        arm.Init()
        time.sleep(args.settle)
        summary("+arm_client", burst_move(loco, args.burst_count, args.burst_hz))

    # --- stage 3: + lowstate subscriber ---
    lowstate_holder = {"msg": None}

    def _on_lowstate(msg):
        lowstate_holder["msg"] = msg

    lowstate_sub = None
    if "lowstate_sub" not in skip:
        time.sleep(args.settle)
        print("\n[stage 3] + ChannelSubscriber('rt/lowstate', LowState_) @ 500Hz")
        lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_sub.Init(_on_lowstate, 10)
        # wait until lowstate flowing so we measure with a real receive load
        deadline = time.time() + 3.0
        while lowstate_holder["msg"] is None and time.time() < deadline:
            time.sleep(0.05)
        if lowstate_holder["msg"] is None:
            print("  warn: no lowstate received within 3s; continuing anyway")
        else:
            print("  lowstate flowing")
        time.sleep(args.settle)
        summary("+lowstate_sub", burst_move(loco, args.burst_count, args.burst_hz))

    # --- stage 4: + arm_sdk publisher ---
    arm_pub = None
    if "arm_sdk_pub" not in skip:
        time.sleep(args.settle)
        print("\n[stage 4] + ChannelPublisher('rt/arm_sdk', LowCmd_)")
        arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        arm_pub.Init()
        time.sleep(args.settle)
        summary("+arm_sdk_pub", burst_move(loco, args.burst_count, args.burst_hz))

    # --- stage 5: close lowstate sub and burst again ---
    if "close_then_burst" not in skip and lowstate_sub is not None:
        time.sleep(args.settle)
        print("\n[stage 5] lowstate_sub.Close() then burst Move(0,0,0)")
        try:
            lowstate_sub.Close()
        except Exception as exc:
            print(f"  warn: Close() raised: {exc!r}")
        # let DDS reader fully unwind and any pending samples drain
        time.sleep(max(args.settle, 0.5))
        summary("close_then_burst", burst_move(loco, args.burst_count, args.burst_hz))

    print("\n=== interpretation ===")
    print("If p99 jumps at:")
    print("  +arm_client       -> G1ArmActionClient is fighting LocoClient on sport response.")
    print("  +lowstate_sub     -> rt/lowstate (500Hz) is starving the DDS receive thread.")
    print("  +arm_sdk_pub      -> arm_sdk publisher is interfering (less likely).")
    print("  loco_only         -> something outside this script (env, bot_mind ghost) is at play.")
    print("close_then_burst should drop p99 back near ~2ms — this validates that the")
    print("on-demand-subscribe fix in unitree_sdk_bridge.py will actually work.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
