#!/usr/bin/env python3
"""
独立 SDK 调用脚本 —— 测试向 PC1 发送动作指令的反应延迟。

按键命令：
    1  握手   (arm action_id = 27, shake_hand)
    2  挥手   (arm action_id = 25, wave_under_head)
    3  前进 0.25 m/s 持续 5s
    4  复原姿态 (arm action_id = 99, release_arm)
    s  紧急停止 (StopMove)
    h  显示帮助
    q  退出

打印的每条命令包含三个时间戳：
    T_press   — 按键回车被捕获的时刻 (asyncio loop 时钟)
    T_sdk_in  — SDK 调用开始
    T_sdk_out — SDK 调用返回
你可以同时用秒表观察机器人实际开始动作的时刻，结合这三个时间戳估算
"应用层调用开销" 与 "PC1 实际响应延迟" 的差值。

用法：
    python3 sdk_test/test_pc1_latency.py --net-if enP8p1s0
注意：必须在能与 PC1 通过 DDS 通信的机器上运行（通常就是 PC1 本机或同网段）。
"""

import argparse
import asyncio
import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

try:
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
except ImportError:
    G1ArmActionClient = None


ACTION_HANDSHAKE = 27
ACTION_WAVE = 25
ACTION_RELEASE = 99
FORWARD_VX = 0.25
FORWARD_DURATION = 5.0
FORWARD_TICK = 0.1


def now_ms():
    return time.monotonic() * 1000.0


HELP_TEXT = """
可用命令：
  1   握手 (action_id=27)
  2   挥手 (action_id=25)
  3   前进 0.25 m/s, 持续 5s
  4   复原姿态 (action_id=99)
  s   紧急停止 (StopMove)
  h   显示帮助
  q   退出
"""


class Tester:
    def __init__(self, net_if, domain_id):
        if G1ArmActionClient is None:
            raise RuntimeError(
                "无法导入 G1ArmActionClient — 请确认已安装 unitree_sdk2py"
            )

        print(f"[init] ChannelFactoryInitialize(domain_id={domain_id}, net_if={net_if})")
        t0 = now_ms()
        ChannelFactoryInitialize(domain_id, net_if)
        print(f"[init] ChannelFactory OK, {now_ms()-t0:.1f}ms")

        t0 = now_ms()
        self.loco = LocoClient()
        self.loco.SetTimeout(10.0)
        self.loco.Init()
        print(f"[init] LocoClient OK, {now_ms()-t0:.1f}ms")

        t0 = now_ms()
        self.arm = G1ArmActionClient()
        self.arm.Init()
        print(f"[init] ArmActionClient OK, {now_ms()-t0:.1f}ms")

        self._sdk_lock: asyncio.Lock = None
        self._stop_event: asyncio.Event = None

    def bind_loop(self):
        self._sdk_lock = asyncio.Lock()
        self._stop_event = asyncio.Event()

    # ---------- 阻塞调用（在线程池里跑） ----------
    def _call_arm(self, action_id):
        t_in = now_ms()
        result = self.arm.ExecuteAction(action_id)
        t_out = now_ms()
        return result, t_in, t_out

    def _call_move_once(self, vx, vy, wz):
        t_in = now_ms()
        self.loco.Move(vx, vy, wz)
        return now_ms() - t_in

    def _call_stop(self):
        t_in = now_ms()
        self.loco.StopMove()
        return now_ms() - t_in

    # ---------- 异步包装 ----------
    async def arm_action(self, label, action_id, t_press):
        async with self._sdk_lock:
            t_dispatch = now_ms()
            print(
                f"[{label}] dispatch  Δpress→dispatch={t_dispatch-t_press:.1f}ms"
            )
            result, t_in, t_out = await asyncio.to_thread(self._call_arm, action_id)
            print(
                f"[{label}] sdk_done  result={result}  "
                f"Δpress→sdk_in={t_in-t_press:.1f}ms  "
                f"sdk阻塞={t_out-t_in:.1f}ms  "
                f"Δpress→sdk_out={t_out-t_press:.1f}ms"
            )

    async def forward(self, t_press):
        label = "前进5s"
        async with self._sdk_lock:
            self._stop_event.clear()
            t_dispatch = now_ms()
            print(
                f"[{label}] dispatch  Δpress→dispatch={t_dispatch-t_press:.1f}ms"
            )

            first_dt = await asyncio.to_thread(self._call_move_once, FORWARD_VX, 0.0, 0.0)
            t_first = now_ms()
            print(
                f"[{label}] 首次 Move 已发出  "
                f"Δpress→first_move={t_first-t_press:.1f}ms  "
                f"调用阻塞={first_dt:.1f}ms"
            )

            deadline = t_first + FORWARD_DURATION * 1000.0
            ticks = 0
            while now_ms() < deadline:
                if self._stop_event.is_set():
                    print(f"[{label}] 收到紧急停止，提前退出循环")
                    break
                await asyncio.to_thread(self._call_move_once, FORWARD_VX, 0.0, 0.0)
                ticks += 1
                try:
                    await asyncio.wait_for(
                        self._stop_event.wait(), timeout=FORWARD_TICK
                    )
                    break
                except asyncio.TimeoutError:
                    pass

            stop_dt = await asyncio.to_thread(self._call_stop)
            t_done = now_ms()
            print(
                f"[{label}] 完成  ticks={ticks}  StopMove阻塞={stop_dt:.1f}ms  "
                f"Δpress→done={t_done-t_press:.1f}ms"
            )

    async def emergency_stop(self, t_press):
        label = "急停"
        # 不抢锁，立即并发发送 StopMove；同时唤醒 forward 循环
        if self._stop_event is not None:
            self._stop_event.set()
        stop_dt = await asyncio.to_thread(self._call_stop)
        t_done = now_ms()
        print(
            f"[{label}] StopMove 已发  "
            f"Δpress→stop_done={t_done-t_press:.1f}ms  阻塞={stop_dt:.1f}ms"
        )


async def stdin_reader(prompt="> "):
    loop = asyncio.get_running_loop()
    sys.stdout.write(prompt)
    sys.stdout.flush()
    line = await loop.run_in_executor(None, sys.stdin.readline)
    return line, now_ms()


async def input_loop(tester):
    print(HELP_TEXT)
    pending = set()
    while True:
        line, t_press = await stdin_reader()
        if not line:
            print("[exit] EOF")
            break
        key = line.strip().lower()
        if key in ("q", "quit", "exit"):
            print("[exit] 用户请求退出")
            break
        if key in ("", "h", "help", "?"):
            print(HELP_TEXT)
            continue

        if key == "1":
            task = asyncio.create_task(
                tester.arm_action("握手", ACTION_HANDSHAKE, t_press)
            )
        elif key == "2":
            task = asyncio.create_task(
                tester.arm_action("挥手", ACTION_WAVE, t_press)
            )
        elif key == "3":
            task = asyncio.create_task(tester.forward(t_press))
        elif key == "4":
            task = asyncio.create_task(
                tester.arm_action("复原", ACTION_RELEASE, t_press)
            )
        elif key == "s":
            task = asyncio.create_task(tester.emergency_stop(t_press))
        else:
            print(f"[error] 未识别命令: {key!r}")
            continue

        pending.add(task)
        task.add_done_callback(pending.discard)

    if pending:
        print(f"[exit] 等待 {len(pending)} 个未完成任务...")
        await asyncio.gather(*pending, return_exceptions=True)


async def amain(args):
    tester = Tester(args.net_if, args.domain_id)
    tester.bind_loop()
    print("[ready] 已就绪")
    try:
        await input_loop(tester)
    finally:
        try:
            await asyncio.to_thread(tester._call_stop)
            print("[shutdown] StopMove 已发送")
        except Exception as exc:
            print(f"[shutdown] StopMove 失败: {exc}")


def parse_args():
    p = argparse.ArgumentParser(description="PC1 SDK 反应延迟测试")
    p.add_argument("--net-if", default="enP8p1s0", help="DDS 网卡名 (默认 enP8p1s0)")
    p.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID 兼容 / DDS domain")
    return p.parse_args()


def main():
    args = parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        print("\n[exit] Ctrl-C")
    # stdin.readline 在 executor 线程里阻塞且无法取消，asyncio.run 的清理会卡住，
    # 所以这里直接强退（StopMove 已经在 amain 的 finally 里发过了）。
    os._exit(0)


if __name__ == "__main__":
    main()
