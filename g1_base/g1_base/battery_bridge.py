#!/usr/bin/env python3
"""Battery Bridge: domain 0 的 /lf/battery_alarm → domain 42 的 sensor_msgs/BatteryState。

G1 的 unitree_hg LowState 里没有电池字段(实机确认过)，SDK 也不发
sensor_msgs/BatteryState。电池数据只存在于 domain 0 的 Unitree DDS 上：
/lf/battery_alarm (std_msgs/String)，data 字段是 JSON，含 cell_voltages。

但没有直接的 SOC 百分比字段，只有 13 节电芯电压(mV)。这里用分段线性
近似锂电池 OCV 曲线，取最低电芯(BMS 按最低电芯截止)反算 SOC。

架构跟 dds_domain_bridge 一样：fork 子进程在 domain 0 订阅，
父进程在 domain 42 发布，中间用 multiprocessing.Queue 传递。
rclpy 不在模块层 import，避免 fork 时继承 DDS 状态。
"""
from __future__ import annotations

import json
import logging
import multiprocessing as mp
import os
import signal
import sys
import time
from typing import List, Optional

logger = logging.getLogger("battery_bridge")

SOURCE_TOPIC = "/lf/battery_alarm"
TARGET_TOPIC = "/battery_state"

QUEUE_MAXSIZE = 20
CHILD_READY_TIMEOUT = 10.0


# ── 电压 → SOC ────────────────────────────────────────────────────
# 锂电池 OCV 曲线的分段线性近似(电压 V → SOC %)。
# 取最低电芯电压，因为 BMS 按最低电芯截止放电。
# 动态放电时电压因内阻偏低，反算的 SOC 会略保守，但显示用途够用。
_OCV_POINTS = [
    (3.00, 0),
    (3.30, 5),
    (3.50, 15),
    (3.70, 40),
    (3.80, 60),
    (3.90, 75),
    (4.00, 90),
    (4.20, 100),
]


def voltage_to_soc(cell_voltages_mv: List[int]) -> Optional[int]:
    """从电芯电压(mV)估算电量百分比。取最低电芯，分段线性插值。"""
    if not cell_voltages_mv:
        return None
    v_min = min(cell_voltages_mv) / 1000.0  # mV → V

    if v_min <= _OCV_POINTS[0][0]:
        return 0
    if v_min >= _OCV_POINTS[-1][0]:
        return 100

    for i in range(len(_OCV_POINTS) - 1):
        v1, s1 = _OCV_POINTS[i]
        v2, s2 = _OCV_POINTS[i + 1]
        if v1 <= v_min <= v2:
            soc = s1 + (s2 - s1) * (v_min - v1) / (v2 - v1)
            return max(0, min(100, int(round(soc))))
    return 0


# ── Subscriber (domain 0) ──────────────────────────────────────────


def _run_subscriber(queue: mp.Queue, ready: mp.Event):
    """子进程：domain 0 订阅 /lf/battery_alarm，把 data 字符串放进 queue。"""
    # fork 继承的信号处理器引用父进程对象，子进程重置避免死锁
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    os.environ["ROS_DOMAIN_ID"] = "0"

    import rclpy  # noqa: PLC0415 — deferred to avoid fork/DDS conflict
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles
    from std_msgs.msg import String

    rclpy.init(args=["--ros-args", "--log-level", "warn"])
    node = Node("battery_bridge_sub")

    def _on_battery_alarm(msg):
        try:
            if queue.full():
                # 电量数据新鲜度 > 完整性，满了丢最旧的
                try:
                    queue.get_nowait()
                except Exception:
                    pass
            queue.put_nowait(msg.data)
        except Exception:
            pass

    node.create_subscription(
        String, SOURCE_TOPIC, _on_battery_alarm,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    node.get_logger().info(
        "battery bridge subscriber on domain 0: %s" % SOURCE_TOPIC
    )
    ready.set()

    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Publisher (domain 42) ──────────────────────────────────────────


def _run_publisher(queue: mp.Queue, child_proc: mp.Process):
    """父进程：domain 42 从 queue 读 data，解析 JSON，反算 SOC，发 BatteryState。

    继承容器的 ROS_DOMAIN_ID=42。
    """
    import rclpy  # noqa: PLC0415
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import BatteryState

    rclpy.init(args=["--ros-args", "--log-level", "info"])
    node = Node("battery_bridge_pub")

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(BatteryState, TARGET_TOPIC, qos)
    node.get_logger().info(
        "battery bridge publisher on domain 42: %s" % TARGET_TOPIC
    )

    last_soc: Optional[int] = None

    def _drain_and_publish():
        nonlocal last_soc

        if not child_proc.is_alive():
            node.get_logger().error(
                "subscriber process died (exitcode=%s), exiting"
                % child_proc.exitcode
            )
            os._exit(1)

        while True:
            try:
                raw = queue.get_nowait()
            except Exception:
                break

            try:
                d = json.loads(raw)
                cell_v = d.get("cell_voltages", [])
                if not cell_v:
                    continue

                soc = voltage_to_soc(cell_v)
                if soc is None:
                    continue

                msg = BatteryState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "battery"
                msg.voltage = sum(cell_v) / 1000.0  # mV → V，总电压
                msg.percentage = float(soc) / 100.0  # 0.0-1.0
                msg.present = True
                msg.cell_voltage = [float(v) / 1000.0 for v in cell_v]

                pub.publish(msg)

                if soc != last_soc:
                    last_soc = soc
                    avg_v = sum(cell_v) / len(cell_v) / 1000.0
                    min_v = min(cell_v) / 1000.0
                    node.get_logger().info(
                        "battery: %d%% (%.3fV/cell avg, %.3fV min)"
                        % (soc, avg_v, min_v)
                    )
            except Exception as exc:
                node.get_logger().debug("parse/publish error: %s" % exc)

    # 5Hz drain，电量话题本身频率不高，够用
    node.create_timer(0.2, _drain_and_publish)

    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    try:
        executor.spin()
    except SystemExit:
        raise
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Main ──────────────────────────────────────────────────────────


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] [battery_bridge] %(message)s",
        stream=sys.stdout,
    )
    logger.info("Starting battery bridge (domain 0 → 42)")

    mp.set_start_method("fork", force=True)

    queue: mp.Queue = mp.Queue(maxsize=QUEUE_MAXSIZE)
    ready: mp.Event = mp.Event()

    child = mp.Process(
        target=_run_subscriber, args=(queue, ready),
        name="battery_bridge_sub", daemon=True,
    )

    def _shutdown(signum=None, frame=None):
        logger.info("Received signal %s, shutting down", signum)
        if child.is_alive():
            child.terminate()
            child.join(timeout=5)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    child.start()
    logger.info("Subscriber process started (pid=%d)", child.pid)

    if not ready.wait(timeout=CHILD_READY_TIMEOUT):
        logger.error(
            "Subscriber did not signal ready within %.0fs — exiting",
            CHILD_READY_TIMEOUT,
        )
        child.terminate()
        child.join(timeout=5)
        sys.exit(1)

    logger.info("Subscriber confirmed ready")

    try:
        _run_publisher(queue, child)
    except SystemExit:
        logger.info("Publisher shutting down")
        sys.exit(1)
    except Exception:
        logger.exception("Publisher process failed")
    finally:
        _shutdown()


if __name__ == "__main__":
    main()
