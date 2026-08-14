import argparse
import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from g1_base.common import yaw_from_quaternion_msg


def hz_from_window(window, span=3.0):
    now = time.time()
    recent = [stamp for stamp in window if now - stamp <= span]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


def append_window(window, limit=200):
    window.append(time.time())
    while len(window) > limit:
        window.popleft()


def finite_or_inf(value):
    if value is None or not math.isfinite(value):
        return "inf"
    return f"{value:.2f}"


class CostmapDecayDiag(Node):
    def __init__(self, args):
        super().__init__("diag_costmap_decay")
        self.args = args

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_stamps = deque()
        self.costmap_stamps = deque()
        self.raw_costmap_stamps = deque()

        self.latest_scan = {
            "stamp": 0.0,
            "front_valid": 0,
            "front_min_range": float("inf"),
            "blocked": False,
        }
        self.latest_costmap = None
        self.latest_raw_costmap = None

        self.last_scan_blocked = False
        self.pending_clear_start = None
        self.last_report_time = 0.0

        self.create_subscription(LaserScan, args.scan_topic, self._scan_cb, 10)
        self.create_subscription(
            OccupancyGrid, args.costmap_topic, self._costmap_cb, 10
        )
        if args.raw_costmap_topic:
            self.create_subscription(
                OccupancyGrid, args.raw_costmap_topic, self._raw_costmap_cb, 10
            )

        self.create_timer(args.report_period, self._tick)

        self.get_logger().info(
            "diag_costmap_decay 已启动: "
            f"scan={args.scan_topic}, costmap={args.costmap_topic}, "
            f"raw_costmap={args.raw_costmap_topic or 'disabled'}"
        )

    def _scan_cb(self, msg):
        append_window(self.scan_stamps)

        half_angle = math.radians(self.args.front_angle_deg)
        valid = 0
        min_range = float("inf")

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if not (msg.range_min < distance < msg.range_max):
                continue

            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) > half_angle:
                continue

            valid += 1
            min_range = min(min_range, distance)

        blocked = math.isfinite(min_range) and min_range <= self.args.scan_block_distance
        now = time.time()
        self.latest_scan = {
            "stamp": now,
            "front_valid": valid,
            "front_min_range": min_range,
            "blocked": blocked,
        }

        if self.last_scan_blocked and not blocked:
            self.pending_clear_start = now
            self.get_logger().warning(
                "[event] /scan 前方障碍已离开，开始等待 local_costmap 清空"
            )
        elif (not self.last_scan_blocked) and blocked:
            self.pending_clear_start = None
            self.get_logger().warning(
                f"[event] /scan 前方出现障碍，min_range={min_range:.2f}m"
            )

        self.last_scan_blocked = blocked

    def _costmap_cb(self, msg):
        append_window(self.costmap_stamps)
        self.latest_costmap = {"stamp": time.time(), "msg": msg}

    def _raw_costmap_cb(self, msg):
        append_window(self.raw_costmap_stamps)
        self.latest_raw_costmap = {"stamp": time.time(), "msg": msg}

    def _lookup_robot_pose(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.args.base_frame,
                Time(),
            )
        except TransformException as exc:
            raise RuntimeError(str(exc)) from exc

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = yaw_from_quaternion_msg(transform.transform.rotation)
        return tx, ty, yaw

    def _region_stats(self, msg):
        frame_id = msg.header.frame_id or self.args.costmap_frame_fallback
        robot_x, robot_y, robot_yaw = self._lookup_robot_pose(frame_id)

        resolution = float(msg.info.resolution)
        width = int(msg.info.width)
        height = int(msg.info.height)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        data = msg.data

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        front_nonzero = 0
        front_lethal = 0
        front_inflated = 0
        back_nonzero = 0
        back_lethal = 0
        back_inflated = 0

        half_width = self.args.region_half_width
        front_distance = self.args.front_distance
        back_distance = self.args.back_distance

        for row in range(height):
            wy = origin_y + (row + 0.5) * resolution
            base_index = row * width
            for col in range(width):
                value = data[base_index + col]
                if value <= 0:
                    continue

                wx = origin_x + (col + 0.5) * resolution
                dx = wx - robot_x
                dy = wy - robot_y

                bx = cos_yaw * dx + sin_yaw * dy
                by = -sin_yaw * dx + cos_yaw * dy

                if abs(by) > half_width:
                    continue

                if 0.0 <= bx <= front_distance:
                    front_nonzero += 1
                    if value >= 100:
                        front_lethal += 1
                    else:
                        front_inflated += 1
                elif -back_distance <= bx < 0.0:
                    back_nonzero += 1
                    if value >= 100:
                        back_lethal += 1
                    else:
                        back_inflated += 1

        return {
            "frame_id": frame_id,
            "front_nonzero": front_nonzero,
            "front_lethal": front_lethal,
            "front_inflated": front_inflated,
            "back_nonzero": back_nonzero,
            "back_lethal": back_lethal,
            "back_inflated": back_inflated,
        }

    def _infer_suspect(self, scan_age, costmap_age, raw_costmap_age, costmap_stats, raw_stats):
        suspects = []

        if scan_age > max(1.0, 2.0 * self.args.report_period):
            suspects.append("scan_stale")
        if hz_from_window(self.scan_stamps) < 3.0:
            suspects.append("scan_hz_low")
        if costmap_age > max(1.0, 2.0 * self.args.report_period):
            suspects.append("costmap_publish_slow")
        if raw_costmap_age is not None and raw_costmap_age > max(1.0, 2.0 * self.args.report_period):
            suspects.append("raw_costmap_publish_slow")

        if not self.latest_scan["blocked"]:
            if raw_stats is not None and raw_stats["front_nonzero"] > 0:
                suspects.append("raw_costmap_not_clearing")
            elif raw_stats is None and costmap_stats is not None and costmap_stats["front_nonzero"] > 0:
                suspects.append("costmap_not_clearing")
            if (
                raw_stats is not None
                and raw_stats["front_nonzero"] == 0
                and costmap_stats is not None
                and costmap_stats["front_nonzero"] > 0
            ):
                suspects.append("inflation_or_visualization_residual")

        return suspects or ["none"]

    def _maybe_report_clear_delay(self, costmap_stats, raw_stats):
        if self.pending_clear_start is None:
            return

        raw_cleared = raw_stats is None or raw_stats["front_nonzero"] == 0
        costmap_cleared = costmap_stats is not None and costmap_stats["front_nonzero"] == 0
        if not raw_cleared and not costmap_cleared:
            return

        delay = time.time() - self.pending_clear_start
        self.get_logger().warning(
            "[event] 前方障碍从 /scan 消失到 local_costmap 前方区域清空 "
            f"耗时 {delay:.2f}s "
            f"(raw_cleared={'yes' if raw_cleared else 'no'}, "
            f"costmap_cleared={'yes' if costmap_cleared else 'no'})"
        )
        self.pending_clear_start = None

    def _tick(self):
        now = time.time()
        scan_age = now - self.latest_scan["stamp"] if self.latest_scan["stamp"] > 0 else float("inf")
        costmap_age = (
            now - self.latest_costmap["stamp"] if self.latest_costmap is not None else float("inf")
        )
        raw_costmap_age = (
            now - self.latest_raw_costmap["stamp"] if self.latest_raw_costmap is not None else None
        )

        costmap_stats = None
        raw_stats = None
        tf_error = None

        try:
            if self.latest_costmap is not None:
                costmap_stats = self._region_stats(self.latest_costmap["msg"])
            if self.latest_raw_costmap is not None:
                raw_stats = self._region_stats(self.latest_raw_costmap["msg"])
        except RuntimeError as exc:
            tf_error = str(exc)

        suspects = self._infer_suspect(
            scan_age, costmap_age, raw_costmap_age, costmap_stats, raw_stats
        )

        parts = [
            f"scan={hz_from_window(self.scan_stamps):.1f}Hz",
            f"scan_age={scan_age:.2f}s" if math.isfinite(scan_age) else "scan_age=inf",
            f"front_min={finite_or_inf(self.latest_scan['front_min_range'])}",
            f"scan_blocked={'yes' if self.latest_scan['blocked'] else 'no'}",
        ]

        if raw_stats is not None:
            parts.extend(
                [
                    f"raw={hz_from_window(self.raw_costmap_stamps):.1f}Hz",
                    f"raw_age={raw_costmap_age:.2f}s",
                    f"raw_front={raw_stats['front_nonzero']}",
                    f"raw_front_lethal={raw_stats['front_lethal']}",
                ]
            )

        if costmap_stats is not None:
            parts.extend(
                [
                    f"costmap={hz_from_window(self.costmap_stamps):.1f}Hz",
                    f"costmap_age={costmap_age:.2f}s",
                    f"front={costmap_stats['front_nonzero']}",
                    f"front_lethal={costmap_stats['front_lethal']}",
                    f"front_inflated={costmap_stats['front_inflated']}",
                    f"back={costmap_stats['back_nonzero']}",
                ]
            )

        parts.append(f"suspect={','.join(suspects)}")

        if tf_error:
            parts.append(f"tf_error={tf_error}")

        self.get_logger().info(" | ".join(parts))
        self._maybe_report_clear_delay(costmap_stats, raw_stats)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="诊断 local costmap 障碍残留/清障滞后的原因。"
    )
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--costmap-topic", default="/local_costmap/costmap")
    parser.add_argument("--raw-costmap-topic", default="/local_costmap/costmap_raw")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--costmap-frame-fallback", default="map")
    parser.add_argument("--front-angle-deg", type=float, default=45.0)
    parser.add_argument("--scan-block-distance", type=float, default=2.0)
    parser.add_argument("--front-distance", type=float, default=2.0)
    parser.add_argument("--back-distance", type=float, default=1.0)
    parser.add_argument("--region-half-width", type=float, default=0.7)
    parser.add_argument("--report-period", type=float, default=0.5)
    return parser.parse_known_args(argv)


def main(args=None):
    parsed, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    node = CostmapDecayDiag(parsed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
