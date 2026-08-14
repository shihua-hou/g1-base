import math
import time
from collections import deque
from statistics import mean

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


NEAR_THRESHOLD = 3.0
DANGER_THRESHOLD = 1.5
FRONT_HALF_ANGLE = 45.0
FRONT_WIDE_ANGLE = 90.0
SAMPLE_INTERVAL = 0.2


def append_window(window, value, limit=200):
    window.append((time.time(), value))
    while len(window) > limit:
        window.popleft()


def hz_from_window(window, span=3.0):
    now = time.time()
    recent = [stamp for stamp, _ in window if now - stamp <= span]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


class DiagObstacleNode(Node):
    def __init__(self):
        super().__init__("diag_obstacle")
        self.declare_parameter("cloud_topic", "/lio/cloud_world")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("planner_cmd_topic", "/cmd_vel")
        self.declare_parameter("executed_cmd_topic", "/cmd_vel_executed")
        self.declare_parameter("motion_source_topic", "/motion_source")

        self.cloud_window = deque()
        self.scan_window = deque()
        self.costmap_window = deque()
        self.plan_window = deque()
        self.planner_window = deque()
        self.executed_window = deque()

        self.latest = {
            "cloud_points": 0,
            "scan_valid": 0,
            "scan_near": 0,
            "scan_min_range": float("inf"),
            "scan_min_angle": 0.0,
            "scan_front45_near": 0,
            "scan_front90_near": 0,
            "scan_front45_min_r": float("inf"),
            "scan_front90_min_r": float("inf"),
            "costmap_lethal": 0,
            "costmap_inflated": 0,
            "plan_points": 0,
            "planner_vx": 0.0,
            "planner_wz": 0.0,
            "executed_vx": 0.0,
            "executed_wz": 0.0,
            "motion_source": "unknown",
        }
        self.samples = []

        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self.cloud_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            self.costmap_callback,
            10,
        )
        self.create_subscription(
            Path, self.get_parameter("plan_topic").value, self.plan_callback, 10
        )
        self.create_subscription(
            Twist,
            self.get_parameter("planner_cmd_topic").value,
            self.planner_cmd_callback,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("executed_cmd_topic").value,
            self.executed_cmd_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("motion_source_topic").value,
            self.motion_source_callback,
            10,
        )
        self.create_timer(SAMPLE_INTERVAL, self.sample)

        self.get_logger().info("diag_obstacle 已启动，开始采样导航与避障状态。")

    def cloud_callback(self, msg):
        append_window(self.cloud_window, msg.width * msg.height)
        self.latest["cloud_points"] = msg.width * msg.height

    def scan_callback(self, msg):
        append_window(self.scan_window, len(msg.ranges))
        valid = 0
        near = 0
        min_r = float("inf")
        min_angle = 0.0
        front45_near = 0
        front90_near = 0
        front45_min_r = float("inf")
        front90_min_r = float("inf")

        for index, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if not (msg.range_min < distance < msg.range_max):
                continue
            valid += 1
            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)

            if distance < NEAR_THRESHOLD:
                near += 1
                if abs(angle_deg) <= FRONT_HALF_ANGLE:
                    front45_near += 1
                    front45_min_r = min(front45_min_r, distance)
                if abs(angle_deg) <= FRONT_WIDE_ANGLE:
                    front90_near += 1
                    front90_min_r = min(front90_min_r, distance)

            if distance < min_r:
                min_r = distance
                min_angle = angle_deg

        self.latest.update(
            {
                "scan_valid": valid,
                "scan_near": near,
                "scan_min_range": min_r,
                "scan_min_angle": min_angle,
                "scan_front45_near": front45_near,
                "scan_front90_near": front90_near,
                "scan_front45_min_r": front45_min_r,
                "scan_front90_min_r": front90_min_r,
            }
        )

    def costmap_callback(self, msg):
        append_window(self.costmap_window, len(msg.data))
        lethal = sum(1 for value in msg.data if value >= 100)
        inflated = sum(1 for value in msg.data if 1 <= value < 100)
        self.latest["costmap_lethal"] = lethal
        self.latest["costmap_inflated"] = inflated

    def plan_callback(self, msg):
        append_window(self.plan_window, len(msg.poses))
        self.latest["plan_points"] = len(msg.poses)

    def planner_cmd_callback(self, msg):
        append_window(self.planner_window, msg.linear.x)
        self.latest["planner_vx"] = msg.linear.x
        self.latest["planner_wz"] = msg.angular.z

    def executed_cmd_callback(self, msg):
        append_window(self.executed_window, msg.linear.x)
        self.latest["executed_vx"] = msg.linear.x
        self.latest["executed_wz"] = msg.angular.z

    def motion_source_callback(self, msg):
        self.latest["motion_source"] = msg.data

    def sample(self):
        snapshot = {
            "cloud_hz": hz_from_window(self.cloud_window),
            "scan_hz": hz_from_window(self.scan_window),
            "costmap_hz": hz_from_window(self.costmap_window),
            "plan_hz": hz_from_window(self.plan_window),
            "planner_hz": hz_from_window(self.planner_window),
            "executed_hz": hz_from_window(self.executed_window),
            **self.latest,
        }
        self.samples.append(snapshot)

        scan_min = snapshot["scan_min_range"]
        front_min = snapshot["scan_front45_min_r"]
        self.get_logger().info(
            " | ".join(
                [
                    f"cloud={snapshot['cloud_hz']:.1f}Hz",
                    f"scan={snapshot['scan_hz']:.1f}Hz",
                    f"costmap={snapshot['costmap_hz']:.1f}Hz",
                    f"plan={snapshot['plan_points']}",
                    f"front45={snapshot['scan_front45_near']}",
                    f"front45_min={front_min:.2f}" if math.isfinite(front_min) else "front45_min=inf",
                    f"planner=({snapshot['planner_vx']:.2f},{snapshot['planner_wz']:.2f})",
                    f"executed=({snapshot['executed_vx']:.2f},{snapshot['executed_wz']:.2f})",
                    f"source={snapshot['motion_source']}",
                ]
            )
        )

    def print_report(self):
        if not self.samples:
            self.get_logger().warning("没有采到任何诊断样本。")
            return

        scan_mins = [s["scan_min_range"] for s in self.samples if math.isfinite(s["scan_min_range"])]
        front_mins = [
            s["scan_front45_min_r"]
            for s in self.samples
            if math.isfinite(s["scan_front45_min_r"])
        ]
        blocked_count = sum(
            1
            for s in self.samples
            if s["scan_front45_near"] > 0 and abs(s["executed_vx"]) < 0.01
        )
        planner_exec_gap = [
            abs(s["planner_vx"] - s["executed_vx"]) for s in self.samples
        ]

        lines = [
            "=" * 72,
            "diag_obstacle 分析报告",
            f"总样本数: {len(self.samples)}",
            f"点云平均频率: {mean(s['cloud_hz'] for s in self.samples):.2f} Hz",
            f"Scan 平均频率: {mean(s['scan_hz'] for s in self.samples):.2f} Hz",
            f"局部代价地图平均频率: {mean(s['costmap_hz'] for s in self.samples):.2f} Hz",
            f"全局路径平均点数: {mean(s['plan_points'] for s in self.samples):.1f}",
            (
                f"最近激光最小距离: min={min(scan_mins):.2f}m avg={mean(scan_mins):.2f}m"
                if scan_mins
                else "最近激光最小距离: 无有效数据"
            ),
            (
                f"前方 ±45° 最小距离: min={min(front_mins):.2f}m avg={mean(front_mins):.2f}m"
                if front_mins
                else "前方 ±45° 最小距离: 无有效数据"
            ),
            f"前方受阻且执行速度接近零的样本数: {blocked_count}",
            f"planner/executed 线速度平均差值: {mean(planner_exec_gap):.3f}",
            "=" * 72,
        ]
        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = DiagObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
