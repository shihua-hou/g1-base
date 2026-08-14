#!/usr/bin/env python3
"""
最小订阅测试：只订阅 /lio/cloud_world 和 /lio/robo/odom，
每秒打印一次最近一条消息的时间戳（time.time 的 wallclock，秒）。
不做任何业务逻辑，不用 executor 魔法，不用 tf。

运行：
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 diag_sub_test.py
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class DiagSub(Node):
    def __init__(self):
        super().__init__("diag_sub_test")
        self.cloud_stamp = 0.0
        self.odom_stamp = 0.0
        self.cloud_count = 0
        self.odom_count = 0

        self.create_subscription(
            PointCloud2, "/lio/cloud_world", self._cloud_cb, 10
        )
        self.create_subscription(
            Odometry, "/lio/robo/odom", self._odom_cb, 10
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            "subscribed to /lio/cloud_world and /lio/robo/odom; printing every 1s"
        )

    def _cloud_cb(self, _msg):
        self.cloud_stamp = time.time()
        self.cloud_count += 1

    def _odom_cb(self, _msg):
        self.odom_stamp = time.time()
        self.odom_count += 1

    def _tick(self):
        now = time.time()
        cloud_age = now - self.cloud_stamp if self.cloud_stamp > 0 else -1
        odom_age = now - self.odom_stamp if self.odom_stamp > 0 else -1
        self.get_logger().info(
            f"cloud: count={self.cloud_count} age={cloud_age:.2f}s | "
            f"odom: count={self.odom_count} age={odom_age:.2f}s"
        )


def main():
    rclpy.init()
    node = DiagSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
