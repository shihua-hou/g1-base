import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from g1_base.common import quaternion_dict_from_yaw


class CmdVelMockNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_mock")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom_2d")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("dt", 0.02)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()
        self.stats_lock = threading.Lock()

        self.vx_peak = 0.0
        self.vy_peak = 0.0
        self.wz_peak = 0.0
        self.vx_sum = 0.0
        self.sample_count = 0
        self.last_nonzero_time = 0.0

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.dt = float(self.get_parameter("dt").value)

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.create_timer(self.dt, self.integration_step)
        self.create_timer(5.0, self.print_stats)

        self.get_logger().info("=" * 60)
        self.get_logger().info("cmd_vel_mock 已启动，进入干跑模式")
        self.get_logger().info(f"订阅 {cmd_vel_topic}，发布 {odom_topic} 与 TF")
        self.get_logger().info("=" * 60)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        with self.lock:
            self.last_cmd = (vx, vy, wz)

        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().info(
                f"[cmd_vel] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}"
            )
            with self.stats_lock:
                self.vx_peak = max(self.vx_peak, abs(vx))
                self.vy_peak = max(self.vy_peak, abs(vy))
                self.wz_peak = max(self.wz_peak, abs(wz))
                self.vx_sum += abs(vx)
                self.sample_count += 1
                self.last_nonzero_time = time.time()

    def integration_step(self):
        with self.lock:
            vx, vy, wz = self.last_cmd

        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * self.dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * self.dt
        dyaw = wz * self.dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        q = quaternion_dict_from_yaw(self.yaw)
        now = self.get_clock().now().to_msg()
        parent_frame = self.get_parameter("parent_frame").value
        child_frame = self.get_parameter("child_frame").value

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q["x"]
        transform.transform.rotation.y = q["y"]
        transform.transform.rotation.z = q["z"]
        transform.transform.rotation.w = q["w"]
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = parent_frame
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q["x"]
        odom.pose.pose.orientation.y = q["y"]
        odom.pose.pose.orientation.z = q["z"]
        odom.pose.pose.orientation.w = q["w"]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def print_stats(self):
        with self.stats_lock:
            if self.sample_count == 0:
                return
            avg_vx = self.vx_sum / self.sample_count
            idle = time.time() - self.last_nonzero_time
            message = (
                f"[统计] vx峰值={self.vx_peak:.3f} vy峰值={self.vy_peak:.3f} "
                f"wz峰值={self.wz_peak:.3f} vx均值={avg_vx:.3f} "
                f"采样数={self.sample_count} 空闲={idle:.1f}s"
            )
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
