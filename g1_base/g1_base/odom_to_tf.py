import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from g1_base.common import quaternion_dict_from_yaw, yaw_from_quaternion_msg


class OdomToTFNode(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.declare_parameter("input_odom_topic", "/lio/robo/odom")
        self.declare_parameter("output_odom_topic", "/odom_2d")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("publish_rate", 20.0)

        input_topic = self.get_parameter("input_odom_topic").value
        output_topic = self.get_parameter("output_odom_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.odom_callback, odom_qos
        )
        self.publish_timer = self.create_timer(
            1.0 / max(publish_rate, 1.0), self.publish_latest
        )
        self._last_invalid_log_time = 0.0
        self._last_stale_log_time = 0.0
        self._last_input_time = 0.0
        self._last_source_stamp = None
        self._latest_pose = None
        self._latest_twist = None
        self.get_logger().info(
            f"[odom_to_tf] 已启动: {input_topic} -> TF({self.parent_frame}->{self.child_frame}) + "
            f"{output_topic}, publish_rate={publish_rate:.1f}Hz"
        )

    def odom_callback(self, msg):
        if not rclpy.ok():
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        input_values = (
            x,
            y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in input_values):
            self._warn_invalid_odom(x, y, orientation)
            return

        yaw = yaw_from_quaternion_msg(msg.pose.pose.orientation)
        if not math.isfinite(yaw):
            self._warn_invalid_odom(x, y, orientation)
            return
        q2d = quaternion_dict_from_yaw(yaw)
        if not all(math.isfinite(q2d[key]) for key in ("x", "y", "z", "w")):
            self._warn_invalid_odom(x, y, orientation)
            return

        self._latest_pose = {
            "x": x,
            "y": y,
            "qx": q2d["x"],
            "qy": q2d["y"],
            "qz": q2d["z"],
            "qw": q2d["w"],
            "pose_covariance": list(msg.pose.covariance),
        }
        self._latest_twist = msg.twist
        self._last_source_stamp = msg.header.stamp
        self._last_input_time = time.time()

    def publish_latest(self):
        if not rclpy.ok() or self._latest_pose is None:
            return

        now = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self._latest_pose["x"]
        transform.transform.translation.y = self._latest_pose["y"]
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = self._latest_pose["qx"]
        transform.transform.rotation.y = self._latest_pose["qy"]
        transform.transform.rotation.z = self._latest_pose["qz"]
        transform.transform.rotation.w = self._latest_pose["qw"]
        try:
            self.tf_broadcaster.sendTransform(transform)
        except Exception:
            if not rclpy.ok():
                return
            raise

        odom_2d = Odometry()
        odom_2d.header.stamp = now
        odom_2d.header.frame_id = self.parent_frame
        odom_2d.child_frame_id = self.child_frame
        odom_2d.pose.pose.position.x = self._latest_pose["x"]
        odom_2d.pose.pose.position.y = self._latest_pose["y"]
        odom_2d.pose.pose.position.z = 0.0
        odom_2d.pose.pose.orientation.x = self._latest_pose["qx"]
        odom_2d.pose.pose.orientation.y = self._latest_pose["qy"]
        odom_2d.pose.pose.orientation.z = self._latest_pose["qz"]
        odom_2d.pose.pose.orientation.w = self._latest_pose["qw"]
        odom_2d.pose.covariance = self._latest_pose["pose_covariance"]
        if self._latest_twist is not None:
            odom_2d.twist = self._latest_twist
        try:
            self.odom_pub.publish(odom_2d)
        except Exception:
            if not rclpy.ok():
                return
            raise

        self._warn_if_input_stale()

    def _warn_invalid_odom(self, x, y, orientation):
        now = time.time()
        if now - self._last_invalid_log_time < 1.0:
            return

        self._last_invalid_log_time = now
        self.get_logger().warning(
            "[odom_to_tf] 跳过无效里程计: "
            f"x={x}, y={y}, "
            f"q=({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})"
        )

    def _warn_if_input_stale(self):
        if self._last_input_time <= 0.0:
            return
        now = time.time()
        age = now - self._last_input_time
        if age < 1.5 or now - self._last_stale_log_time < 2.0:
            return
        self._last_stale_log_time = now
        self.get_logger().warning(
            f"[odom_to_tf] 输入里程计已 {age:.1f}s 未更新，继续重发最近一次位姿"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
