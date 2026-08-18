"""Runtime gravity alignment monitor for the navigation TF chain.

The monitor compares the gravity-aligned orientation from Madgwick
(`/imu/data`) with Super-LIO odometry (`/lio/odom`). It only reports
health; it never modifies TF.
"""

import json
import math
import sys
import time
from dataclasses import dataclass

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    DiagnosticArray = None
    DiagnosticStatus = None
    KeyValue = None
    Odometry = None
    ExternalShutdownException = Exception
    Node = object
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    Imu = None
    String = None


STATUS_WAITING = "waiting"
STATUS_OK = "ok"
STATUS_WARN = "warn"


@dataclass(frozen=True)
class GravityHealthState:
    status: str
    angle_deg: float | None
    sustained_sec: float
    message: str


def orientation_tuple(orientation):
    """Return a normalized (x, y, z, w) quaternion tuple, or None."""

    try:
        quat = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    return _normalize_quaternion(quat)


def gravity_alignment_angle_deg(imu_orientation, odom_orientation):
    """Angle between IMU-estimated up and the LIO world +Z axis.

    `imu_orientation` is treated as the Madgwick body->gravity-world rotation.
    `odom_orientation` is the Super-LIO body->world rotation. If both agree,
    odom * inverse(imu) maps gravity-world +Z back onto LIO world +Z.
    """

    imu_q = _normalize_quaternion(imu_orientation)
    odom_q = _normalize_quaternion(odom_orientation)
    if imu_q is None or odom_q is None:
        return None

    up_body = _rotate_vector((0.0, 0.0, 1.0), _quat_conjugate(imu_q))
    up_world = _normalize_vector(_rotate_vector(up_body, odom_q))
    if up_world is None:
        return None

    dot = max(-1.0, min(1.0, up_world[2]))
    return math.degrees(math.acos(dot))


class GravityHealthMonitor:
    def __init__(self, warning_angle_deg=3.0, warning_duration_sec=5.0):
        self.warning_angle_deg = float(warning_angle_deg)
        self.warning_duration_sec = float(warning_duration_sec)
        self.imu_orientation = None
        self.odom_orientation = None
        self.exceeded_since_sec = None

    def update_imu_orientation(self, orientation):
        quat = _normalize_quaternion(orientation)
        if quat is None:
            return False
        self.imu_orientation = quat
        return True

    def update_odom_orientation(self, orientation):
        quat = _normalize_quaternion(orientation)
        if quat is None:
            return False
        self.odom_orientation = quat
        return True

    def evaluate(self, now_sec):
        now_sec = float(now_sec)
        if self.imu_orientation is None or self.odom_orientation is None:
            self.exceeded_since_sec = None
            return GravityHealthState(
                STATUS_WAITING,
                None,
                0.0,
                "waiting for IMU and odometry orientation",
            )

        angle_deg = gravity_alignment_angle_deg(
            self.imu_orientation, self.odom_orientation
        )
        if angle_deg is None:
            self.exceeded_since_sec = None
            return GravityHealthState(
                STATUS_WAITING,
                None,
                0.0,
                "invalid IMU or odometry orientation",
            )

        if angle_deg > self.warning_angle_deg:
            if self.exceeded_since_sec is None:
                self.exceeded_since_sec = now_sec
            sustained_sec = max(0.0, now_sec - self.exceeded_since_sec)
            if sustained_sec >= self.warning_duration_sec:
                return GravityHealthState(
                    STATUS_WARN,
                    angle_deg,
                    sustained_sec,
                    "gravity alignment warning: "
                    f"{angle_deg:.2f} deg > {self.warning_angle_deg:.2f} deg "
                    f"for {sustained_sec:.1f}s",
                )
            return GravityHealthState(
                STATUS_OK,
                angle_deg,
                sustained_sec,
                "gravity alignment above threshold, waiting for duration: "
                f"{angle_deg:.2f} deg for {sustained_sec:.1f}s",
            )

        self.exceeded_since_sec = None
        return GravityHealthState(
            STATUS_OK,
            angle_deg,
            0.0,
            f"gravity alignment OK: {angle_deg:.2f} deg",
        )


class GravityHealthNode(Node):
    def __init__(self):
        super().__init__("gravity_health")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/lio/odom")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("detail_topic", "/navigation_manager/detail")
        self.declare_parameter("warning_angle_deg", 3.0)
        self.declare_parameter("warning_duration_sec", 5.0)
        self.declare_parameter("max_imu_age", 1.0)
        self.declare_parameter("max_odom_age", 1.0)
        self.declare_parameter("publish_period_sec", 1.0)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self.detail_topic = self.get_parameter("detail_topic").value
        self.max_imu_age = float(self.get_parameter("max_imu_age").value)
        self.max_odom_age = float(self.get_parameter("max_odom_age").value)
        self.monitor = GravityHealthMonitor(
            warning_angle_deg=self.get_parameter("warning_angle_deg").value,
            warning_duration_sec=self.get_parameter("warning_duration_sec").value,
        )

        self.latest_imu_wall_time = 0.0
        self.latest_odom_wall_time = 0.0
        self.last_detail_status = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        detail_qos = QoSProfile(depth=1)
        detail_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        detail_qos.reliability = ReliabilityPolicy.RELIABLE

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, sensor_qos
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )
        self.detail_pub = self.create_publisher(String, self.detail_topic, detail_qos)

        publish_period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(max(0.1, publish_period), self._publish_health)
        self.get_logger().info(
            "gravity_health started: "
            f"{self.imu_topic} + {self.odom_topic}, "
            f"threshold={self.monitor.warning_angle_deg:.2f}deg/"
            f"{self.monitor.warning_duration_sec:.1f}s"
        )

    def _on_imu(self, msg):
        quat = orientation_tuple(msg.orientation)
        if quat is None:
            self.get_logger().warning("ignoring invalid IMU orientation")
            return
        self.monitor.update_imu_orientation(quat)
        self.latest_imu_wall_time = time.monotonic()

    def _on_odom(self, msg):
        quat = orientation_tuple(msg.pose.pose.orientation)
        if quat is None:
            self.get_logger().warning("ignoring invalid odometry orientation")
            return
        self.monitor.update_odom_orientation(quat)
        self.latest_odom_wall_time = time.monotonic()

    def _publish_health(self):
        now = time.monotonic()
        state = self._input_state(now)
        if state is None:
            state = self.monitor.evaluate(now)

        self._publish_diagnostic(state)
        if state.status == STATUS_WARN or self.last_detail_status == STATUS_WARN:
            self._publish_detail(state)
        self.last_detail_status = state.status

    def _input_state(self, now):
        missing = []
        if self.latest_imu_wall_time <= 0.0:
            missing.append(f"imu={self.imu_topic}")
        elif now - self.latest_imu_wall_time > self.max_imu_age:
            missing.append(f"imu_age={now - self.latest_imu_wall_time:.2f}s")

        if self.latest_odom_wall_time <= 0.0:
            missing.append(f"odom={self.odom_topic}")
        elif now - self.latest_odom_wall_time > self.max_odom_age:
            missing.append(f"odom_age={now - self.latest_odom_wall_time:.2f}s")

        if not missing:
            return None

        self.monitor.exceeded_since_sec = None
        return GravityHealthState(
            STATUS_WAITING,
            None,
            0.0,
            "waiting for fresh gravity health inputs: " + ", ".join(missing),
        )

    def _publish_diagnostic(self, state):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "g1_base/gravity_health"
        status.hardware_id = "g1"
        status.level = (
            DiagnosticStatus.WARN
            if state.status in (STATUS_WARN, STATUS_WAITING)
            else DiagnosticStatus.OK
        )
        status.message = state.message
        status.values = [
            KeyValue(
                key="angle_deg",
                value="nan" if state.angle_deg is None else f"{state.angle_deg:.3f}",
            ),
            KeyValue(key="status", value=state.status),
            KeyValue(key="sustained_sec", value=f"{state.sustained_sec:.3f}"),
            KeyValue(
                key="warning_angle_deg",
                value=f"{self.monitor.warning_angle_deg:.3f}",
            ),
            KeyValue(
                key="warning_duration_sec",
                value=f"{self.monitor.warning_duration_sec:.3f}",
            ),
        ]
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def _publish_detail(self, state):
        detail_state = {
            STATUS_WARN: "GRAVITY_WARNING",
            STATUS_OK: "GRAVITY_OK",
            STATUS_WAITING: "GRAVITY_WAITING",
        }.get(state.status, "GRAVITY_UNKNOWN")
        payload = {
            "source": "gravity_health",
            "state": detail_state,
            "status": state.status,
            "ready": state.status == STATUS_OK,
            "angle_deg": state.angle_deg,
            "threshold_deg": self.monitor.warning_angle_deg,
            "sustained_sec": state.sustained_sec,
            "message": state.message,
            "stamp": time.time(),
        }
        self.detail_pub.publish(
            String(data=json.dumps(payload, sort_keys=True, ensure_ascii=False))
        )


def _normalize_quaternion(quat):
    if quat is None:
        return None
    try:
        x, y, z, w = [float(value) for value in quat]
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_conjugate(quat):
    x, y, z, w = quat
    return (-x, -y, -z, w)


def _rotate_vector(vector, quat):
    qx, qy, qz, qw = quat
    vx, vy, vz = vector

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _normalize_vector(vector):
    x, y, z = vector
    norm = math.sqrt(x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    return (x / norm, y / norm, z / norm)


def main(args=None):
    if rclpy is None:
        print(
            "rclpy and diagnostic_msgs are required to run gravity_health",
            file=sys.stderr,
        )
        return 1

    rclpy.init(args=args)
    node = GravityHealthNode()
    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
