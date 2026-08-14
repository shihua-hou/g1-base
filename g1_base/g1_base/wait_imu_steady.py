"""Wait for Livox IMU samples to be steady before LIO initialization.

BMI088 zero-g offset can be as large as +/-50mg. The acceleration magnitude
gate is intentionally a functional sanity band, while gyro and variance carry
the main "is the robot still moving?" decision.
"""

import math
import sys
import time
from collections import deque
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSPresetProfiles
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None
    ExternalShutdownException = Exception
    Node = object
    QoSPresetProfiles = None
    Imu = None


STATUS_WAITING = "waiting"
STATUS_PASSED = "passed"
STATUS_TIMEOUT = "timeout"
EXIT_IMU_NOT_STEADY = 4


@dataclass(frozen=True)
class ImuWindowStats:
    sample_count: int = 0
    duration_sec: float = 0.0
    gyro_norm: float = float("inf")
    accel_norm: float = float("inf")
    max_gyro_norm: float = float("inf")
    min_accel_norm: float = float("inf")
    max_accel_norm: float = float("inf")
    gyro_var: float = float("inf")
    accel_var: float = float("inf")


class ImuSteadyDetector:
    def __init__(
        self,
        window_sec=2.0,
        window_slack_sec=0.1,
        gyro_max_rad_s=0.05,
        # G1 站立时双足平衡控制器会产生轻微抖动，需放宽 BMI088 零偏（±50mg）之外的余量。
        accel_min_g=0.950,
        accel_max_g=1.050,
        gyro_var_max=1e-4,
        accel_var_max=8e-4,
    ):
        self.window_sec = float(window_sec)
        self.window_slack_sec = max(0.0, float(window_slack_sec))
        self.gyro_max_rad_s = float(gyro_max_rad_s)
        self.accel_min_g = float(accel_min_g)
        self.accel_max_g = float(accel_max_g)
        self.gyro_var_max = float(gyro_var_max)
        self.accel_var_max = float(accel_var_max)
        self.samples = deque()
        self.latest_stats = ImuWindowStats()
        self.ready = False

    def add_sample(self, stamp_sec, gyro_xyz, accel_xyz):
        gyro_norm = self._norm3(gyro_xyz)
        accel_norm = self._norm3(accel_xyz)
        self.samples.append((float(stamp_sec), gyro_norm, accel_norm))
        self._drop_old_samples(float(stamp_sec))
        self.latest_stats = self._compute_stats()
        self.ready = self._is_steady(self.latest_stats)
        return self.ready

    def status(self, now_sec, start_sec, timeout_sec):
        if self.ready:
            return STATUS_PASSED
        if float(now_sec) - float(start_sec) >= float(timeout_sec):
            return STATUS_TIMEOUT
        return STATUS_WAITING

    @staticmethod
    def _norm3(values):
        x, y, z = values
        return math.sqrt(x * x + y * y + z * z)

    def _drop_old_samples(self, stamp_sec):
        cutoff = stamp_sec - self.window_sec
        while self.samples and self.samples[0][0] < cutoff - 1e-9:
            self.samples.popleft()

    def _compute_stats(self):
        if not self.samples:
            return ImuWindowStats()

        stamps = [item[0] for item in self.samples]
        gyro_values = [item[1] for item in self.samples]
        accel_values = [item[2] for item in self.samples]
        return ImuWindowStats(
            sample_count=len(self.samples),
            duration_sec=stamps[-1] - stamps[0],
            gyro_norm=gyro_values[-1],
            accel_norm=accel_values[-1],
            max_gyro_norm=max(gyro_values),
            min_accel_norm=min(accel_values),
            max_accel_norm=max(accel_values),
            gyro_var=self._variance(gyro_values),
            accel_var=self._variance(accel_values),
        )

    @staticmethod
    def _variance(values):
        if not values:
            return float("inf")
        mean = sum(values) / len(values)
        return sum((value - mean) ** 2 for value in values) / len(values)

    def _is_steady(self, stats):
        required_duration = max(0.0, self.window_sec - self.window_slack_sec)
        return (
            stats.duration_sec >= required_duration
            and stats.max_gyro_norm < self.gyro_max_rad_s
            and stats.min_accel_norm >= self.accel_min_g
            and stats.max_accel_norm <= self.accel_max_g
            and stats.gyro_var < self.gyro_var_max
            and stats.accel_var < self.accel_var_max
        )


class WaitImuSteadyNode(Node):
    def __init__(self):
        super().__init__("wait_imu_steady")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("window_sec", 2.0)
        self.declare_parameter("window_slack_sec", 0.1)
        self.declare_parameter("timeout_sec", 10.0)
        self.declare_parameter("gyro_max_rad_s", 0.05)
        self.declare_parameter("accel_min_g", 0.950)
        self.declare_parameter("accel_max_g", 1.050)
        self.declare_parameter("gyro_var_max", 1e-4)
        self.declare_parameter("accel_var_max", 8e-4)
        self.declare_parameter("log_period_sec", 1.0)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.start_time = time.monotonic()
        self.last_log_time = 0.0
        self.exit_code = None
        self.detector = ImuSteadyDetector(
            window_sec=self.get_parameter("window_sec").value,
            window_slack_sec=self.get_parameter("window_slack_sec").value,
            gyro_max_rad_s=self.get_parameter("gyro_max_rad_s").value,
            accel_min_g=self.get_parameter("accel_min_g").value,
            accel_max_g=self.get_parameter("accel_max_g").value,
            gyro_var_max=self.get_parameter("gyro_var_max").value,
            accel_var_max=self.get_parameter("accel_var_max").value,
        )

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.subscription = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos
        )
        self.timer = self.create_timer(0.1, self._check_timeout)
        self.get_logger().info(
            "[wait_imu_steady] waiting for steady IMU: "
            f"topic={self.imu_topic}, window={self.detector.window_sec:.1f}s, "
            f"slack={self.detector.window_slack_sec:.2f}s, "
            f"timeout={self.timeout_sec:.1f}s, "
            f"gyro_max={self.detector.gyro_max_rad_s:.4f}rad/s, "
            f"accel_range=[{self.detector.accel_min_g:.4f},"
            f"{self.detector.accel_max_g:.4f}]g"
        )

    def _on_imu(self, msg):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        gyro = msg.angular_velocity
        accel = msg.linear_acceleration
        if self.detector.add_sample(
            now,
            (gyro.x, gyro.y, gyro.z),
            (accel.x, accel.y, accel.z),
        ):
            stats = self.detector.latest_stats
            self.get_logger().info(
                "[wait_imu_steady] steady: "
                f"duration={stats.duration_sec:.3f}s "
                f"samples={stats.sample_count} "
                f"gyro={stats.gyro_norm:.4f}rad/s "
                f"max_gyro={stats.max_gyro_norm:.4f}rad/s "
                f"accel={stats.accel_norm:.4f}g "
                f"accel_range=[{stats.min_accel_norm:.4f},"
                f"{stats.max_accel_norm:.4f}]g "
                f"var=({stats.gyro_var:.6g},{stats.accel_var:.6g})"
            )
            self.exit_code = 0
            return
        self._log_waiting(now)

    def _check_timeout(self):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        if self.detector.status(now, self.start_time, self.timeout_sec) == STATUS_TIMEOUT:
            self.get_logger().error(
                "[wait_imu_steady] timeout; 请保持机器人静止后重试"
            )
            self.exit_code = EXIT_IMU_NOT_STEADY
            return
        self._log_waiting(now)

    def _log_waiting(self, now):
        if now - self.last_log_time < self.log_period_sec:
            return
        self.last_log_time = now
        stats = self.detector.latest_stats
        if stats.sample_count == 0:
            self.get_logger().warning(
                f"[wait_imu_steady] waiting for {self.imu_topic} data"
            )
            return
        self.get_logger().warning(
            "[wait_imu_steady] still moving: "
            f"duration={stats.duration_sec:.3f}s "
            f"samples={stats.sample_count} "
            f"gyro={stats.gyro_norm:.4f}rad/s "
            f"max_gyro={stats.max_gyro_norm:.4f}rad/s "
            f"accel={stats.accel_norm:.4f}g "
            f"accel_range=[{stats.min_accel_norm:.4f},"
            f"{stats.max_accel_norm:.4f}]g "
            f"var=({stats.gyro_var:.6g},{stats.accel_var:.6g})"
        )


def main(args=None):
    if rclpy is None:
        print("rclpy is required to run wait_imu_steady", file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = WaitImuSteadyNode()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.exit_code is not None else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
