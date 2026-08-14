import math
import time
from collections import defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _finite(value):
    return math.isfinite(float(value))


class NavObstacleCloudFilter(Node):
    """Publish a Nav2-friendly obstacle cloud in the Super-LIO world frame.

    Super-LIO publishes /lio/cloud_world in its 3D world frame. This node keeps
    the original world coordinates and labels the output as world. Height
    filtering is still done against a locally estimated floor plane, but the
    floor estimate is updated slowly and reused between frames.

    The node does not accumulate obstacle points. If odometry is missing or the
    stable floor plane has failed for too long it publishes an empty cloud,
    leaving obstacle expiry to STVL's voxel decay.
    """

    def __init__(self):
        super().__init__("nav_obstacle_cloud_filter")

        self.declare_parameter("input_cloud_topic", "/lio/cloud_world")
        self.declare_parameter("odom_topic", "/lio/robo/odom")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("output_cloud_topic", "/nav/obstacle_cloud")
        self.declare_parameter("output_frame", "world")

        self.declare_parameter("min_range", 0.45)
        self.declare_parameter("max_range", 3.0)
        self.declare_parameter("ground_fit_min_range", 0.7)
        self.declare_parameter("ground_fit_max_range", 4.0)
        self.declare_parameter("obstacle_min_height", 0.15)
        self.declare_parameter("obstacle_max_height", 1.6)

        self.declare_parameter("ground_ransac_iterations", 72)
        self.declare_parameter("ground_inlier_distance", 0.10)
        self.declare_parameter("ground_min_inliers", 80)
        self.declare_parameter("ground_min_normal_z", 0.55)
        self.declare_parameter("ground_fit_period_sec", 1.0)
        self.declare_parameter("ground_ema_alpha", 0.25)
        self.declare_parameter("ground_prior_max_angle_deg", 25.0)
        self.declare_parameter("imu_max_age", 1.0)

        self.declare_parameter("voxel_leaf_size", 0.08)
        self.declare_parameter("voxel_min_points", 1)
        self.declare_parameter("max_odom_age", 0.5)
        self.declare_parameter("publish_empty_on_failure", True)

        self.input_cloud_topic = self.get_parameter("input_cloud_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.output_cloud_topic = self.get_parameter("output_cloud_topic").value
        self.output_frame = self.get_parameter("output_frame").value

        self.latest_odom = None
        self.latest_odom_wall_time = 0.0
        self.latest_imu_accel = None
        self.latest_imu_wall_time = 0.0
        self.latest_cloud = None
        self.latest_cloud_wall_time = 0.0
        self.stable_ground_plane = None
        self.stable_ground_wall_time = 0.0
        self.consecutive_fail_count = 0
        self.max_ground_fit_failures = 30
        self.last_ground_fit_iterations = 0
        self.last_ground_fit_used_prior = False
        self.warn_times = defaultdict(float)
        self.rng = np.random.default_rng(42)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self._on_cloud, qos
        )
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)
        ground_fit_period = float(self.get_parameter("ground_fit_period_sec").value)
        self.ground_fit_timer = self.create_timer(
            max(0.1, ground_fit_period), self._on_ground_fit_timer
        )

        self.get_logger().info(
            "nav_obstacle_cloud_filter started: "
            f"{self.input_cloud_topic} + {self.odom_topic} + {self.imu_topic} -> "
            f"{self.output_cloud_topic} ({self.output_frame})"
        )

    def _on_odom(self, msg):
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(_finite(value) for value in values):
            self._warn_throttled("invalid_odom", "ignoring invalid odometry")
            return
        self.latest_odom = msg
        self.latest_odom_wall_time = time.monotonic()

    def _on_imu(self, msg):
        accel = msg.linear_acceleration
        values = (accel.x, accel.y, accel.z)
        if not all(_finite(value) for value in values):
            self._warn_throttled("invalid_imu", "ignoring invalid IMU acceleration")
            return
        accel_vec = np.array([accel.x, accel.y, accel.z], dtype=np.float64)
        if np.linalg.norm(accel_vec) < 1e-6:
            self._warn_throttled("invalid_imu", "ignoring near-zero IMU acceleration")
            return
        self.latest_imu_accel = accel_vec
        self.latest_imu_wall_time = time.monotonic()

    def _on_cloud(self, msg):
        odom = self.latest_odom
        if odom is None:
            self._warn_throttled("no_odom", "no odometry yet; publishing empty cloud")
            self._publish_empty(msg)
            return

        odom_age = time.monotonic() - self.latest_odom_wall_time
        max_odom_age = float(self.get_parameter("max_odom_age").value)
        if odom_age > max_odom_age:
            self._warn_throttled(
                "stale_odom",
                f"odometry age {odom_age:.2f}s exceeds {max_odom_age:.2f}s; "
                "publishing empty cloud",
            )
            self._publish_empty(msg)
            return

        cloud = self._cloud_to_array(msg)
        if cloud.size == 0:
            self._publish_empty(msg)
            return

        self.latest_cloud = cloud
        self.latest_cloud_wall_time = time.monotonic()

        output = self._filter_obstacles(cloud, odom)
        self._publish_points(output, msg)

    def _on_ground_fit_timer(self):
        odom = self.latest_odom
        cloud = self.latest_cloud
        if odom is None or cloud is None or cloud.size == 0:
            return

        odom_age = time.monotonic() - self.latest_odom_wall_time
        max_odom_age = float(self.get_parameter("max_odom_age").value)
        if odom_age > max_odom_age:
            return

        prior_normal = self._gravity_prior_normal(odom)
        if prior_normal is None:
            if self.stable_ground_plane is None:
                self._warn_throttled(
                    "no_ground_prior",
                    "no IMU/odom gravity prior yet; waiting to fit ground plane",
                    period=5.0,
                )
            return

        odom_pos = odom.pose.pose.position
        odom_xy = np.array([odom_pos.x, odom_pos.y], dtype=np.float64)
        odom_xyz = np.array([odom_pos.x, odom_pos.y, odom_pos.z], dtype=np.float64)
        rel_xy = cloud[:, :2] - odom_xy
        range_xy = np.linalg.norm(rel_xy, axis=1)
        ground_plane = self._fit_ground_plane(
            cloud, range_xy, odom_xyz, prior_normal=prior_normal
        )
        if ground_plane is None:
            self._record_ground_fit_failure()
            return

        self._accept_ground_plane(*ground_plane)

    def _filter_obstacles(self, cloud, odom):
        odom_pos = odom.pose.pose.position
        odom_xy = np.array([odom_pos.x, odom_pos.y], dtype=np.float64)

        rel_xy = cloud[:, :2] - odom_xy
        range_xy = np.linalg.norm(rel_xy, axis=1)
        ground_plane = self._ground_plane_for_filter()
        if ground_plane is None:
            self._warn_throttled(
                "ground_fit",
                "no stable ground plane available; publishing empty cloud",
            )
            return np.empty((0, 3), dtype=np.float32)

        plane_point, normal = ground_plane
        heights = (cloud - plane_point) @ normal

        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        min_height = float(self.get_parameter("obstacle_min_height").value)
        max_height = float(self.get_parameter("obstacle_max_height").value)
        mask = (
            (range_xy >= min_range)
            & (range_xy <= max_range)
            & (heights >= min_height)
            & (heights <= max_height)
        )
        selected = cloud[mask]
        if selected.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        return self._voxel_filter(selected.astype(np.float32, copy=False))

    def _ground_plane_for_filter(self):
        if self.stable_ground_plane is None:
            return None
        if self.consecutive_fail_count > self.max_ground_fit_failures:
            return None
        return self.stable_ground_plane

    def _accept_ground_plane(self, plane_point, normal):
        plane_point = np.asarray(plane_point, dtype=np.float64)
        normal = self._normalized(normal)
        if normal is None:
            self._record_ground_fit_failure()
            return

        alpha = float(self.get_parameter("ground_ema_alpha").value)
        alpha = max(0.0, min(1.0, alpha))
        if self.stable_ground_plane is None:
            smoothed_point = plane_point
            smoothed_normal = normal
        else:
            prev_point, prev_normal = self.stable_ground_plane
            if float(np.dot(prev_normal, normal)) < 0.0:
                normal = -normal
            smoothed_point = (1.0 - alpha) * prev_point + alpha * plane_point
            smoothed_normal = self._normalized(
                (1.0 - alpha) * prev_normal + alpha * normal
            )
            if smoothed_normal is None:
                smoothed_normal = prev_normal

        self.stable_ground_plane = (smoothed_point, smoothed_normal)
        self.stable_ground_wall_time = time.monotonic()
        if self.consecutive_fail_count > 0:
            self.get_logger().info(
                "ground plane fit recovered after "
                f"{self.consecutive_fail_count} consecutive failures"
            )
        self.consecutive_fail_count = 0

    def _record_ground_fit_failure(self):
        self.consecutive_fail_count += 1
        if self.consecutive_fail_count > self.max_ground_fit_failures:
            self._warn_throttled(
                "ground_fit_forced_empty",
                "ground plane fit failed "
                f"{self.consecutive_fail_count} consecutive times; "
                "publishing empty obstacle clouds",
                period=2.0,
            )
        elif self.consecutive_fail_count > 5:
            self._warn_throttled(
                "ground_fit_reuse",
                "ground plane fit failed "
                f"{self.consecutive_fail_count} consecutive times; "
                "reusing the last stable plane",
                period=5.0,
            )

    def _gravity_prior_normal(self, odom):
        if self.latest_imu_accel is None:
            return None

        imu_age = time.monotonic() - self.latest_imu_wall_time
        max_imu_age = float(self.get_parameter("imu_max_age").value)
        if imu_age > max_imu_age:
            return None

        gravity_body = self._normalized(-self.latest_imu_accel)
        if gravity_body is None:
            return None

        orientation = odom.pose.pose.orientation
        normal_world = self._rotate_body_to_world(
            gravity_body,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        normal_world = self._normalized(normal_world)
        if normal_world is None:
            return None
        return normal_world

    def _rotate_body_to_world(self, vector, qx, qy, qz, qw):
        quat = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(quat)
        if norm < 1e-9:
            return None
        qx, qy, qz, qw = quat / norm
        q_vec = np.array([qx, qy, qz], dtype=np.float64)
        t = 2.0 * np.cross(q_vec, vector)
        return vector + qw * t + np.cross(q_vec, t)

    def _normalized(self, vector):
        if vector is None:
            return None
        arr = np.asarray(vector, dtype=np.float64)
        norm = np.linalg.norm(arr)
        if not math.isfinite(float(norm)) or norm < 1e-9:
            return None
        return arr / norm

    def _fit_ground_plane(self, cloud, range_xy, odom_xyz, prior_normal=None):
        fit_min = float(self.get_parameter("ground_fit_min_range").value)
        fit_max = float(self.get_parameter("ground_fit_max_range").value)
        iterations = int(self.get_parameter("ground_ransac_iterations").value)
        inlier_distance = float(self.get_parameter("ground_inlier_distance").value)
        min_inliers = int(self.get_parameter("ground_min_inliers").value)
        min_normal_z = float(self.get_parameter("ground_min_normal_z").value)
        prior_max_angle = float(self.get_parameter("ground_prior_max_angle_deg").value)
        prior_cos = math.cos(math.radians(max(0.0, min(89.0, prior_max_angle))))
        prior_normal = self._normalized(prior_normal)

        self.last_ground_fit_iterations = 0
        self.last_ground_fit_used_prior = prior_normal is not None

        range_mask = (range_xy >= fit_min) & (range_xy <= fit_max)
        candidates = cloud[range_mask]
        if candidates.shape[0] < max(3, min_inliers):
            return None

        if prior_normal is not None:
            seeded = self._fit_ground_plane_from_prior(
                candidates, odom_xyz, prior_normal, inlier_distance, min_inliers
            )
            if seeded is not None:
                self.last_ground_fit_iterations = 0
                self.last_ground_fit_used_prior = True
                return seeded

        best_inliers = None
        best_count = 0
        for _ in range(max(iterations, 1)):
            self.last_ground_fit_iterations += 1
            sample_idx = self.rng.choice(candidates.shape[0], 3, replace=False)
            p0, p1, p2 = candidates[sample_idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            if prior_normal is not None:
                prior_dot = float(np.dot(normal, prior_normal))
                if prior_dot < 0.0:
                    normal = -normal
                    prior_dot = -prior_dot
                if prior_dot < prior_cos:
                    continue
            else:
                if abs(normal[2]) < min_normal_z:
                    continue

            distances = np.abs((candidates - p0) @ normal)
            inliers = distances <= inlier_distance
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count < min_inliers:
            return None

        inlier_points = candidates[best_inliers]
        return self._refine_ground_plane(inlier_points, odom_xyz, prior_normal)

    def _fit_ground_plane_from_prior(
        self, candidates, odom_xyz, prior_normal, inlier_distance, min_inliers
    ):
        projection = candidates @ prior_normal
        seed_distance = float(np.percentile(projection, 10.0))
        distances = np.abs(projection - seed_distance)
        inliers = distances <= inlier_distance
        if int(np.count_nonzero(inliers)) < min_inliers:
            return None
        return self._refine_ground_plane(candidates[inliers], odom_xyz, prior_normal)

    def _refine_ground_plane(self, inlier_points, odom_xyz, prior_normal=None):
        min_normal_z = float(self.get_parameter("ground_min_normal_z").value)
        centroid = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vh[-1]
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            return None
        normal = normal / norm
        if prior_normal is not None:
            if float(np.dot(normal, prior_normal)) < 0.0:
                normal = -normal
            prior_max_angle = float(
                self.get_parameter("ground_prior_max_angle_deg").value
            )
            prior_cos = math.cos(math.radians(max(0.0, min(89.0, prior_max_angle))))
            if float(np.dot(normal, prior_normal)) < prior_cos:
                return None
        elif abs(normal[2]) < min_normal_z:
            return None
        elif (odom_xyz - centroid) @ normal < 0.0:
            normal = -normal
        return centroid, normal

    def _voxel_filter(self, points):
        leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        min_points = int(self.get_parameter("voxel_min_points").value)
        if points.size == 0 or leaf_size <= 0.0:
            return points

        keys = np.floor(points / leaf_size).astype(np.int64)
        accum = {}
        counts = defaultdict(int)
        for key, point in zip(map(tuple, keys), points):
            if key not in accum:
                accum[key] = point.astype(np.float64)
            else:
                accum[key] += point
            counts[key] += 1

        filtered = [
            (accum[key] / count).astype(np.float32)
            for key, count in counts.items()
            if count >= min_points
        ]
        if not filtered:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(filtered)

    def _cloud_to_array(self, msg):
        direct_cloud = self._cloud_to_array_from_buffer(msg)
        if direct_cloud is not None:
            return direct_cloud

        raw_points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if isinstance(raw_points, np.ndarray):
            if raw_points.dtype.names:
                cloud = np.column_stack(
                    [raw_points["x"], raw_points["y"], raw_points["z"]]
                )
            else:
                cloud = np.asarray(raw_points)[:, :3]
            cloud = cloud.astype(np.float64, copy=False)
            finite = np.isfinite(cloud).all(axis=1)
            return cloud[finite]

        points = []
        for point in raw_points:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
        if not points:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(points, dtype=np.float64)

    def _cloud_to_array_from_buffer(self, msg):
        fields = {field.name: field for field in msg.fields}
        xyz_fields = [fields.get(name) for name in ("x", "y", "z")]
        if any(field is None for field in xyz_fields):
            return None
        if any(field.datatype != PointField.FLOAT32 for field in xyz_fields):
            return None

        row_points_size = msg.point_step * msg.width
        if msg.height > 1 and msg.row_step != row_points_size:
            return None

        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
                "offsets": [field.offset for field in xyz_fields],
                "itemsize": msg.point_step,
            }
        )
        try:
            count = msg.width * msg.height
            raw = np.frombuffer(msg.data, dtype=dtype, count=count)
        except (TypeError, ValueError):
            return None

        cloud = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
            np.float64, copy=False
        )
        finite = np.isfinite(cloud).all(axis=1)
        return cloud[finite]

    def _publish_empty(self, source_msg):
        if bool(self.get_parameter("publish_empty_on_failure").value):
            self._publish_points(np.empty((0, 3), dtype=np.float32), source_msg)

    def _publish_points(self, points, source_msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.output_frame
        msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.cloud_pub.publish(msg)

    def _warn_throttled(self, key, message, period=2.0):
        now = time.monotonic()
        if now - self.warn_times[key] >= period:
            self.warn_times[key] = now
            self.get_logger().warning(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavObstacleCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
