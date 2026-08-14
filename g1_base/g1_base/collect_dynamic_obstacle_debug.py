import argparse
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_DURATION = 45.0
DEFAULT_SAMPLE_INTERVAL = 0.2
DEFAULT_LOOKAHEAD = 3.0
DEFAULT_FRONT_HALF_WIDTH = 0.6
DEFAULT_GLOBAL_WINDOW = 4.0


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def now_sec():
    return time.time()


def hz_from_window(window, span=3.0):
    now = now_sec()
    recent = [stamp for stamp in window if now - stamp <= span]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)


def append_stamp(window, limit=300):
    window.append(now_sec())
    while len(window) > limit:
        window.popleft()


def msg_age(stamp):
    if stamp <= 0.0:
        return None
    return max(0.0, now_sec() - stamp)


def twist_to_dict(msg):
    return {
        "vx": float(msg.linear.x),
        "vy": float(msg.linear.y),
        "wz": float(msg.angular.z),
    }


def pose_to_dict(x, y, yaw, source):
    return {
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
        "source": source,
    }


class CostmapState:
    def __init__(self):
        self.msg = None
        self.stamp = 0.0
        self.window = deque()

    def update(self, msg):
        self.msg = msg
        self.stamp = now_sec()
        append_stamp(self.window)

    @property
    def hz(self):
        return hz_from_window(self.window)


class PathState:
    def __init__(self):
        self.points = np.empty((0, 2), dtype=np.float64)
        self.stamp = 0.0
        self.window = deque()

    def update(self, msg):
        pts = []
        for pose in msg.poses:
            pts.append((pose.pose.position.x, pose.pose.position.y))
        self.points = np.asarray(pts, dtype=np.float64) if pts else np.empty((0, 2))
        self.stamp = now_sec()
        append_stamp(self.window)

    @property
    def hz(self):
        return hz_from_window(self.window)


class DynamicObstacleDebug(Node):
    def __init__(self, args):
        super().__init__("collect_dynamic_obstacle_debug")
        self.args = args
        self.started_at = now_sec()
        self.end_at = self.started_at + args.duration if args.duration > 0 else None

        self.out_dir = Path(args.out_dir).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = self.out_dir / "samples.jsonl"
        self.events_path = self.out_dir / "events.jsonl"
        self.summary_path = self.out_dir / "summary.txt"
        self.samples_file = self.samples_path.open("a", encoding="utf-8")
        self.events_file = self.events_path.open("a", encoding="utf-8")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cmd = {}
        self.latest_cmd_stamp = {}
        self.cmd_windows = {}
        self.motion_source = "unknown"
        self.nav_status = []
        self.odom_pose = None
        self.odom_stamp = 0.0

        self.local_costmap = CostmapState()
        self.global_costmap = CostmapState()
        self.global_path = PathState()
        self.local_path = PathState()
        self.received_global_path = PathState()

        self.obstacle_cloud = {
            "stamp": 0.0,
            "hz": 0.0,
            "points": 0,
            "min_dist": None,
            "front_count": 0,
            "front_min_x": None,
            "front_min_dist": None,
        }
        self.cloud_window = deque()

        self.last_print = 0.0
        self.stop_like_samples = 0
        self.path_blocked_samples = 0
        self.cmd_nonzero_exec_zero_samples = 0
        self.total_samples = 0

        self._create_subscriptions()
        self.create_timer(args.sample_interval, self.sample)
        self.snapshot_thread = threading.Thread(
            target=self._snapshot_environment, daemon=True
        )
        self.snapshot_thread.start()

        self.get_logger().info(
            f"collect_dynamic_obstacle_debug started, out={self.out_dir}, "
            f"duration={args.duration:.1f}s"
        )

    def _create_subscriptions(self):
        reliable_qos = 10
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb("/cmd_vel"), 10)
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", self._cmd_cb("/cmd_vel_smoothed"), 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_executed", self._cmd_cb("/cmd_vel_executed"), 10
        )
        self.create_subscription(
            Odometry, "/odom_2d", self._odom_callback, reliable_qos
        )
        self.create_subscription(
            String, "/motion_source", self._motion_source_callback, reliable_qos
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav_status_callback,
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.local_costmap.update,
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.global_costmap.update,
            reliable_qos,
        )
        self.create_subscription(NavPath, "/plan", self.global_path.update, reliable_qos)
        self.create_subscription(
            NavPath, "/local_plan", self.local_path.update, reliable_qos
        )
        self.create_subscription(
            NavPath,
            "/received_global_plan",
            self.received_global_path.update,
            reliable_qos,
        )
        self.create_subscription(
            PointCloud2, "/nav/obstacle_cloud", self._obstacle_cloud_callback, sensor_qos
        )

    def _cmd_cb(self, topic):
        def callback(msg):
            self.latest_cmd[topic] = twist_to_dict(msg)
            self.latest_cmd_stamp[topic] = now_sec()
            window = self.cmd_windows.setdefault(topic, deque())
            append_stamp(window)

        return callback

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pose = pose_to_dict(p.x, p.y, yaw_from_quaternion(q), "odom_2d")
        self.odom_stamp = now_sec()

    def _motion_source_callback(self, msg):
        self.motion_source = str(msg.data)

    def _nav_status_callback(self, msg):
        self.nav_status = [int(status.status) for status in msg.status_list]

    def _obstacle_cloud_callback(self, msg):
        append_stamp(self.cloud_window)
        pose = self._current_pose()
        if pose is None:
            self.obstacle_cloud.update(
                {
                    "stamp": now_sec(),
                    "hz": hz_from_window(self.cloud_window),
                    "points": msg.width * msg.height,
                    "min_dist": None,
                    "front_count": 0,
                    "front_min_x": None,
                    "front_min_dist": None,
                }
            )
            return

        points = self._cloud_xyz(msg)
        if points.size == 0:
            metrics = {
                "points": 0,
                "min_dist": None,
                "front_count": 0,
                "front_min_x": None,
                "front_min_dist": None,
            }
        else:
            rel = self._to_robot_frame(points[:, :2], pose)
            dist = np.linalg.norm(rel, axis=1)
            front = (
                (rel[:, 0] > 0.0)
                & (rel[:, 0] <= self.args.lookahead)
                & (np.abs(rel[:, 1]) <= self.args.front_half_width)
            )
            front_count = int(np.count_nonzero(front))
            metrics = {
                "points": int(points.shape[0]),
                "min_dist": float(np.min(dist)) if dist.size else None,
                "front_count": front_count,
                "front_min_x": float(np.min(rel[front, 0])) if front_count else None,
                "front_min_dist": float(np.min(dist[front])) if front_count else None,
            }

        self.obstacle_cloud.update(
            {
                "stamp": now_sec(),
                "hz": hz_from_window(self.cloud_window),
                **metrics,
            }
        )

    def _cloud_xyz(self, msg):
        fields = {field.name: field for field in msg.fields}
        xyz_fields = [fields.get(name) for name in ("x", "y", "z")]
        if any(field is None for field in xyz_fields):
            return np.empty((0, 3), dtype=np.float64)
        if any(field.datatype != PointField.FLOAT32 for field in xyz_fields):
            return np.empty((0, 3), dtype=np.float64)
        row_points_size = msg.point_step * msg.width
        if msg.height > 1 and msg.row_step != row_points_size:
            return np.empty((0, 3), dtype=np.float64)

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
            raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        except (TypeError, ValueError):
            return np.empty((0, 3), dtype=np.float64)
        cloud = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
            np.float64, copy=False
        )
        finite_mask = np.isfinite(cloud).all(axis=1)
        return cloud[finite_mask]

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            t = transform.transform.translation
            yaw = yaw_from_quaternion(transform.transform.rotation)
            return pose_to_dict(t.x, t.y, yaw, "tf_map_base_link")
        except TransformException:
            pass
        if self.odom_pose is not None and msg_age(self.odom_stamp) is not None:
            return self.odom_pose
        return None

    def _to_robot_frame(self, xy, pose):
        dx = xy[:, 0] - pose["x"]
        dy = xy[:, 1] - pose["y"]
        c = math.cos(pose["yaw"])
        s = math.sin(pose["yaw"])
        return np.column_stack((c * dx + s * dy, -s * dx + c * dy))

    def _costmap_metrics(self, state, pose, path_points):
        msg = state.msg
        if msg is None:
            return {
                "age": None,
                "hz": state.hz,
                "available": False,
            }

        data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        local_window = self._costmap_window_indices(
            msg, pose, self.args.global_window
        )
        if local_window is None:
            window_data = data
            x0 = 0
            y0 = 0
        else:
            x0, x1, y0, y1 = local_window
            window_data = data[y0:y1, x0:x1]

        lethal = int(np.count_nonzero(window_data >= 100))
        high = int(np.count_nonzero(window_data >= 80))
        inflated = int(np.count_nonzero((window_data > 0) & (window_data < 100)))
        unknown = int(np.count_nonzero(window_data < 0))
        if pose is None:
            return {
                "available": True,
                "age": msg_age(state.stamp),
                "hz": state.hz,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "window_lethal": lethal,
                "window_high": high,
                "window_inflated": inflated,
                "window_unknown": unknown,
                "pose_cost": None,
                "min_lethal_dist": None,
                "min_high_dist": None,
                "path": self._path_cost_metrics(msg, data, pose, path_points),
            }
        pose_cost = self._cost_at(msg, data, pose["x"], pose["y"])
        min_lethal = self._min_cost_distance(msg, window_data, x0, y0, pose, 100)
        min_high = self._min_cost_distance(msg, window_data, x0, y0, pose, 80)
        path_cost = self._path_cost_metrics(msg, data, pose, path_points)

        return {
            "available": True,
            "age": msg_age(state.stamp),
            "hz": state.hz,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "window_lethal": lethal,
            "window_high": high,
            "window_inflated": inflated,
            "window_unknown": unknown,
            "pose_cost": pose_cost,
            "min_lethal_dist": min_lethal,
            "min_high_dist": min_high,
            "path": path_cost,
        }

    def _costmap_window_indices(self, msg, pose, radius):
        if pose is None:
            return None
        res = msg.info.resolution
        if res <= 0.0:
            return None
        gx, gy = self._world_to_grid(msg, pose["x"], pose["y"])
        if gx is None:
            return None
        cells = max(1, int(math.ceil(radius / res)))
        x0 = max(0, gx - cells)
        x1 = min(msg.info.width, gx + cells + 1)
        y0 = max(0, gy - cells)
        y1 = min(msg.info.height, gy + cells + 1)
        return x0, x1, y0, y1

    def _world_to_grid(self, msg, x, y):
        res = msg.info.resolution
        if res <= 0.0:
            return None, None
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
            return None, None
        return gx, gy

    def _cost_at(self, msg, data, x, y):
        gx, gy = self._world_to_grid(msg, x, y)
        if gx is None:
            return None
        return int(data[gy, gx])

    def _min_cost_distance(self, msg, window_data, x0, y0, pose, threshold):
        if pose is None or window_data.size == 0:
            return None
        ys, xs = np.where(window_data >= threshold)
        if xs.size == 0:
            return None
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        wx = ox + (xs + x0 + 0.5) * res
        wy = oy + (ys + y0 + 0.5) * res
        dist = np.hypot(wx - pose["x"], wy - pose["y"])
        return float(np.min(dist)) if dist.size else None

    def _path_cost_metrics(self, msg, data, pose, path_points):
        if pose is None or path_points.size == 0:
            return {
                "points": int(path_points.shape[0]) if path_points.size else 0,
                "lookahead_points": 0,
                "max_cost": None,
                "first_high_dist": None,
                "first_lethal_dist": None,
            }
        rel = self._to_robot_frame(path_points, pose)
        ahead = (
            (rel[:, 0] >= 0.0)
            & (rel[:, 0] <= self.args.lookahead)
            & (np.abs(rel[:, 1]) <= max(self.args.front_half_width, 0.8))
        )
        pts = path_points[ahead]
        rel_pts = rel[ahead]
        if pts.size == 0:
            return {
                "points": int(path_points.shape[0]),
                "lookahead_points": 0,
                "max_cost": None,
                "first_high_dist": None,
                "first_lethal_dist": None,
            }

        costs = []
        dists = []
        for point, rel_point in zip(pts, rel_pts):
            cost = self._cost_at(msg, data, point[0], point[1])
            if cost is None:
                continue
            costs.append(cost)
            dists.append(float(math.hypot(rel_point[0], rel_point[1])))
        if not costs:
            return {
                "points": int(path_points.shape[0]),
                "lookahead_points": int(pts.shape[0]),
                "max_cost": None,
                "first_high_dist": None,
                "first_lethal_dist": None,
            }

        first_high = None
        first_lethal = None
        for cost, dist in sorted(zip(costs, dists), key=lambda item: item[1]):
            if first_high is None and cost >= 80:
                first_high = dist
            if first_lethal is None and cost >= 100:
                first_lethal = dist
        return {
            "points": int(path_points.shape[0]),
            "lookahead_points": int(pts.shape[0]),
            "max_cost": int(max(costs)),
            "first_high_dist": first_high,
            "first_lethal_dist": first_lethal,
        }

    def _path_metrics(self, pose, path_state):
        points = path_state.points
        if pose is None or points.size == 0:
            return {
                "age": msg_age(path_state.stamp),
                "hz": path_state.hz,
                "points": int(points.shape[0]) if points.size else 0,
                "dist_to_path": None,
                "remaining_length": None,
            }
        dist_to_points = np.hypot(points[:, 0] - pose["x"], points[:, 1] - pose["y"])
        nearest = int(np.argmin(dist_to_points))
        remaining = points[nearest:]
        if remaining.shape[0] < 2:
            length = 0.0
        else:
            diffs = np.diff(remaining, axis=0)
            length = float(np.sum(np.linalg.norm(diffs, axis=1)))
        return {
            "age": msg_age(path_state.stamp),
            "hz": path_state.hz,
            "points": int(points.shape[0]),
            "dist_to_path": float(dist_to_points[nearest]),
            "remaining_length": length,
        }

    def sample(self):
        pose = self._current_pose()
        path_points = self._select_global_path_points()
        local_costmap = self._costmap_metrics(self.local_costmap, pose, path_points)
        global_costmap = self._costmap_metrics(self.global_costmap, pose, path_points)
        cmd_vel = self.latest_cmd.get("/cmd_vel", {"vx": 0.0, "vy": 0.0, "wz": 0.0})
        executed = self.latest_cmd.get(
            "/cmd_vel_executed", {"vx": None, "vy": None, "wz": None}
        )

        blocked_dist = None
        if local_costmap.get("available"):
            blocked_dist = local_costmap["path"].get("first_high_dist")
        obstacle_front = self.obstacle_cloud.get("front_min_dist")
        stopped = abs(cmd_vel.get("vx", 0.0)) < 0.03 and abs(cmd_vel.get("wz", 0.0)) < 0.08
        blocked = blocked_dist is not None and blocked_dist <= self.args.lookahead
        if stopped:
            self.stop_like_samples += 1
        if blocked:
            self.path_blocked_samples += 1
        if (
            executed.get("vx") is not None
            and abs(cmd_vel.get("vx", 0.0)) > 0.05
            and abs(executed.get("vx", 0.0)) < 0.01
        ):
            self.cmd_nonzero_exec_zero_samples += 1
        self.total_samples += 1

        sample = {
            "t": now_sec(),
            "elapsed": now_sec() - self.started_at,
            "pose": pose,
            "cmd": {
                "/cmd_vel": cmd_vel,
                "/cmd_vel_age": msg_age(self.latest_cmd_stamp.get("/cmd_vel", 0.0)),
                "/cmd_vel_hz": hz_from_window(self.cmd_windows.get("/cmd_vel", [])),
                "/cmd_vel_smoothed": self.latest_cmd.get("/cmd_vel_smoothed"),
                "/cmd_vel_smoothed_age": msg_age(
                    self.latest_cmd_stamp.get("/cmd_vel_smoothed", 0.0)
                ),
                "/cmd_vel_executed": executed,
                "/cmd_vel_executed_age": msg_age(
                    self.latest_cmd_stamp.get("/cmd_vel_executed", 0.0)
                ),
            },
            "motion_source": self.motion_source,
            "nav_status": self.nav_status,
            "obstacle_cloud": {
                **self.obstacle_cloud,
                "age": msg_age(self.obstacle_cloud.get("stamp", 0.0)),
            },
            "paths": {
                "/plan": self._path_metrics(pose, self.global_path),
                "/received_global_plan": self._path_metrics(
                    pose, self.received_global_path
                ),
                "/local_plan": self._path_metrics(pose, self.local_path),
            },
            "costmaps": {
                "local": local_costmap,
                "global": global_costmap,
            },
            "diagnosis_flags": {
                "cmd_stop_like": stopped,
                "local_path_blocked": blocked,
                "front_obstacle_seen": obstacle_front is not None
                and obstacle_front <= self.args.lookahead,
                "planner_cmd_nonzero_but_executed_zero": (
                    executed.get("vx") is not None
                    and abs(cmd_vel.get("vx", 0.0)) > 0.05
                    and abs(executed.get("vx", 0.0)) < 0.01
                ),
            },
        }
        self.samples_file.write(json.dumps(sample, ensure_ascii=False) + "\n")
        self.samples_file.flush()

        if sample["diagnosis_flags"]["cmd_stop_like"] or sample["diagnosis_flags"][
            "local_path_blocked"
        ]:
            self.events_file.write(json.dumps(sample, ensure_ascii=False) + "\n")
            self.events_file.flush()

        self._print_sample(sample)
        if self.end_at is not None and now_sec() >= self.end_at:
            raise KeyboardInterrupt

    def _select_global_path_points(self):
        if self.global_path.points.size:
            return self.global_path.points
        if self.received_global_path.points.size:
            return self.received_global_path.points
        return np.empty((0, 2), dtype=np.float64)

    def _print_sample(self, sample):
        now = now_sec()
        if now - self.last_print < self.args.print_interval:
            return
        self.last_print = now
        cmd = sample["cmd"]["/cmd_vel"]
        local_path = sample["costmaps"]["local"].get("path", {})
        front = sample["obstacle_cloud"].get("front_min_dist")
        blocked = local_path.get("first_high_dist")
        self.get_logger().info(
            " | ".join(
                [
                    f"t={sample['elapsed']:.1f}s",
                    f"cmd=({cmd.get('vx', 0.0):.2f},{cmd.get('vy', 0.0):.2f},{cmd.get('wz', 0.0):.2f})",
                    f"front_obs={front:.2f}m" if front is not None else "front_obs=none",
                    f"path_high={blocked:.2f}m" if blocked is not None else "path_high=none",
                    f"local_hz={sample['costmaps']['local'].get('hz', 0.0):.1f}",
                    f"plan_pts={sample['paths']['/plan'].get('points', 0)}",
                    f"source={sample['motion_source']}",
                ]
            )
        )

    def _snapshot_environment(self):
        meta = {
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "duration": self.args.duration,
            "sample_interval": self.args.sample_interval,
            "lookahead": self.args.lookahead,
            "front_half_width": self.args.front_half_width,
            "note": "read-only collector; no publishers, no costmap clears, no action calls",
        }
        (self.out_dir / "metadata.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )

        commands = {
            "node_list.txt": ["ros2", "node", "list"],
            "topic_list.txt": ["ros2", "topic", "list", "-t"],
            "action_list.txt": ["ros2", "action", "list", "-t"],
        }
        for filename, command in commands.items():
            self._run_snapshot_command(filename, command, timeout_sec=8)

        param_dir = self.out_dir / "params"
        param_dir.mkdir(exist_ok=True)
        for node in (
            "/controller_server",
            "/planner_server",
            "/bt_navigator",
            "/velocity_smoother",
            "/local_costmap/local_costmap",
            "/global_costmap/global_costmap",
            "/navigation_manager",
        ):
            safe = node.strip("/").replace("/", "__") or "root"
            self._run_snapshot_command(
                f"params/{safe}.yaml",
                ["ros2", "param", "dump", node],
                timeout_sec=8,
            )

    def _run_snapshot_command(self, filename, command, timeout_sec=8):
        path = self.out_dir / filename
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_sec,
                check=False,
            )
            path.write_text(result.stdout, encoding="utf-8")
        except Exception as exc:
            path.write_text(f"failed: {exc}\n", encoding="utf-8")

    def write_summary(self):
        if self.total_samples <= 0:
            text = "No samples collected.\n"
        else:
            text = "\n".join(
                [
                    "dynamic obstacle path debug summary",
                    f"out_dir: {self.out_dir}",
                    f"duration_s: {now_sec() - self.started_at:.1f}",
                    f"samples: {self.total_samples}",
                    f"stop_like_samples: {self.stop_like_samples}",
                    f"path_blocked_samples: {self.path_blocked_samples}",
                    "cmd_nonzero_exec_zero_samples: "
                    f"{self.cmd_nonzero_exec_zero_samples}",
                    "",
                    "Interpretation hints:",
                    "- local_path_blocked=true + cmd_stop_like=true: controller sees the global path blocked and is not finding a local way around.",
                    "- front_obstacle_seen=true + local_path_blocked=false: obstacle cloud sees the person but costmap/path scoring may not be marking it on the path.",
                    "- planner_cmd_nonzero_but_executed_zero=true: motion bridge or gait layer is suppressing commands after Nav2.",
                    "- global path remains unchanged while local_path_blocked=true: solve with local controller/replan behavior, not global scan marking.",
                ]
            )
        self.summary_path.write_text(text + "\n", encoding="utf-8")
        self.get_logger().info(text.replace("\n", " | "))

    def close_files(self):
        self.samples_file.close()
        self.events_file.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Collect read-only diagnostics for dynamic obstacles on the global path."
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--sample-interval", type=float, default=DEFAULT_SAMPLE_INTERVAL)
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD)
    parser.add_argument("--front-half-width", type=float, default=DEFAULT_FRONT_HALF_WIDTH)
    parser.add_argument("--global-window", type=float, default=DEFAULT_GLOBAL_WINDOW)
    args, ros_args = parser.parse_known_args(argv)
    if not args.out_dir:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.out_dir = os.path.join("dynamic_obstacle_debug", stamp)
    return args, ros_args


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = DynamicObstacleDebug(args)
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except KeyboardInterrupt:
                break
    finally:
        node.write_summary()
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
