import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from g1_base_interfaces.srv import Relocalize
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSPresetProfiles, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from g1_base.common import package_share_dir


STATE_STOPPED = "STOPPED"
STATE_STARTING_LOCALIZATION = "STARTING_LOCALIZATION"
STATE_STARTING_NAVIGATION = "STARTING_NAVIGATION"
STATE_READY = "READY"
STATE_DEGRADED_LOCALIZATION = "DEGRADED_LOCALIZATION"
STATE_DEGRADED_NAVIGATION = "DEGRADED_NAVIGATION"
STATE_RECOVERING = "RECOVERING"
STATE_ERROR = "ERROR"
STATE_MAPPING = "MAPPING"

# 时间戳化地图制品的命名约定与 manifest 配置
MAP_NAME_SUFFIX = "exhibit_2d_map"
MANIFEST_FILENAME = "current_map.json"
MAP_RETENTION_KEEP = 100
INITIAL_POSE_DEFAULT_DURATION_SEC = 30.0
INITIAL_POSE_DEFAULT_RATE_HZ = 8.0
IMU_NOT_STEADY_REASON = "imu_not_steady"
IMU_NOT_STEADY_MESSAGE = "请保持机器人静止后重试"


def _script_candidates(script_name):
    share_dir = Path(package_share_dir())
    return [
        share_dir / script_name,
        share_dir.parent.parent / "src" / "g1_base" / script_name,
        Path(__file__).resolve().parent.parent / script_name,
    ]


class NavigationManager(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.args = args
        self.lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.shutting_down = False

        self.localization_proc = None
        self.navigation_proc = None
        self.state = STATE_STOPPED
        self.state_reason = ""
        self.mode = "navigation"  # "navigation" | "mapping"
        # 当前建图轮次的 base_name (YYYYMMDD_HHMMSS)，贯穿 PCD/PGM/YAML 文件命名
        self.current_map_basename = None
        # 周期性 2D 快照器（仅 mapping 模式下挂载）
        self.snapshotter = None
        self.last_error = ""
        self.last_transition_time = time.time()
        self.restart_count = 0

        self.cloud_world_stamp = 0.0
        self.robo_odom_stamp = 0.0
        self.odom_2d_stamp = 0.0

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.ready_pub = self.create_publisher(Bool, "/navigation_manager/ready", qos)
        self.state_pub = self.create_publisher(String, "/navigation_manager/state", qos)
        self.detail_pub = self.create_publisher(String, "/navigation_manager/detail", qos)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            qos,
        )

        self.sub_cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            PointCloud2,
            args.pointcloud_topic,
            self._cloud_cb,
            sensor_qos,
            callback_group=self.sub_cb_group,
        )
        self.create_subscription(
            Odometry,
            args.relocal_odom_topic,
            self._relocal_cb,
            sensor_qos,
            callback_group=self.sub_cb_group,
        )
        self.create_subscription(
            Odometry,
            args.odom_topic,
            self._odom_cb,
            sensor_qos,
            callback_group=self.sub_cb_group,
        )

        self.tf_buffer = Buffer()
        # Use the main executor instead of an internal thread so shutdown stays quiet.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.navigate_action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.bt_navigator_state_client = self.create_client(
            GetState,
            "/bt_navigator/get_state",
            callback_group=self.sub_cb_group,
        )

        self.create_service(
            Trigger, "/navigation_manager/ensure_ready", self._handle_ensure_ready
        )
        self.create_service(
            Trigger, "/navigation_manager/restart_all", self._handle_restart_all
        )
        self.create_service(Trigger, "/navigation_manager/stop_all", self._handle_stop_all)
        self.create_service(
            Trigger, "/navigation_manager/start_mapping", self._handle_start_mapping
        )
        self.create_service(
            Trigger, "/navigation_manager/stop_mapping", self._handle_stop_mapping
        )
        self.create_service(
            Trigger, "/navigation_manager/generate_2d_map", self._handle_generate_2d_map
        )
        self.create_service(
            Relocalize, "/navigation_manager/relocalize", self._handle_relocalize
        )

        self.monitor_timer = self.create_timer(
            1.0 / max(args.monitor_hz, 1.0),
            self._monitor_loop,
            callback_group=self.sub_cb_group,
        )
        self._publish_status()

        if args.auto_ensure:
            threading.Thread(target=self._auto_ensure, daemon=True).start()

    def _cloud_cb(self, _msg):
        with self.lock:
            self.cloud_world_stamp = time.time()

    def _relocal_cb(self, _msg):
        with self.lock:
            self.robo_odom_stamp = time.time()

    def _odom_cb(self, _msg):
        with self.lock:
            self.odom_2d_stamp = time.time()

    def _transition_state(self, new_state, reason=""):
        with self.lock:
            previous_state = self.state
            previous_reason = self.state_reason
            changed = previous_state != new_state
            reason_changed = previous_reason != reason
            if not changed and not reason_changed:
                return
            self.state = new_state
            self.state_reason = reason
            self.last_transition_time = time.time()
        if changed:
            if reason:
                self.get_logger().info(f"[navigation_manager] state -> {new_state}: {reason}")
            else:
                self.get_logger().info(f"[navigation_manager] state -> {new_state}")
        elif reason:
            self.get_logger().info(f"[navigation_manager] {new_state}: {reason}")

    def _set_last_error(self, message):
        with self.lock:
            previous_error = self.last_error
            self.last_error = message
        if message and message != previous_error:
            self.get_logger().warning(f"[navigation_manager] {message}")

    def _clear_last_error(self):
        with self.lock:
            self.last_error = ""

    def _managed_proc_alive(self, proc):
        return proc is not None and proc.poll() is None

    def _check_tf_ready(self):
        try:
            return self.tf_buffer.can_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return False
        except Exception:
            return False

    def _check_nav_ready(self):
        try:
            return self.navigate_action.wait_for_server(timeout_sec=0.2)
        except Exception:
            return False

    def _get_bt_navigator_state(self):
        try:
            if not self.bt_navigator_state_client.wait_for_service(timeout_sec=0.2):
                return None, "service_unavailable"
            future = self.bt_navigator_state_client.call_async(GetState.Request())
            deadline = time.time() + 1.0
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                try:
                    self.bt_navigator_state_client.remove_pending_request(future)
                except Exception:
                    pass
                return None, "query_timeout"
            response = future.result()
            if response is None:
                return None, "query_failed"
            state_id = response.current_state.id
            state_label = response.current_state.label or "unknown"
            is_active = state_id == LifecycleState.PRIMARY_STATE_ACTIVE
            return is_active, state_label
        except Exception as exc:
            return None, f"error:{exc}"

    def _build_snapshot(self):
        now = time.time()
        with self.lock:
            cloud_stamp = self.cloud_world_stamp
            relocal_stamp = self.robo_odom_stamp
            odom_stamp = self.odom_2d_stamp
            state = self.state
            last_error = self.last_error
            last_transition_time = self.last_transition_time
            restart_count = self.restart_count
            localization_proc = self.localization_proc
            navigation_proc = self.navigation_proc
            mode = self.mode

        localization_alive = bool(self._managed_proc_alive(localization_proc))
        navigation_alive = bool(self._managed_proc_alive(navigation_proc))
        cloud_fresh = bool(
            cloud_stamp > 0.0 and now - cloud_stamp <= self.args.freshness_window
        )
        relocal_fresh = bool(
            relocal_stamp > 0.0 and now - relocal_stamp <= self.args.freshness_window
        )
        odom_fresh = bool(
            odom_stamp > 0.0 and now - odom_stamp <= self.args.freshness_window
        )
        localization_ready = bool(cloud_fresh and relocal_fresh)
        tf_ready = bool(self._check_tf_ready())
        nav_action_ready = bool(self._check_nav_ready())
        bt_navigator_active, bt_navigator_state = self._get_bt_navigator_state()
        nav_stack_ready = bool(
            odom_fresh
            and tf_ready
            and nav_action_ready
            and bt_navigator_active is True
        )
        nav_ready = bool(localization_ready and nav_stack_ready)
        ready = bool(localization_ready and nav_stack_ready)

        return {
            "mode": mode,
            "state": state,
            "state_reason": self._describe_state(localization_ready=localization_ready, nav_ready=nav_ready, cloud_fresh=cloud_fresh, relocal_fresh=relocal_fresh, odom_fresh=odom_fresh, tf_ready=tf_ready, nav_action_ready=nav_action_ready, bt_navigator_active=bt_navigator_active, bt_navigator_state=bt_navigator_state, localization_alive=localization_alive, navigation_alive=navigation_alive, ready=ready),
            "ready": ready,
            "localization_ready": localization_ready,
            "navigation_ready": nav_ready,
            "nav_stack_ready": nav_stack_ready,
            "cloud_world_fresh": cloud_fresh,
            "relocal_odom_fresh": relocal_fresh,
            "odom_2d_fresh": odom_fresh,
            "tf_ready": tf_ready,
            "nav_action_ready": nav_action_ready,
            "bt_navigator_active": bt_navigator_active,
            "bt_navigator_state": bt_navigator_state,
            "restart_count": restart_count,
            "last_error": last_error,
            "last_transition_time": last_transition_time,
            "managed_localization_alive": localization_alive,
            "managed_navigation_alive": navigation_alive,
        }

    @staticmethod
    def _join_parts(parts):
        return ", ".join(part for part in parts if part)

    def _describe_state(
        self,
        *,
        localization_ready,
        nav_ready,
        cloud_fresh,
        relocal_fresh,
        odom_fresh,
        tf_ready,
        nav_action_ready,
        bt_navigator_active,
        bt_navigator_state,
        localization_alive,
        navigation_alive,
        ready,
    ):
        if ready:
            return "ready"

        process_parts = []
        if not localization_alive:
            process_parts.append("localization_proc=down")
        if not navigation_alive:
            process_parts.append("navigation_proc=down")

        if not localization_ready:
            localization_parts = []
            if not cloud_fresh:
                localization_parts.append("cloud_world=stale")
            if not relocal_fresh:
                localization_parts.append("relocal_odom=stale")
            return self._join_parts(process_parts + localization_parts) or "localization_unready"

        if not nav_ready:
            nav_parts = []
            if not odom_fresh:
                nav_parts.append("odom_2d=stale")
            if not tf_ready:
                nav_parts.append("tf=missing")
            if not nav_action_ready:
                nav_parts.append("navigate_to_pose=down")
            if bt_navigator_active is not True:
                nav_parts.append(f"bt_navigator={bt_navigator_state}")
            return self._join_parts(process_parts + nav_parts) or "navigation_unready"

        return self._join_parts(process_parts) or "unknown"

    def _publish_snapshot(self, snapshot):
        if self.shutting_down or not rclpy.ok():
            return

        self.ready_pub.publish(Bool(data=bool(snapshot["ready"])))
        self.state_pub.publish(String(data=snapshot["state"]))
        self.detail_pub.publish(
            String(data=json.dumps(snapshot, sort_keys=True, ensure_ascii=False))
        )

    def _publish_status(self):
        self._publish_snapshot(self._build_snapshot())

    def _monitor_loop(self):
        if self.shutting_down or not rclpy.ok():
            return

        with self.lock:
            current_state = self.state
            current_mode = self.mode

        # 在 STOPPED 或 ERROR 状态下，跳过昂贵的健康检查（TF lookup、
        # action server wait、bt_navigator service call），避免空转消耗 CPU。
        # 等待 bot_mind 调用 ensure_ready / restart_all 来重新启动。
        if current_state in (STATE_STOPPED, STATE_ERROR):
            return

        # 建图模式：只做轻量的进程存活检查，不做 Nav2 健康检查
        if current_mode == "mapping":
            with self.lock:
                proc = self.localization_proc
            if not self._managed_proc_alive(proc):
                self._transition_state(
                    STATE_ERROR, "mapping process exited unexpectedly"
                )
            return

        # 导航模式：完整的健康检查
        snapshot = self._build_snapshot()
        target_state = snapshot["state"]
        if snapshot["ready"]:
            target_state = STATE_READY
            self._transition_state(target_state, snapshot["state_reason"])
        elif snapshot["managed_localization_alive"] or snapshot["managed_navigation_alive"]:
            if not snapshot["localization_ready"]:
                target_state = STATE_DEGRADED_LOCALIZATION
                self._transition_state(target_state, snapshot["state_reason"])
            else:
                target_state = STATE_DEGRADED_NAVIGATION
                self._transition_state(target_state, snapshot["state_reason"])
        snapshot["state"] = target_state
        self._publish_snapshot(snapshot)

    def _resolve_script(self, script_name):
        for candidate in _script_candidates(script_name):
            if candidate.is_file():
                return str(candidate)
        return None

    def _start_script(self, script_name, label, extra_env=None):
        script_path = self._resolve_script(script_name)
        if script_path is None:
            self._set_last_error(f"{label} script not found: {script_name}")
            return None

        env = os.environ.copy()
        env.setdefault("G1_BASE_ROOT", str(Path(script_path).resolve().parent))
        if extra_env:
            env.update(extra_env)
        self.get_logger().info(f"[navigation_manager] starting {label}: {script_path}")
        try:
            return subprocess.Popen(
                ["/bin/bash", script_path],
                cwd=str(Path(script_path).resolve().parent),
                env=env,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            self._set_last_error(f"failed to start {label}: {exc}")
            return None

    def _stop_process(self, proc, label):
        if not self._managed_proc_alive(proc):
            return

        self.get_logger().info(f"[navigation_manager] stopping {label}")
        try:
            pgid = os.getpgid(proc.pid)
        except OSError:
            pgid = None

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                if pgid is not None:
                    os.killpg(pgid, sig)
                else:
                    proc.send_signal(sig)
            except OSError:
                break

            try:
                proc.wait(timeout=5.0)
                return
            except subprocess.TimeoutExpired:
                continue

        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGKILL)
            else:
                proc.kill()
        except Exception:
            pass

    def _stop_managed_processes(self):
        with self.lock:
            navigation_proc = self.navigation_proc
            localization_proc = self.localization_proc
            self.navigation_proc = None
            self.localization_proc = None
            self.cloud_world_stamp = 0.0
            self.robo_odom_stamp = 0.0
            self.odom_2d_stamp = 0.0

        self._stop_process(navigation_proc, "navigation stack")
        self._stop_process(localization_proc, "localization stack")

    def _wait_until(self, predicate, timeout_s, abort_if=None):
        deadline = time.time() + timeout_s
        while time.time() < deadline and rclpy.ok():
            snapshot = self._build_snapshot()
            self._publish_snapshot(snapshot)
            if predicate(snapshot):
                return True, snapshot
            if abort_if is not None:
                abort_reason = abort_if()
                if abort_reason:
                    snapshot["abort_reason"] = abort_reason
                    return False, snapshot
            time.sleep(1.0 / max(self.args.monitor_hz, 1.0))
        return False, self._build_snapshot()

    @staticmethod
    def _script_exit_failure(proc, label):
        if proc is None:
            return None
        code = proc.poll()
        if code is None:
            return None
        if code != 0:
            if code == 4:
                return {
                    "reason": IMU_NOT_STEADY_REASON,
                    "message": (
                        f"{label} exited with code {code}: {IMU_NOT_STEADY_MESSAGE}"
                    ),
                }
            return {
                "reason": "process_exited",
                "message": f"{label} exited with code {code}",
            }
        return {
            "reason": "process_exited",
            "message": f"{label} exited before readiness",
        }

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _validate_float(value, name):
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a number")
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        return number

    def _initial_pose_seen_lio_data(self):
        now = time.time()
        with self.lock:
            cloud_stamp = self.cloud_world_stamp
            relocal_stamp = self.robo_odom_stamp
        return bool(
            (cloud_stamp > 0.0 and now - cloud_stamp <= self.args.freshness_window)
            or (
                relocal_stamp > 0.0
                and now - relocal_stamp <= self.args.freshness_window
            )
        )

    def _publish_initial_pose_once(self, initial_pose):
        x, y, yaw = initial_pose
        yaw = self._normalize_angle(yaw)
        half = yaw / 2.0

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        self.initial_pose_pub.publish(msg)

    def _initial_pose_publish_loop(self, initial_pose, stop_event, duration_sec, rate_hz):
        interval = 1.0 / max(float(rate_hz), 0.1)
        deadline = time.time() + max(0.1, float(duration_sec))
        published = 0
        self.get_logger().info(
            "[navigation_manager] initialpose stream started: "
            f"x={initial_pose[0]:.3f}, y={initial_pose[1]:.3f}, "
            f"yaw={math.degrees(initial_pose[2]):.1f}deg, "
            f"duration={duration_sec:.1f}s, rate={rate_hz:.1f}Hz"
        )
        while rclpy.ok() and not stop_event.is_set() and time.time() < deadline:
            self._publish_initial_pose_once(initial_pose)
            published += 1
            if self._initial_pose_seen_lio_data():
                break
            stop_event.wait(interval)

        self.get_logger().info(
            f"[navigation_manager] initialpose stream stopped after {published} publishes"
        )

    def _start_initial_pose_stream(self, initial_pose, duration_sec, rate_hz):
        if initial_pose is None:
            return None, None
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._initial_pose_publish_loop,
            args=(initial_pose, stop_event, duration_sec, rate_hz),
            name="initialpose-stream",
            daemon=True,
        )
        thread.start()
        return stop_event, thread

    @staticmethod
    def _stop_initial_pose_stream(stop_event, thread):
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def _start_localization_stack(self):
        proc = self._start_script("start_pc2_localization.sh", "localization stack")
        if proc is None:
            return False
        with self.lock:
            self.localization_proc = proc
        return True

    def _start_navigation_stack(self):
        # Localization already starts odom_to_tf; reuse it when navigation_manager
        # brings up the full stack to avoid duplicate /odom_to_tf nodes.
        proc = self._start_script(
            "start_navigation.sh",
            "navigation stack",
            extra_env={"SKIP_ODOM_TO_TF": "1"},
        )
        if proc is None:
            return False
        with self.lock:
            self.navigation_proc = proc
        return True

    def _bringup_stack(
        self,
        force_restart,
        initial_pose=None,
        initial_pose_duration=INITIAL_POSE_DEFAULT_DURATION_SEC,
        initial_pose_rate=INITIAL_POSE_DEFAULT_RATE_HZ,
    ):
        snapshot = self._build_snapshot()
        if snapshot["ready"] and not force_restart:
            self._transition_state(STATE_READY, "already_ready")
            self._clear_last_error()
            return True, "bottom navigation stack already ready"

        for attempt in range(1, self.args.max_bringup_attempts + 1):
            with self.lock:
                self.restart_count += 1
            self._transition_state(
                STATE_RECOVERING if force_restart or attempt > 1 else STATE_STARTING_LOCALIZATION,
                f"bringup attempt {attempt}/{self.args.max_bringup_attempts}",
            )
            self._stop_managed_processes()

            initial_pose_stop_event, initial_pose_thread = self._start_initial_pose_stream(
                initial_pose,
                initial_pose_duration,
                initial_pose_rate,
            )
            if not self._start_localization_stack():
                self._stop_initial_pose_stream(
                    initial_pose_stop_event,
                    initial_pose_thread,
                )
                continue

            self._transition_state(
                STATE_STARTING_LOCALIZATION, "waiting for localization readiness"
            )
            ok, snapshot = self._wait_until(
                lambda s: s["localization_ready"],
                self.args.localization_timeout,
                abort_if=lambda: self._script_exit_failure(
                    self.localization_proc,
                    "localization stack",
                ),
            )
            self._stop_initial_pose_stream(
                initial_pose_stop_event,
                initial_pose_thread,
            )
            if not ok:
                failure = snapshot.get("abort_reason")
                if failure:
                    self._set_last_error(failure["message"])
                else:
                    self._set_last_error(
                        "localization stack did not become ready in time"
                    )
                continue

            if not self._start_navigation_stack():
                continue

            self._transition_state(
                STATE_STARTING_NAVIGATION, "waiting for navigation readiness"
            )
            ok, snapshot = self._wait_until(
                lambda s: s["nav_stack_ready"], self.args.navigation_timeout
            )
            if ok:
                self._clear_last_error()
                self._transition_state(STATE_READY, "bringup_complete")
                return True, "bottom navigation stack is ready"

            self._set_last_error("navigation stack did not become ready in time")

        self._transition_state(STATE_ERROR, self.last_error or "bringup failed")
        self._stop_managed_processes()
        return False, self.last_error or "bringup failed"

    def _with_operation(self, operation, func):
        if not self.operation_lock.acquire(blocking=False):
            return False, f"{operation} rejected: another operation is already running"
        try:
            return func()
        finally:
            self.operation_lock.release()

    def _handle_ensure_ready(self, _request, response):
        with self.lock:
            if self.mode == "mapping":
                response.success = False
                response.message = "currently in mapping mode, call stop_mapping first"
                return response
        success, message = self._with_operation(
            "ensure_ready", lambda: self._bringup_stack(force_restart=False)
        )
        response.success = success
        response.message = message
        return response

    def _handle_restart_all(self, _request, response):
        with self.lock:
            if self.mode == "mapping":
                response.success = False
                response.message = "currently in mapping mode, call stop_mapping first"
                return response
        success, message = self._with_operation(
            "restart_all", lambda: self._bringup_stack(force_restart=True)
        )
        response.success = success
        response.message = message
        return response

    def _handle_relocalize(self, request, response):
        with self.lock:
            if self.mode == "mapping":
                response.success = False
                response.message = "currently in mapping mode, call stop_mapping first"
                return response

        try:
            x = self._validate_float(request.x, "x")
            y = self._validate_float(request.y, "y")
            yaw = self._normalize_angle(self._validate_float(request.yaw, "yaw"))
            duration = self._validate_float(request.duration_sec, "duration_sec")
            rate = self._validate_float(request.rate_hz, "rate_hz")
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response

        if duration <= 0.0:
            duration = INITIAL_POSE_DEFAULT_DURATION_SEC
        if rate <= 0.0:
            rate = INITIAL_POSE_DEFAULT_RATE_HZ
        duration = min(max(duration, 1.0), 120.0)
        rate = min(max(rate, 0.5), 20.0)

        initial_pose = (x, y, yaw)
        success, message = self._with_operation(
            "relocalize",
            lambda: self._bringup_stack(
                force_restart=True,
                initial_pose=initial_pose,
                initial_pose_duration=duration,
                initial_pose_rate=rate,
            ),
        )
        response.success = success
        if success:
            response.message = (
                f"relocalize completed with initial pose "
                f"x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}deg: {message}"
            )
        else:
            response.message = message
        return response

    def _handle_stop_all(self, _request, response):
        success, message = self._with_operation(
            "stop_all", self._stop_all_impl
        )
        response.success = success
        response.message = message
        return response

    def _stop_all_impl(self):
        with self.lock:
            was_mapping = self.mode == "mapping"
        if was_mapping:
            self._stop_mapping_gracefully()
        else:
            self._stop_managed_processes()
        with self.lock:
            self.mode = "navigation"
        self._transition_state(STATE_STOPPED, "stopped by request")
        self._clear_last_error()
        self._publish_status()
        return True, "stopped"

    def _auto_ensure(self):
        time.sleep(1.0)
        success, message = self._bringup_stack(force_restart=False)
        if success:
            self.get_logger().info(f"[navigation_manager] auto ensure succeeded: {message}")
        else:
            self.get_logger().warning(f"[navigation_manager] auto ensure failed: {message}")

    # ── 建图模式 ──

    def _stop_mapping_gracefully(self):
        """停止建图进程，发 SIGINT 让 Super-LIO 保存 map.pcd。

        只对 bash 脚本进程发 SIGINT，让 bash cleanup 自然传播信号给
        ros2 launch → super_lio_node，避免 killpg 导致双重信号竞争。
        """
        with self.lock:
            proc = self.localization_proc
            self.localization_proc = None
            self.cloud_world_stamp = 0.0
            self.robo_odom_stamp = 0.0
            self.odom_2d_stamp = 0.0

        if not self._managed_proc_alive(proc):
            return

        self.get_logger().info(
            "[navigation_manager] stopping mapping (SIGINT for map save)"
        )

        # 只给 bash 脚本发 SIGINT，不用 killpg。
        # bash trap 会触发 cleanup → SIGTERM 给 ros2 launch →
        # ros2 launch 再 SIGINT 给 super_lio_node → 保存地图后退出。
        try:
            proc.send_signal(signal.SIGINT)
        except OSError:
            return

        try:
            try:
                map_save_timeout = float(os.environ.get("MAP_SAVE_TIMEOUT", "30"))
            except ValueError:
                map_save_timeout = 30.0
            proc.wait(timeout=max(45.0, map_save_timeout + 15.0))
            self.get_logger().info(
                "[navigation_manager] mapping process exited, map should be saved"
            )
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "[navigation_manager] mapping process did not exit in 30s, force killing"
            )

        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def _start_mapping_impl(self):
        """切换到建图模式：停止导航栈，启动建图 LIO。"""
        # 如果已在建图，先停掉
        with self.lock:
            was_mapping = self.mode == "mapping"
        if was_mapping:
            self._stop_mapping_gracefully()
        else:
            self._stop_managed_processes()

        # 为本轮建图生成统一时间戳（贯穿 PCD/PGM/YAML 命名），写 manifest 让上层
        # 可以立刻知道即将产出的文件名。
        base_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        with self.lock:
            self.current_map_basename = base_name
            self.mode = "mapping"
        self._write_map_manifest(base_name, status="mapping")
        self._prune_old_maps()

        self._transition_state(STATE_MAPPING, "starting mapping LIO")

        # 使用建图专用脚本（cleanup 会等待 super_lio 保存地图）
        proc = self._start_script(
            "start_pc2_mapping.sh",
            "mapping stack",
        )
        if proc is None:
            with self.lock:
                self.mode = "navigation"
            self._write_map_manifest(
                base_name, status="failed", error="failed to start mapping script"
            )
            self._transition_state(STATE_ERROR, "failed to start mapping script")
            return False, "failed to start mapping script"

        with self.lock:
            self.localization_proc = proc

        # 等待 LIO topic 确认建图已启动
        ok, _ = self._wait_until(
            lambda s: s["cloud_world_fresh"] and s["relocal_odom_fresh"],
            self.args.localization_timeout,
            abort_if=lambda: self._script_exit_failure(proc, "mapping stack"),
        )
        if not ok:
            failure = self._script_exit_failure(proc, "mapping stack")
            message = (
                failure["message"]
                if failure
                else "mapping LIO did not start in time"
            )
            reason = failure["reason"] if failure else None
            self._stop_mapping_gracefully()
            with self.lock:
                self.mode = "navigation"
            self._write_map_manifest(
                base_name,
                status="failed",
                error=message,
                reason=reason,
            )
            self._transition_state(STATE_ERROR, message)
            return False, message

        # 启动周期性 2D 快照（错误隔离：失败不影响建图主流程）
        try:
            from g1_base.mapping_snapshotter import MappingSnapshotter

            self.snapshotter = MappingSnapshotter(
                node=self,
                maps_dir=self._resolve_maps_dir(),
                base_name=base_name,
                callback_group=self.sub_cb_group,
                manifest_writer=lambda ts, bn=base_name: self._write_map_manifest(
                    bn, status="mapping", last_snapshot_at=ts
                ),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[navigation_manager] snapshotter init failed: {exc}"
            )
            self.snapshotter = None

        self._transition_state(STATE_MAPPING, "mapping active")
        self._publish_status()
        return True, f"mapping started ({base_name}), walk the robot to build the map"

    def _stop_mapping_impl(self):
        """停止建图，回到 STOPPED 状态，并自动生成 2D 地图。"""
        with self.lock:
            if self.mode != "mapping":
                return False, "not in mapping mode"
            base_name = self.current_map_basename

        # 先停快照再停 LIO，避免 LIO 退出过程中累积错误数据 + 释放 set 内存
        if self.snapshotter is not None:
            try:
                self.snapshotter.stop()
            except Exception as exc:
                self.get_logger().warning(
                    f"[navigation_manager] snapshotter stop failed: {exc}"
                )
            self.snapshotter = None

        self._stop_mapping_gracefully()
        with self.lock:
            self.mode = "navigation"
        self._transition_state(STATE_STOPPED, "mapping stopped, map.pcd saved")
        self._clear_last_error()
        self._publish_status()

        # 兜底：如果 base_name 丢失（异常路径），现场补一个但不写 mapping。
        # 此刻建图已停，对外应只呈现 ready/failed 终态，避免短时回退到 mapping 误导消费者。
        if not base_name:
            base_name = datetime.now().strftime("%Y%m%d_%H%M%S")
            with self.lock:
                self.current_map_basename = base_name

        pcd_path = self._resolve_map_pcd_path()
        if not pcd_path or not Path(pcd_path).is_file():
            self._write_map_manifest(base_name, status="failed", error="map.pcd not found")
            return False, "mapping stopped but map.pcd not found"

        threading.Thread(
            target=self._generate_and_publish_2d_map,
            args=(pcd_path, base_name),
            daemon=True,
        ).start()

        return True, f"mapping stopped, 2D map generation started ({base_name})"

    def _handle_start_mapping(self, _request, response):
        success, message = self._with_operation(
            "start_mapping", self._start_mapping_impl
        )
        response.success = success
        response.message = message
        return response

    def _handle_stop_mapping(self, _request, response):
        success, message = self._with_operation(
            "stop_mapping", self._stop_mapping_impl
        )
        response.success = success
        response.message = message
        return response

    # ── 2D 地图生成 ──

    def _resolve_map_pcd_path(self):
        """定位 map.pcd 路径（Super-LIO 标准位置）。

        容器里 Super-LIO 装在 $LIO_WORKSPACE_ROOT（/root/lio_ws），
        不在下面两个裸机历史位置下，所以先认环境变量再退回历史路径 ——
        否则停止建图后的自动 2D 生成会报 "map.pcd not found"。
        """
        candidates = []
        lio_root = os.environ.get("LIO_WORKSPACE_ROOT", "").strip()
        if lio_root:
            candidates.append(
                Path(lio_root).expanduser() / "src" / "Super-LIO" / "src" / "super_lio" / "map" / "map.pcd"
            )
        candidates += [
            Path.home() / "ros2_ws" / "src" / "Super-LIO" / "src" / "super_lio" / "map" / "map.pcd",
            Path.home() / "Super-LIO" / "src" / "super_lio" / "map" / "map.pcd",
        ]
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        return None

    def _resolve_maps_dir(self):
        """g1_base 内可写地图目录（PCD 复制件、PGM、YAML、manifest 都落在这）。"""
        configured = os.environ.get("G1_MAPS_DIR", "").strip()
        if configured:
            return Path(configured).expanduser()

        root = os.environ.get("G1_BASE_ROOT", "").strip()
        if root:
            return Path(root).expanduser() / "config" / "maps"

        return Path(package_share_dir()) / "config" / "maps"

    def _generate_2d_map(self, pcd_path, base_name):
        """同步执行：复制 PCD + 生成 PGM/YAML，全部按 base_name 命名落到 maps 目录。

        返回 (pgm_path, yaml_path, pcd_copy_path)；失败抛异常。
        """
        from g1_base.pcd_to_2d_map import (
            LEGACY_Z_MAX,
            LEGACY_Z_MIN,
            convert_pcd_to_2d_map,
        )

        maps_dir = self._resolve_maps_dir()
        maps_dir.mkdir(parents=True, exist_ok=True)

        pcd_target = maps_dir / f"{base_name}_map.pcd"
        shutil.copy2(pcd_path, pcd_target)

        pgm, yaml_f = convert_pcd_to_2d_map(
            pcd_path=str(pcd_target),
            output_dir=str(maps_dir),
            output_name=f"{base_name}_{MAP_NAME_SUFFIX}",
            z_min=LEGACY_Z_MIN,
            z_max=LEGACY_Z_MAX,
        )
        return pgm, yaml_f, str(pcd_target)

    def _generate_and_publish_2d_map(self, pcd_path, base_name):
        """后台线程入口：跑生成流程，结束后更新 manifest 状态。"""
        from g1_base.pcd_to_2d_map import TiltedWorldError

        try:
            pgm, yaml_f, pcd_copy = self._generate_2d_map(pcd_path, base_name)
            self.get_logger().info(
                f"[navigation_manager] 2D map ready: {pgm}"
            )
            self._write_map_manifest(base_name, status="ready")
            self.get_logger().info(
                f"[navigation_manager] manifest updated: status=ready ({base_name})"
            )
        except TiltedWorldError as exc:
            self.get_logger().error(
                f"[navigation_manager] 2D map generation failed: {exc}"
            )
            self._write_map_manifest(
                base_name,
                status="failed",
                reason="tilted_world",
                error=str(exc),
                tilt=exc.tilt_deg,
            )
        except Exception as exc:
            self.get_logger().error(
                f"[navigation_manager] 2D map generation failed: {exc}"
            )
            self._write_map_manifest(base_name, status="failed", error=str(exc))

    def _generate_2d_map_impl(self):
        """手动触发 2D 地图生成（不依赖 start/stop_mapping 流程）。"""
        pcd_path = self._resolve_map_pcd_path()
        if not pcd_path or not Path(pcd_path).is_file():
            return False, "map.pcd not found"
        base_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        with self.lock:
            self.current_map_basename = base_name
        self._write_map_manifest(base_name, status="mapping")
        self._prune_old_maps()
        self._generate_and_publish_2d_map(pcd_path, base_name)
        return True, f"2D map generation completed ({base_name})"

    def _write_map_manifest(
        self,
        base_name,
        status,
        error=None,
        last_snapshot_at=None,
        reason=None,
        tilt=None,
    ):
        """原子写入 G1_MAPS_DIR/current_map.json，作为 bot_mind 取最新地图的指针。

        status: "mapping" | "ready" | "failed"
        last_snapshot_at: 由 MappingSnapshotter 在每次 render 后传入，仅当 status=mapping 时有意义。
        """
        maps_dir = self._resolve_maps_dir()
        maps_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = maps_dir / MANIFEST_FILENAME

        existing = {}
        if manifest_path.is_file():
            try:
                existing = json.loads(manifest_path.read_text(encoding="utf-8"))
            except Exception:
                existing = {}

        now = datetime.now().isoformat(timespec="seconds")
        started_at = (
            existing.get("started_at")
            if existing.get("base_name") == base_name
            else None
        )
        if status == "mapping":
            # 仅在新会话首次写 mapping 时设置 started_at；同会话后续的快照刷新不重置
            if started_at is None:
                started_at = now
        finished_at = now if status in ("ready", "failed") else None

        # last_snapshot_at: 显式传入则覆盖；否则保留 existing 中已有值（终态写入也保留快照历史）
        snapshot_ts = (
            last_snapshot_at
            if last_snapshot_at is not None
            else existing.get("last_snapshot_at")
            if existing.get("base_name") == base_name
            else None
        )

        payload = {
            "base_name": base_name,
            "status": status,
            "pgm": f"{base_name}_{MAP_NAME_SUFFIX}.pgm",
            "yaml": f"{base_name}_{MAP_NAME_SUFFIX}.yaml",
            "pcd": f"{base_name}_map.pcd",
            "started_at": started_at,
            "finished_at": finished_at,
            "last_snapshot_at": snapshot_ts,
            "reason": reason,
            "error": error,
            "tilt": f"{float(tilt):.2f}°" if tilt is not None else None,
        }

        tmp_path = manifest_path.with_name(manifest_path.name + ".tmp")
        tmp_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(tmp_path, manifest_path)

    def _prune_old_maps(self, keep=MAP_RETENTION_KEEP):
        """按时间戳保留最近 keep 份地图三件套（pgm/yaml/pcd），其余删除。

        以 *_<MAP_NAME_SUFFIX>.pgm 为索引：文件名前缀即 base_name，倒序保留。
        """
        maps_dir = self._resolve_maps_dir()
        if not maps_dir.is_dir():
            return

        suffix = f"_{MAP_NAME_SUFFIX}.pgm"
        # 文件名按时间戳前缀字典序倒序，等价于按时间倒序
        pgms = sorted(
            maps_dir.glob(f"*{suffix}"),
            key=lambda p: p.name,
            reverse=True,
        )
        for stale in pgms[keep:]:
            base = stale.name[: -len(suffix)]
            for path in (
                stale,
                maps_dir / f"{base}_{MAP_NAME_SUFFIX}.yaml",
                maps_dir / f"{base}_map.pcd",
            ):
                try:
                    if path.is_file():
                        path.unlink()
                except Exception as exc:
                    self.get_logger().warning(
                        f"[navigation_manager] failed to prune {path}: {exc}"
                    )

    def _handle_generate_2d_map(self, _request, response):
        success, message = self._with_operation(
            "generate_2d_map", self._generate_2d_map_impl
        )
        response.success = success
        response.message = message
        return response

    def shutdown(self):
        self.shutting_down = True
        self.monitor_timer.cancel()
        self.get_logger().info("[navigation_manager] shutting down")
        if self.snapshotter is not None:
            try:
                self.snapshotter.stop()
            except Exception:
                pass
            self.snapshotter = None
        with self.lock:
            mode = self.mode
        if mode == "mapping":
            self._stop_mapping_gracefully()
        else:
            self._stop_managed_processes()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Bottom navigation manager for PC2")
    parser.add_argument("--auto-ensure", action="store_true")
    parser.add_argument("--node-name", default="navigation_manager")
    parser.add_argument("--monitor-hz", type=float, default=2.0)
    parser.add_argument("--freshness-window", type=float, default=3.0)
    parser.add_argument("--localization-timeout", type=float, default=60.0)
    parser.add_argument("--navigation-timeout", type=float, default=60.0)
    parser.add_argument("--max-bringup-attempts", type=int, default=2)
    parser.add_argument("--pointcloud-topic", default="/lio/cloud_world")
    parser.add_argument("--relocal-odom-topic", default="/lio/robo/odom")
    parser.add_argument("--odom-topic", default="/odom_2d")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    return parser.parse_known_args(argv)


def main(args=None):
    parsed, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    node = NavigationManager(parsed)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
