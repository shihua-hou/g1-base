import argparse
import json
import math
import threading
import time
from pathlib import Path

from g1_base.logging import get_logger, log_elapsed

_logger = get_logger("control_server")

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from g1_base.common import config_file, movement_dir, normalize_angle
from g1_base.nav_core import MissionNode, RobotController, force_robot_stop
from g1_base_interfaces.action import NavigateToTarget
from g1_base_interfaces.srv import (
    ExecuteArmAction,
    ExecuteCustomAction,
    GetFsmId,
    MoveRobot,
    PlayNamedAction,
    RotateRobot,
    RunMovementScript,
    SetVolume,
    Speak,
    SetFsmId,
    SquatRobot,
    StopRobot,
)


SERVICE_STATUS_SUCCESS = "success"
SERVICE_STATUS_ERROR = "error"
SERVICE_STATUS_CANCELED = "canceled"
STATUS_TOPIC = "/g1_control/status"
STATUS_PUBLISH_HZ = 2.0

# 摇杆遥操作：网页端发 Twist 到这个话题
TELEOP_CMD_VEL_TOPIC = "/g1_control/teleop_cmd_vel"
# 手动速度覆盖的有效期。收不到新指令超过这个时间，nav_core 自动让覆盖失效，
# 机器人停下 —— 这是网页断连时唯一的保护，别调大。
TELEOP_WATCHDOG_SEC = 0.4
NAMED_ACTION_TO_ARM_ACTION = {
    "wave": 25,
}


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def parse_control_server_args(argv=None):
    parser = argparse.ArgumentParser(description="G1 ROS2 control server")
    parser.add_argument("--net-if", default="enP8p1s0")
    parser.add_argument("--node-name", default="g1_control_server")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    return parser.parse_known_args(argv)


class G1ControlServer(MissionNode):
    def __init__(self, args):
        super().__init__(args)
        self.set_external_spin(True)
        self._server_group = ReentrantCallbackGroup()
        self._runtime_lock = threading.RLock()
        self._script_cancel_event = threading.Event()
        self._navigation_cancel_event = threading.Event()
        self._named_action_dir = Path(movement_dir()) / "motions"
        # 音频原本是关的（基线提交带进来的，没留原因），于是 speak() 一直走
        # [speech-disabled] 分支——网页上点「试听」、巡航到点讲解、事件播报
        # 全都没声音，而且不报错。推测当初关掉是因为 _wakeup_audio() 会阻塞
        # 最多 10 秒拖慢启动；现在唤醒改成后台线程，就没有关掉的理由了。
        # 真要关（比如展厅要求静音）用环境变量 G1_DISABLE_AUDIO=1。
        self.robot_controller = RobotController(
            self,
            args.net_if,
            enable_arm=True,
        )

        # ── 活动状态跟踪 ──
        self._current_activity = "idle"
        self._current_activity_detail = ""
        self._activity_start_time = 0.0
        self._status_lock = threading.Lock()

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, status_qos)
        self._status_timer = self.create_timer(
            1.0 / STATUS_PUBLISH_HZ, self._publish_status
        )

        # ── 摇杆遥操作 ──
        # 网页端按住摇杆时约 10Hz 发 Twist；这里转成带 deadline 的手动速度覆盖，
        # 由 nav_core 的 _select_command 在 deadline 到点后自动失效。也就是说
        # 平板断网 / 息屏 / 应用崩了，机器人在 TELEOP_WATCHDOG_SEC 内自己停下，
        # 不需要网页再发一条停止命令。
        self._teleop_sub = self.create_subscription(
            Twist,
            TELEOP_CMD_VEL_TOPIC,
            self._on_teleop_cmd_vel,
            10,
            callback_group=self._server_group,
        )
        self._teleop_active = False
        self._teleop_blocked_warned = False

        self._navigate_action = ActionServer(
            self,
            NavigateToTarget,
            "/g1_control/navigate_to_target",
            execute_callback=self._execute_navigate_to_target,
            goal_callback=self._accept_navigation_goal,
            cancel_callback=self._cancel_navigation_goal,
            callback_group=self._server_group,
        )
        self._execute_arm_service = self.create_service(
            ExecuteArmAction,
            "/g1_control/execute_arm_action",
            self._handle_execute_arm_action,
            callback_group=self._server_group,
        )
        self._play_named_action_service = self.create_service(
            PlayNamedAction,
            "/g1_control/play_named_action",
            self._handle_play_named_action,
            callback_group=self._server_group,
        )
        self._move_robot_service = self.create_service(
            MoveRobot,
            "/g1_control/move_robot",
            self._handle_move_robot,
            callback_group=self._server_group,
        )
        self._rotate_robot_service = self.create_service(
            RotateRobot,
            "/g1_control/rotate_robot",
            self._handle_rotate_robot,
            callback_group=self._server_group,
        )
        self._stop_robot_service = self.create_service(
            StopRobot,
            "/g1_control/stop_robot",
            self._handle_stop_robot,
            callback_group=self._server_group,
        )
        self._execute_custom_action_service = self.create_service(
            ExecuteCustomAction,
            "/g1_control/execute_custom_action",
            self._handle_execute_custom_action,
            callback_group=self._server_group,
        )
        self._run_movement_script_service = self.create_service(
            RunMovementScript,
            "/g1_control/run_movement_script",
            self._handle_run_movement_script,
            callback_group=self._server_group,
        )
        self._squat_robot_service = self.create_service(
            SquatRobot,
            "/g1_control/squat_robot",
            self._handle_squat_robot,
            callback_group=self._server_group,
        )
        self._set_fsm_id_service = self.create_service(
            SetFsmId,
            "/g1_control/set_fsm_id",
            self._handle_set_fsm_id,
            callback_group=self._server_group,
        )
        self._get_fsm_id_service = self.create_service(
            GetFsmId,
            "/g1_control/get_fsm_id",
            self._handle_get_fsm_id,
            callback_group=self._server_group,
        )
        self._speak_service = self.create_service(
            Speak,
            "/g1_control/speak",
            self._handle_speak,
            callback_group=self._server_group,
        )
        self._set_volume_service = self.create_service(
            SetVolume,
            "/g1_control/set_volume",
            self._handle_set_volume,
            callback_group=self._server_group,
        )

    # ── 活动状态管理 ──

    def _set_activity(self, activity, detail=""):
        """标记当前正在执行的活动，并立即发布一次状态。"""
        with self._status_lock:
            self._current_activity = activity
            self._current_activity_detail = detail
            self._activity_start_time = time.time()
        self._publish_status()

    def _clear_activity(self):
        """将活动重置为 idle，并立即发布一次状态。"""
        with self._status_lock:
            self._current_activity = "idle"
            self._current_activity_detail = ""
            self._activity_start_time = 0.0
        self._publish_status()

    def _on_teleop_cmd_vel(self, msg):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        idle = abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3

        if self.robot_controller.is_squatting:
            if self._teleop_active:
                self.robot_controller.motion.clear_manual_override()
                self._teleop_active = False
            # 摇杆按住时是 10Hz，只在状态翻转时记一条，别刷日志
            if not self._teleop_blocked_warned:
                self._teleop_blocked_warned = True
                _logger.warning("teleop 忽略：机器人处于蹲下状态，请先站起来")
            return
        self._teleop_blocked_warned = False

        if idle:
            # 松手：先把速度打到 0，再交还控制权（与 _move_robot 收尾一致）
            if self._teleop_active:
                self.robot_controller.motion.set_manual_velocity(
                    0.0, 0.0, 0.0, timeout=TELEOP_WATCHDOG_SEC, source="teleop_pad_stop"
                )
                self.robot_controller.motion.clear_manual_override()
                self._teleop_active = False
                self._clear_activity()
            return

        if not self._teleop_active:
            self.robot_controller.motion.end_navigation_goal()
            self.robot_controller.motion.release_stop_latch("teleop_pad")
            self._teleop_active = True
            self._set_activity("teleop", "摇杆遥操作")

        self.robot_controller.motion.set_manual_velocity(
            vx, vy, wz, timeout=TELEOP_WATCHDOG_SEC, source="teleop_pad"
        )

    def _check_squatting_guard(self, action_name, response):
        """检查机器人是否处于蹲下状态，若是则填充拒绝响应并返回 True。"""
        if self.robot_controller.is_squatting:
            msg = f"机器人处于蹲下状态，无法执行 {action_name}，请先站起来"
            _logger.warning("%s 拒绝 (蹲下状态): %s", action_name, msg)
            self._fill_service_response(response, {
                "status": SERVICE_STATUS_ERROR,
                "message": msg,
            })
            return True
        return False

    def _publish_status(self):
        """将当前活动状态发布到 /g1_control/status 话题。"""
        with self._status_lock:
            activity = self._current_activity
            detail = self._current_activity_detail
            start_time = self._activity_start_time

        motion_source = "unknown"
        stop_latched = False
        navigation_failure_reason = ""
        close_obstacle_active = False
        close_obstacle_front_min = None
        close_obstacle_trigger_distance = None
        close_obstacle_release_distance = None
        sdk_snapshot = {}
        try:
            motion = self.robot_controller.motion
            motion_source = motion.last_sent_source
            stop_latched = motion.is_stop_latched()
            navigation_failure_reason = motion.navigation_failure_reason
            command_snapshot = motion.get_command_snapshot()
            close_obstacle_active = bool(
                command_snapshot.get("close_obstacle_active", False)
            )
            raw_front_min = command_snapshot.get("close_obstacle_front_min")
            if isinstance(raw_front_min, (int, float)) and math.isfinite(raw_front_min):
                close_obstacle_front_min = round(float(raw_front_min), 3)
            policy = getattr(motion, "motion_policy", None)
            if policy is not None:
                close_obstacle_trigger_distance = getattr(
                    policy, "close_obstacle_trigger_distance", None
                )
                close_obstacle_release_distance = getattr(
                    policy, "close_obstacle_release_distance", None
                )
            sdk_snapshot = self.robot_controller.loco.GetHealthSnapshot()
        except Exception:
            pass

        status = {
            "activity": activity,
            "activity_detail": detail,
            "activity_duration": round(time.time() - start_time, 1) if start_time > 0 else 0.0,
            "motion_source": motion_source,
            "is_squatting": self.robot_controller.is_squatting,
            "stop_latched": stop_latched,
            "sdk_last_loco_latency_ms": sdk_snapshot.get("last_loco_latency_ms", 0.0),
            "sdk_loco_latency_p95_ms": sdk_snapshot.get("loco_latency_p95_ms", 0.0),
            "sdk_slow_count": sdk_snapshot.get("loco_slow_count", 0),
            "navigation_failure_reason": navigation_failure_reason,
            "close_obstacle_active": close_obstacle_active,
            "close_obstacle_front_min": close_obstacle_front_min,
            "close_obstacle_trigger_distance": close_obstacle_trigger_distance,
            "close_obstacle_release_distance": close_obstacle_release_distance,
            "timestamp": time.time(),
        }
        self._status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    def shutdown(self):
        try:
            self._status_timer.cancel()
        except Exception:
            pass
        try:
            self._navigate_action.destroy()
        except Exception:
            pass
        try:
            self.robot_controller.shutdown()
        finally:
            self.shutdown_resources()

    @staticmethod
    def _fill_service_response(response, result):
        response.status = str(result.get("status") or SERVICE_STATUS_ERROR)
        response.message = str(result.get("message") or "")
        response.success = response.status == SERVICE_STATUS_SUCCESS
        return response

    def _accept_navigation_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_navigation_goal(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _publish_feedback(self, goal_handle, phase, distance_to_goal):
        feedback = NavigateToTarget.Feedback()
        feedback.phase = str(phase)
        feedback.distance_to_goal = float(distance_to_goal)
        goal_handle.publish_feedback(feedback)

    def _build_action_result(self, goal_handle, result):
        ros_result = NavigateToTarget.Result()
        ros_result.status = str(result.get("status") or SERVICE_STATUS_ERROR)
        ros_result.message = str(result.get("message") or "")
        ros_result.success = ros_result.status == SERVICE_STATUS_SUCCESS

        if ros_result.status == SERVICE_STATUS_SUCCESS:
            goal_handle.succeed()
        elif ros_result.status == SERVICE_STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return ros_result

    def _execute_navigate_to_target(self, goal_handle):
        request = goal_handle.request
        waypoint_name = str(request.waypoint_name or "").strip() or "target"
        self._navigation_cancel_event.clear()

        # ── 蹲下拦截 ──
        if self.robot_controller.is_squatting:
            _logger.warning("导航拒绝 (蹲下状态)")
            result = {
                "status": SERVICE_STATUS_ERROR,
                "message": "机器人处于蹲下状态，无法导航，请先站起来",
            }
            return self._build_action_result(goal_handle, result)

        waypoint = {
            "name": waypoint_name,
            "x": float(request.target_pose.pose.position.x),
            "y": float(request.target_pose.pose.position.y),
            "yaw": self._yaw_from_pose(request.target_pose),
            "align_final_yaw": bool(request.align_final_yaw),
            # 之前这两项写死成 None/""，而 action 里也没有对应字段——
            # 于是巡航点上配的「动作 31 · 定位区」是死数据，机器人走到了
            # 既不做动作也不说话。现在由调用方（巡航）传进来。
            "action_id": int(request.action_id),
            "say_text": str(request.say_text or ""),
        }
        perform_interaction = bool(request.perform_interaction)

        with self._runtime_lock:
            self._set_activity("navigating", f"目标: {waypoint_name}")
            try:
                self.ensure_navigation_stack_ready_with_manager()
                result = self.navigate_to_waypoint(
                    waypoint,
                    self.robot_controller,
                    perform_interaction=perform_interaction,
                    announce_failures=False,
                    feedback_cb=lambda phase, distance: self._publish_feedback(
                        goal_handle, phase, distance
                    ),
                    cancel_requested=lambda: bool(goal_handle.is_cancel_requested)
                    or self._navigation_cancel_event.is_set(),
                )
            except Exception as exc:
                self.get_logger().error(f"导航执行失败: {exc}")
                result = {
                    "status": SERVICE_STATUS_ERROR,
                    "message": str(exc),
                }
            finally:
                self._navigation_cancel_event.clear()
                self._clear_activity()
        return self._build_action_result(goal_handle, result)

    @staticmethod
    def _yaw_from_pose(pose_stamped):
        q = pose_stamped.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _resolve_named_action(self, action_name):
        normalized = str(action_name or "").strip()
        if not normalized:
            raise ValueError("action_name 不能为空")

        action_path = self._named_action_dir / (
            normalized if normalized.endswith(".jsonl") else f"{normalized}.jsonl"
        )
        if not action_path.exists():
            raise FileNotFoundError(f"动作文件不存在: {action_path}")

        rows = self._load_jsonl(action_path)
        if not rows:
            raise ValueError(f"动作文件为空: {action_path}")

        first = rows[0]

        # 向后兼容：如果第一行有 sdk_action_id 且没有 q 字段，走预设动作
        if "sdk_action_id" in first and "q" not in first:
            return {"type": "preset", "action_id": int(first["sdk_action_id"])}

        # 否则尝试通过名称查找预设动作映射（兼容旧配置）
        action_id = NAMED_ACTION_TO_ARM_ACTION.get(action_path.stem)
        if action_id is not None and "q" not in first:
            return {"type": "preset", "action_id": action_id}

        # 如果第一行既没有轨迹数据(q/joints)，也没有 sdk_action_id —— 无效文件
        if "q" not in first and "joints" not in first:
            raise ValueError(
                f"动作文件 {action_path.name} 既不是预设动作 (缺 sdk_action_id)，"
                f"也不是自定义轨迹 (缺 joints/q)。"
                f"请检查 JSONL 格式"
            )

        # 自定义轨迹
        trajectory_rows = self._load_trajectory_rows(rows, action_path)
        return {"type": "trajectory", "rows": trajectory_rows}

    @staticmethod
    def _load_jsonl(path):
        rows = []
        with open(path, "r", encoding="utf-8") as fp:
            for line in fp:
                stripped = line.strip()
                if stripped:
                    rows.append(json.loads(stripped))
        return rows

    @staticmethod
    def _load_trajectory_rows(raw_rows, source_path):
        if not raw_rows:
            raise ValueError(f"轨迹为空: {source_path}")

        first = raw_rows[0]
        if "joints" not in first:
            raise ValueError(f"第一行缺少 joints: {source_path}")

        joints = [int(v) for v in first["joints"]]
        if not joints:
            raise ValueError(f"joints 不能为空: {source_path}")

        normalized = []
        previous_t = None
        default_group = str(first.get("group", "unknown"))

        for index, row in enumerate(raw_rows):
            if "t" not in row or "q" not in row:
                raise ValueError(f"轨迹行 {index} 缺少 t 或 q: {source_path}")

            t_value = float(row["t"])
            q_values = [float(v) for v in row["q"]]

            if len(q_values) != len(joints):
                raise ValueError(
                    f"轨迹行 {index} q 长度 {len(q_values)} != 关节数 {len(joints)}"
                )
            if previous_t is not None and t_value < previous_t:
                raise ValueError(f"轨迹行 {index} 时间戳不单调")
            previous_t = t_value

            normalized.append({
                "t": t_value,
                "q": q_values,
                "joints": joints,
                "group": str(row.get("group", default_group)),
            })

        return normalized

    def _handle_execute_arm_action(self, request, response):
        _logger.info("收到 execute_arm_action 请求: action_id=%s", request.action_id)
        if self._check_squatting_guard("execute_arm_action", response):
            return response
        with self._runtime_lock:
            self._set_activity("arm_action", f"动作ID: {request.action_id}")
            try:
                with log_elapsed(_logger, f"execute_arm_action(id={request.action_id})"):
                    result = self.robot_controller.execute_arm_action_safe(
                        request.action_id,
                        reset_after=True,
                    )
            except Exception as exc:
                _logger.error("execute_arm_action 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("execute_arm_action 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_speak(self, request, response):
        text = str(request.text or "").strip()
        if not text:
            response.success = False
            response.status = SERVICE_STATUS_ERROR
            response.message = "播报内容为空"
            return response
        # 不进 _runtime_lock：播报和动作/移动互不冲突，
        # 而且导航过程中的事件播报正需要能插进来。
        try:
            spoken = self.robot_controller.speak(text, int(request.voice_id))
        except Exception as exc:
            _logger.error("speak 失败: %s", exc, exc_info=True)
            response.success = False
            response.status = SERVICE_STATUS_ERROR
            response.message = str(exc)
            return response
        response.success = bool(spoken)
        response.status = SERVICE_STATUS_SUCCESS if spoken else SERVICE_STATUS_ERROR
        response.message = "已播报" if spoken else "音频客户端不可用"
        return response

    def _handle_set_volume(self, request, response):
        try:
            if int(request.volume) < 0:
                # 负数 = 只查询
                value = self.robot_controller.get_volume()
            else:
                value = self.robot_controller.set_volume(int(request.volume))
        except Exception as exc:
            _logger.error("set_volume 失败: %s", exc, exc_info=True)
            response.success = False
            response.volume = -1
            response.status = SERVICE_STATUS_ERROR
            response.message = str(exc)
            return response
        response.success = value is not None
        response.volume = int(value) if value is not None else -1
        response.status = SERVICE_STATUS_SUCCESS if value is not None else SERVICE_STATUS_ERROR
        response.message = "" if value is not None else "音量不可读"
        return response

    def _handle_play_named_action(self, request, response):
        _logger.info("收到 play_named_action 请求: action_name=%s", request.action_name)
        if self._check_squatting_guard("play_named_action", response):
            return response
        with self._runtime_lock:
            self._set_activity("trajectory", f"动作: {request.action_name}")
            try:
                with log_elapsed(_logger, f"resolve_named_action({request.action_name})"):
                    resolved = self._resolve_named_action(request.action_name)
                _logger.info("动作解析结果: type=%s", resolved["type"])
                if resolved["type"] == "preset":
                    with log_elapsed(_logger, f"execute_arm_action_safe(id={resolved['action_id']})"):
                        result = self.robot_controller.execute_arm_action_safe(
                            resolved["action_id"],
                            reset_after=True,
                        )
                else:
                    with log_elapsed(_logger, f"play_trajectory_safe(frames={len(resolved['rows'])})"):
                        result = self.robot_controller.play_trajectory_safe(
                            resolved["rows"],
                        )
            except Exception as exc:
                _logger.error("play_named_action 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("play_named_action 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_move_robot(self, request, response):
        _logger.info("收到 move_robot 请求: direction=%s, distance=%.2f, speed_scale=%.2f",
                     request.direction, request.distance_m, request.speed_scale)
        if self._check_squatting_guard("move_robot", response):
            return response
        with self._runtime_lock:
            self._set_activity(
                "moving", f"{request.direction} {request.distance_m:.2f}m"
            )
            try:
                with log_elapsed(_logger, f"move_robot({request.direction}, {request.distance_m:.2f}m)"):
                    result = self._move_robot(
                        request.direction,
                        request.distance_m,
                    )
            except Exception as exc:
                _logger.error("move_robot 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("move_robot 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_rotate_robot(self, request, response):
        _logger.info("收到 rotate_robot 请求: angle_deg=%.1f, speed_scale=%.2f",
                     request.angle_deg, request.speed_scale)
        if self._check_squatting_guard("rotate_robot", response):
            return response
        with self._runtime_lock:
            self._set_activity("rotating", f"{request.angle_deg:.1f}°")
            try:
                with log_elapsed(_logger, f"rotate_robot({request.angle_deg:.1f}°)"):
                    result = self._rotate_robot(
                        request.angle_deg,
                        request.speed_scale,
                    )
            except Exception as exc:
                _logger.error("rotate_robot 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("rotate_robot 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_stop_robot(self, _request, response):
        _logger.info("收到 stop_robot 请求")
        self._script_cancel_event.set()
        self._navigation_cancel_event.set()
        with self._status_lock:
            stopping_navigation = self._current_activity == "navigating"
        try:
            with log_elapsed(_logger, "stop_robot"):
                self.robot_controller.motion.emergency_stop_latched("stop_robot")
                if not stopping_navigation:
                    self._navigation_cancel_event.clear()
            result = {"status": SERVICE_STATUS_SUCCESS, "message": "已请求停止"}
        except Exception as exc:
            _logger.error("stop_robot 失败: %s", exc, exc_info=True)
            result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
        return self._fill_service_response(response, result)

    def _handle_execute_custom_action(self, request, response):
        _logger.info("收到 execute_custom_action 请求: action_name=%s", request.action_name)
        if self._check_squatting_guard("execute_custom_action", response):
            return response
        with self._runtime_lock:
            self._set_activity("custom_action", f"示教动作: {request.action_name}")
            try:
                with log_elapsed(_logger, f"execute_custom_action({request.action_name})"):
                    result = self.robot_controller.execute_custom_action_safe(
                        request.action_name,
                    )
            except Exception as exc:
                _logger.error("execute_custom_action 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("execute_custom_action 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_run_movement_script(self, request, response):
        _logger.info("收到 run_movement_script 请求: script_path=%s", request.script_path)
        self._script_cancel_event.clear()
        if self._check_squatting_guard("run_movement_script", response):
            return response
        with self._runtime_lock:
            self._set_activity("movement_script", f"脚本: {request.script_path}")
            try:
                with log_elapsed(_logger, f"run_movement_script({request.script_path})"):
                    result = self.robot_controller.run_movement_script_safe(
                        request.script_path,
                        movement_base_dir=movement_dir(),
                        cancel_event=self._script_cancel_event,
                    )
            except Exception as exc:
                _logger.error("run_movement_script 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("run_movement_script 响应: %s", result)
        return self._fill_service_response(response, result)

    _SQUAT_ACTIONS = {
        "squat":    {"label": "squatting",    "detail": "蹲下中",      "verb": "蹲下"},
        "stand_up": {"label": "standing_up",  "detail": "站起来中",    "verb": "站起来"},
        "damp":     {"label": "damping",      "detail": "切换阻尼中",  "verb": "切换阻尼模式"},
        "start":    {"label": "starting",     "detail": "切换站立中",  "verb": "切换站立模式"},
    }

    def _handle_squat_robot(self, request, response):
        action = str(request.action or "").strip().lower()
        _logger.info("收到 squat_robot 请求: action=%s", action)

        action_info = self._SQUAT_ACTIONS.get(action)
        if action_info is None:
            return self._fill_service_response(response, {
                "status": SERVICE_STATUS_ERROR,
                "message": f"无效动作: {action}，支持: {', '.join(self._SQUAT_ACTIONS)}",
            })

        # ── 安全前置检查：必须 idle 才允许执行 ──
        with self._status_lock:
            current = self._current_activity
        if current != "idle":
            msg = f"机器人正在执行 {current}，无法{action_info['verb']}"
            _logger.warning("squat_robot 拒绝: %s", msg)
            return self._fill_service_response(response, {
                "status": SERVICE_STATUS_ERROR,
                "message": msg,
            })

        with self._runtime_lock:
            self._set_activity(action_info["label"], action_info["detail"])
            try:
                with log_elapsed(_logger, f"squat_robot({action})"):
                    if action == "squat":
                        result = self.robot_controller.squat_safe()
                    elif action == "stand_up":
                        result = self.robot_controller.stand_up_safe()
                    elif action == "damp":
                        result = self.robot_controller.damp_safe()
                    else:  # start
                        result = self.robot_controller.start_safe()
            except Exception as exc:
                _logger.error("squat_robot 失败: %s", exc, exc_info=True)
                result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
            finally:
                self._clear_activity()
        _logger.info("squat_robot 响应: %s", result)
        return self._fill_service_response(response, result)

    def _handle_set_fsm_id(self, request, response):
        fsm_id = int(request.fsm_id)
        _logger.info("收到 set_fsm_id 请求: fsm_id=%s", fsm_id)
        try:
            with log_elapsed(_logger, f"set_fsm_id({fsm_id})"):
                result = self.robot_controller.set_fsm_id_safe(fsm_id)
        except Exception as exc:
            _logger.error("set_fsm_id 失败: %s", exc, exc_info=True)
            result = {"status": SERVICE_STATUS_ERROR, "message": str(exc)}
        response = self._fill_service_response(response, result)
        response.return_code = int(result.get("return_code", 0)) if isinstance(result, dict) else 0
        _logger.info("set_fsm_id 响应: status=%s return_code=%s", response.status, response.return_code)
        return response

    def _handle_get_fsm_id(self, _request, response):
        try:
            fsm_id = int(self.robot_controller.get_fsm_id_safe())
            response.fsm_id = fsm_id
            return self._fill_service_response(response, {
                "status": SERVICE_STATUS_SUCCESS,
                "message": f"FSM ID = {fsm_id}",
            })
        except Exception as exc:
            _logger.error("get_fsm_id 失败: %s", exc, exc_info=True)
            response.fsm_id = -1
            return self._fill_service_response(response, {
                "status": SERVICE_STATUS_ERROR,
                "message": str(exc),
            })

    def _move_robot(self, direction, distance_m):
        normalized_direction = str(direction or "").strip().lower()
        if normalized_direction not in {"forward", "backward", "left", "right"}:
            raise ValueError("direction 仅支持: forward/backward/left/right")

        target_distance = float(distance_m)
        if target_distance <= 0.0:
            return {"status": SERVICE_STATUS_SUCCESS, "message": "无需移动"}

        wm = self.robot_controller.motion.walking_mode
        linear_speed = wm.linear_speed
        vx, vy = {
            "forward": (linear_speed, 0.0),
            "backward": (-linear_speed, 0.0),
            "left": (0.0, linear_speed),
            "right": (0.0, -linear_speed),
        }[normalized_direction]

        self.robot_controller.motion.end_navigation_goal()
        self.robot_controller.motion.release_stop_latch("manual_move")
        start_pose = None
        try:
            start_pose = self.lookup_current_pose()
        except Exception:
            pass

        expected_duration = target_distance / linear_speed
        deadline = time.time() + expected_duration + wm.move_deadline_margin
        start_time = time.time()
        try:
            while rclpy.ok() and time.time() < deadline:
                self.robot_controller.motion.set_manual_velocity(
                    vx,
                    vy,
                    0.0,
                    timeout=0.2,
                    source=f"g1_control_move_{normalized_direction}",
                )
                if start_pose is not None:
                    try:
                        cur_x, cur_y, _ = self.lookup_current_pose()
                        moved = math.hypot(cur_x - start_pose[0], cur_y - start_pose[1])
                        if moved >= target_distance:
                            break
                    except Exception:
                        pass
                elif time.time() - start_time >= expected_duration:
                    break
                time.sleep(0.05)
        finally:
            self.robot_controller.motion.set_manual_velocity(
                0.0, 0.0, 0.0, timeout=0.2, source="g1_control_move_stop"
            )
            time.sleep(0.2)
            self.robot_controller.motion.clear_manual_override()

        return {
            "status": SERVICE_STATUS_SUCCESS,
            "message": f"已完成 {normalized_direction} {target_distance:.2f} 米",
        }

    def _rotate_robot(self, angle_deg, speed_scale):
        angle_deg = float(angle_deg)
        if abs(angle_deg) < 1e-3:
            return {"status": SERVICE_STATUS_SUCCESS, "message": "无需旋转"}

        speed = _clamp(abs(float(speed_scale) or 1.0), 0.2, 1.5)
        wm = self.robot_controller.motion.walking_mode
        wz = _clamp(wm.rotate_wz_base * speed, wm.rotate_wz_min, wm.rotate_wz_max)
        wz = wz if angle_deg >= 0 else -wz
        duration = abs(math.radians(angle_deg) / abs(wz))

        self.robot_controller.motion.end_navigation_goal()
        self.robot_controller.motion.release_stop_latch("manual_rotate")
        deadline = time.time() + duration + wm.rotate_deadline_margin
        try:
            while rclpy.ok() and time.time() < deadline:
                self.robot_controller.motion.set_manual_velocity(
                    0.0,
                    0.0,
                    wz,
                    timeout=0.2,
                    source="g1_control_rotate",
                )
                time.sleep(0.05)
        finally:
            self.robot_controller.motion.set_manual_velocity(
                0.0, 0.0, 0.0, timeout=0.2, source="g1_control_rotate_stop"
            )
            time.sleep(0.2)
            self.robot_controller.motion.clear_manual_override()

        return {
            "status": SERVICE_STATUS_SUCCESS,
            "message": f"已旋转 {angle_deg:.1f} 度",
        }


def main(argv=None):
    args, ros_args = parse_control_server_args(argv)
    rclpy.init(args=ros_args)
    node = G1ControlServer(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
