import argparse
import json
import subprocess
import sys
import threading
import time
import traceback


class UnitreeSdkBridge:
    LOCO_DISPATCH_HZ = 5.0
    LOCO_SLOW_LOG_MS = 200.0
    LOCO_LATENCY_WINDOW = 30
    LOCO_ACTIVE_PLANNER_SOURCES = {"planner", "lateral_assist", "rotate_recovery"}

    def __init__(
        self,
        node,
        network_interface,
        python_executable,
        domain_id=0,
        enable_audio=True,
        enable_arm=True,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._python_executable = python_executable
        self._response_lock = threading.Lock()
        self._stdin_lock = threading.Lock()
        self._request_lock = self._response_lock
        self._next_request_id = 1
        self._loco_dispatch_lock = threading.Condition()
        self._loco_dispatch_latest = None
        self._loco_dispatch_epoch = 0
        self._loco_dispatch_stop = False
        self._loco_min_interval_sec = 1.0 / self.LOCO_DISPATCH_HZ
        self._loco_latency_lock = threading.Lock()
        self._loco_latency_samples_ms = []
        self._loco_latency_active_samples_ms = []
        self._last_loco_latency_ms = 0.0
        self._last_loco_latency_sample_time = 0.0
        self._last_active_loco_latency_sample_time = 0.0
        self._loco_slow_count = 0

        try:
            self._process = subprocess.Popen(
                [
                    self._python_executable,
                    "-m",
                    "g1_base.unitree_sdk_bridge",
                    "--worker",
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            raise RuntimeError(
                f"无法启动 Unitree SDK 子进程: {self._python_executable}: {exc}"
            ) from exc

        self._stderr_thread = threading.Thread(
            target=self._pump_stderr,
            name="unitree-sdk-stderr",
            daemon=True,
        )
        self._stderr_thread.start()

        capabilities = self.request(
            "init",
            network_interface=network_interface,
            domain_id=domain_id,
            enable_audio=enable_audio,
            enable_arm=enable_arm,
        )
        self._loco_dispatch_thread = threading.Thread(
            target=self._run_loco_dispatcher,
            name="unitree-loco-dispatcher",
            daemon=True,
        )
        self._loco_dispatch_thread.start()
        self.loco_client = _LocoClientProxy(self)
        self.audio_client = _AudioClientProxy(self) if capabilities.get("audio") else None
        self.arm_client = _ArmClientProxy(self) if capabilities.get("arm") else None

    def _write_message(self, message):
        process = getattr(self, "_process", None)
        if process is None or process.poll() is not None:
            raise RuntimeError(
                f"Unitree SDK 子进程已退出，退出码 {getattr(process, 'returncode', None)}"
            )
        try:
            with self._stdin_lock:
                assert process.stdin is not None
                process.stdin.write(json.dumps(message, ensure_ascii=True) + "\n")
                process.stdin.flush()
        except BrokenPipeError as exc:
            raise RuntimeError("Unitree SDK 子进程管道已断开") from exc

    def _fire_and_forget(self, command, **payload):
        message = {"id": -1, "command": command}
        message.update(payload)
        self._write_message(message)

    def queue_loco_move(self, vx, vy, wz, source="planner"):
        with self._loco_dispatch_lock:
            self._loco_dispatch_latest = {
                "vx": float(vx),
                "vy": float(vy),
                "wz": float(wz),
                "source": str(source or "planner"),
                "epoch": self._loco_dispatch_epoch,
            }
            self._loco_dispatch_lock.notify()

    def urgent_loco_stop(self, reason=""):
        source = f"urgent_stop:{reason or 'stop'}"
        with self._loco_dispatch_lock:
            self._loco_dispatch_epoch += 1
            self._loco_dispatch_latest = None
            self._loco_dispatch_lock.notify()
        self._fire_and_forget("loco_move", vx=0.0, vy=0.0, wz=0.0, source=source)
        self._fire_and_forget("loco_stop", source=source)

    def _run_loco_dispatcher(self):
        last_sent = 0.0
        while True:
            with self._loco_dispatch_lock:
                while self._loco_dispatch_latest is None and not self._loco_dispatch_stop:
                    self._loco_dispatch_lock.wait()
                if self._loco_dispatch_stop:
                    return

                wait_sec = last_sent + self._loco_min_interval_sec - time.monotonic()
                if wait_sec > 0.0:
                    self._loco_dispatch_lock.wait(timeout=wait_sec)
                    continue

                command = self._loco_dispatch_latest
                self._loco_dispatch_latest = None

            if command["epoch"] != self._loco_dispatch_epoch:
                continue

            try:
                self.request(
                    "loco_move",
                    vx=command["vx"],
                    vy=command["vy"],
                    wz=command["wz"],
                    source=command["source"],
                )
            except Exception as exc:
                try:
                    self._logger.error(f"[sdk-bridge] loco dispatcher Move 失败: {exc}")
                except Exception:
                    pass
            finally:
                last_sent = time.monotonic()

    def _record_loco_latency(self, command, total_ms, source=""):
        if command != "loco_move":
            return
        now = time.monotonic()
        source = str(source or "")
        latency_ms = float(total_ms)
        with self._loco_latency_lock:
            self._last_loco_latency_ms = latency_ms
            self._last_loco_latency_sample_time = now
            self._loco_latency_samples_ms.append(latency_ms)
            if len(self._loco_latency_samples_ms) > self.LOCO_LATENCY_WINDOW:
                self._loco_latency_samples_ms = self._loco_latency_samples_ms[
                    -self.LOCO_LATENCY_WINDOW :
                ]
            if source in self.LOCO_ACTIVE_PLANNER_SOURCES:
                self._last_active_loco_latency_sample_time = now
                self._loco_latency_active_samples_ms.append(latency_ms)
                if len(self._loco_latency_active_samples_ms) > self.LOCO_LATENCY_WINDOW:
                    self._loco_latency_active_samples_ms = (
                        self._loco_latency_active_samples_ms[-self.LOCO_LATENCY_WINDOW :]
                    )
            if latency_ms >= self.LOCO_SLOW_LOG_MS:
                self._loco_slow_count += 1

    def reset_loco_health(self):
        with self._loco_latency_lock:
            self._loco_latency_samples_ms = []
            self._loco_latency_active_samples_ms = []
            self._last_loco_latency_ms = 0.0
            self._last_loco_latency_sample_time = 0.0
            self._last_active_loco_latency_sample_time = 0.0
            self._loco_slow_count = 0

    @staticmethod
    def _latency_p95_ms(samples):
        if not samples:
            return 0.0
        ordered = sorted(samples)
        return ordered[int((len(ordered) - 1) * 0.95)]

    @staticmethod
    def _sample_age_sec(now, sample_time):
        if sample_time <= 0.0:
            return float("inf")
        return round(max(0.0, now - sample_time), 3)

    def get_loco_health_snapshot(self):
        now = time.monotonic()
        with self._loco_latency_lock:
            samples = list(self._loco_latency_samples_ms)
            active_samples = list(self._loco_latency_active_samples_ms)
            last_ms = self._last_loco_latency_ms
            last_sample_time = self._last_loco_latency_sample_time
            last_active_sample_time = self._last_active_loco_latency_sample_time
            slow_count = self._loco_slow_count
        p95_ms = self._latency_p95_ms(samples)
        active_p95_ms = self._latency_p95_ms(active_samples)
        return {
            "last_loco_latency_ms": round(last_ms, 1),
            "loco_latency_p95_ms": round(p95_ms, 1),
            "loco_slow_count": int(slow_count),
            "loco_latency_sample_count": len(samples),
            "last_loco_latency_age_sec": self._sample_age_sec(now, last_sample_time),
            "active_loco_latency_p95_ms": round(active_p95_ms, 1),
            "active_loco_latency_sample_count": len(active_samples),
            "active_loco_latency_age_sec": self._sample_age_sec(
                now, last_active_sample_time
            ),
        }

    def send_arm_q_async(self, joints, q_target, kp=60.0, kd=1.5, dq_target=0.0, tau_ff=0.0):
        """Fire-and-forget arm command (no response wait)."""
        self._fire_and_forget(
            "send_arm_q",
            joints=[int(j) for j in joints],
            q_target=[float(v) for v in q_target],
            kp=kp,
            kd=kd,
            dq_target=dq_target,
            tau_ff=tau_ff,
        )


    def _pump_stderr(self):
        if self._process.stderr is None:
            return
        for raw_line in self._process.stderr:
            line = raw_line.rstrip()
            if not line:
                continue
            try:
                self._logger.info(f"[sdk-worker] {line}")
            except Exception:
                pass

    def request(self, command, **payload):
        t_enter = time.monotonic()
        with self._response_lock:
            t_acquired = time.monotonic()
            if self._process.poll() is not None:
                raise RuntimeError(
                    f"Unitree SDK 子进程已退出，退出码 {self._process.returncode}"
                )

            request_id = self._next_request_id
            self._next_request_id += 1
            message = {"id": request_id, "command": command}
            message.update(payload)

            self._write_message(message)
            t_written = time.monotonic()

            assert self._process.stdout is not None
            while True:
                line = self._process.stdout.readline()
                if line == "":
                    raise RuntimeError(
                        "Unitree SDK 子进程在响应前退出。"
                        f" 退出码: {self._process.poll()}"
                    )

                stripped = line.strip()
                if not stripped:
                    continue

                try:
                    response = json.loads(stripped)
                except json.JSONDecodeError:
                    self._logger.warning(f"[sdk-worker/stdout] {stripped}")
                    continue

                if response.get("id") != request_id:
                    # Async fire-and-forget commands (id == -1) only surface
                    # failures here.  Promote those — and any unmatched failure
                    # response — to error so they aren't lost in the warning
                    # noise of legitimately reordered messages.
                    if response.get("id") == -1 or response.get("ok") is False:
                        self._logger.error(
                            f"[sdk-worker/async-error] {response.get('error')!r} "
                            f"id={response.get('id')} "
                            f"traceback={response.get('traceback')}"
                        )
                    else:
                        self._logger.warning(f"[sdk-worker/protocol] 忽略乱序消息: {response}")
                    continue

                if response.get("ok"):
                    t_done = time.monotonic()
                    lock_wait_ms = (t_acquired - t_enter) * 1000
                    write_ms = (t_written - t_acquired) * 1000
                    read_ms = (t_done - t_written) * 1000
                    total_ms = (t_done - t_enter) * 1000
                    self._record_loco_latency(
                        command, total_ms, source=payload.get("source", "")
                    )
                    if total_ms >= self.LOCO_SLOW_LOG_MS:
                        extra = ""
                        if command == "loco_move":
                            extra = (
                                f" vx={float(payload.get('vx', 0.0)):.3f}"
                                f" vy={float(payload.get('vy', 0.0)):.3f}"
                                f" wz={float(payload.get('wz', 0.0)):.3f}"
                                f" source={payload.get('source', '')}"
                            )
                        self._logger.warning(
                            f"[sdk-bridge/slow] cmd={command}{extra} "
                            f"lock_wait={lock_wait_ms:.1f}ms "
                            f"write={write_ms:.1f}ms "
                            f"read={read_ms:.1f}ms "
                            f"total={total_ms:.1f}ms"
                        )
                    return response.get("result")

                error = response.get("error", "unknown error")
                detail = response.get("traceback")
                if detail:
                    raise RuntimeError(f"{error}\n{detail}")
                raise RuntimeError(error)

    def close(self):
        process = getattr(self, "_process", None)
        if process is None:
            return

        with self._loco_dispatch_lock:
            self._loco_dispatch_stop = True
            self._loco_dispatch_lock.notify_all()
        thread = getattr(self, "_loco_dispatch_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)

        if process.poll() is None:
            try:
                self.request("shutdown")
            except Exception:
                try:
                    process.terminate()
                except Exception:
                    pass

        try:
            if process.stdin is not None:
                process.stdin.close()
        except Exception:
            pass
        try:
            if process.stdout is not None:
                process.stdout.close()
        except Exception:
            pass
        try:
            if process.stderr is not None:
                process.stderr.close()
        except Exception:
            pass


class _LocoClientProxy:
    def __init__(self, bridge):
        self._bridge = bridge

    def SetTimeout(self, timeout_sec):
        self._bridge.request("loco_set_timeout", timeout_sec=timeout_sec)

    def Init(self):
        self._bridge.request("loco_init")

    def SetSpeedMode(self, mode):
        return self._bridge.request("loco_set_speed_mode", mode=mode)

    def Move(self, vx, vy, wz, source=None):
        self._bridge.request(
            "loco_move",
            vx=vx,
            vy=vy,
            wz=wz,
            source=str(source or "direct"),
        )

    def QueueMove(self, vx, vy, wz, source=None):
        self._bridge.queue_loco_move(vx, vy, wz, source=str(source or "planner"))

    def EmergencyStop(self, reason=""):
        self._bridge.urgent_loco_stop(reason=reason)

    def StopMove(self):
        self._bridge.urgent_loco_stop(reason="stop_move")

    def GetHealthSnapshot(self):
        return self._bridge.get_loco_health_snapshot()

    def ResetHealthSnapshot(self):
        return self._bridge.reset_loco_health()

    def StandUp2Squat(self):
        self._bridge.request("loco_squat")

    def Squat2StandUp(self):
        self._bridge.request("loco_stand_up")

    def GetFsmId(self):
        """查询当前 FSM ID（如 1=Damp, 200=Start, 801=站立稳态）。"""
        return self._bridge.request("loco_get_fsm_id")

    def SetFsmId(self, fsm_id):
        """切换到指定 FSM 状态（透传 LocoClient.SetFsmId）。

        返回底层 SDK 的返回码（int），0 通常表示请求被接收。
        到位判定由调用方通过 GetFsmId 轮询完成。
        """
        return self._bridge.request("loco_set_fsm_id", fsm_id=int(fsm_id))

    def CheckZeroTorque(self, threshold=0.5):
        """检查所有关节力矩是否接近零。

        返回 dict: {"is_zero": bool, "max_abs_tau": float, "details": str}
        threshold: 力矩绝对值阈值 (Nm)，低于此值视为零力矩。
        """
        return self._bridge.request("check_zero_torque", threshold=threshold)

    def Damp(self):
        """切换到阻尼模式 (FSM 1)。"""
        self._bridge.request("loco_damp")

    def Start(self):
        """切换到正常站立/行走模式 (FSM 200)。"""
        self._bridge.request("loco_start")


class _AudioClientProxy:
    def __init__(self, bridge):
        self._bridge = bridge

    def Init(self):
        self._bridge.request("audio_init")

    def SetTimeout(self, timeout_sec):
        self._bridge.request("audio_set_timeout", timeout_sec=timeout_sec)

    def GetVolume(self):
        return self._bridge.request("audio_get_volume")

    def SetVolume(self, volume):
        self._bridge.request("audio_set_volume", volume=volume)

    def TtsMaker(self, text, voice_id):
        self._bridge.request("audio_tts", text=text, voice_id=voice_id)


class _ArmClientProxy:
    def __init__(self, bridge):
        self._bridge = bridge

    def Init(self):
        self._bridge.request("arm_init")

    def ExecuteAction(self, action_id):
        self._bridge.request("arm_execute_action", action_id=action_id)

    def ExecuteCustomAction(self, action_name):
        return self._bridge.request("arm_execute_custom_action", action_name=action_name)

    def StopCustomAction(self):
        return self._bridge.request("arm_stop_custom_action")

    def GetActionList(self):
        return self._bridge.request("arm_get_action_list")

    def PlayTrajectory(self, rows, params, hold_secs=0.0, release_time=1.0, dt=0.01):
        return self._bridge.request(
            "play_trajectory",
            rows=rows,
            params=params,
            hold_secs=hold_secs,
            release_time=release_time,
            dt=dt,
        )

    def StartArmSequence(self, joints, frames, dt=0.01, release=False, release_time=1.0):
        return self._bridge.request(
            "start_arm_sequence",
            joints=joints,
            frames=frames,
            dt=dt,
            release=release,
            release_time=release_time,
        )

    def CancelArmSequence(self, playback_id):
        return self._bridge.request(
            "cancel_arm_sequence",
            playback_id=playback_id,
        )

    def GetArmSequenceStatus(self, playback_id):
        return self._bridge.request(
            "get_arm_sequence_status",
            playback_id=playback_id,
        )


class _WorkerState:
    def __init__(self):
        self._channel_factory_initialize = None
        self._loco_cls = None
        self._audio_cls = None
        self._arm_cls = None
        self._loco = None
        self._audio = None
        self._arm = None
        self._channel_subscriber_cls = None
        self._lowstate_msg_cls = None
        self._lowstate_sub = None
        self._arm_pub = None
        self._crc = None
        self._latest_lowstate = None
        self._arm_sequence_lock = threading.Lock()
        self._arm_sequence_jobs = {}
        self._arm_sequence_jobs_lock = threading.Lock()
        self._next_arm_sequence_id = 1

    def _log(self, message):
        print(message, file=sys.stderr, flush=True)

    def _import_sdk(self):
        if self._channel_factory_initialize is not None:
            return

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        try:
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        except ImportError:
            AudioClient = None

        try:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        except ImportError:
            G1ArmActionClient = None

        self._channel_factory_initialize = ChannelFactoryInitialize
        self._loco_cls = LocoClient
        self._audio_cls = AudioClient
        self._arm_cls = G1ArmActionClient

    def handle(self, command, payload):
        if command == "init":
            return self._handle_init(payload)
        if command == "loco_set_timeout":
            self._require(self._loco, "运动客户端")
            self._loco.SetTimeout(float(payload["timeout_sec"]))
            return None
        if command == "loco_init":
            self._require(self._loco, "运动客户端")
            self._loco.Init()
            return None
        if command == "loco_set_speed_mode":
            self._require(self._loco, "运动客户端")
            return self._loco.SetSpeedMode(int(payload["mode"]))
        if command == "loco_move":
            self._require(self._loco, "运动客户端")
            self._loco.Move(
                float(payload["vx"]),
                float(payload["vy"]),
                float(payload["wz"]),
            )
            return None
        if command == "loco_stop":
            self._require(self._loco, "运动客户端")
            self._loco.StopMove()
            return None
        if command == "loco_squat":
            self._require(self._loco, "运动客户端")
            self._loco.StandUp2Squat()
            return None
        if command == "loco_stand_up":
            self._require(self._loco, "运动客户端")
            self._loco.Squat2StandUp()
            return None
        if command == "loco_get_fsm_id":
            self._require(self._loco, "运动客户端")
            import json as _json
            code, data = self._loco._Call(7001, "{}")
            if code != 0:
                raise RuntimeError(f"GetFsmId 失败, code={code}")
            parsed = _json.loads(data) if isinstance(data, str) else data
            return int(parsed.get("data", -1))
        if command == "loco_set_fsm_id":
            self._require(self._loco, "运动客户端")
            ret = self._loco.SetFsmId(int(payload["fsm_id"]))
            return int(ret) if ret is not None else 0
        if command == "check_zero_torque":
            threshold = float(payload.get("threshold", 0.5))
            return self._check_zero_torque(threshold)
        if command == "loco_damp":
            self._require(self._loco, "运动客户端")
            self._loco.Damp()
            return None
        if command == "loco_start":
            self._require(self._loco, "运动客户端")
            self._loco.Start()
            return None
        if command == "audio_init":
            self._require(self._audio, "音频客户端")
            self._audio.Init()
            return None
        if command == "audio_set_timeout":
            self._require(self._audio, "音频客户端")
            self._audio.SetTimeout(float(payload["timeout_sec"]))
            return None
        if command == "audio_get_volume":
            self._require(self._audio, "音频客户端")
            return self._audio.GetVolume()
        if command == "audio_set_volume":
            self._require(self._audio, "音频客户端")
            self._audio.SetVolume(int(payload["volume"]))
            return None
        if command == "audio_tts":
            self._require(self._audio, "音频客户端")
            self._audio.TtsMaker(str(payload["text"]), int(payload["voice_id"]))
            return None
        if command == "arm_init":
            self._require(self._arm, "动作客户端")
            self._arm.Init()
            return None
        if command == "arm_execute_action":
            self._require(self._arm, "动作客户端")
            self._arm.ExecuteAction(int(payload["action_id"]))
            return None
        if command == "arm_execute_custom_action":
            self._require(self._arm, "动作客户端")
            import json as _json
            p = {"action_name": str(payload["action_name"])}
            code, _data = self._arm._Call(7108, _json.dumps(p))
            return code
        if command == "arm_stop_custom_action":
            self._require(self._arm, "动作客户端")
            import json as _json
            code, _data = self._arm._Call(7113, _json.dumps({}))
            return code
        if command == "arm_get_action_list":
            self._require(self._arm, "动作客户端")
            code, data = self._arm.GetActionList()
            return {"code": code, "data": data}
        if command == "start_lowstate":
            self._start_lowstate()
            return None
        if command == "stop_lowstate":
            self._stop_lowstate()
            return None
        if command == "get_joint_q":
            joints = [int(j) for j in payload["joints"]]
            return self._get_joint_q(joints)
        if command == "get_joint_dq":
            joints = [int(j) for j in payload["joints"]]
            if self._latest_lowstate is None:
                raise RuntimeError("lowstate 不可用")
            return [float(self._latest_lowstate.motor_state[j].dq) for j in joints]
        if command == "get_joint_tau_est":
            joints = [int(j) for j in payload["joints"]]
            if self._latest_lowstate is None:
                raise RuntimeError("lowstate 不可用")
            return [float(self._latest_lowstate.motor_state[j].tau_est) for j in joints]
        if command == "send_arm_q":
            joints = [int(j) for j in payload["joints"]]
            q_target = [float(v) for v in payload["q_target"]]
            kp = payload.get("kp", 60.0)
            kd = payload.get("kd", 1.5)
            dq_target = payload.get("dq_target", 0.0)
            tau_ff = payload.get("tau_ff", 0.0)
            self._send_arm_q_extended(joints, q_target, kp=kp, kd=kd, dq_target=dq_target, tau_ff=tau_ff)
            return None
        if command == "release_arm":
            joints = [int(j) for j in payload["joints"]]
            hold_q = [float(v) for v in payload["hold_q"]]
            release_time = float(payload.get("release_time", 1.0))
            dt = float(payload.get("dt", 0.02))
            kp = payload.get("kp", 60.0)
            kd = payload.get("kd", 1.5)
            self._release_arm(joints, hold_q, release_time, dt, kp, kd)
            return None
        if command == "play_trajectory":
            return self._handle_play_trajectory(payload)
        if command == "start_arm_sequence":
            return self._handle_start_arm_sequence(payload)
        if command == "cancel_arm_sequence":
            return self._handle_cancel_arm_sequence(payload)
        if command == "get_arm_sequence_status":
            return self._handle_get_arm_sequence_status(payload)
        if command == "shutdown":
            self.shutdown()
            return None
        raise RuntimeError(f"未知命令: {command}")

    def _handle_init(self, payload):
        self._import_sdk()

        domain_id = int(payload["domain_id"])
        network_interface = payload["network_interface"]
        enable_audio = bool(payload.get("enable_audio", True))
        enable_arm = bool(payload.get("enable_arm", True))

        self._log(
            f"初始化 ChannelFactory: domain_id={domain_id}, network_interface={network_interface}"
        )
        self._channel_factory_initialize(domain_id, network_interface)
        self._log("ChannelFactory 初始化完成")

        self._log("创建运动客户端...")
        self._loco = self._loco_cls()
        self._loco.SetTimeout(0.2)
        self._log("运动客户端 Init()...")
        self._loco.Init()
        self._log("运动客户端初始化完成 (Loco SetTimeout=0.2s)")

        if enable_audio and self._audio_cls is not None:
            self._log("创建音频客户端...")
            self._audio = self._audio_cls()
            self._audio.Init()
            self._audio.SetTimeout(10.0)
            self._log("音频客户端初始化完成")
        else:
            self._audio = None
            self._log("音频客户端已禁用或不可用")

        if enable_arm and self._arm_cls is not None:
            self._log("创建动作客户端...")
            self._arm = self._arm_cls()
            self._arm.Init()
            # 注册 Python SDK 未暴露的 C++ API
            try:
                self._arm._RegistApi(7108, 0)  # ExecuteCustomAction
                self._arm._RegistApi(7113, 0)  # StopCustomAction
                self._log("已注册扩展 API 7108/7113")
            except Exception as exc:
                self._log(f"注册扩展 API 失败（非致命）: {exc}")
            self._log("动作客户端初始化完成")
        else:
            self._arm = None
            self._log("动作客户端已禁用或不可用")

        self._log("初始化轨迹播放基础设施（lowstate 订阅延迟到首次 play_trajectory）...")
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
            from unitree_sdk2py.utils.crc import CRC

            self._channel_subscriber_cls = ChannelSubscriber
            self._lowstate_msg_cls = LowState_
            self._arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._arm_pub.Init()
            self._crc = CRC()
            self._log("轨迹播放发布器/CRC 初始化完成")
        except Exception as exc:
            self._log(f"轨迹播放基础设施初始化失败（非致命）: {exc}")

        return {
            "audio": self._audio is not None,
            "arm": self._arm is not None,
        }

    def _require(self, value, label):
        if value is None:
            raise RuntimeError(f"{label}尚未初始化")

    def _on_lowstate(self, msg):
        self._latest_lowstate = msg

    def _start_lowstate(self, wait_timeout=5.0):
        if self._lowstate_sub is not None:
            return
        if self._channel_subscriber_cls is None or self._lowstate_msg_cls is None:
            raise RuntimeError("lowstate 订阅类未初始化")
        self._latest_lowstate = None
        sub = self._channel_subscriber_cls("rt/lowstate", self._lowstate_msg_cls)
        sub.Init(self._on_lowstate, 10)
        self._lowstate_sub = sub
        self._wait_lowstate(timeout=wait_timeout)

    def _stop_lowstate(self):
        sub = self._lowstate_sub
        self._lowstate_sub = None
        self._latest_lowstate = None
        if sub is None:
            return
        try:
            sub.Close()
        except Exception as exc:
            self._log(f"关闭 lowstate 订阅失败（忽略）: {exc}")

    # G1 机器人共 29 个有效关节 (0~28)，关节 29 用于特殊标志位
    _G1_ALL_JOINTS = list(range(29))

    def _check_zero_torque(self, threshold=0.5):
        """检查所有关节的力矩估计值是否接近零。

        临时启动 lowstate 订阅，读取一次 tau_est，然后停止订阅。
        返回 {"is_zero": bool, "max_abs_tau": float, "details": str}。
        """
        owned_sub = self._lowstate_sub is None
        try:
            if owned_sub:
                self._start_lowstate(wait_timeout=5.0)
            else:
                # 确保数据已更新
                self._wait_lowstate(timeout=2.0)

            ls = self._latest_lowstate
            if ls is None:
                raise RuntimeError("lowstate 不可用，无法检查力矩")

            taus = [abs(float(ls.motor_state[j].tau_est)) for j in self._G1_ALL_JOINTS]
            max_abs_tau = max(taus)
            is_zero = max_abs_tau < threshold

            # 找出超过阈值的关节
            over = [
                (j, taus[j]) for j in range(len(taus)) if taus[j] >= threshold
            ]
            if over:
                details = ", ".join(f"关节{j}={tau:.2f}Nm" for j, tau in over)
            else:
                details = f"所有关节力矩 < {threshold}Nm"

            self._log(
                f"[check_zero_torque] max_abs_tau={max_abs_tau:.3f}, "
                f"threshold={threshold}, is_zero={is_zero}"
            )
            return {
                "is_zero": is_zero,
                "max_abs_tau": round(max_abs_tau, 4),
                "details": details,
            }
        finally:
            if owned_sub:
                self._stop_lowstate()

    def _wait_lowstate(self, timeout=5.0):
        deadline = time.time() + timeout
        while self._latest_lowstate is None:
            if time.time() >= deadline:
                raise TimeoutError("等待 lowstate 超时")
            time.sleep(0.05)
        return self._latest_lowstate

    def _get_joint_q(self, joints):
        if self._latest_lowstate is None:
            raise RuntimeError("lowstate 不可用")
        return [float(self._latest_lowstate.motor_state[j].q) for j in joints]

    def _make_low_cmd(self):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        return unitree_hg_msg_dds__LowCmd_()

    @staticmethod
    def _expand_param(x, n):
        if isinstance(x, (int, float)):
            return [float(x)] * n
        if len(x) != n:
            raise ValueError(f"参数长度 {len(x)} != 关节数 {n}")
        return [float(v) for v in x]

    def _send_arm_q(self, joints, q_target, kp=60.0, kd=1.5):
        cmd = self._make_low_cmd()
        kp_list = self._expand_param(kp, len(joints))
        kd_list = self._expand_param(kd, len(joints))
        cmd.motor_cmd[29].q = 1.0  # kNotUsedJoint = 29
        for i, joint in enumerate(joints):
            cmd.motor_cmd[joint].tau = 0.0
            cmd.motor_cmd[joint].q = float(q_target[i])
            cmd.motor_cmd[joint].dq = 0.0
            cmd.motor_cmd[joint].kp = kp_list[i]
            cmd.motor_cmd[joint].kd = kd_list[i]
        cmd.crc = self._crc.Crc(cmd)
        self._arm_pub.Write(cmd)

    def _send_arm_q_extended(self, joints, q_target, kp=60.0, kd=1.5, dq_target=0.0, tau_ff=0.0):
        """Extended version supporting dq_target and tau_ff for teach playback."""
        cmd = self._make_low_cmd()
        kp_list = self._expand_param(kp, len(joints))
        kd_list = self._expand_param(kd, len(joints))
        dq_list = self._expand_param(dq_target, len(joints))
        tau_list = self._expand_param(tau_ff, len(joints))
        cmd.motor_cmd[29].q = 1.0  # kNotUsedJoint = 29
        for i, joint in enumerate(joints):
            cmd.motor_cmd[joint].tau = tau_list[i]
            cmd.motor_cmd[joint].q = float(q_target[i])
            cmd.motor_cmd[joint].dq = dq_list[i]
            cmd.motor_cmd[joint].kp = kp_list[i]
            cmd.motor_cmd[joint].kd = kd_list[i]
        cmd.crc = self._crc.Crc(cmd)
        self._arm_pub.Write(cmd)

    def _release_arm(self, joints, hold_q, release_time, dt, kp, kd):
        steps = max(1, int(release_time / dt))
        kp_list = self._expand_param(kp, len(joints))
        kd_list = self._expand_param(kd, len(joints))
        for k in range(steps):
            ratio = 1.0 - (k + 1) / steps
            cmd = self._make_low_cmd()
            cmd.motor_cmd[29].q = ratio  # kNotUsedJoint = 29
            for i, joint in enumerate(joints):
                cmd.motor_cmd[joint].tau = 0.0
                cmd.motor_cmd[joint].q = float(hold_q[i])
                cmd.motor_cmd[joint].dq = 0.0
                cmd.motor_cmd[joint].kp = kp_list[i]
                cmd.motor_cmd[joint].kd = kd_list[i]
            cmd.crc = self._crc.Crc(cmd)
            self._arm_pub.Write(cmd)
            time.sleep(dt)

    def _normalize_arm_sequence_frames(self, joints, frames):
        if not frames:
            raise ValueError("arm sequence frames 不能为空")

        n = len(joints)
        normalized = []
        for index, raw in enumerate(frames):
            if not isinstance(raw, dict):
                raise ValueError(f"arm sequence frame {index} 必须是对象")

            q_target = [float(v) for v in raw.get("q", [])]
            if len(q_target) != n:
                raise ValueError(
                    f"arm sequence frame {index} q 长度 {len(q_target)} != 关节数 {n}"
                )

            normalized.append({
                "q": q_target,
                "kp": self._expand_param(raw.get("kp", 60.0), n),
                "kd": self._expand_param(raw.get("kd", 1.5), n),
                "dq_target": self._expand_param(
                    raw.get("dq_target", raw.get("dq", 0.0)), n
                ),
                "tau_ff": self._expand_param(raw.get("tau_ff", 0.0), n),
            })

        return normalized

    def _prune_arm_sequence_jobs_locked(self, keep=16):
        finished = [
            (job.get("finished_at") or 0.0, job_id)
            for job_id, job in self._arm_sequence_jobs.items()
            if job.get("state") in {"succeeded", "canceled", "error"}
        ]
        if len(finished) <= keep:
            return
        for _finished_at, job_id in sorted(finished)[: len(finished) - keep]:
            self._arm_sequence_jobs.pop(job_id, None)

    def _copy_arm_sequence_status(self, job):
        started_at = job.get("started_at")
        finished_at = job.get("finished_at")
        now = time.time()
        if started_at is None:
            elapsed = 0.0
        else:
            elapsed = (finished_at or now) - started_at
        status = {
            "playback_id": job["id"],
            "state": job["state"],
            "frame_count": job["frame_count"],
            "completed_frames": job.get("completed_frames", 0),
            "elapsed": elapsed,
        }
        if job.get("error"):
            status["error"] = job["error"]
            status["traceback"] = job.get("traceback")
        if job.get("result") is not None:
            status["result"] = job["result"]
        return status

    def _running_arm_sequence_locked(self):
        for job in self._arm_sequence_jobs.values():
            if job.get("state") in {"queued", "running"}:
                return job
        return None

    def _handle_start_arm_sequence(self, payload):
        if self._arm_pub is None or self._crc is None:
            raise RuntimeError("arm sequence 基础设施未初始化")

        joints = [int(j) for j in payload["joints"]]
        if not joints:
            raise ValueError("arm sequence joints 不能为空")

        frames = self._normalize_arm_sequence_frames(joints, payload["frames"])
        dt = float(payload.get("dt", 0.01))
        if dt <= 0.0:
            raise ValueError("arm sequence dt must be > 0")
        release = bool(payload.get("release", False))
        release_time = float(payload.get("release_time", 1.0))
        if release_time < 0.0:
            raise ValueError("arm sequence release_time must be >= 0")

        with self._arm_sequence_jobs_lock:
            running = self._running_arm_sequence_locked()
            if running is not None:
                raise RuntimeError(
                    f"已有 arm sequence 正在运行: id={running['id']}"
                )

            playback_id = self._next_arm_sequence_id
            self._next_arm_sequence_id += 1
            job = {
                "id": playback_id,
                "state": "queued",
                "frame_count": len(frames),
                "completed_frames": 0,
                "cancel_event": threading.Event(),
                "started_at": None,
                "finished_at": None,
                "result": None,
                "error": None,
                "traceback": None,
            }
            self._arm_sequence_jobs[playback_id] = job
            self._prune_arm_sequence_jobs_locked()

        thread = threading.Thread(
            target=self._run_arm_sequence_job,
            args=(job, joints, frames, dt, release, release_time),
            name=f"arm-sequence-{playback_id}",
            daemon=True,
        )
        thread.start()
        self._log(
            f"[arm_sequence] started id={playback_id} "
            f"frames={len(frames)} dt={dt:.3f}s release={release}"
        )
        return {"playback_id": playback_id, "frame_count": len(frames)}

    def _handle_cancel_arm_sequence(self, payload):
        playback_id = int(payload["playback_id"])
        with self._arm_sequence_jobs_lock:
            job = self._arm_sequence_jobs.get(playback_id)
            if job is None:
                raise RuntimeError(f"arm sequence 不存在: id={playback_id}")
            job["cancel_event"].set()
            return self._copy_arm_sequence_status(job)

    def _handle_get_arm_sequence_status(self, payload):
        playback_id = int(payload["playback_id"])
        with self._arm_sequence_jobs_lock:
            job = self._arm_sequence_jobs.get(playback_id)
            if job is None:
                raise RuntimeError(f"arm sequence 不存在: id={playback_id}")
            return self._copy_arm_sequence_status(job)

    def _run_arm_sequence_job(self, job, joints, frames, dt, release, release_time):
        playback_id = job["id"]
        cancel_event = job["cancel_event"]
        q_last = list(frames[0]["q"])
        kp_last = list(frames[0]["kp"])
        kd_last = list(frames[0]["kd"])
        canceled = False
        started_monotonic = time.monotonic()

        with self._arm_sequence_jobs_lock:
            job["state"] = "running"
            job["started_at"] = time.time()

        try:
            with self._arm_sequence_lock:
                next_tick = time.monotonic()
                for index, frame in enumerate(frames):
                    if cancel_event.is_set():
                        canceled = True
                        break

                    q_last = list(frame["q"])
                    kp_last = list(frame["kp"])
                    kd_last = list(frame["kd"])
                    self._send_arm_q_extended(
                        joints,
                        q_last,
                        kp=kp_last,
                        kd=kd_last,
                        dq_target=frame["dq_target"],
                        tau_ff=frame["tau_ff"],
                    )

                    with self._arm_sequence_jobs_lock:
                        job["completed_frames"] = index + 1

                    next_tick += dt
                    wait_time = next_tick - time.monotonic()
                    if wait_time > 0.0 and cancel_event.wait(wait_time):
                        canceled = True
                        break

                if release and q_last is not None:
                    self._release_arm(
                        joints,
                        q_last,
                        release_time,
                        dt,
                        kp_last,
                        kd_last,
                    )

            elapsed = time.monotonic() - started_monotonic
            result = {
                "joints": list(joints),
                "q_final": list(q_last),
                "kp": list(kp_last),
                "kd": list(kd_last),
                "control_dt": dt,
                "frame_count": len(frames),
                "completed_frames": job.get("completed_frames", 0),
                "elapsed": elapsed,
                "canceled": canceled,
            }
            with self._arm_sequence_jobs_lock:
                job["state"] = "canceled" if canceled else "succeeded"
                job["finished_at"] = time.time()
                job["result"] = result
            self._log(
                f"[arm_sequence] finished id={playback_id} "
                f"state={'canceled' if canceled else 'succeeded'} "
                f"frames={result['completed_frames']}/{len(frames)} "
                f"elapsed={elapsed:.2f}s"
            )
        except Exception as exc:
            with self._arm_sequence_jobs_lock:
                job["state"] = "error"
                job["finished_at"] = time.time()
                job["error"] = str(exc)
                job["traceback"] = traceback.format_exc()
                job["result"] = {
                    "joints": list(joints),
                    "q_final": list(q_last),
                    "kp": list(kp_last),
                    "kd": list(kd_last),
                    "control_dt": dt,
                    "frame_count": len(frames),
                    "completed_frames": job.get("completed_frames", 0),
                    "elapsed": time.monotonic() - started_monotonic,
                    "canceled": canceled,
                }
            self._log(f"[arm_sequence] error id={playback_id}: {exc}")

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def _play_rows(self, joints, rows, q_now, params, hold_secs, dt):
        kp = params["kp"]
        kd = params["kd"]
        speed_scale = params["speed_scale"]
        blend_time = params["blend_time"]

        ts = [float(row["t"]) for row in rows]
        qs = [[float(v) for v in row["q"]] for row in rows]

        q_start = qs[0]
        q_last = list(q_now)
        blend_steps = max(1, int(blend_time / dt))

        self._log("blend 到轨迹起始帧")
        for step in range(blend_steps):
            alpha = (step + 1) / blend_steps
            q = [
                (1.0 - alpha) * q_now[i] + alpha * q_start[i]
                for i in range(len(joints))
            ]
            q_last = list(q)
            self._send_arm_q(joints, q, kp=kp, kd=kd)
            time.sleep(dt)

        self._log("轨迹播放开始")
        start = time.time()
        idx = 0
        final_t = ts[-1]

        while True:
            now = (time.time() - start) * speed_scale
            if now >= final_t:
                q = qs[-1]
                q_last = list(q)
                self._send_arm_q(joints, q, kp=kp, kd=kd)
                break

            while idx + 1 < len(ts) and ts[idx + 1] <= now:
                idx += 1

            if idx >= len(ts) - 1:
                q = qs[-1]
            else:
                t0 = ts[idx]
                t1 = ts[idx + 1]
                q0 = qs[idx]
                q1 = qs[idx + 1]
                if t1 <= t0:
                    alpha = 0.0
                else:
                    alpha = self._clamp((now - t0) / (t1 - t0), 0.0, 1.0)
                q = [
                    (1.0 - alpha) * q0[i] + alpha * q1[i]
                    for i in range(len(joints))
                ]

            q_last = list(q)
            self._send_arm_q(joints, q, kp=kp, kd=kd)
            time.sleep(dt)

        if hold_secs > 0:
            hold_steps = max(1, int(hold_secs / dt))
            for _ in range(hold_steps):
                self._send_arm_q(joints, q_last, kp=kp, kd=kd)
                time.sleep(dt)

        return q_last

    def _handle_play_trajectory(self, payload):
        if self._arm_pub is None or self._crc is None:
            raise RuntimeError("轨迹播放基础设施未初始化")

        rows = payload["rows"]
        params = payload["params"]
        hold_secs = float(payload.get("hold_secs", 0.0))
        release_time = float(payload.get("release_time", 1.0))
        dt = float(payload.get("dt", 0.01))

        if not rows:
            raise ValueError("轨迹为空")

        joints = rows[0]["joints"]
        self._log(
            f"开始播放轨迹: joints={joints}, frames={len(rows)}, "
            f"duration={rows[-1]['t']:.2f}s"
        )

        try:
            self._start_lowstate(wait_timeout=5.0)
            q_now = self._get_joint_q(joints)

            playback_error = None
            q_last = q_now
            started_at = time.time()
            try:
                q_last = self._play_rows(joints, rows, q_now, params, hold_secs, dt)
            except Exception as exc:
                playback_error = exc
                raise
            finally:
                try:
                    self._release_arm(
                        joints, q_last, release_time, dt,
                        params["kp"], params["kd"],
                    )
                except Exception as exc:
                    self._log(f"释放 arm_sdk 失败: {exc}")
                    if playback_error is None:
                        raise
        finally:
            self._stop_lowstate()

        elapsed = time.time() - started_at
        self._log(f"轨迹播放完成: elapsed={elapsed:.2f}s")
        return {
            "frame_count": len(rows),
            "duration": rows[-1]["t"],
            "elapsed": elapsed,
        }

    def shutdown(self):
        if self._loco is not None:
            try:
                self._loco.Move(0.0, 0.0, 0.0)
            except Exception:
                pass
            try:
                self._loco.StopMove()
            except Exception:
                pass
        self._stop_lowstate()


def _run_worker():
    state = _WorkerState()

    for raw_line in sys.stdin:
        line = raw_line.strip()
        if not line:
            continue

        response = {"id": None, "ok": False}
        try:
            request = json.loads(line)
            if request.get("id") == -1:
                state.handle(request["command"], request)
                continue
            response["id"] = request.get("id")
            result = state.handle(request["command"], request)
            response["ok"] = True
            response["result"] = result
        except Exception as exc:
            response["error"] = str(exc)
            response["traceback"] = traceback.format_exc()

        print(json.dumps(response, ensure_ascii=True), flush=True)

    state.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Unitree SDK worker bridge")
    parser.add_argument("--worker", action="store_true")
    args = parser.parse_args()
    if args.worker:
        _run_worker()


if __name__ == "__main__":
    main()
