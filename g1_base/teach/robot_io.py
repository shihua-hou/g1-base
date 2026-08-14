"""Robot I/O adapter routed through the Unitree SDK bridge.

All Unitree SDK/DDS access stays in the bridge subprocess so g1_base_manager
only initializes the Unitree SDK once.
"""

import time


class RobotSession:
    """Thin adapter presenting the same interface as g1_teach_v2's RobotSession.

    Arm commands and lowstate reads go through the bridge subprocess."""

    def __init__(self, sdk_bridge, use_direct_arm=False, iface=None, domain=0,
                 direct_arm_pub=None, direct_arm_crc=None, direct_arm_cmd_factory=None):
        self._bridge = sdk_bridge
        self._lowstate_started = False
        if use_direct_arm or any(
            resource is not None
            for resource in (direct_arm_pub, direct_arm_crc, direct_arm_cmd_factory)
        ):
            raise RuntimeError(
                "direct arm publishing is disabled in g1_base_manager; "
                "use UnitreeSdkBridge so the Unitree SDK is initialized once"
            )

    # -- lowstate lifecycle ---------------------------------------------------

    def _ensure_lowstate(self):
        if not self._lowstate_started:
            self._bridge.request("start_lowstate")
            self._lowstate_started = True

    def wait_lowstate(self, poll_dt=0.05):
        self._ensure_lowstate()
        # The bridge blocks until the first lowstate arrives.
        return True

    def stop_lowstate(self):
        if self._lowstate_started:
            try:
                self._bridge.request("stop_lowstate")
            except Exception:
                pass
            self._lowstate_started = False

    # -- joint reads ----------------------------------------------------------

    def get_joint_q(self, joints):
        self._ensure_lowstate()
        return self._bridge.request(
            "get_joint_q", joints=[int(j) for j in joints]
        )

    def get_joint_dq(self, joints):
        self._ensure_lowstate()
        return self._bridge.request(
            "get_joint_dq", joints=[int(j) for j in joints]
        )

    def get_joint_tau_est(self, joints):
        self._ensure_lowstate()
        return self._bridge.request(
            "get_joint_tau_est", joints=[int(j) for j in joints]
        )

    # -- arm command ----------------------------------------------------------

    @staticmethod
    def _expand_param(x, n):
        """Expand a scalar gain value to a per-joint list of length n."""
        if isinstance(x, (int, float)):
            return [float(x)] * n
        if len(x) != n:
            raise ValueError(f"parameter length {len(x)} != joint count {n}")
        return [float(v) for v in x]

    def send_arm_q(self, joints, q_target, kp=60.0, kd=1.5, dq_target=0.0, tau_ff=0.0):
        n = len(joints)
        if len(q_target) != n:
            raise ValueError(
                f"send_arm_q: q_target length {len(q_target)} != joints length {n}"
            )
        # Normalise gain/feed-forward inputs in the parent process so the async
        # worker can never see a length mismatch silently.
        kp = self._expand_param(kp, n)
        kd = self._expand_param(kd, n)
        dq_target = self._expand_param(dq_target, n)
        tau_ff = self._expand_param(tau_ff, n)
        fast = getattr(self._bridge, "send_arm_q_async", None)
        if fast is not None:
            fast(joints, q_target, kp=kp, kd=kd, dq_target=dq_target, tau_ff=tau_ff)
            return
        self._bridge.request(
            "send_arm_q",
            joints=[int(j) for j in joints],
            q_target=[float(v) for v in q_target],
            kp=kp,
            kd=kd,
            dq_target=dq_target,
            tau_ff=tau_ff,
        )

    def play_arm_sequence(
        self,
        joints,
        frames,
        dt=0.01,
        release=False,
        release_time=1.0,
        cancel_event=None,
        poll_dt=0.02,
    ):
        """Run a precomputed arm command sequence inside the SDK worker.

        This keeps the high-frequency publish loop in the worker process and
        leaves this process responsible only for orchestration and cancellation.
        """
        if not frames:
            return {
                "joints": [int(j) for j in joints],
                "q_final": [],
                "kp": [],
                "kd": [],
                "control_dt": float(dt),
                "canceled": bool(cancel_event and cancel_event.is_set()),
            }

        start = self._bridge.request(
            "start_arm_sequence",
            joints=[int(j) for j in joints],
            frames=frames,
            dt=float(dt),
            release=bool(release),
            release_time=float(release_time),
        )
        playback_id = int(start["playback_id"])
        cancel_sent = False
        poll_dt = max(0.005, min(0.1, float(poll_dt)))

        while True:
            if cancel_event is not None and cancel_event.is_set() and not cancel_sent:
                self._bridge.request(
                    "cancel_arm_sequence",
                    playback_id=playback_id,
                )
                cancel_sent = True

            status = self._bridge.request(
                "get_arm_sequence_status",
                playback_id=playback_id,
            )
            state = str(status.get("state", ""))
            if state in {"succeeded", "canceled"}:
                result = status.get("result") or {}
                if not result:
                    raise RuntimeError(f"arm sequence {playback_id} finished without result")
                return result
            if state == "error":
                detail = status.get("traceback") or status.get("error") or "unknown error"
                raise RuntimeError(f"arm sequence {playback_id} failed: {detail}")

            time.sleep(poll_dt)

    def release_arm_sdk(self, joints, hold_q, release_time=1.0, dt=0.02, kp=60.0, kd=1.5):
        self._bridge.request(
            "release_arm",
            joints=[int(j) for j in joints],
            hold_q=[float(v) for v in hold_q],
            release_time=release_time,
            dt=dt,
            kp=kp,
            kd=kd,
        )
