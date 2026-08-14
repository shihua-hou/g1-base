"""Thin robot I/O wrapper around Unitree DDS channels."""

import time

from .iface_utils import AUTO_IFACE, resolve_network_interface
from .joints import G1JointIndex


class RobotSession:
    def __init__(self, iface=AUTO_IFACE, domain=0, enable_pub=True):
        # Import SDK lazily so help text and file-only operations stay lightweight.
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self._low_state = None
        self._cmd_factory = unitree_hg_msg_dds__LowCmd_
        self.iface = resolve_network_interface(iface, verbose=True)

        # Each session initializes DDS independently so CLI subcommands can start and stop cleanly.
        ChannelFactoryInitialize(domain, self.iface)

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._lowstate_handler, 10)

        self.pub = None
        self.crc = None
        if enable_pub:
            self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self.pub.Init()
            self.crc = CRC()

    def _lowstate_handler(self, msg):
        self._low_state = msg

    @property
    def low_state(self):
        return self._low_state

    def wait_lowstate(self, poll_dt=0.05):
        while self._low_state is None:
            time.sleep(poll_dt)
        return self._low_state

    def get_joint_q(self, joints):
        self.wait_lowstate()
        return [float(self._low_state.motor_state[j].q) for j in joints]

    def get_joint_dq(self, joints):
        self.wait_lowstate()
        return [float(self._low_state.motor_state[j].dq) for j in joints]

    def get_joint_tau_est(self, joints):
        self.wait_lowstate()
        return [float(self._low_state.motor_state[j].tau_est) for j in joints]

    def _expand_param(self, x, n):
        if isinstance(x, (int, float)):
            return [float(x)] * n
        if len(x) != n:
            raise ValueError(f"parameter length {len(x)} != joint count {n}")
        return [float(v) for v in x]

    def build_arm_cmd(self, joints, q_target, kp=60.0, kd=1.5, dq_target=0.0, tau_ff=0.0):
        cmd = self._cmd_factory()
        # Unitree arm_sdk uses this reserved slot to claim control ownership.
        cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        kp_list = self._expand_param(kp, len(joints))
        kd_list = self._expand_param(kd, len(joints))
        dq_list = self._expand_param(dq_target, len(joints))
        tau_list = self._expand_param(tau_ff, len(joints))

        for idx, joint in enumerate(joints):
            cmd.motor_cmd[joint].tau = tau_list[idx]
            cmd.motor_cmd[joint].q = float(q_target[idx])
            cmd.motor_cmd[joint].dq = dq_list[idx]
            cmd.motor_cmd[joint].kp = kp_list[idx]
            cmd.motor_cmd[joint].kd = kd_list[idx]
        return cmd

    def send_arm_q(self, joints, q_target, kp=60.0, kd=1.5, dq_target=0.0, tau_ff=0.0):
        if self.pub is None or self.crc is None:
            raise RuntimeError("arm publisher is not enabled for this session")
        cmd = self.build_arm_cmd(joints, q_target, kp=kp, kd=kd, dq_target=dq_target, tau_ff=tau_ff)
        cmd.crc = self.crc.Crc(cmd)
        self.pub.Write(cmd)

    def release_arm_sdk(self, joints, hold_q, release_time=1.0, dt=0.02, kp=60.0, kd=1.5):
        if self.pub is None or self.crc is None:
            return

        steps = max(1, int(release_time / dt))
        kp_list = self._expand_param(kp, len(joints))
        kd_list = self._expand_param(kd, len(joints))

        for step in range(steps):
            # Fade the reserved ownership slot from 1 to 0 to avoid a hard handoff at release time.
            ratio = 1.0 - (step + 1) / steps
            cmd = self._cmd_factory()
            cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = ratio
            for idx, joint in enumerate(joints):
                cmd.motor_cmd[joint].tau = 0.0
                cmd.motor_cmd[joint].q = float(hold_q[idx])
                cmd.motor_cmd[joint].dq = 0.0
                cmd.motor_cmd[joint].kp = kp_list[idx]
                cmd.motor_cmd[joint].kd = kd_list[idx]
            cmd.crc = self.crc.Crc(cmd)
            self.pub.Write(cmd)
            time.sleep(dt)
