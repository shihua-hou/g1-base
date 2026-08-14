"""Joint index definitions used by g1_teach_v2.

这里集中维护上肢相关关节编号和常用分组，避免录制、回放、snapshot
这些模块各自写一套硬编码索引。
"""


class G1JointIndex:
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    # arm_sdk 会占用这个保留槽位来声明控制权。
    kNotUsedJoint = 29


LEFT_ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
]

RIGHT_ARM_JOINTS = [
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]

BOTH_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
WAIST_YAW_JOINTS = [G1JointIndex.WaistYaw]
WAIST_EXTRA_JOINTS = [
    G1JointIndex.WaistRoll,
    G1JointIndex.WaistPitch,
]
WAIST_SUPPORT_JOINTS = list(WAIST_EXTRA_JOINTS)
UPPER_JOINTS = WAIST_YAW_JOINTS + BOTH_ARM_JOINTS

ARM_GROUPS = {
    "left": LEFT_ARM_JOINTS,
    "right": RIGHT_ARM_JOINTS,
    "both": BOTH_ARM_JOINTS,
    "upper": UPPER_JOINTS,
}


def get_group_joints(group):
    """Return a copy of the configured joint list for one logical arm group."""
    if group not in ARM_GROUPS:
        raise ValueError(f"unknown group: {group}, choose from {list(ARM_GROUPS.keys())}")
    return list(ARM_GROUPS[group])


def with_waist_support(joints):
    """Prefix roll/pitch waist support joints while keeping the target list unique."""
    merged = list(WAIST_SUPPORT_JOINTS)
    for joint in joints:
        if joint not in merged:
            merged.append(joint)
    return merged
