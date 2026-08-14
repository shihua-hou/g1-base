"""宇树 G1 内置手臂动作服务适配器。

该模块与 g1_teach_v2 的自定义动作体系（snapshot/motion/script）互斥：
  - 内置动作通过固件 RPC 服务执行，依赖内置运控
  - 自定义动作通过 rt/arm_sdk 话题关节级控制执行
两者不能同时工作。本模块仅提供独立的 CLI 调用能力。

Python SDK (unitree_sdk2py) 的 G1ArmActionClient 只实现了 ExecuteAction(int)
和 GetActionList() 两个接口。参照 C++ SDK (unitree_sdk2) 中完整的头文件定义，
本模块在 g1_teach_v2 内部扩展出 ExecuteCustomAction 和 StopCustomAction，
不修改 unitree_sdk2_python 本身。
"""

import json
import time

from .iface_utils import resolve_network_interface


# ---------------------------------------------------------------------------
# C++ SDK 中定义但 Python SDK 未暴露的 API ID
# 来源: unitree_sdk2/include/unitree/robot/g1/arm/g1_arm_action_api.hpp
# ---------------------------------------------------------------------------
_API_ID_EXECUTE_CUSTOM_ACTION = 7108
_API_ID_STOP_CUSTOM_ACTION = 7113

# ---------------------------------------------------------------------------
# 内置动作 ID 映射表（与 C++ SDK action_map 及官方文档对齐）
# ---------------------------------------------------------------------------
BUILTIN_ACTIONS = {
    99: "release_arm",       # 恢复初始手臂位姿
    11: "two_hand_kiss",     # 双手飞吻
    12: "single_kiss",       # 单手飞吻
    13: "right_kiss",        # 右手飞吻
    15: "hands_up",          # 平举
    17: "clap",              # 鼓掌
    18: "high_five",         # 击掌
    19: "hug",               # 拥抱
    20: "double_heart",      # 双手比心
    21: "right_heart",       # 右手比心
    22: "reject",            # 双手打X
    23: "right_hand_up",     # 右手平举
    24: "x_ray",             # 动感光波
    25: "chest_wave",        # 胸前挥手
    26: "high_wave",         # 高举挥手
    27: "shake_hand",        # 握手
}

# 反向映射：名称 → ID
BUILTIN_ACTION_BY_NAME = {name: aid for aid, name in BUILTIN_ACTIONS.items()}

# 中文描述，用于 --local-list 和日志
BUILTIN_DESCRIPTIONS = {
    99: "恢复初始手臂位姿",
    11: "双手飞吻",
    12: "单手飞吻",
    13: "右手飞吻",
    15: "平举",
    17: "鼓掌",
    18: "击掌",
    19: "拥抱",
    20: "双手比心",
    21: "右手比心",
    22: "双手打X",
    23: "右手平举",
    24: "动感光波",
    25: "胸前挥手",
    26: "高举挥手",
    27: "握手",
}

# ---------------------------------------------------------------------------
# 错误码（与 C++ g1_arm_action_error.hpp 对齐）
# ---------------------------------------------------------------------------
ARM_ACTION_ERRORS = {
    7400: "话题 rt/arm_sdk 被占用：有动作正在执行",
    7401: "手臂正举起，请使用 ID=99 或相同的上次动作 ID 恢复",
    7402: "动作 ID 不存在",
    7404: "当前 FsmID 不可触发此动作（部分动作在走跑运控下不可用）",
}


def _format_error(code):
    """将错误码转换为可读信息。"""
    msg = ARM_ACTION_ERRORS.get(code)
    return f"错误码 {code}: {msg}" if msg else f"未知错误码: {code}"


# ---------------------------------------------------------------------------
# 扩展客户端：在 g1_teach_v2 内部补齐 Python SDK 缺失的 API
# ---------------------------------------------------------------------------

class _ExtendedArmActionClient:
    """在不修改 unitree_sdk2_python 的前提下，补齐 C++ SDK 中有而 Python SDK 缺失的接口。

    通过继承 G1ArmActionClient 并在 Init 后额外注册 API 7108 / 7113 来实现。
    """

    def __init__(self):
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        self._client = G1ArmActionClient()

    def SetTimeout(self, timeout):
        self._client.SetTimeout(timeout)

    def Init(self):
        self._client.Init()
        # 补充注册 Python SDK 中未暴露的 API
        self._client._RegistApi(_API_ID_EXECUTE_CUSTOM_ACTION, 0)
        self._client._RegistApi(_API_ID_STOP_CUSTOM_ACTION, 0)

    def ExecuteAction(self, action_id: int):
        """执行内置预设动作（阻塞）。"""
        return self._client.ExecuteAction(action_id)

    def ExecuteCustomAction(self, action_name: str):
        """执行 APP 端录制的示教动作（非阻塞）。

        参数格式与 C++ SDK 一致: {"action_name": "<name>"}
        """
        p = {"action_name": action_name}
        parameter = json.dumps(p)
        code, _data = self._client._Call(_API_ID_EXECUTE_CUSTOM_ACTION, parameter)
        return code

    def StopCustomAction(self):
        """停止当前正在执行的示教动作。停止后手臂回到初始位置。"""
        p = {}
        parameter = json.dumps(p)
        code, _data = self._client._Call(_API_ID_STOP_CUSTOM_ACTION, parameter)
        return code

    def GetActionList(self):
        """获取当前固件可用的全部动作列表。"""
        return self._client.GetActionList()


# ---------------------------------------------------------------------------
# 内部工具
# ---------------------------------------------------------------------------

def _create_client(iface, domain=0):
    """创建并初始化扩展版 ArmActionClient。

    延迟导入 SDK，保持 --help / --dry-run / --local-list 轻量。
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    iface = resolve_network_interface(iface, verbose=True)
    ChannelFactoryInitialize(domain, iface)
    client = _ExtendedArmActionClient()
    client.SetTimeout(10.0)
    client.Init()
    return client


# ---------------------------------------------------------------------------
# 公开 API
# ---------------------------------------------------------------------------

def execute_builtin_action(iface, action_id, auto_release=False,
                           hold_time=2.0):
    """执行一个内置预设动作。

    Args:
        iface: 网卡接口名
        action_id: 动作 ID（整数）
        auto_release: 执行后是否自动恢复初始位姿（ID=99）
        hold_time: auto_release 时动作完成后等待的时间（秒）

    Returns:
        0 表示成功，否则为错误码。
    """
    client = _create_client(iface)
    name = BUILTIN_ACTIONS.get(action_id, f"unknown({action_id})")
    print(f"[ARM_ACTION] 执行内置动作: {name} (ID={action_id})")

    code = client.ExecuteAction(action_id)
    if code != 0:
        print(f"[ARM_ACTION] 执行失败: {_format_error(code)}")
        return code

    print("[ARM_ACTION] 执行成功")

    if auto_release and action_id != 99:
        print(f"[ARM_ACTION] 等待 {hold_time:.1f}s 后恢复初始位姿...")
        time.sleep(hold_time)
        release_code = client.ExecuteAction(99)
        if release_code != 0:
            print(f"[ARM_ACTION] 恢复失败: {_format_error(release_code)}")
            return release_code
        print("[ARM_ACTION] 已恢复初始位姿")

    return 0


def execute_custom_action(iface, action_name, wait=None):
    """执行一个 APP 端录制的示教动作（非阻塞）。

    Args:
        iface: 网卡接口名
        action_name: 示教动作名称（区分大小写）
        wait: 可选等待时间（秒），示教动作非阻塞，需手动等待

    Returns:
        0 表示成功，否则为错误码。
    """
    client = _create_client(iface)
    print(f"[ARM_ACTION] 执行示教动作: {action_name}")

    code = client.ExecuteCustomAction(action_name)
    if code != 0:
        print(f"[ARM_ACTION] 执行失败: {_format_error(code)}")
        return code

    print("[ARM_ACTION] 示教动作已触发（非阻塞）")

    if wait and wait > 0:
        print(f"[ARM_ACTION] 等待 {wait:.1f}s...")
        time.sleep(wait)

    return 0


def stop_custom_action(iface):
    """停止当前正在执行的示教动作。

    Returns:
        0 表示成功，否则为错误码。
    """
    client = _create_client(iface)
    print("[ARM_ACTION] 停止示教动作...")

    code = client.StopCustomAction()
    if code != 0:
        print(f"[ARM_ACTION] 停止失败: {_format_error(code)}")
        return code

    print("[ARM_ACTION] 示教动作已停止，手臂回到初始位置")
    return 0

import os as _os

# def_motion.json 默认路径（与本文件同级）
_DEF_MOTION_PATH = _os.path.join(_os.path.dirname(__file__), "def_motion.json")


def _parse_action_list(data):
    """解析 GetActionList 返回的数据。

    实机返回的格式是一个包含两个列表的数组：
      [
        [ {id, name, ...}, ... ],      # 内置动作
        [ {name, time, ...}, ... ],     # APP 示教动作
      ]

    Returns:
        (builtin_list, custom_list) 两个列表。
    """
    if isinstance(data, list) and len(data) == 2:
        return data[0] or [], data[1] or []
    # 兼容其他可能的格式
    if isinstance(data, dict):
        return data.get("actions", []), data.get("custom_actions", [])
    return [], []


def list_actions(iface):
    """从机器人查询当前固件可用的全部动作（含示教动作）。

    Returns:
        0 表示成功，否则为错误码。
    """
    client = _create_client(iface)
    print("[ARM_ACTION] 查询可用动作列表...")

    code, data = client.GetActionList()
    if code != 0:
        print(f"[ARM_ACTION] 查询失败: {_format_error(code)}")
        return code

    builtin_list, custom_list = _parse_action_list(data)

    if builtin_list:
        print("\n  === 内置动作 ===")
        for action in builtin_list:
            aid = action.get("id", "?")
            aname = action.get("name", "?")
            desc = BUILTIN_DESCRIPTIONS.get(aid, "")
            extra = ""
            if "fsm" in action:
                extra += f"  fsm={action['fsm']}"
            if "mode_machine" in action:
                extra += f"  mode={action['mode_machine']}"
            print(f"    ID={aid:>3}  {aname:<30}  {desc}{extra}")

    if custom_list:
        print("\n  === APP 示教动作 ===")
        for action in custom_list:
            aname = action.get("name", "?")
            duration = action.get("time", "?")
            print(f"    名称={aname:<30}  时长={duration}s")

    if not builtin_list and not custom_list:
        print(f"  {data}")

    return 0


def sync_actions(iface, def_motion_path=None):
    """从机器人查询动作列表，自动同步 def_motion.json 中的 app_action 部分。

    同步规则：
      - app_action 中 comment 以 "id:" 开头的条目 → 内置动作，保持不变
      - 其余条目 → 删除旧的，用机器人返回的 APP 示教动作替换

    Args:
        iface: 网卡接口名
        def_motion_path: def_motion.json 路径，默认为项目同级文件

    Returns:
        0 表示成功，否则为错误码。
    """
    if def_motion_path is None:
        def_motion_path = _DEF_MOTION_PATH

    # 1. 从机器人查询
    client = _create_client(iface)
    print("[ARM_ACTION] 查询机器人动作列表...")

    code, data = client.GetActionList()
    if code != 0:
        print(f"[ARM_ACTION] 查询失败: {_format_error(code)}")
        return code

    _builtin_list, custom_list = _parse_action_list(data)

    # 2. 读取现有 def_motion.json
    try:
        with open(def_motion_path, "r", encoding="utf-8-sig") as f:
            def_motion = json.load(f)
    except FileNotFoundError:
        print(f"[ARM_ACTION] 文件不存在: {def_motion_path}")
        return 1

    # 3. 找到 app_action 分组
    app_group = None
    for group in def_motion:
        if group.get("type") == "app_action":
            app_group = group
            break

    if app_group is None:
        print("[ARM_ACTION] def_motion.json 中未找到 type=app_action 分组")
        return 1

    # 4. 拆分：保留内置动作条目（comment 以 "id:" 开头），删除旧的 APP 示教条目
    old_actions = app_group.get("actions", [])
    builtin_entries = [a for a in old_actions if a.get("comment", "").startswith("id:")]
    old_custom_entries = [a for a in old_actions if not a.get("comment", "").startswith("id:")]

    # 5. 从机器人查询结果生成新的 APP 示教条目
    new_custom_entries = []
    for action in custom_list:
        aname = action.get("name", "")
        atime = action.get("time", "")
        new_custom_entries.append({
            "actionName": aname,
            "comment": f"{atime}s"
        })

    # 6. 合并：内置动作 + 新的 APP 示教动作
    app_group["actions"] = builtin_entries + new_custom_entries

    # 7. 写回文件
    with open(def_motion_path, "w", encoding="utf-8") as f:
        json.dump(def_motion, f, ensure_ascii=False, indent=2)
        f.write("\n")

    # 8. 打印同步报告
    old_names = {a["actionName"] for a in old_custom_entries}
    new_names = {a["actionName"] for a in new_custom_entries}

    added = new_names - old_names
    removed = old_names - new_names
    kept = old_names & new_names

    print(f"[ARM_ACTION] 同步完成 → {def_motion_path}")
    print(f"  内置动作（不变）: {len(builtin_entries)} 条")
    print(f"  APP 示教动作:    {len(new_custom_entries)} 条")
    if added:
        print(f"    + 新增: {', '.join(sorted(added))}")
    if removed:
        print(f"    - 移除: {', '.join(sorted(removed))}")
    if kept:
        print(f"    = 保留: {', '.join(sorted(kept))}")

    return 0


def list_builtin_actions():
    """打印本地内置动作映射表（不需要连接机器人）。"""
    print("[ARM_ACTION] 本地内置动作映射表:")
    print(f"  {'ID':>3}  {'名称':<20}  说明")
    print(f"  {'---':>3}  {'----':<20}  ----")
    for aid in sorted(BUILTIN_ACTIONS.keys()):
        name = BUILTIN_ACTIONS[aid]
        desc = BUILTIN_DESCRIPTIONS.get(aid, "")
        print(f"  {aid:>3}  {name:<20}  {desc}")

# ---------------------------------------------------------------------------
# �������� �� g1_base ���ݲ�
# g1_base �� nav_core.py ֱ�� import format_error / parse_action_list��
# �°��Ϊ _format_error / _parse_action_list���˴����ֹ��� API ���á�
# ---------------------------------------------------------------------------
format_error = _format_error
parse_action_list = _parse_action_list
API_ID_EXECUTE_CUSTOM_ACTION = _API_ID_EXECUTE_CUSTOM_ACTION
API_ID_STOP_CUSTOM_ACTION = _API_ID_STOP_CUSTOM_ACTION
