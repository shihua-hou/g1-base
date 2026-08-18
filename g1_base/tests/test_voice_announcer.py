"""播报引擎的状态机行为。

这类"按状态变化触发"的代码最容易出两种错：变化那一拍漏播，或者状态
抖动时连珠炮。两者在现场都很难复现，所以在这里钉死。
"""

import importlib.util
import sys
import types
from pathlib import Path


def _load_bridge():
    """只加载模块里的纯逻辑部分，不拉起 rclpy。"""
    root = Path(__file__).resolve().parents[1]
    src = (root / "g1_base" / "g1_web_bridge.py").read_text(encoding="utf-8")
    # 截取到 VoiceAnnouncer 结束为止的独立片段，避免 import ROS
    start = src.index("DEFAULT_VOICE_PROMPTS = {")
    end = src.index("# ── 设置：速度 / 走路模式 ──")
    body = src[start:end]
    mod = types.ModuleType("voice_frag")
    mod.__dict__["time"] = __import__("time")
    mod.__dict__["copy"] = __import__("copy")
    # get_voice_prompts 在片段里依赖文件读取，测试中直接替换掉
    exec(compile(body, "voice_frag", "exec"), mod.__dict__)
    return mod


MOD = _load_bridge()


def _make(cfg=None):
    said = []
    cfg = cfg or MOD.DEFAULT_VOICE_PROMPTS
    MOD.get_voice_prompts = lambda: __import__("copy").deepcopy(cfg)
    ann = MOD.VoiceAnnouncer(node=None, say=said.append)
    return ann, said


def _make_split(output):
    """分别收集"发给机器人"和"发给浏览器"的两路。"""
    robot, browser = [], []
    cfg = __import__("copy").deepcopy(MOD.DEFAULT_VOICE_PROMPTS)
    cfg["output"] = output
    MOD.get_voice_prompts = lambda: __import__("copy").deepcopy(cfg)
    ann = MOD.VoiceAnnouncer(node=None, say=robot.append, enqueue=browser.append)
    return ann, robot, browser


def _status(**kw):
    base = {
        "navigate": {"active": False},
        "patrol": {"running": False},
        "navigation_manager": {"state": "READY"},
        "control": {"stop_latched": False},
        "system": {},
    }
    for k, v in kw.items():
        base.setdefault(k, {}).update(v)
    return base


def test_nav_start_fires_once_not_every_tick():
    ann, said = _make()
    ann.tick(_status())                                   # 基线
    ann.tick(_status(navigate={"active": True}))          # 起跑
    for _ in range(5):
        ann.tick(_status(navigate={"active": True}))      # 持续跑，不该再播
    assert said == ["开始导航"]


def test_arrival_and_failure_are_distinguished():
    ann, said = _make()
    ann.tick(_status(navigate={"active": True}))
    ann.tick(_status(navigate={"active": False, "result_status": "success",
                               "result_success": True}))
    assert said[-1] == "已到达"

    ann2, said2 = _make()
    ann2.tick(_status(navigate={"active": True}))
    ann2.tick(_status(navigate={"active": False, "result_status": "error",
                                "result_success": False}))
    assert said2[-1] == "导航失败"


def test_cooldown_suppresses_flapping():
    """导航栈在 ERROR 和 DEGRADED 之间反复抖动时不能连珠炮。"""
    ann, said = _make()
    for _ in range(6):
        ann.tick(_status(navigation_manager={"state": "ERROR"}))
        ann.tick(_status(navigation_manager={"state": "DEGRADED_LOCALIZATION"}))
    assert said.count("导航系统异常，请检查") == 1


def test_disabled_globally_says_nothing():
    cfg = __import__("copy").deepcopy(MOD.DEFAULT_VOICE_PROMPTS)
    cfg["enabled"] = False
    ann, said = _make(cfg)
    ann.tick(_status(navigate={"active": True}))
    ann.tick(_status(navigate={"active": False, "result_success": True,
                               "result_status": "success"}))
    assert said == []


def test_single_event_can_be_disabled():
    cfg = __import__("copy").deepcopy(MOD.DEFAULT_VOICE_PROMPTS)
    cfg["events"]["nav_start"]["enabled"] = False
    ann, said = _make(cfg)
    ann.tick(_status())
    ann.tick(_status(navigate={"active": True}))
    assert said == []


def test_battery_alert_silent_when_battery_not_wired():
    """电量未接入时 battery_percent 为 None，这条告警必须完全沉默。"""
    ann, said = _make()
    for _ in range(10):
        ann.tick(_status(system={"battery_percent": None}))
    assert said == []


def test_battery_alert_repeats_on_cooldown_not_once():
    """低电是持续状态：只在跨越阈值那一拍播的话，漏了就再也不提醒。"""
    cfg = __import__("copy").deepcopy(MOD.DEFAULT_VOICE_PROMPTS)
    cfg["alerts"]["battery_low"]["cooldown_sec"] = 0
    ann, said = _make(cfg)
    for _ in range(3):
        ann.tick(_status(system={"battery_percent": 5}))
    assert said.count("电量不足，请及时充电") == 3


def test_estop_fires_on_latch_edge():
    ann, said = _make()
    ann.tick(_status())
    ann.tick(_status(control={"stop_latched": True}))
    ann.tick(_status(control={"stop_latched": True}))
    assert said.count("急停已触发") == 1


# ── 播报去向 ──

def test_output_robot_only():
    ann, robot, browser = _make_split("robot")
    ann.tick(_status())
    ann.tick(_status(navigate={"active": True}))
    assert robot == ["开始导航"] and browser == []


def test_output_browser_only():
    """机器人音频服务挂了的时候，这条路必须还能响。"""
    ann, robot, browser = _make_split("browser")
    ann.tick(_status())
    ann.tick(_status(navigate={"active": True}))
    assert robot == [] and browser == ["开始导航"]


def test_output_both_goes_to_both():
    ann, robot, browser = _make_split("both")
    ann.tick(_status())
    ann.tick(_status(navigate={"active": True}))
    assert robot == ["开始导航"] and browser == ["开始导航"]


def test_cooldown_is_shared_across_outputs():
    """冷却按事件算，不能因为有两路就播两遍。"""
    ann, robot, browser = _make_split("both")
    for _ in range(6):
        ann.tick(_status(navigation_manager={"state": "ERROR"}))
        ann.tick(_status(navigation_manager={"state": "DEGRADED_LOCALIZATION"}))
    assert len(robot) == 1 and len(browser) == 1
