"""巡航讲解的参数传递。

「多点巡航」原来只是连续走点：NavigateToTarget.action 里没有讲解字段，
g1_control_server 把 action_id/say_text 写死成 None/""，于是巡航点上配的
动作和讲解词全是死数据，机器人走到了既不做动作也不说话。这组测试钉住
整条传参链路，任何一环退回去都会红。
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
G1 = ROOT / "g1_base"


def _read(rel):
    return (G1 / rel).read_text(encoding="utf-8")


def test_action_definition_carries_interaction_fields():
    text = _read("g1_base_interfaces/action/NavigateToTarget.action")
    assert "bool perform_interaction" in text
    assert "int32 action_id" in text
    assert "string say_text" in text


def test_control_server_uses_request_fields_not_hardcoded_blanks():
    text = _read("g1_base/g1_control_server.py")
    assert '"action_id": int(request.action_id)' in text
    assert '"say_text": str(request.say_text or "")' in text
    # 曾经写死的那两行不能回来
    assert '"action_id": None,' not in text
    assert 'perform_interaction=perform_interaction' in text
    assert 'perform_interaction=False,\n                    announce_failures' not in text


def test_patrol_passes_waypoint_action_and_speech():
    text = _read("g1_base/g1_web_bridge.py")
    assert "perform_interaction=True," in text
    assert 'action_id=wp.get("action_id", 0)' in text
    assert 'say_text=wp.get("say_text", "")' in text


def test_single_point_navigation_stays_silent():
    """网页上点一个目标点只是让它过去，不该突然做动作说话。"""
    text = _read("g1_base/g1_web_bridge.py")
    assert "goal.perform_interaction = False" in text


def test_interaction_tolerates_missing_action_or_speech():
    """巡航点只填讲解词、或只填动作，都要正常工作。"""
    text = _read("g1_base/nav_core.py")
    assert "do_action = self.arm_client is not None and action_id > 0" in text
    assert "if not do_action and not text:" in text
    # 没做动作就不该白跑一次复位
    assert "# 没做动作就没什么好复位的" in text
