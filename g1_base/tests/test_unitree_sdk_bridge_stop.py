import json
from pathlib import Path
import sys
import threading


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base.unitree_sdk_bridge import UnitreeSdkBridge


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Stdin:
    def __init__(self):
        self.lines = []

    def write(self, data):
        self.lines.append(data)

    def flush(self):
        pass


class _Process:
    def __init__(self):
        self.stdin = _Stdin()

    def poll(self):
        return None


def _bare_bridge():
    bridge = object.__new__(UnitreeSdkBridge)
    bridge._logger = _Logger()
    bridge._process = _Process()
    bridge._stdin_lock = threading.Lock()
    bridge._response_lock = threading.Lock()
    bridge._next_request_id = 1
    bridge._loco_dispatch_lock = threading.Condition()
    bridge._loco_dispatch_latest = None
    bridge._loco_dispatch_epoch = 0
    bridge._loco_min_interval_sec = 0.2
    bridge._loco_dispatch_stop = False
    bridge._loco_latency_lock = threading.Lock()
    bridge._loco_latency_samples_ms = []
    bridge._loco_latency_active_samples_ms = []
    bridge._last_loco_latency_ms = 0.0
    bridge._last_loco_latency_sample_time = 0.0
    bridge._last_active_loco_latency_sample_time = 0.0
    bridge._loco_slow_count = 0
    return bridge


def _written_commands(bridge):
    return [json.loads(line) for line in bridge._process.stdin.lines]


def test_urgent_loco_stop_writes_without_waiting_for_response_lock():
    bridge = _bare_bridge()
    bridge._response_lock.acquire()
    try:
        thread = threading.Thread(
            target=bridge.urgent_loco_stop,
            kwargs={"reason": "unit_test"},
        )
        thread.start()
        thread.join(0.2)
        assert not thread.is_alive()
    finally:
        bridge._response_lock.release()

    commands = _written_commands(bridge)
    assert [cmd["command"] for cmd in commands] == ["loco_move", "loco_stop"]
    assert all(cmd["id"] == -1 for cmd in commands)
    assert commands[0]["vx"] == 0.0
    assert commands[0]["vy"] == 0.0
    assert commands[0]["wz"] == 0.0
    assert commands[0]["source"] == "urgent_stop:unit_test"


def test_queue_loco_move_keeps_latest_command_only():
    bridge = _bare_bridge()

    bridge.queue_loco_move(0.1, 0.0, 0.0, source="first")
    bridge.queue_loco_move(0.2, 0.0, 0.3, source="second")

    queued = bridge._loco_dispatch_latest
    assert queued["vx"] == 0.2
    assert queued["wz"] == 0.3
    assert queued["source"] == "second"


def test_loco_health_snapshot_tracks_active_planner_samples_separately():
    bridge = _bare_bridge()

    bridge._record_loco_latency("loco_move", 245.0, source="planner_timeout")
    bridge._record_loco_latency("loco_move", 510.0, source="force_stop_hold")
    bridge._record_loco_latency("loco_move", 203.4, source="planner")

    snapshot = bridge.get_loco_health_snapshot()

    assert snapshot["loco_latency_sample_count"] == 3
    assert snapshot["loco_latency_p95_ms"] == 245.0
    assert snapshot["active_loco_latency_sample_count"] == 1
    assert snapshot["active_loco_latency_p95_ms"] == 203.4
    assert snapshot["last_loco_latency_age_sec"] >= 0.0
    assert snapshot["active_loco_latency_age_sec"] >= 0.0


def test_loco_health_reset_prevents_planner_timeout_sample_from_polluting_next_navigation():
    bridge = _bare_bridge()

    bridge._record_loco_latency("loco_move", 245.8, source="planner_timeout")
    assert bridge.get_loco_health_snapshot()["loco_latency_p95_ms"] == 245.8

    bridge.reset_loco_health()
    bridge._record_loco_latency("loco_move", 203.6, source="planner")

    snapshot = bridge.get_loco_health_snapshot()
    assert snapshot["loco_latency_sample_count"] == 1
    assert snapshot["loco_latency_p95_ms"] == 203.6
    assert snapshot["active_loco_latency_sample_count"] == 1
    assert snapshot["active_loco_latency_p95_ms"] == 203.6
    assert snapshot["loco_slow_count"] == 1
