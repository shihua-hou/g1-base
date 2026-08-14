import math
from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base.gravity_health import (
    STATUS_OK,
    STATUS_WARN,
    GravityHealthMonitor,
    orientation_tuple,
)


def _quat_from_roll(deg):
    half = math.radians(deg) * 0.5
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def _orientation(quat):
    return SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


def _imu(quat):
    return SimpleNamespace(orientation=_orientation(quat))


def _odom(quat):
    return SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(orientation=_orientation(quat)))
    )


def test_gravity_health_reports_ok_under_three_degrees():
    monitor = GravityHealthMonitor(warning_angle_deg=3.0, warning_duration_sec=5.0)
    monitor.update_imu_orientation(orientation_tuple(_imu(_quat_from_roll(2.0)).orientation))
    monitor.update_odom_orientation(orientation_tuple(_odom((0.0, 0.0, 0.0, 1.0)).pose.pose.orientation))

    state = monitor.evaluate(0.0)

    assert state.status == STATUS_OK
    assert state.angle_deg is not None
    assert state.angle_deg < 3.0


def test_gravity_health_compares_imu_to_lio_orientation_not_absolute_roll():
    monitor = GravityHealthMonitor(warning_angle_deg=3.0, warning_duration_sec=5.0)
    roll = _quat_from_roll(8.0)
    monitor.update_imu_orientation(orientation_tuple(_imu(roll).orientation))
    monitor.update_odom_orientation(orientation_tuple(_odom(roll).pose.pose.orientation))

    state = monitor.evaluate(0.0)

    assert state.status == STATUS_OK
    assert state.angle_deg is not None
    assert state.angle_deg < 0.001


def test_gravity_health_warns_after_five_seconds_over_three_degrees():
    monitor = GravityHealthMonitor(warning_angle_deg=3.0, warning_duration_sec=5.0)
    monitor.update_imu_orientation(orientation_tuple(_imu(_quat_from_roll(4.0)).orientation))
    monitor.update_odom_orientation(orientation_tuple(_odom((0.0, 0.0, 0.0, 1.0)).pose.pose.orientation))

    assert monitor.evaluate(10.0).status == STATUS_OK
    assert monitor.evaluate(14.9).status == STATUS_OK

    warning = monitor.evaluate(15.1)
    assert warning.status == STATUS_WARN
    assert warning.angle_deg is not None
    assert warning.angle_deg > 3.0
    assert warning.sustained_sec >= 5.0
