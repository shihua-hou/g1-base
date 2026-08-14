from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_base.wait_imu_steady import (
    STATUS_PASSED,
    STATUS_TIMEOUT,
    ImuSteadyDetector,
)


def _feed(detector, start_sec, duration_sec, hz, gyro_xyz, accel_xyz):
    count = int(duration_sec * hz) + 1
    for index in range(count):
        detector.add_sample(start_sec + index / hz, gyro_xyz, accel_xyz)
    return start_sec + (count - 1) / hz


def test_stable_imu_passes_after_full_window():
    detector = ImuSteadyDetector(window_sec=2.0)
    now = _feed(detector, 0.0, 2.2, 50.0, (0.001, 0.0, 0.0), (0.0, 0.0, -1.0))

    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED
    assert detector.latest_stats.duration_sec >= 2.0


def test_near_full_window_passes_with_sampling_jitter_slack():
    detector = ImuSteadyDetector(window_sec=2.0, window_slack_sec=0.1)
    now = 0.0
    for index in range(200):
        now = 0.015 + index / 100.0
        detector.add_sample(now, (0.001, 0.0, 0.0), (0.0, 0.0, -1.0))

    assert detector.latest_stats.duration_sec < 2.0
    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED


def test_continuous_motion_times_out():
    detector = ImuSteadyDetector(window_sec=2.0)
    now = _feed(detector, 0.0, 10.2, 50.0, (0.08, 0.0, 0.0), (0.0, 0.0, -1.08))

    assert detector.status(now, 0.0, 10.0) == STATUS_TIMEOUT
    assert not detector.ready


def test_imu_passes_after_motion_settles():
    detector = ImuSteadyDetector(window_sec=2.0)
    _feed(detector, 0.0, 1.0, 50.0, (0.08, 0.0, 0.0), (0.0, 0.0, -1.08))
    now = _feed(detector, 1.1, 2.4, 50.0, (0.001, 0.0, 0.0), (0.0, 0.0, -0.998))

    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED
    assert detector.latest_stats.max_gyro_norm < 0.05


def test_field_stationary_gyro_noise_passes_default_threshold():
    detector = ImuSteadyDetector(window_sec=2.0)
    now = _feed(detector, 0.0, 2.2, 50.0, (0.018, 0.0, 0.0), (0.0, 0.0, -1.004))

    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED


def test_field_stationary_accel_range_passes_default_threshold():
    low_offset = ImuSteadyDetector(window_sec=2.0)
    low_now = _feed(
        low_offset,
        0.0,
        2.2,
        50.0,
        (0.009, 0.0, 0.0),
        (0.0, 0.0, -0.975),
    )
    assert low_offset.status(low_now, 0.0, 10.0) == STATUS_PASSED

    high_offset = ImuSteadyDetector(window_sec=2.0)
    high_now = _feed(
        high_offset,
        0.0,
        2.2,
        50.0,
        (0.009, 0.0, 0.0),
        (0.0, 0.0, -1.025),
    )
    assert high_offset.status(high_now, 0.0, 10.0) == STATUS_PASSED


def test_field_standing_sway_accel_variance_passes_default_threshold():
    detector = ImuSteadyDetector(window_sec=2.0)
    now = 0.0
    hz = 100.0
    for index in range(int(2.2 * hz) + 1):
        now = index / hz
        accel_norm = 1.020 + 0.018 * math.sin(index * 0.37)
        detector.add_sample(now, (0.018, 0.0, 0.0), (0.0, 0.0, -accel_norm))

    assert 1e-4 < detector.latest_stats.accel_var < 5e-4
    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED


def test_pc2_observed_standing_sway_passes_default_threshold():
    detector = ImuSteadyDetector(window_sec=2.0)
    now = 0.0
    hz = 100.0
    for index in range(int(2.2 * hz) + 1):
        now = index / hz
        accel_norm = 1.004 + 0.033 * math.sin(index * 0.37)
        detector.add_sample(now, (0.028, 0.0, 0.0), (0.0, 0.0, -accel_norm))

    assert 5e-4 < detector.latest_stats.accel_var < 8e-4
    assert detector.status(now, 0.0, 10.0) == STATUS_PASSED
