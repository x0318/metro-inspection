import math

import pytest

from metro_detection.odometry_guard import (
    NANOSECONDS_PER_SECOND,
    OdometryGuard,
    OdometryGuardState,
)


def seconds(value: float) -> int:
    return int(value * NANOSECONDS_PER_SECOND)


def make_guard() -> OdometryGuard:
    return OdometryGuard(
        timeout_sec=1.0,
        max_age_sec=0.5,
        future_tolerance_sec=0.05,
        start_steady_sec=0.0,
    )


def observe(
    guard: OdometryGuard,
    *,
    stamp: float,
    ros_now: float,
    steady_now: float,
    values=(0.0, 0.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
) -> bool:
    return guard.observe(
        stamp_ns=seconds(stamp),
        ros_now_ns=seconds(ros_now),
        steady_now_sec=steady_now,
        numeric_values=values,
        orientation_xyzw=orientation,
    )


def test_normal_stream_permits_motion_while_receive_time_is_fresh():
    guard = make_guard()

    assert not guard.check_freshness(0.5)
    assert observe(
        guard, stamp=1.0, ros_now=1.1, steady_now=0.6
    )
    assert guard.state == OdometryGuardState.MONITORING
    assert guard.check_freshness(1.5)


def test_missing_initial_odometry_latches_fault_stop():
    guard = make_guard()

    assert not guard.check_freshness(1.01)
    assert guard.state == OdometryGuardState.FAULT_STOP
    assert guard.reason == "odometry_not_received"


def test_simulation_pause_times_out_and_recovery_requires_rearm_and_new_data():
    guard = make_guard()
    assert observe(
        guard, stamp=10.0, ros_now=10.0, steady_now=0.1
    )

    # ROS time may be frozen by a paused simulator; steady time still expires.
    assert not guard.check_freshness(1.11)
    assert guard.reason == "odometry_stream_stale"

    # A recovered stream refreshes health but cannot clear the latched stop.
    assert observe(
        guard, stamp=11.0, ros_now=11.0, steady_now=1.2
    )
    assert guard.state == OdometryGuardState.FAULT_STOP
    assert not guard.motion_permitted

    success, _ = guard.rearm(
        steady_now_sec=1.21, ros_now_ns=seconds(11.01)
    )
    assert success
    assert guard.state == OdometryGuardState.WAITING_FOR_NEW_ODOMETRY
    assert not guard.check_freshness(1.22)

    assert observe(
        guard, stamp=11.1, ros_now=11.1, steady_now=1.3
    )
    assert guard.motion_permitted


@pytest.mark.parametrize("invalid_value", [math.nan, math.inf, -math.inf])
def test_non_finite_odometry_latches_fault(invalid_value):
    guard = make_guard()

    assert not observe(
        guard,
        stamp=1.0,
        ros_now=1.0,
        steady_now=0.1,
        values=(0.0, invalid_value),
    )
    assert guard.reason == "odometry_contains_non_finite_value"


@pytest.mark.parametrize(
    ("stamp", "ros_now", "expected_reason"),
    [
        (1.0, 1.6, "odometry_stamp_too_old"),
        (1.1, 1.0, "odometry_stamp_from_future"),
    ],
)
def test_stale_or_future_header_stamp_latches_fault(
    stamp, ros_now, expected_reason
):
    guard = make_guard()

    assert not observe(
        guard, stamp=stamp, ros_now=ros_now, steady_now=0.1
    )
    assert guard.reason == expected_reason


@pytest.mark.parametrize("second_stamp", [5.0, 4.9])
def test_repeated_or_backwards_header_stamp_latches_fault(second_stamp):
    guard = make_guard()
    assert observe(
        guard, stamp=5.0, ros_now=5.0, steady_now=0.1
    )

    assert not observe(
        guard,
        stamp=second_stamp,
        ros_now=5.0,
        steady_now=0.2,
    )
    assert guard.reason == "odometry_stamp_not_increasing"


def test_invalid_quaternion_latches_fault():
    guard = make_guard()

    assert not observe(
        guard,
        stamp=1.0,
        ros_now=1.0,
        steady_now=0.1,
        orientation=(0.0, 0.0, 0.0, 0.0),
    )
    assert guard.reason == "odometry_orientation_invalid"


def test_rearm_is_rejected_until_odometry_has_recovered():
    guard = make_guard()
    assert observe(
        guard, stamp=1.0, ros_now=1.0, steady_now=0.1
    )
    assert not guard.check_freshness(1.2)

    success, reason = guard.rearm(
        steady_now_sec=1.2, ros_now_ns=seconds(2.0)
    )
    assert not success
    assert reason == "odometry_still_stale"

    assert observe(
        guard, stamp=2.1, ros_now=2.1, steady_now=1.3
    )
    success, _ = guard.rearm(
        steady_now_sec=1.31, ros_now_ns=seconds(2.11)
    )
    assert success
    assert not guard.motion_permitted
