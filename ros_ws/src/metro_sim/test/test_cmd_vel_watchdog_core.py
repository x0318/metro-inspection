from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_watchdog_core import (  # noqa: E402
    CommandWatchdog,
    InputFreshness,
    WatchdogState,
)


def test_normal_command_is_forwarded_until_timeout():
    watchdog = CommandWatchdog(timeout_sec=0.5)

    assert not watchdog.may_forward(1.0)
    assert watchdog.receive_command(1.0)
    assert watchdog.may_forward(1.49)
    assert not watchdog.may_forward(1.51)
    assert watchdog.state == WatchdogState.COMMAND_TIMEOUT
    assert not watchdog.have_command


def test_new_command_restores_stream_after_timeout():
    watchdog = CommandWatchdog(timeout_sec=0.5)
    assert watchdog.receive_command(1.0)
    assert not watchdog.may_forward(2.0)

    assert watchdog.receive_command(2.1)
    assert watchdog.may_forward(2.1)
    assert watchdog.state == WatchdogState.ACTIVE


def test_estop_discards_old_and_in_stop_commands():
    watchdog = CommandWatchdog(timeout_sec=0.5)
    assert watchdog.receive_command(1.0)
    assert watchdog.may_forward(1.1)

    assert watchdog.set_estop(True)
    assert not watchdog.may_forward(1.2)
    assert not watchdog.receive_command(1.3)
    assert watchdog.state == WatchdogState.ESTOP

    assert watchdog.set_estop(False)
    assert watchdog.state == WatchdogState.WAITING_FOR_REARM
    assert not watchdog.may_forward(1.4)
    assert not watchdog.receive_command(1.5)

    success, _ = watchdog.rearm()
    assert success
    assert watchdog.state == WatchdogState.WAITING_FOR_COMMAND
    assert not watchdog.may_forward(1.6)

    assert watchdog.receive_command(1.7)
    assert watchdog.may_forward(1.7)


def test_repeated_clear_estop_message_does_not_clear_active_command():
    watchdog = CommandWatchdog(timeout_sec=0.5)
    assert watchdog.receive_command(1.0)

    assert not watchdog.set_estop(False)
    assert watchdog.may_forward(1.1)


def test_rearm_is_rejected_while_estop_is_active():
    watchdog = CommandWatchdog(timeout_sec=0.5)
    assert watchdog.set_estop(True)

    success, reason = watchdog.rearm()
    assert not success
    assert reason == "estop_is_still_active"


def test_steady_clock_regression_fails_closed():
    watchdog = CommandWatchdog(timeout_sec=0.5)
    assert watchdog.receive_command(2.0)

    assert not watchdog.may_forward(1.0)
    assert watchdog.state == WatchdogState.COMMAND_TIMEOUT


def test_required_input_must_arrive_and_remain_fresh():
    freshness = InputFreshness(timeout_sec=1.0)

    assert not freshness.is_fresh(0.5)
    assert freshness.observe(0.5)
    assert freshness.is_fresh(1.5)
    assert not freshness.is_fresh(1.5001)


def test_required_input_rejects_invalid_or_regressed_steady_time():
    freshness = InputFreshness(timeout_sec=1.0)

    assert not freshness.observe(float("nan"))
    assert freshness.observe(2.0)
    assert not freshness.is_fresh(1.0)
