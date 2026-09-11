"""ROS-independent state machine for the final drive watchdog."""

from enum import Enum
import math


class WatchdogState(str, Enum):
    WAITING_FOR_COMMAND = "waiting_for_command"
    WAITING_FOR_REARM = "waiting_for_rearm"
    ACTIVE = "active"
    COMMAND_TIMEOUT = "command_timeout"
    ESTOP = "estop"


class InputFreshness:
    """Track whether a required input has arrived recently on a steady clock."""

    def __init__(self, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self.timeout_sec = timeout_sec
        self.last_input_steady_sec: float | None = None

    def observe(self, steady_now_sec: float) -> bool:
        if not math.isfinite(steady_now_sec):
            return False
        self.last_input_steady_sec = steady_now_sec
        return True

    def is_fresh(self, steady_now_sec: float) -> bool:
        if self.last_input_steady_sec is None:
            return False
        age_sec = steady_now_sec - self.last_input_steady_sec
        return math.isfinite(age_sec) and 0.0 <= age_sec <= self.timeout_sec


class CommandWatchdog:
    """Decide whether the most recently received command may be forwarded."""

    def __init__(self, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self.timeout_sec = timeout_sec
        self.state = WatchdogState.WAITING_FOR_COMMAND
        self.estop_active = False
        self.rearm_required = False
        self.have_command = False
        self.last_command_steady_sec: float | None = None

    def receive_command(self, steady_now_sec: float) -> bool:
        if (
            self.estop_active
            or self.rearm_required
            or not math.isfinite(steady_now_sec)
        ):
            return False
        self.have_command = True
        self.last_command_steady_sec = steady_now_sec
        self.state = WatchdogState.ACTIVE
        return True

    def set_estop(self, active: bool) -> bool:
        """Apply an e-stop edge and discard every pre-edge command."""

        if active == self.estop_active:
            return False
        self.estop_active = active
        self.clear_command()
        if active:
            self.rearm_required = True
            self.state = WatchdogState.ESTOP
        else:
            self.state = WatchdogState.WAITING_FOR_REARM
        return True

    def rearm(self) -> tuple[bool, str]:
        if self.estop_active:
            return False, "estop_is_still_active"
        if not self.rearm_required:
            return False, "no_estop_rearm_required"

        self.clear_command()
        self.rearm_required = False
        self.state = WatchdogState.WAITING_FOR_COMMAND
        return True, "rearmed_waiting_for_new_command"

    def clear_command(self) -> None:
        self.have_command = False
        self.last_command_steady_sec = None

    def may_forward(self, steady_now_sec: float) -> bool:
        if self.estop_active:
            self.state = WatchdogState.ESTOP
            return False
        if self.rearm_required:
            self.state = WatchdogState.WAITING_FOR_REARM
            return False
        if not self.have_command or self.last_command_steady_sec is None:
            return False

        command_age_sec = steady_now_sec - self.last_command_steady_sec
        if (
            not math.isfinite(command_age_sec)
            or command_age_sec < 0.0
            or command_age_sec > self.timeout_sec
        ):
            self.clear_command()
            self.state = WatchdogState.COMMAND_TIMEOUT
            return False

        self.state = WatchdogState.ACTIVE
        return True
