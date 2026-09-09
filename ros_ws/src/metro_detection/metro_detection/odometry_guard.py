"""State machine for fail-safe odometry validation."""

from enum import Enum
import math
from typing import Iterable, Sequence


NANOSECONDS_PER_SECOND = 1_000_000_000


class OdometryGuardState(str, Enum):
    """Safety states used by the automatic coverage driver."""

    WAITING_FOR_ODOMETRY = "waiting_for_odometry"
    MONITORING = "monitoring"
    FAULT_STOP = "fault_stop"
    WAITING_FOR_NEW_ODOMETRY = "waiting_for_new_odometry"


class OdometryGuard:
    """Validate odometry and latch faults until an explicit rearm."""

    def __init__(
        self,
        timeout_sec: float,
        max_age_sec: float,
        future_tolerance_sec: float,
        start_steady_sec: float,
        quaternion_norm_tolerance: float = 0.1,
    ) -> None:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if max_age_sec <= 0.0:
            raise ValueError("max_age_sec must be positive")
        if future_tolerance_sec < 0.0:
            raise ValueError("future_tolerance_sec cannot be negative")
        if not 0.0 < quaternion_norm_tolerance < 1.0:
            raise ValueError(
                "quaternion_norm_tolerance must be between zero and one"
            )

        self.timeout_sec = timeout_sec
        self.max_age_ns = int(max_age_sec * NANOSECONDS_PER_SECOND)
        self.future_tolerance_ns = int(
            future_tolerance_sec * NANOSECONDS_PER_SECOND
        )
        self.quaternion_norm_tolerance = quaternion_norm_tolerance
        self.start_steady_sec = start_steady_sec

        self.state = OdometryGuardState.WAITING_FOR_ODOMETRY
        self.reason = "waiting_for_first_odometry"
        self.last_valid_receive_steady_sec: float | None = None
        self.last_valid_stamp_ns: int | None = None
        self.rearm_after_stamp_ns: int | None = None

    @property
    def motion_permitted(self) -> bool:
        return self.state == OdometryGuardState.MONITORING

    @property
    def fault_latched(self) -> bool:
        return self.state == OdometryGuardState.FAULT_STOP

    def _latch_fault(self, reason: str) -> None:
        self.state = OdometryGuardState.FAULT_STOP
        self.reason = reason

    def observe(
        self,
        *,
        stamp_ns: int,
        ros_now_ns: int,
        steady_now_sec: float,
        numeric_values: Iterable[float],
        orientation_xyzw: Sequence[float],
    ) -> bool:
        """Validate one message and update freshness tracking.

        A valid message refreshes the stored stream even while a fault is
        latched. It never clears the fault by itself.
        """

        error = self._validate_observation(
            stamp_ns=stamp_ns,
            ros_now_ns=ros_now_ns,
            steady_now_sec=steady_now_sec,
            numeric_values=numeric_values,
            orientation_xyzw=orientation_xyzw,
        )
        if error is not None:
            self._latch_fault(error)
            return False

        self.last_valid_stamp_ns = stamp_ns
        self.last_valid_receive_steady_sec = steady_now_sec

        if self.state == OdometryGuardState.WAITING_FOR_ODOMETRY:
            self.state = OdometryGuardState.MONITORING
            self.reason = "odometry_valid"
        elif self.state == OdometryGuardState.WAITING_FOR_NEW_ODOMETRY:
            if (
                self.rearm_after_stamp_ns is None
                or stamp_ns <= self.rearm_after_stamp_ns
            ):
                self._latch_fault("odometry_not_new_after_rearm")
                return False
            self.state = OdometryGuardState.MONITORING
            self.reason = "new_odometry_received_after_rearm"

        return True

    def _validate_observation(
        self,
        *,
        stamp_ns: int,
        ros_now_ns: int,
        steady_now_sec: float,
        numeric_values: Iterable[float],
        orientation_xyzw: Sequence[float],
    ) -> str | None:
        if not math.isfinite(steady_now_sec):
            return "steady_clock_invalid"
        if stamp_ns <= 0:
            return "odometry_stamp_zero_or_negative"
        if ros_now_ns < 0:
            return "ros_clock_invalid"
        if any(not math.isfinite(float(value)) for value in numeric_values):
            return "odometry_contains_non_finite_value"
        if len(orientation_xyzw) != 4:
            return "odometry_orientation_invalid"

        quaternion_norm = math.sqrt(
            sum(float(component) ** 2 for component in orientation_xyzw)
        )
        if (
            not math.isfinite(quaternion_norm)
            or abs(quaternion_norm - 1.0)
            > self.quaternion_norm_tolerance
        ):
            return "odometry_orientation_invalid"

        if ros_now_ns > 0:
            message_age_ns = ros_now_ns - stamp_ns
            if message_age_ns > self.max_age_ns:
                return "odometry_stamp_too_old"
            if message_age_ns < -self.future_tolerance_ns:
                return "odometry_stamp_from_future"

        if (
            self.last_valid_stamp_ns is not None
            and stamp_ns <= self.last_valid_stamp_ns
        ):
            return "odometry_stamp_not_increasing"
        return None

    def check_freshness(self, steady_now_sec: float) -> bool:
        """Latch a stop when the odometry stream is absent or stale."""

        if self.fault_latched:
            return False

        if self.last_valid_receive_steady_sec is None:
            if steady_now_sec - self.start_steady_sec > self.timeout_sec:
                self._latch_fault("odometry_not_received")
            return False

        if (
            steady_now_sec - self.last_valid_receive_steady_sec
            > self.timeout_sec
        ):
            if self.state == OdometryGuardState.WAITING_FOR_NEW_ODOMETRY:
                reason = "odometry_timeout_after_rearm"
            else:
                reason = "odometry_stream_stale"
            self._latch_fault(reason)
            return False

        return self.motion_permitted

    def rearm(
        self, *, steady_now_sec: float, ros_now_ns: int
    ) -> tuple[bool, str]:
        """Clear a latched fault only after the stream itself has recovered."""

        if not self.fault_latched:
            return False, "no_latched_odometry_fault"
        if (
            self.last_valid_receive_steady_sec is None
            or self.last_valid_stamp_ns is None
        ):
            return False, "no_valid_odometry_available"
        if (
            steady_now_sec - self.last_valid_receive_steady_sec
            > self.timeout_sec
        ):
            return False, "odometry_still_stale"

        if ros_now_ns > 0:
            message_age_ns = ros_now_ns - self.last_valid_stamp_ns
            if message_age_ns > self.max_age_ns:
                return False, "odometry_stamp_still_too_old"
            if message_age_ns < -self.future_tolerance_ns:
                return False, "odometry_stamp_still_from_future"

        self.rearm_after_stamp_ns = self.last_valid_stamp_ns
        self.state = OdometryGuardState.WAITING_FOR_NEW_ODOMETRY
        self.reason = "rearmed_waiting_for_new_odometry"
        return True, self.reason
