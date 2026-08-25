"""Pure control primitives for quadrature-encoder feedback and closed-loop speed.

No hardware imports — fully unit-testable on any platform. These convert
encoder counts to physical quantities and run the closed-loop speed PID.
"""

from __future__ import annotations

import math

_CPR_POSITIVE = "counts_per_rev must be positive"


def counts_to_revolutions(counts: int, counts_per_rev: float) -> float:
    """Output-shaft revolutions for a raw quadrature count."""
    if counts_per_rev <= 0:
        raise ValueError(_CPR_POSITIVE)
    return counts / counts_per_rev


def revolutions_to_distance(revolutions: float, wheel_diameter_m: float) -> float:
    """Linear wheel distance (metres) for a number of output-shaft revolutions."""
    return revolutions * math.pi * wheel_diameter_m


def counts_to_distance(counts: int, counts_per_rev: float, wheel_diameter_m: float) -> float:
    """Linear wheel distance (metres) for a raw quadrature count."""
    return revolutions_to_distance(counts_to_revolutions(counts, counts_per_rev), wheel_diameter_m)


class SpeedEstimator:
    """Estimate signed output-shaft RPM from successive encoder counts.

    Applies light exponential smoothing to suppress quantisation jitter.
    """

    def __init__(self, counts_per_rev: float, smoothing: float = 0.3) -> None:
        if counts_per_rev <= 0:
            raise ValueError(_CPR_POSITIVE)
        if not 0.0 < smoothing <= 1.0:
            msg = "smoothing must be in (0, 1]"
            raise ValueError(msg)
        self._counts_per_rev = counts_per_rev
        self._smoothing = smoothing
        self._prev_counts: int | None = None
        self._rpm = 0.0

    def reset(self) -> None:
        """Forget history (call when the encoder is reset or motion stops)."""
        self._prev_counts = None
        self._rpm = 0.0

    def update(self, counts: int, dt: float) -> float:
        """Fold in a new count reading and return the smoothed RPM."""
        if dt <= 0:
            return self._rpm
        if self._prev_counts is None:
            self._prev_counts = counts
            return self._rpm
        delta = counts - self._prev_counts
        self._prev_counts = counts
        revs = delta / self._counts_per_rev
        rpm_raw = revs / dt * 60.0
        self._rpm = self._smoothing * rpm_raw + (1.0 - self._smoothing) * self._rpm
        return self._rpm


class PIDController:
    """Clamped PI(D) controller with back-calculation anti-windup.

    Pure function of (setpoint, measurement, dt). An optional feed-forward term
    proportional to the setpoint gives the integrator less work to do.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float = 0.0,
        *,
        output_min: float = -1.0,
        output_max: float = 1.0,
        feedforward: float = 0.0,
    ) -> None:
        if output_min >= output_max:
            msg = "output_min must be < output_max"
            raise ValueError(msg)
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._output_min = output_min
        self._output_max = output_max
        self._feedforward = feedforward
        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        """Clear integral and derivative state."""
        self._integral = 0.0
        self._prev_error = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """Compute the clamped control output for one tick."""
        if dt <= 0:
            msg = "dt must be positive"
            raise ValueError(msg)

        error = setpoint - measurement
        proportional = self._kp * error
        derivative = 0.0
        if self._prev_error is not None:
            derivative = self._kd * (error - self._prev_error) / dt
        self._prev_error = error

        feedforward = self._feedforward * setpoint
        integral_candidate = self._integral + error * dt
        raw = proportional + self._ki * integral_candidate + derivative + feedforward

        clamped = max(self._output_min, min(self._output_max, raw))
        # Anti-windup: only accumulate the integral when not saturated, so the
        # integrator cannot keep growing while the output is railed.
        if clamped == raw:
            self._integral = integral_candidate
        return clamped
