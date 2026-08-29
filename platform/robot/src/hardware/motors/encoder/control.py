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

    Accumulates counts/dt over a minimum window before computing a rate, then
    applies light exponential smoothing to what's left. A single caller tick
    (e.g. a ~20ms PID period) is not enough window on its own at low RPM and
    coarse counts_per_rev: bench data at 86 counts_per_rev / 13.6rpm target
    (2026-08-28) averages ~0.39 counts per ~20ms tick, so a per-tick rate is
    computed from a raw 0-vs-1 count difference -- a >100% relative swing
    that smoothing alone cannot remove, since it damps a noisy signal rather
    than fixing the signal's own resolution. Widening the window to
    min_window_s trades responsiveness (a real lag of up to that long between
    a real speed change and it showing up here) for a much lower noise floor
    (at the same operating point, a 0.1s window averages ~1.95 counts, so a
    +-1 count difference is a ~50% swing instead of >100%). Default chosen as
    a reasoned middle ground, NOT live-verified against real oscillation
    behavior -- re-check via base.py's "PID step" debug log after any
    counts_per_rev/target-rpm change before trusting it.
    """

    def __init__(
        self,
        counts_per_rev: float,
        smoothing: float = 0.3,
        min_window_s: float = 0.1,
    ) -> None:
        if counts_per_rev <= 0:
            raise ValueError(_CPR_POSITIVE)
        if not 0.0 < smoothing <= 1.0:
            msg = "smoothing must be in (0, 1]"
            raise ValueError(msg)
        if min_window_s < 0.0:
            msg = "min_window_s must be >= 0"
            raise ValueError(msg)
        self._counts_per_rev = counts_per_rev
        self._smoothing = smoothing
        self._min_window_s = min_window_s
        self._prev_counts: int | None = None
        self._window_counts = 0
        self._window_dt = 0.0
        self._rpm = 0.0

    def reset(self) -> None:
        """Forget history (call when the encoder is reset or motion stops)."""
        self._prev_counts = None
        self._window_counts = 0
        self._window_dt = 0.0
        self._rpm = 0.0

    def update(self, counts: int, dt: float) -> float:
        """Fold in a new count reading; return the smoothed RPM once enough
        window has accumulated, otherwise the held value from the last
        completed window."""
        if dt <= 0:
            return self._rpm
        if self._prev_counts is None:
            self._prev_counts = counts
            return self._rpm
        delta = counts - self._prev_counts
        self._prev_counts = counts
        self._window_counts += delta
        self._window_dt += dt
        if self._window_dt < self._min_window_s:
            return self._rpm
        revs = self._window_counts / self._counts_per_rev
        rpm_raw = revs / self._window_dt * 60.0
        self._rpm = self._smoothing * rpm_raw + (1.0 - self._smoothing) * self._rpm
        self._window_counts = 0
        self._window_dt = 0.0
        return self._rpm


class PIDController:
    """Clamped PI(D) controller with back-calculation anti-windup.

    Pure function of (setpoint, measurement, dt). An optional feed-forward term
    gives the integrator less work to do. It is AFFINE, not proportional --
    ``offset + gain * setpoint`` -- because a real drivetrain has a duty
    deadband below which the motor does not turn, so the duty a given rpm needs
    does not pass through the origin. See :meth:`_feed_forward_for`.
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
        feedforward_offset: float = 0.0,
    ) -> None:
        if output_min >= output_max:
            msg = "output_min must be < output_max"
            raise ValueError(msg)
        if feedforward_offset < 0.0:
            msg = "feedforward_offset must be >= 0 (its sign follows the setpoint)"
            raise ValueError(msg)
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._output_min = output_min
        self._output_max = output_max
        self._feedforward = feedforward
        self._feedforward_offset = feedforward_offset
        self._integral = 0.0
        self._prev_error: float | None = None

    def reset(self) -> None:
        """Clear integral and derivative state."""
        self._integral = 0.0
        self._prev_error = None

    def _feed_forward_for(self, setpoint: float) -> float:
        """Open-loop duty estimate for ``setpoint``, as offset + gain * setpoint.

        The offset exists because a real drivetrain has a DEADBAND: below some
        duty the motor does not turn at all, so duty is affine in rpm rather
        than proportional to it. Measured 2026-08-29 on this chassis, loaded:

            rpm = 434.6 * duty - 86.7    ->    duty = 0.200 + rpm / 434.6

        A pure ``gain * setpoint`` term cannot represent that, and the error it
        leaves is what the integrator has to absorb on every tick.

        Returns 0.0 for a zero setpoint rather than the offset. Without that,
        commanding a stop would still ask for the deadband duty and the chassis
        would creep -- and ``stop_drive()`` would not stop it.

        The offset is unsigned in config and takes the setpoint's sign here, so
        one value serves both directions instead of the caller managing it.
        """
        if setpoint == 0.0:
            return 0.0
        return math.copysign(self._feedforward_offset, setpoint) + self._feedforward * setpoint

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

        feedforward = self._feed_forward_for(setpoint)
        integral_candidate = self._integral + error * dt
        raw = proportional + self._ki * integral_candidate + derivative + feedforward

        clamped = max(self._output_min, min(self._output_max, raw))
        # Anti-windup: only accumulate the integral when not saturated, so the
        # integrator cannot keep growing while the output is railed.
        if clamped == raw:
            self._integral = integral_candidate
        return clamped
