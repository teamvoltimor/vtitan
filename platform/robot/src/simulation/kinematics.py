"""Ackermann bicycle-model kinematics for the headless car simulation.

Integrates the same motion a real LEGO Bugatti Bolide chassis produces from a
``(linear_speed, steering)`` command, honouring the physical limits the actual
hardware imposes:

* **Steering slew** — the servo cannot snap to an angle instantly; it ramps at
  ``MAX_STEERING_RATE`` (rad/s).
* **Drive acceleration** — the drive motor cannot change speed instantly; it is
  clamped to ``max_accel`` (m/s²), matching the JerkLimiter the controller uses.
* **Steering limit** — front-wheel angle saturates at ``MAX_STEERING_ANGLE``.

The bicycle model uses the rear-axle reference point::

    x   += v * cos(yaw) * dt
    y   += v * sin(yaw) * dt
    yaw += (v / wheelbase) * tan(steer) * dt

Integration is sub-stepped for accuracy at the 20 Hz control rate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from shared.config.constants import RobotSpecs

_DEFAULT_MAX_STEER_RATE = 2.0  # rad/s (NavigationTuning.pursuit.MAX_STEERING_RATE)
_DEFAULT_MAX_ACCEL = 2.0  # m/s² (NavigationTuning.speed / JerkLimiter default)


@dataclass(frozen=True, slots=True)
class AckermannState:
    """Full kinematic state of the simulated car."""

    x: float
    y: float
    yaw: float
    v: float = 0.0  # current linear speed (m/s)
    steer: float = 0.0  # current front-wheel angle (radians)


class AckermannKinematics:
    """Sub-stepped bicycle-model integrator with hardware rate limits."""

    def __init__(
        self,
        wheelbase: float = RobotSpecs.WHEELBASE,
        max_steer: float = RobotSpecs.MAX_STEERING_ANGLE,
        max_steer_rate: float = _DEFAULT_MAX_STEER_RATE,
        max_accel: float = _DEFAULT_MAX_ACCEL,
        substeps: int = 5,
    ) -> None:
        self._wheelbase = wheelbase
        self._max_steer = max_steer
        self._max_steer_rate = max_steer_rate
        self._max_accel = max_accel
        self._substeps = max(1, substeps)

    def step(
        self,
        state: AckermannState,
        target_speed: float,
        target_steer_norm: float,
        dt: float,
    ) -> AckermannState:
        """Advance the state by ``dt`` under a controller command.

        Args:
            state: Current kinematic state.
            target_speed: Commanded linear speed (m/s); may be negative (reverse).
            target_steer_norm: Commanded steering in the controller's normalised
                ``[-1, 1]`` range (as published in ``Velocity.angular``).
            dt: Control interval (seconds).

        Returns:
            The new :class:`AckermannState`.
        """
        target_steer = _clamp(target_steer_norm, -1.0, 1.0) * self._max_steer

        x, y, yaw = state.x, state.y, state.yaw
        v, steer = state.v, state.steer

        h = dt / self._substeps
        for _ in range(self._substeps):
            # Servo steering slew toward the target angle.
            steer = _approach(steer, target_steer, self._max_steer_rate * h)
            steer = _clamp(steer, -self._max_steer, self._max_steer)
            # Drive acceleration clamp toward the target speed.
            v = _approach(v, target_speed, self._max_accel * h)

            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += (v / self._wheelbase) * math.tan(steer) * h

        yaw = _wrap_angle(yaw)
        return replace(state, x=x, y=y, yaw=yaw, v=v, steer=steer)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _approach(current: float, target: float, max_delta: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_delta``."""
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def _wrap_angle(angle: float) -> float:
    """Wrap to ``[-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))
