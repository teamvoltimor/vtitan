"""Counter-phase four-wheel-steer kinematics for the headless car simulation.

Integrates the same motion a real LEGO Bugatti Bolide chassis produces from a
``(linear_speed, steering)`` command, honouring the physical limits the actual
hardware imposes:

* **Steering slew** — the servo cannot snap to an angle instantly; it ramps at
  ``MAX_STEERING_RATE`` (rad/s).
* **Drive acceleration** — the drive motor cannot change speed instantly; it is
  clamped to ``max_accel`` (m/s²), the physical acceleration limit of the drive motor.
* **Steering limit** — front-wheel angle saturates at ``MAX_STEERING_ANGLE``.

**Both axles steer, in opposite directions and by the same amount** -- confirmed
on the real chassis 2026-07-25. That is not the textbook bicycle model, and the
difference is not subtle: counter-phase steering moves the instantaneous centre
of rotation from the rear axle to the chassis centre, so the robot yaws *twice
as fast* as a front-steer car at the same steering angle::

    x += v * cos(yaw) * dt
    y += v * sin(yaw) * dt
    yaw += (v / L_eff) * tan(steer) * dt      # L_eff = wheelbase / (1 + rear_ratio)

With ``rear_steer_ratio = 1.0`` that is ``wheelbase / 2``. Modelling this as a
front-steer car (the previous behaviour) made the simulation turn half as
sharply as the hardware, so any gain tuned against it -- notably
``WaypointController.steer_kp`` -- is hotter on the real robot than in sim.

Integration is sub-stepped for accuracy at the 20 Hz control rate.

Note the reference point: with symmetric counter-steer the body rotates about
its own centre, so ``(x, y)`` tracks the chassis centre rather than the rear
axle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from shared.config.constants import RobotSpecs

_DEFAULT_MAX_STEER_RATE = 2.0  # rad/s (NavigationTuning.pursuit.MAX_STEERING_RATE)
_DEFAULT_MAX_ACCEL = 2.0  # m/s² (drive motor's physical acceleration limit)

_DEFAULT_MAX_SPEED_MPS = 0.156
"""Top speed the real drivetrain reaches, measured 2026-07-25 (0.796 m / 5.11 s).

The simulator previously integrated whatever speed it was handed, and the
tuning profiles asked for 0.7-0.8 m/s -- roughly 6x what the hardware can do.
Clamping here means a profile that over-asks produces the same saturated
behaviour in sim as on the robot instead of a lap time that cannot happen.

Sags with battery charge (0.129 m/s measured on a tired pack), so this is a
ceiling rather than a guarantee.
"""

_DEFAULT_REAR_STEER_RATIO = 1.0
"""Rear steering magnitude relative to the front, counter-phase.

1.0 = rear wheels turn equally and oppositely to the front (confirmed on the
real chassis 2026-07-25: both axles steer, at the same ratio, in opposite
directions). 0.0 would be a conventional front-only car.
"""


@dataclass(frozen=True, slots=True)
class AckermannState:
    """Full kinematic state of the simulated car."""

    x: float
    y: float
    yaw: float
    v: float = 0.0  # current linear speed (m/s)
    steer: float = 0.0  # current front-wheel angle (radians)


class AckermannKinematics:
    """Sub-stepped counter-phase four-wheel-steer integrator with hardware rate limits."""

    def __init__(
        self,
        wheelbase: float = RobotSpecs.WHEELBASE,
        max_steer: float = RobotSpecs.MAX_STEERING_ANGLE,
        max_steer_rate: float = _DEFAULT_MAX_STEER_RATE,
        max_accel: float = _DEFAULT_MAX_ACCEL,
        substeps: int = 5,
        rear_steer_ratio: float = _DEFAULT_REAR_STEER_RATIO,
        max_speed_mps: float = _DEFAULT_MAX_SPEED_MPS,
    ) -> None:
        self._max_speed = max_speed_mps
        self._wheelbase = wheelbase
        self._max_steer = max_steer
        self._max_steer_rate = max_steer_rate
        self._max_accel = max_accel
        self._substeps = max(1, substeps)
        self._rear_steer_ratio = rear_steer_ratio
        # Effective turn length: the yaw rate is v/L_eff * tan(steer). Front-only
        # steering pivots about the rear axle (L_eff = L); counter-phase steering
        # with equal angles pivots about the chassis centre (L_eff = L/2), i.e.
        # twice the yaw rate for the same steering angle.
        self._turn_reference_len = wheelbase / (1.0 + abs(rear_steer_ratio))

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
            v = _approach(v, _clamp(target_speed, -self._max_speed, self._max_speed), self._max_accel * h)

            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += (v / self._turn_reference_len) * math.tan(steer) * h

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
