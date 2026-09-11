"""Counter-phase four-wheel-steer kinematics for the headless car simulation.

Integrates the same motion a real vTitan chassis produces from a
``(linear_speed, steering)`` command, honouring the physical limits the actual
hardware imposes:

* **Steering slew** — the servo cannot snap to an angle instantly; it ramps at
  ``MAX_STEERING_RATE`` (rad/s).
* **Drive acceleration** — the drive motor cannot change speed instantly; it is
  clamped to ``max_accel`` (m/s²), the physical acceleration limit of the drive motor.
* **Drive lag** — and it does not track the command even within that clamp: it
  approaches a new setpoint as a first-order lag with time constant
  ``speed_tau_s``. Measured 2026-08-29; see below.
* **Steering limit** — front-wheel angle saturates at ``MAX_STEERING_ANGLE``.
* **Yaw gain** — the geometry above is zero-slip, and the real chassis is not.
  ``yaw_gain`` scales the predicted yaw rate by the fraction actually
  delivered. Measured 2026-08-29; see below.

The last two exist because this model was written from first principles and
first checked against a bag on 2026-08-29 (``run_20260829_140424``, via
``scripts/bag/diag_bag_sim_fidelity.py``). Replaying that run's own command
stream through this integrator produced 3512° of yaw against the IMU's 1918° --
the simulator cornered 1.83x harder than the hardware it was standing in for,
and reached commanded speed in a fraction of the ~0.35 s the drivetrain takes.
Both gaps flattered the robot, so any tuning validated only in sim before that
date was validated against a car that turns better than the real one.

**Both axles steer, in opposite directions and by the same amount** -- confirmed
on the real chassis 2026-07-25. That is not the textbook bicycle model, and the
difference is not subtle: counter-phase steering moves the instantaneous centre
of rotation from the rear axle to the chassis centre, so the robot yaws *twice
as fast* as a front-steer car at the same steering angle::

    x += v * cos(yaw) * dt
    y += v * sin(yaw) * dt
    yaw += yaw_gain * (v / L_eff) * tan(steer) * dt  # L_eff = wheelbase / (1 + rear_ratio)

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
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.utils import (
    clamp as _clamp,
    wrap_angle as _wrap_angle,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning


@dataclass(frozen=True, slots=True)
class _KinematicsConstants:
    """Tuning-derived kinematics constants, computed on-demand instead of frozen at module level."""

    max_steer_rate: float
    max_accel: float
    max_speed_mps: float
    rear_steer_ratio: float
    speed_tau_s: float
    yaw_gain: float
    min_turn_radius_m: float
    radius_tracks_speed: bool

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning | None = None) -> _KinematicsConstants:
        tuning = get_tuning(tuning)
        return cls(
            max_steer_rate=tuning.pursuit.MAX_STEERING_RATE,
            max_accel=RobotSpecs.MAX_ACCEL_MPS2,
            max_speed_mps=RobotSpecs.MAX_SPEED_MPS,
            rear_steer_ratio=RobotSpecs.REAR_STEER_RATIO,
            speed_tau_s=RobotSpecs.SPEED_RESPONSE_TAU_S,
            yaw_gain=RobotSpecs.YAW_GAIN,
            min_turn_radius_m=get_tuning(tuning).simulation.MIN_TURN_RADIUS_M,
            radius_tracks_speed=get_tuning(tuning).simulation.MIN_TURN_RADIUS_TRACKS_SPEED,
        )


class KinematicsContext(TuningContext[_KinematicsConstants]):
    """Context holding tuning-derived kinematics constants."""

    _constants_cls = _KinematicsConstants


_DEFAULT_KINEMATICS_CONTEXT = KinematicsContext()


@dataclass(frozen=True, slots=True)
class AckermannState:
    """Full kinematic state of the simulated car."""

    x: float
    y: float
    yaw: float
    v: float = 0.0  # current linear speed (m/s)
    steer: float = 0.0  # current front-wheel angle (radians)


@dataclass(frozen=True, slots=True)
class WheelPose:
    """One road wheel's chassis-frame mount point and its own steer angle.

    ``name`` matches the corresponding URDF link in
    ``apps/gazebo/runtime/robot_description/wro_robot.urdf.xacro`` so the
    two descriptions of the same wheel can be lined up by eye.
    """

    name: str
    x: float  # +x forward, from the chassis centre
    y: float  # +y left, from the chassis centre
    steer: float  # this wheel's own angle (radians, +ve = turning left)


def wheel_poses(
    steer: float,
    wheelbase: float = RobotSpecs.WHEELBASE,
    track_width: float = RobotSpecs.TRACK_WIDTH,
    rear_steer_ratio: float | None = None,
    context: KinematicsContext | None = None,
) -> tuple[WheelPose, WheelPose, WheelPose, WheelPose]:
    """Place the four road wheels for a bicycle-equivalent front angle.

    Both wheels on an axle carry the SAME angle. The vTitan turns each axle with
    one servo through one linkage, and :class:`AckermannKinematics` integrates a
    single ``steer``, so there is no inner/outer Ackermann differential to
    render here -- deriving one from the instantaneous centre of rotation would
    draw geometry neither the chassis nor the model has.

    The rear axle is the front angle negated and scaled by ``rear_steer_ratio``.
    That counter-phase is the whole reason this chassis yaws twice as fast as a
    front-steer car at the same angle (see the module docstring), and it is
    exactly what a single rigid box in RViz cannot show.

    Coordinates are relative to the chassis centre, matching
    :class:`AckermannState`'s reference point.
    """
    if rear_steer_ratio is None:
        rear_steer_ratio = (context or _DEFAULT_KINEMATICS_CONTEXT).constants.rear_steer_ratio
    rear_steer = -steer * rear_steer_ratio
    half_wheelbase = wheelbase / 2.0
    half_track = track_width / 2.0
    return (
        WheelPose("front_left_wheel", half_wheelbase, half_track, steer),
        WheelPose("front_right_wheel", half_wheelbase, -half_track, steer),
        WheelPose("rear_left_wheel", -half_wheelbase, half_track, rear_steer),
        WheelPose("rear_right_wheel", -half_wheelbase, -half_track, rear_steer),
    )


class AckermannKinematics:
    """Sub-stepped counter-phase four-wheel-steer integrator with hardware rate limits."""

    def __init__(
        self,
        wheelbase: float = RobotSpecs.WHEELBASE,
        max_steer: float = RobotSpecs.MAX_STEERING_ANGLE,
        max_steer_rate: float | None = None,
        max_accel: float | None = None,
        substeps: int = 5,
        rear_steer_ratio: float | None = None,
        max_speed_mps: float | None = None,
        speed_tau_s: float | None = None,
        yaw_gain: float | None = None,
        min_turn_radius_m: float | None = None,
        radius_tracks_speed: bool | None = None,
        context: KinematicsContext | None = None,
    ) -> None:
        if context is None:
            context = _DEFAULT_KINEMATICS_CONTEXT
        c = context.constants

        if max_steer_rate is None:
            max_steer_rate = c.max_steer_rate
        if max_accel is None:
            max_accel = c.max_accel
        if rear_steer_ratio is None:
            rear_steer_ratio = c.rear_steer_ratio
        if max_speed_mps is None:
            max_speed_mps = c.max_speed_mps
        if speed_tau_s is None:
            speed_tau_s = c.speed_tau_s
        if yaw_gain is None:
            yaw_gain = c.yaw_gain
        if min_turn_radius_m is None:
            min_turn_radius_m = c.min_turn_radius_m
        if radius_tracks_speed is None:
            radius_tracks_speed = c.radius_tracks_speed

        self._speed_tau_s = speed_tau_s
        self._yaw_gain = yaw_gain
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
        self._min_turn_radius_m = min_turn_radius_m
        self._radius_tracks_speed = radius_tracks_speed

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
            # Drive response: a first-order lag toward the setpoint, then the
            # acceleration clamp on top. Ordered that way because they model
            # different things -- the lag is how this drivetrain habitually
            # answers a command, the clamp is a ceiling it may not cross -- and
            # a lag that produced an impossible acceleration would still be
            # impossible.
            setpoint = _clamp(target_speed, -self._max_speed, self._max_speed)
            lagged = setpoint if self._speed_tau_s <= 0.0 else v + (setpoint - v) * min(h / self._speed_tau_s, 1.0)
            v = _approach(v, lagged, self._max_accel * h)

            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            # Curvature, floored by the chassis's MINIMUM TURN RADIUS. The
            # bicycle term alone has no floor: at the shipped 85 deg lock it
            # gives L_eff / (tan(85) * yaw_gain) = 1.5 cm of radius, which a
            # 30 x 19.4 cm four-wheeled chassis cannot do.
            #
            # Measured from `/joint_states` drive-wheel travel against pose yaw
            # over five hardware bags, the real radius SATURATES:
            #
            #   |steer|   effective R   model R    ratio
            #    15-30       66.0 cm     41.7 cm    1.6x
            #    30-45       38.2 cm     22.5 cm    1.7x
            #    75-90       28.9 cm      2.3 cm   12.7x
            #
            # so past ~30 deg the real car buys almost nothing while the model
            # keeps rewarding lock. Without this floor every full-lock manoeuvre
            # in simulation is optimistic by more than an order of magnitude --
            # the in-bay exit completes in a deterministic 91 ticks in sim where
            # hardware takes 5-44 s and once managed 2.2 deg in 44.1 s.
            #
            # AND THE FLOOR IS NOT A CONSTANT. Re-measured 2026-09-10 over 33
            # bags with IMU yaw, the achieved radius rises with speed and then
            # saturates -- 0.105 m at 0.025 m/s, 0.298 at 0.132, 0.43 above
            # 0.22 -- so 0.29 is the curve's value at ~0.118 m/s, near corridor
            # speed. The bay exit creeps, and there the constant is nearly 2x
            # too large. `v` is the SUBSTEP's speed, so the floor tracks the
            # chassis through an acceleration rather than being fixed per call.
            # Off by default; see `MIN_TURN_RADIUS_TRACKS_SPEED`.
            floor = self._min_turn_radius_m
            if self._radius_tracks_speed:
                floor = min(
                    RobotSpecs.MIN_TURN_RADIUS_CAP_M,
                    RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M + RobotSpecs.MIN_TURN_RADIUS_SLOPE_S * abs(v),
                )
            curvature = math.tan(steer) * self._yaw_gain / self._turn_reference_len
            if floor > 0.0:
                limit = 1.0 / floor
                curvature = _clamp(curvature, -limit, limit)
            yaw += curvature * v * h

        yaw = _wrap_angle(yaw)
        return replace(state, x=x, y=y, yaw=yaw, v=v, steer=steer)


def _approach(current: float, target: float, max_delta: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_delta``."""
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target
