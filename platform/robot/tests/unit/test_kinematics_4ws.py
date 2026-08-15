"""Counter-phase four-wheel steering invariants.

Both axles steer, by the same angle in opposite directions -- confirmed on the
real chassis 2026-07-25, and the 1:1 front-to-rear angle re-verified by hand
2026-08-15. That is not the textbook bicycle model, and the difference is not
subtle: the instantaneous centre of rotation moves from the rear axle to the
chassis centre, so the robot yaws TWICE as fast at the same steering angle.

These are regression tests rather than derivations, because this exact property
has already been wrong in shipped code once: the simulator modelled a front-steer
car, turned half as sharply as the hardware, and every gain fitted against it was
hotter on the real robot than in sim. Nothing failed at the time -- the sim was
simply a different vehicle. There was no test covering it until this file.

The geometry that the Obstacles sign-clearance analysis rests on
(docs/sign-avoidance-investigation.md) assumes all of the below: a centre
reference point, a symmetric footprint, and rear swing-out equal to nose swing-in.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs

from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import _rect_corners

_DT = 0.05
"""The 20 Hz control interval the simulator runs at."""

_STEER_NORM = 0.5
"""Mid-range steering: away from the saturation limit, so the rate limiter and
the angle clamp are not what is being measured."""


def _radius_of_curvature(kin: AckermannKinematics, steer_norm: float) -> float:
    """Steady-state turn radius, measured by integrating rather than asserted.

    Driven to steady state first: ``step`` slews the servo toward the commanded
    angle at ``max_steer_rate`` and accelerates toward the commanded speed, so
    the first ticks are a transient that would understate the curvature.
    """
    state = AckermannState(x=0.0, y=0.0, yaw=0.0)
    for _ in range(200):
        state = kin.step(state, target_speed=0.1, target_steer_norm=steer_norm, dt=_DT)

    yaw_before, v = state.yaw, state.v
    state = kin.step(state, target_speed=0.1, target_steer_norm=steer_norm, dt=_DT)
    yaw_rate = (state.yaw - yaw_before) / _DT
    return v / yaw_rate


class TestCounterPhaseDoublesTheYawRate:
    """The headline property, and the one that regressed before."""

    def test_counter_phase_turns_twice_as_sharply_as_front_steer(self):
        counter = AckermannKinematics(rear_steer_ratio=1.0)
        front_only = AckermannKinematics(rear_steer_ratio=0.0)

        assert _radius_of_curvature(counter, _STEER_NORM) == pytest.approx(
            _radius_of_curvature(front_only, _STEER_NORM) / 2, rel=1e-3
        )

    def test_turn_radius_follows_the_effective_wheelbase(self):
        """radius = L_eff / tan(steer), with L_eff = wheelbase / (1 + rear_ratio)."""
        for ratio in (0.0, 0.5, 1.0):
            kin = AckermannKinematics(rear_steer_ratio=ratio)
            expected_l_eff = RobotSpecs.WHEELBASE / (1.0 + ratio)
            steer = _STEER_NORM * RobotSpecs.MAX_STEERING_ANGLE

            assert _radius_of_curvature(kin, _STEER_NORM) == pytest.approx(
                expected_l_eff / math.tan(steer), rel=1e-3
            ), f"rear_steer_ratio={ratio}"

    def test_shipped_ratio_is_full_counter_phase(self):
        """1.0, hand-verified on the chassis: rear angle equals front angle.

        Guarded because it is a physical fact about the car, not a tunable --
        anything else silently changes the rotation centre, and with it every
        clearance number in the sign-avoidance analysis.
        """
        assert RobotSpecs.REAR_STEER_RATIO == 1.0


class TestRotationIsAboutTheChassisCentre:
    """Why (x, y) is the centre, and why the tail swings as far as the nose."""

    def test_the_collision_rectangle_is_centred_on_the_pose(self):
        corners = _rect_corners(0.0, 0.0, 0.0, RobotSpecs.LENGTH, RobotSpecs.WIDTH)

        xs = [x for x, _ in corners]
        ys = [y for _, y in corners]
        assert min(xs) == pytest.approx(-RobotSpecs.LENGTH / 2)
        assert max(xs) == pytest.approx(+RobotSpecs.LENGTH / 2)
        assert min(ys) == pytest.approx(-RobotSpecs.WIDTH / 2)
        assert max(ys) == pytest.approx(+RobotSpecs.WIDTH / 2)

    def test_the_tail_swings_out_as_far_as_the_nose_swings_in(self):
        """The property that constrains "pass the sign square, then turn".

        With a centre-referenced body, rotating in place sweeps the rear corner
        outward by exactly what the front corner sweeps inward. Clearing a sign
        with the nose therefore does NOT clear the vehicle -- unlike a
        front-steer car, whose rear tracks inside the front's path.
        """
        yaw = math.radians(30)
        corners = _rect_corners(0.0, 0.0, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)

        lateral = sorted(y for _, y in corners)
        assert lateral[0] == pytest.approx(-lateral[-1]), "footprint is not symmetric about the pose"

    def test_lateral_half_extent_matches_the_clearance_formula(self):
        """(L/2)|sin th| + (W/2)|cos th| -- the basis of the 28 deg yaw budget.

        Derived independently here from the rectangle the collision checker
        actually uses, so the analysis and the simulator cannot drift apart.
        """
        for degrees in (0, 20, 28, 40, 60, 84):
            yaw = math.radians(degrees)
            corners = _rect_corners(0.0, 0.0, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
            measured = max(y for _, y in corners)
            predicted = (RobotSpecs.LENGTH / 2) * abs(math.sin(yaw)) + (RobotSpecs.WIDTH / 2) * abs(math.cos(yaw))

            assert measured == pytest.approx(predicted), f"at {degrees} deg"
