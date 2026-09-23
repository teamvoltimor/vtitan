"""Counter-phase four-wheel steering invariants.

Both axles steer, by the same angle in opposite directions -- confirmed on the
real chassis, with the 1:1 front-to-rear angle re-verified by hand. That is not
the textbook bicycle model, and the difference is not
subtle: the instantaneous centre of rotation moves from the rear axle to the
chassis centre, so the robot yaws TWICE as fast at the same steering angle.

These are regression tests rather than derivations, because this exact property
has already been wrong in shipped code once: the simulator modelled a front-steer
car, turned half as sharply as the hardware, and every gain fitted against it was
hotter on the real robot than in sim. Nothing failed at the time -- the sim was
simply a different vehicle. There was no test covering it until this file.

The geometry that the Obstacles sign-clearance analysis rests on
(adr:0051-sign-lane-planner) assumes all of the below: a centre
reference point, a symmetric footprint, and rear swing-out equal to nose swing-in.
See adr:0076-drivetrain-and-steering-hardware.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs

from src.simulation.kinematics import AckermannKinematics, AckermannState, wheel_poses
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
        counter = AckermannKinematics(rear_steer_ratio=1.0, min_turn_radius_m=0.0)
        front_only = AckermannKinematics(rear_steer_ratio=0.0, min_turn_radius_m=0.0)

        assert _radius_of_curvature(counter, _STEER_NORM) == pytest.approx(
            _radius_of_curvature(front_only, _STEER_NORM) / 2, rel=1e-3
        )

    def test_turn_radius_follows_the_effective_wheelbase(self):
        """radius = L_eff / tan(steer), with L_eff = wheelbase / (1 + rear_ratio).

        ``yaw_gain`` is pinned to 1.0 because this asserts the GEOMETRY. The
        shipped gain is a measured slip factor sitting on top of it (see
        :class:`TestMeasuredDeparturesFromTheIdealModel`); leaving it in would
        make a geometry regression and a re-measured tyre look identical here.

        ``min_turn_radius_m`` is pinned to 0.0 for the same reason. The shipped
        floor (``simulation.MIN_TURN_RADIUS_M`` 0.29, the MEASURED saturation of
        the real chassis) clamps the curvature this formula predicts -- at the
        steer used here the ideal geometry gives 0.207 m and the floor returns
        0.29 -- so leaving it on would assert the clamp rather than the model.
        """
        for ratio in (0.0, 0.5, 1.0):
            kin = AckermannKinematics(rear_steer_ratio=ratio, yaw_gain=1.0, min_turn_radius_m=0.0)
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


class TestMeasuredDeparturesFromTheIdealModel:
    """The two terms added when the model was first checked against a bag.

    Replaying a bag's own commands through the integrator produced far more yaw
    than the IMU measured, and reached commanded speed far sooner than the
    drivetrain does. Both errors flattered the robot, which is the direction
    that matters: a sim that corners better than the car certifies tuning the
    car cannot execute. Re-measure with ``scripts/bag/diag_bag_sim_fidelity.py``.
    See adr:0086-simulator-realism.
    """

    def test_yaw_gain_scales_the_turn_radius_inversely(self):
        """Half the yaw for the same speed and angle is twice the radius.

        The turn-radius floor is off here: a clamp would cap BOTH arms at 0.29 m
        and the doubling this asserts would vanish into it.
        """
        ideal = AckermannKinematics(yaw_gain=1.0, min_turn_radius_m=0.0)
        slipping = AckermannKinematics(yaw_gain=0.5, min_turn_radius_m=0.0)

        assert _radius_of_curvature(slipping, _STEER_NORM) == pytest.approx(
            _radius_of_curvature(ideal, _STEER_NORM) * 2, rel=1e-3
        )

    def test_the_shipped_gain_is_below_one(self):
        """A measured slip factor, not a tunable -- 1.0 would be the zero-slip model back."""
        assert 0.0 < RobotSpecs.YAW_GAIN < 1.0

    def test_the_drivetrain_lags_a_step_by_its_time_constant(self):
        """One tau after a step, a first-order lag has closed ~63% of it.

        ``max_speed_mps`` is pinned here for the same reason ``max_accel`` is:
        this measures the LAG, so every other limiter has to be taken out of
        the way. Left unset it inherits the profile ceiling, and conftest pins
        the retired generic-motor-1500rpm at 0.156 m/s -- below the 0.3 target,
        so the step was clamped and the test measured 63% of the wrong number.
        """
        tau = 0.4
        kin = AckermannKinematics(speed_tau_s=tau, max_accel=1e6, max_speed_mps=1e6)
        target = 0.3

        state = AckermannState(x=0.0, y=0.0, yaw=0.0)
        for _ in range(round(tau / _DT)):
            state = kin.step(state, target_speed=target, target_steer_norm=0.0, dt=_DT)

        assert state.v == pytest.approx(target * 0.632, rel=0.05)

    def test_zero_tau_reproduces_the_old_instant_response(self):
        """What the retired motor's profile ships, so its recorded results stay comparable.

        ``max_speed_mps`` pinned for the same reason as the test above: the
        assertion is that zero tau reaches target in ONE step, which the
        profile's 0.156 m/s ceiling silently contradicted for a 0.3 target.
        """
        kin = AckermannKinematics(speed_tau_s=0.0, max_accel=1e6, max_speed_mps=1e6)

        state = kin.step(AckermannState(x=0.0, y=0.0, yaw=0.0), target_speed=0.3, target_steer_norm=0.0, dt=_DT)

        assert state.v == pytest.approx(0.3)

    def test_the_acceleration_clamp_still_bounds_the_lag(self):
        """The lag says "late", the clamp says "never faster than this" -- both apply.

        Ordering matters: a large step early in the lag asks for an acceleration
        the motor cannot produce, and the clamp is what refuses it.
        """
        kin = AckermannKinematics(speed_tau_s=0.01, max_accel=0.5)

        state = kin.step(AckermannState(x=0.0, y=0.0, yaw=0.0), target_speed=1.0, target_steer_norm=0.0, dt=_DT)

        assert state.v == pytest.approx(0.5 * _DT, rel=1e-6)


def _steady_radius_at(kin: AckermannKinematics, speed: float) -> float:
    """Steady-state |radius| at full lock and a SIGNED speed, integrated like above."""
    state = AckermannState(x=0.0, y=0.0, yaw=0.0)
    for _ in range(200):
        state = kin.step(state, target_speed=speed, target_steer_norm=1.0, dt=_DT)
    yaw_before, v = state.yaw, state.v
    state = kin.step(state, target_speed=speed, target_steer_norm=1.0, dt=_DT)
    return abs(v / ((state.yaw - yaw_before) / _DT))


class TestTheTurnRadiusFloorIsDirectional:
    """The speed curve's cap differs by direction, and the difference is measured.

    Split by the encoder's sign over run_20260915_140852/141413/140358, the
    chassis at 30-45 deg holds R 0.27-0.33 m forward at 0.15-0.21 m/s, which
    the 0.35 m forward cap reproduces. In REVERSE it yaws ~1.0 rad/s at every
    speed, so R grows with v (0.08 m at 0.08 m/s, 0.19 at 0.18, 0.24 at 0.25):
    a pivot the radius model cannot express. 0.20 m is the fit at the escape's
    own 0.18-0.20 m/s, where the forward cap made the simulator hold 0.32-0.34
    and rotate half of what the IMU records.
    """

    def test_reverse_holds_a_tighter_radius_than_forward_at_the_same_speed(self):
        kin = AckermannKinematics(min_turn_radius_m=RobotSpecs.MIN_TURN_RADIUS_M, radius_tracks_speed=True, max_speed_mps=1e6)
        forward = _steady_radius_at(kin, 0.2)
        reverse = _steady_radius_at(kin, -0.2)

        # At 0.2 m/s the linear term is 0.053 + 1.86 * 0.2 = 0.42 m, above both
        # caps, so each direction reads its own cap and nothing else.
        assert forward == pytest.approx(RobotSpecs.MIN_TURN_RADIUS_CAP_M, rel=1e-2)
        assert reverse == pytest.approx(RobotSpecs.MIN_TURN_RADIUS_REVERSE_CAP_M, rel=1e-2)
        assert reverse < forward

    def test_the_shipped_reverse_cap_is_the_measured_one(self):
        """0.20 m is the measured fit at escape speed (three rounds, n=125..2422), not a tunable."""
        assert RobotSpecs.MIN_TURN_RADIUS_REVERSE_CAP_M == pytest.approx(0.20)
        assert RobotSpecs.MIN_TURN_RADIUS_REVERSE_CAP_M < RobotSpecs.MIN_TURN_RADIUS_CAP_M

    def test_a_disabled_floor_stays_disabled_in_reverse(self):
        """``min_turn_radius_m = 0`` asked for no clamp; the direction switch must not resurrect one."""
        kin = AckermannKinematics(min_turn_radius_m=0.0, radius_tracks_speed=True, yaw_gain=1.0, max_speed_mps=1e6)
        ideal = (RobotSpecs.WHEELBASE / (1.0 + RobotSpecs.REAR_STEER_RATIO)) / math.tan(RobotSpecs.MAX_STEERING_ANGLE)
        assert _steady_radius_at(kin, -0.2) == pytest.approx(ideal, rel=1e-2)


class TestWheelPosesShowTheCounterPhase:
    """``wheel_poses`` exists so RViz can draw what this file asserts.

    The live visualizer once drew the robot as a single rigid box, so the
    property this whole module is about -- the two axles turning against each
    other -- was the one thing you could not see while watching a run. See
    adr:0076-drivetrain-and-steering-hardware.
    """

    def test_the_axles_steer_in_opposite_directions(self):
        front_left, front_right, rear_left, rear_right = wheel_poses(math.radians(20))

        assert front_left.steer == pytest.approx(math.radians(20))
        assert rear_left.steer == pytest.approx(-math.radians(20))
        assert front_right.steer == pytest.approx(front_left.steer), "one servo per axle"
        assert rear_right.steer == pytest.approx(rear_left.steer), "one servo per axle"

    def test_the_rear_angle_scales_with_the_ratio(self):
        for ratio in (0.0, 0.5, 1.0):
            *_, rear_left, _ = wheel_poses(math.radians(20), rear_steer_ratio=ratio)
            assert rear_left.steer == pytest.approx(-math.radians(20) * ratio), f"ratio={ratio}"

    def test_the_wheels_sit_on_the_measured_axle_geometry(self):
        """Straight from robot.toml, and centre-referenced like AckermannState."""
        front_left, front_right, rear_left, rear_right = wheel_poses(0.0)

        assert front_left.x == pytest.approx(+RobotSpecs.WHEELBASE / 2)
        assert rear_left.x == pytest.approx(-RobotSpecs.WHEELBASE / 2)
        assert front_left.y == pytest.approx(+RobotSpecs.TRACK_WIDTH / 2)
        assert front_right.y == pytest.approx(-RobotSpecs.TRACK_WIDTH / 2)
        assert rear_right.y == pytest.approx(-RobotSpecs.TRACK_WIDTH / 2)
        assert all(w.steer == 0.0 for w in (front_left, front_right, rear_left, rear_right))

    def test_the_names_match_the_urdf_links(self):
        """So the marker view and wro_robot.urdf.xacro can be lined up by eye."""
        assert [w.name for w in wheel_poses(0.0)] == [
            "front_left_wheel",
            "front_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
        ]


class TestRotationIsAboutTheChassisCentre:
    """Why (x, y) is the centre, and why the tail swings as far as the nose."""

    def test_the_collision_rectangle_is_centred_on_the_pose(self):
        corners = _rect_corners(0.0, 0.0, 0.0, RobotSpecs.LENGTH, RobotSpecs.WIDTH)

        xs = [c.x for c in corners]
        ys = [c.y for c in corners]
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

        lateral = sorted(c.y for c in corners)
        assert lateral[0] == pytest.approx(-lateral[-1]), "footprint is not symmetric about the pose"

    def test_lateral_half_extent_matches_the_clearance_formula(self):
        """(L/2)|sin th| + (W/2)|cos th| -- the basis of the 28 deg yaw budget.

        Derived independently here from the rectangle the collision checker
        actually uses, so the analysis and the simulator cannot drift apart.
        """
        for degrees in (0, 20, 28, 40, 60, 84):
            yaw = math.radians(degrees)
            corners = _rect_corners(0.0, 0.0, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
            measured = max(c.y for c in corners)
            predicted = (RobotSpecs.LENGTH / 2) * abs(math.sin(yaw)) + (RobotSpecs.WIDTH / 2) * abs(math.cos(yaw))

            assert measured == pytest.approx(predicted), f"at {degrees} deg"
