"""Wheel odometry reaches the navigation port, in both adapters.

Nothing consumes this yet -- it is the data path only, landed separately from
the localizer change that will use it so that a pass-rate movement can be
attributed to one or the other. What it enables immediately is measuring
commanded speed against actual, which is the wheel-slip and actuation error
the simulator does not model and the robot could not previously observe.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs
from shared.config.enums import Direction, Section

from src.navigation.ports import DriveCommand, WheelOdometry
from src.simulation.gateway import SimulatedHardwareGateway
from src.simulation.kinematics import AckermannState
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.track_model import TrackModel

_WIDE_MM = 1000
_DT = 0.05


def _gateway() -> SimulatedHardwareGateway:
    metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
    from src.navigation.track_geometry import corridor_widths_from_metadata

    track = TrackModel(corridor_widths_from_metadata(metadata.model_dump()))
    return SimulatedHardwareGateway(
        track=track,
        initial_state=AckermannState(x=1.5, y=0.45, yaw=0.0),
    )


def _drive(gw: SimulatedHardwareGateway, speed: float, ticks: int) -> None:
    gw.publish_drive(DriveCommand(speed_mps=speed, steering_norm=0.0))
    for _ in range(ticks):
        gw.advance(_DT)


class TestSimulatedWheelOdometry:
    def test_starts_at_zero(self) -> None:
        odom = _gateway().get_wheel_odometry()
        assert odom.distance_m == pytest.approx(0.0)
        assert odom.speed_mps == pytest.approx(0.0)

    def test_distance_accumulates_while_driving(self) -> None:
        gw = _gateway()
        _drive(gw, 0.15, 40)
        assert gw.get_wheel_odometry().distance_m > 0.2

    def test_distance_is_path_length_not_displacement(self) -> None:
        """An encoder counts wheel rotation, so a turn still accrues distance.

        Displacement from the start would under-report on any curved path and
        read zero on a closed loop.
        """
        gw = _gateway()
        gw.publish_drive(DriveCommand(speed_mps=0.15, steering_norm=1.0))
        for _ in range(120):
            gw.advance(_DT)
        state = gw.state
        displacement = math.hypot(state.x - 1.5, state.y - 0.45)
        assert gw.get_wheel_odometry().distance_m > displacement

    def test_reversing_still_adds_distance(self) -> None:
        """Wheel travel is unsigned; direction lives in the speed sign."""
        gw = _gateway()
        _drive(gw, 0.15, 20)
        forward = gw.get_wheel_odometry().distance_m
        _drive(gw, -0.15, 20)
        assert gw.get_wheel_odometry().distance_m > forward

    def test_speed_is_signed(self) -> None:
        gw = _gateway()
        _drive(gw, -0.15, 20)
        assert gw.get_wheel_odometry().speed_mps < 0

    def test_a_stationary_robot_accrues_nothing(self) -> None:
        gw = _gateway()
        _drive(gw, 0.0, 20)
        assert gw.get_wheel_odometry().distance_m == pytest.approx(0.0)

    def test_stamp_advances_with_the_clock(self) -> None:
        """Integrating between LIDAR scans is meaningless without sample times."""
        gw = _gateway()
        first = gw.get_wheel_odometry().stamp_s
        _drive(gw, 0.15, 10)
        assert gw.get_wheel_odometry().stamp_s == pytest.approx(first + 10 * _DT)


class TestCommandedVersusActual:
    """What this data path buys immediately, before anything consumes it."""

    def test_commanded_speed_saturates_at_the_plant_limit(self) -> None:
        """Ask for more than the motor can do and the encoder shows the truth.

        On hardware this same comparison is wheel slip and actuation error --
        neither of which the simulator models, and neither of which the robot
        could observe until the encoder reached the navigator.
        """
        gw = _gateway()
        _drive(gw, 5.0, 40)
        assert gw.last_command.speed_mps == pytest.approx(5.0)
        assert abs(gw.get_wheel_odometry().speed_mps) < 0.2


class TestWheelOdometryModel:
    def test_wheel_angle_converts_to_distance_by_radius(self) -> None:
        """One full wheel revolution is one circumference of travel."""
        radius = RobotSpecs.WHEEL_RADIUS
        one_rev = WheelOdometry(distance_m=2 * math.pi * radius, speed_mps=0.0, stamp_s=0.0)
        assert one_rev.distance_m == pytest.approx(2 * math.pi * radius)

    def test_is_not_a_pose(self) -> None:
        """Guards the design decision: no x/y/yaw on this type.

        The encoder cannot know heading, so a pose here would be dead reckoning
        computed by the component with the least information, competing with
        the localizer's answer.
        """
        fields = WheelOdometry.__dataclass_fields__
        assert set(fields) == {"distance_m", "speed_mps", "stamp_s"}
