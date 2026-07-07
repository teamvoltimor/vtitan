"""Contract tests for the shared steering-command mapping.

``DriveCommand.steering_norm`` carries a normalised steering command in
``[-1, 1]``. The controller, the simulator, and the Build HAT motor adapter must all agree on how
that maps to a physical wheel angle — otherwise the hardware steers by a
different amount than every simulation proved safe. These tests pin the mapping
and guard against the "treated 0.5 as 0.5 radians" regression.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs
from shared.domain.steering import (
    angle_rad_to_steering_norm,
    steering_norm_to_angle_rad,
)

from src.simulation.kinematics import AckermannKinematics, AckermannState

MAX = RobotSpecs.MAX_STEERING_ANGLE


@pytest.mark.parametrize("norm", [-1.0, -0.5, 0.0, 0.25, 0.5, 1.0])
def test_decode_matches_simulator(norm):
    """The actuator decode equals the wheel angle the simulator integrates."""
    # Rate/accel limits removed and a full-second step so the servo reaches its
    # target angle in one integration, exposing the sim's internal decode.
    kin = AckermannKinematics(max_steer=MAX, max_steer_rate=1e9, max_accel=1e9, substeps=1)
    state = kin.step(
        AckermannState(x=0.0, y=0.0, yaw=0.0),
        target_speed=0.0,
        target_steer_norm=norm,
        dt=1.0,
    )
    assert state.steer == pytest.approx(steering_norm_to_angle_rad(norm, MAX))


def test_full_command_is_max_angle():
    assert steering_norm_to_angle_rad(1.0, MAX) == pytest.approx(MAX)
    assert steering_norm_to_angle_rad(-1.0, MAX) == pytest.approx(-MAX)


def test_out_of_range_is_clamped():
    assert steering_norm_to_angle_rad(2.5, MAX) == pytest.approx(MAX)
    assert steering_norm_to_angle_rad(-2.5, MAX) == pytest.approx(-MAX)


def test_half_command_is_half_angle_not_half_radian():
    # Regression: the motor node treated 0.5 as 0.5 rad (~28.6 deg). The contract
    # says 0.5 is half of max steering (~15 deg).
    angle_deg = math.degrees(steering_norm_to_angle_rad(0.5, MAX))
    assert angle_deg == pytest.approx(15.0, abs=0.5)
    assert math.degrees(0.5) == pytest.approx(28.6, abs=0.5)  # the old buggy value


@pytest.mark.parametrize("norm", [-1.0, -0.3, 0.0, 0.3, 1.0])
def test_round_trip(norm):
    angle = steering_norm_to_angle_rad(norm, MAX)
    assert angle_rad_to_steering_norm(angle, MAX) == pytest.approx(norm)
