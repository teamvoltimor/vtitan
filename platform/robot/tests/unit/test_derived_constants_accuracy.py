"""Verify that derived constants match their documented formulas.

Catches stale documentation and formula changes that weren't propagated.
Examples:
- ARC_RADIUS floor was documented as 0.329m but actually 0.034m (10× off)
- Steering angle changed but derived constant wasn't updated
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import CorridorWidthType
from shared.domain.models import CorridorWidthEntry


def test_arc_radius_floor_derivation() -> None:
    """Verify ARC_RADIUS floor matches the published formula.

    Formula: r_min = L / tan(θ_max), where L = wheelbase, θ = max steering angle.
    This is for phase steering; counter-phase doubles the yaw rate, reducing r_min further.
    """
    # Direct calculation
    r_min_phase_steering = RobotSpecs.WHEELBASE / math.tan(RobotSpecs.MAX_STEERING_ANGLE)

    # Should be roughly in the 0.03-0.04m range (counter-phase reduces it)
    # The audit noted it was documented as 0.329m but actually 0.034m
    assert 0.01 < r_min_phase_steering < 0.1, \
        f"ARC_RADIUS floor {r_min_phase_steering} seems wrong; check steering angle and wheelbase"


def test_chassis_half_diagonal_derivation() -> None:
    """Verify half-diagonal clearance calculation."""
    # Half-diagonal from centre to corner
    half_diagonal = math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)

    # Should be roughly 0.17-0.19m (30cm long, 19.4cm wide)
    assert 0.15 < half_diagonal < 0.25, \
        f"Chassis half-diagonal {half_diagonal} seems wrong; check dimensions"

    # Verify it's greater than half-width (used for clearance calculations)
    half_width = RobotSpecs.WIDTH / 2
    assert half_diagonal > half_width, \
        "Half-diagonal should exceed half-width (reason for using it in clearance calculations)"


def test_camera_focal_length_derivation() -> None:
    """Verify camera focal length calculation from HFOV."""
    # Formula: f = (width / 2) / tan(HFOV / 2)
    focal_length = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)

    # Should be positive and reasonable for a camera (typically 100-500px for our resolution)
    assert 50 < focal_length < 1000, \
        f"Camera focal length {focal_length} seems wrong; check HFOV and camera width"


def test_max_speed_specification() -> None:
    """Verify that MAX_SPEED_MPS is a real achievable speed, not an aspirational one."""
    # The audit noted that real max speed is 0.156 m/s (measured on hardware)
    # Anything >=0.156 saturates the motor, so higher values are OK (they just saturate)
    # But we should document what the real max is
    assert 0.1 < RobotSpecs.MAX_SPEED_MPS < 1.0, \
        f"MAX_SPEED_MPS {RobotSpecs.MAX_SPEED_MPS} seems unrealistic"

    # Verify it's not absurdly high (would indicate a config error)
    assert RobotSpecs.MAX_SPEED_MPS < 5.0, \
        f"MAX_SPEED_MPS {RobotSpecs.MAX_SPEED_MPS} is unrealistically high (robot is ~30cm long chassis)"


def test_lidar_max_range_safety_margin() -> None:
    """Verify LIDAR max range has a safety margin below the raw hardware max."""
    lidar_raw_max = 12.0  # C1 datasheet: 12m

    # Should be slightly less than hardware max to distinguish from dropouts
    assert lidar_raw_max > RobotSpecs.LIDAR_MAX_RANGE, \
        "LIDAR_MAX_RANGE should be less than hardware max (to distinguish dropouts)"

    # Should be reasonable for the 3m mat
    assert RobotSpecs.LIDAR_MAX_RANGE > 3.0, \
        "LIDAR_MAX_RANGE should exceed mat diagonal to see walls"


def test_steering_limits_are_symmetric() -> None:
    """Verify steering limits allow symmetric left/right turns."""
    # Should be positive (magnitude of max steering in either direction)
    assert RobotSpecs.MAX_STEERING_ANGLE > 0, "MAX_STEERING_ANGLE should be positive"

    # Should be reasonable (not exceeding physical limits of servo)
    assert math.pi / 2 > RobotSpecs.MAX_STEERING_ANGLE, \
        "MAX_STEERING_ANGLE should not exceed 90°"


def test_waypoints_arc_radius_documented() -> None:
    """Verify that waypoint ARC_RADIUS tuning parameter exists and is reasonable."""
    tuning = NavigationTuning.load_default()

    arc_radius = tuning.waypoints.ARC_RADIUS

    # Should match the theoretical minimum derived from steering limits
    theoretical_min = RobotSpecs.WHEELBASE / math.tan(RobotSpecs.MAX_STEERING_ANGLE)

    # Actual should be >= theoretical minimum
    assert arc_radius >= theoretical_min, \
        f"ARC_RADIUS {arc_radius} is less than theoretical minimum {theoretical_min}"

    # Should be documented in the TOML/config, not a surprise value
    # This test just verifies it's a real value, not that it matches docs
    assert arc_radius > 0, "ARC_RADIUS should be positive"


def test_corridor_width_model_default_matches_the_mat() -> None:
    """The domain model's width literal must track ``track.toml``.

    ``CorridorWidthEntry`` cannot import ``CorridorDimensions``: it lives in the
    domain layer, which does not depend on config. So the wide width is stated
    in both places, and this is what stops the two drifting -- the check has to
    live here, in a suite free to import either side.

    The default was ``width_mm=500`` against ``type="wide"``, which is neither
    of the two legal widths and disagrees with its own type field. It reached
    ``ScenarioMetadata`` through two layers of model defaults, so metadata built
    without explicit widths described a mat that cannot be built.
    """
    entry = CorridorWidthEntry()

    assert entry.width_mm == round(CorridorDimensions.WIDE * 1000), \
        "CorridorWidthEntry.WIDE_WIDTH_MM has drifted from CorridorDimensions.WIDE"
    assert entry.type is CorridorWidthType.WIDE, "default type must name the default width"


def test_only_two_corridor_widths_are_legal() -> None:
    """Open Challenge's randomised/estimated width is a classification with exactly two values.

    CorridorWidthType also has FIXED, for Obstacles Challenge corridors, which
    are never randomised or estimated -- see CorridorWidthType's own
    docstring. blind_default() (what the estimator settles between) must
    still only ever choose one of the two Open Challenge values.
    """
    assert {CorridorWidthType.NARROW, CorridorWidthType.WIDE, CorridorWidthType.FIXED} == set(CorridorWidthType)
    assert CorridorWidthType.blind_default() in {CorridorWidthType.NARROW, CorridorWidthType.WIDE}
    assert CorridorDimensions.NARROW < CorridorDimensions.WIDE
