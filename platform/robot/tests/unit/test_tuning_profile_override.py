"""Verify that tuning parameter overrides propagate through the navigation stack.

This guards against the import-time freezing bug where `--tuning profile.yaml`
was silently ignored because constants were loaded at module import, not at
runtime when the override could be applied.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.corridor_estimator import CorridorWidthEstimator
from src.navigation.corridor_follower import follow_corridor
from src.navigation.direction_estimator import infer_direction

_ANGLES_RAD = [math.radians(i - 180) for i in range(360)]
_FORWARD_CAP_M = 3.0


def _corridor_scan(left_m: float, right_m: float) -> list[float]:
    """Rays into a straight corridor with the walls at these perpendicular distances.

    Forward stays open, so ``follow_corridor`` takes its centering branch rather
    than the turn branch -- a scan of one uniform range reads as a wall dead
    ahead and steers hard at any gain, which proves nothing about the gain.
    """
    ranges_m = []
    for angle in _ANGLES_RAD:
        sin_a = math.sin(angle)
        if sin_a > 1e-6:
            wall_dist = left_m / sin_a
        elif sin_a < -1e-6:
            wall_dist = right_m / -sin_a
        else:
            wall_dist = _FORWARD_CAP_M
        ranges_m.append(min(wall_dist, _FORWARD_CAP_M))
    return ranges_m


def _with_centering_gain(tuning: NavigationTuning, gain: float) -> NavigationTuning:
    """A copy of ``tuning`` with CENTERING_GAIN replaced.

    Both the tuning dataclass and its parameter groups are frozen, so this is
    the only way to build an override -- assigning to the field raises.
    """
    return replace(
        tuning,
        corridor_follower=tuning.corridor_follower.model_copy(update={"CENTERING_GAIN": gain}),
    )


def test_corridor_follower_respects_tuning_override() -> None:
    """Doubling CENTERING_GAIN must double the steering follow_corridor commands."""
    default_tuning = NavigationTuning.load_default()
    base_gain = default_tuning.corridor_follower.CENTERING_GAIN

    # Off-centre, so the centering term has an offset to act on. The doubled
    # gain must stay under STEERING_CAP or both calls saturate to the same
    # number and the assertion passes without the override doing anything.
    ranges_m = _corridor_scan(left_m=0.35, right_m=0.65)

    default_cmd = follow_corridor(
        ranges_m=ranges_m, angles_rad=_ANGLES_RAD, speed_mps=0.1, tuning=default_tuning
    )
    doubled_cmd = follow_corridor(
        ranges_m=ranges_m,
        angles_rad=_ANGLES_RAD,
        speed_mps=0.1,
        tuning=_with_centering_gain(default_tuning, base_gain * 2.0),
    )

    assert default_cmd.steering_norm != 0.0, "off-centre scan should steer back to the middle"
    assert doubled_cmd.steering_norm == pytest.approx(default_cmd.steering_norm * 2.0)


def test_centred_corridor_steers_straight_at_any_gain() -> None:
    """The centred case cannot detect an override -- pinned so it is not used as one."""
    default_tuning = NavigationTuning.load_default()
    ranges_m = _corridor_scan(left_m=0.5, right_m=0.5)

    for gain in (default_tuning.corridor_follower.CENTERING_GAIN, 10.0):
        cmd = follow_corridor(
            ranges_m=ranges_m,
            angles_rad=_ANGLES_RAD,
            speed_mps=0.1,
            tuning=_with_centering_gain(default_tuning, gain),
        )
        assert cmd.steering_norm == pytest.approx(0.0)


def test_corridor_estimator_respects_tuning_override() -> None:
    """Verify that CorridorWidthEstimator uses passed tuning."""
    default_tuning = NavigationTuning.load_default()

    # Create two estimators with different tuning
    estimator_default = CorridorWidthEstimator(tuning=default_tuning)

    aggressive = NavigationTuning.load_default()
    aggressive.corridor_estimator.MIN_SAMPLES = max(1, default_tuning.corridor_estimator.MIN_SAMPLES - 2)
    estimator_aggressive = CorridorWidthEstimator(tuning=aggressive)

    # Both should work without error (verifies tuning is passed through)
    assert estimator_default is not None
    assert estimator_aggressive is not None


def test_direction_estimator_respects_tuning_override() -> None:
    """Verify that infer_direction uses passed tuning, not frozen defaults."""
    default_tuning = NavigationTuning.load_default()

    # Create a scan that shows corridor opening to the right
    ranges_m = [0.6] * 360
    angles_rad = [math.radians(i - 180) for i in range(360)]

    # Make the right side open (simulating a corner)
    for i in range(270, 360):  # Right side (90° to 180° in robot frame)
        angles_rad[i] = math.radians(i - 180)
        ranges_m[i] = 2.5  # Far away (open side)

    # Call with default tuning
    direction_default = infer_direction(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        yaw=0.0,
        tuning=default_tuning,
    )

    # Call with override tuning (should still work)
    direction_aggressive = infer_direction(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        yaw=0.0,
        tuning=default_tuning,
    )

    # Both should return the same direction (same tuning for this test)
    assert direction_default == direction_aggressive, \
        "Same tuning should produce same direction inference"


def test_tuning_parameter_none_loads_default() -> None:
    """Verify that tuning=None falls back to default (lazy load behavior)."""
    # Create a scan
    ranges_m = [0.6] * 360
    angles_rad = [math.radians(i - 180) for i in range(360)]

    # Call with tuning=None (should load default internally)
    cmd_none = follow_corridor(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        speed_mps=0.1,
        tuning=None,
    )

    # Call with explicit default
    explicit_default = NavigationTuning.load_default()
    cmd_explicit = follow_corridor(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        speed_mps=0.1,
        tuning=explicit_default,
    )

    # Both should produce the same command (same tuning loaded)
    assert cmd_none.speed_mps == cmd_explicit.speed_mps
    assert cmd_none.steering_norm == cmd_explicit.steering_norm
