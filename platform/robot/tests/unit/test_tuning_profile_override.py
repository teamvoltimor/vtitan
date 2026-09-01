"""Verify that tuning parameter overrides propagate through the navigation stack.

This guards against the import-time freezing bug where `--tuning profile.yaml`
was silently ignored because constants were loaded at module import, not at
runtime when the override could be applied.
"""

from __future__ import annotations

import math

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


def test_corridor_follower_respects_tuning_override(override_tuning) -> None:
    """Doubling CENTERING_GAIN_DEG_PER_M must double the steering follow_corridor commands.

    The base gain is stated here rather than read from the shipped tuning. The
    shipped value has been deliberately 0.0 since 2026-08-22 -- the blind creep
    holds heading instead of chasing the centreline -- so doubling it doubles
    nothing, both calls return -0.0, and the "off-centre scan should steer"
    guard fails on correct config. What this pins is that the OVERRIDE reaches
    corridor_follower at all; the shipped gain's value is a separate question,
    already pinned by test_tuning_fields_not_none's DELIBERATELY_ZERO entry.
    """
    default_tuning = NavigationTuning.load_default()
    base_gain = 5.0

    # Off-centre, so the centering term has an offset to act on. The doubled
    # gain must stay under STEERING_CAP or both calls saturate to the same
    # number and the assertion passes without the override doing anything.
    ranges_m = _corridor_scan(left_m=0.35, right_m=0.65)

    base_cmd = follow_corridor(
        ranges_m=ranges_m,
        angles_rad=_ANGLES_RAD,
        speed_mps=0.1,
        tuning=override_tuning(default_tuning, corridor_follower={"CENTERING_GAIN_DEG_PER_M": base_gain}),
    )
    doubled_cmd = follow_corridor(
        ranges_m=ranges_m,
        angles_rad=_ANGLES_RAD,
        speed_mps=0.1,
        tuning=override_tuning(default_tuning, corridor_follower={"CENTERING_GAIN_DEG_PER_M": base_gain * 2.0}),
    )

    assert base_cmd.steering_norm != 0.0, "off-centre scan should steer back to the middle"
    assert doubled_cmd.steering_norm == pytest.approx(base_cmd.steering_norm * 2.0)


def test_centred_corridor_steers_straight_at_any_gain(override_tuning) -> None:
    """The centred case cannot detect an override -- pinned so it is not used as one."""
    default_tuning = NavigationTuning.load_default()
    ranges_m = _corridor_scan(left_m=0.5, right_m=0.5)

    for gain in (default_tuning.corridor_follower.CENTERING_GAIN_DEG_PER_M, 10.0):
        cmd = follow_corridor(
            ranges_m=ranges_m,
            angles_rad=_ANGLES_RAD,
            speed_mps=0.1,
            tuning=override_tuning(default_tuning, corridor_follower={"CENTERING_GAIN_DEG_PER_M": gain}),
        )
        assert cmd.steering_norm == pytest.approx(0.0)


def test_corridor_estimator_respects_tuning_override(override_tuning) -> None:
    """CorridorWidthEstimator reads MIN_SAMPLES from the tuning it's given, not a frozen default."""
    default_tuning = NavigationTuning.load_default()
    lower_min_samples = max(1, default_tuning.corridor_estimator.MIN_SAMPLES - 2)
    aggressive = override_tuning(default_tuning, corridor_estimator={"MIN_SAMPLES": lower_min_samples})

    estimator_default = CorridorWidthEstimator(tuning=default_tuning)
    estimator_aggressive = CorridorWidthEstimator(tuning=aggressive)

    assert estimator_default._min_samples == default_tuning.corridor_estimator.MIN_SAMPLES
    assert estimator_aggressive._min_samples == lower_min_samples


def test_direction_estimator_respects_tuning_override(override_tuning) -> None:
    """infer_direction reads MIN_ASYMMETRY_M from the tuning it's given, not a frozen default."""
    default_tuning = NavigationTuning.load_default()

    # Right side open, left side a near wall: a real but modest asymmetry,
    # well above the default MIN_ASYMMETRY_M but not enough to survive a
    # much stricter threshold.
    ranges_m = [0.6] * 360
    angles_rad = [math.radians(i - 180) for i in range(360)]
    for i in range(270, 360):
        ranges_m[i] = 2.5

    direction_default = infer_direction(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        yaw=0.0,
        tuning=default_tuning,
    )

    strict = override_tuning(default_tuning, direction_estimator={"MIN_ASYMMETRY_M": 100.0})
    direction_strict = infer_direction(
        ranges_m=ranges_m,
        angles_rad=angles_rad,
        yaw=0.0,
        tuning=strict,
    )

    assert direction_default is not None, "the crafted asymmetry should be enough under the default threshold"
    assert direction_strict is None, "an unreachable MIN_ASYMMETRY_M override should suppress the same call"


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
