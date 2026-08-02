"""Shared navigation helpers extracted from corridor_estimator, corridor_follower,
direction_estimator to eliminate identical module-private definitions."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.navigation_tuning import NavigationTuning

if TYPE_CHECKING:
    from collections.abc import Sequence


_tuning = NavigationTuning.load_default()
# Sourced from tuning rather than hardcoded, so TOML edits take effect everywhere.
# DIRECTION_ARC_HALF_FOV_DEG, not FRONT_HALF_FOV_DEG: the two are both "how wide
# is forward" but gate different things at different tolerances -- see that
# field's docstring for the run this distinction cost when they were conflated.
_FORWARD_ARC_RAD = math.radians(_tuning.lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG)
_MIN_VALID_RANGE_M = _tuning.lidar_sectors.MIN_VALID_RANGE_M
# Alignment tolerance: 25 deg is HEADING_ERROR_ZONES.MEDIUM from tuning
_ALIGNMENT_TOLERANCE_RAD = _tuning.heading.MEDIUM


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _nearest_ray(ranges_m: Sequence[float], angles_rad: Sequence[float], target: float) -> float:
    index = min(range(len(angles_rad)), key=lambda i: abs(_wrap(angles_rad[i] - target)))
    return ranges_m[index]


def _forward_clearance(ranges_m: Sequence[float], angles_rad: Sequence[float]) -> float:
    forward = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(_wrap(a)) <= _FORWARD_ARC_RAD and r > _MIN_VALID_RANGE_M
    ]
    return min(forward) if forward else math.inf
