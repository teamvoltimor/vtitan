"""Shared navigation helpers extracted from corridor_estimator, corridor_follower,
direction_estimator to eliminate identical module-private definitions."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.navigation_tuning import NavigationTuning

if TYPE_CHECKING:
    from collections.abc import Sequence


_FORWARD_ARC_RAD = math.radians(8.0)
# Read from lidar_sectors.toml rather than restated. This module's callers
# (corridor_follower, direction_estimator) have no tuning-injection path, which
# is why it sits at module level -- but "keep the two values in sync if either
# changes" is not a mechanism, it is a hope, and the same arrangement in the
# sign router already produced a TOML value with no reader at all.
_MIN_VALID_RANGE_M = NavigationTuning.load_default().lidar_sectors.MIN_VALID_RANGE_M
_ALIGNMENT_TOLERANCE_RAD = math.radians(25.0)


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
