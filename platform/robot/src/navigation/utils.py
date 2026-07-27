"""Shared navigation helpers extracted from corridor_estimator, corridor_follower,
direction_estimator to eliminate identical module-private definitions."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence


_FORWARD_ARC_RAD = math.radians(8.0)
_MIN_VALID_RANGE_M = 0.01
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
