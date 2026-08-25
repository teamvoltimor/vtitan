"""Angle normalisation helper for the parking controller."""

from __future__ import annotations

import math


def normalise_angle(angle: float) -> float:
    """Wrap an angle into the ``(-pi, pi]`` range."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
