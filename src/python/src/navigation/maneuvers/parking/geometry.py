"""Angle normalisation helper for the parking controller."""

from __future__ import annotations

from src.navigation.utils import wrap_angle


def normalise_angle(angle: float) -> float:
    """Wrap an angle into the ``(-pi, pi]`` range.

    Delegates to :func:`src.navigation.utils.wrap_angle` so the parking
    controller shares the one angle-wrap definition rather than carrying a
    second loop-based copy.
    """
    return wrap_angle(angle)
