"""Shared geometry/math helpers extracted from kinematics, gateway,
vision_emulator to eliminate identical module-private definitions."""

from __future__ import annotations

import math


def _wrap_angle(angle: float) -> float:
    """Wrap to ``[-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
