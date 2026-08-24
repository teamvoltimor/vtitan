"""Coordinate transformation utilities for WRO track sections.

Eliminates duplicated section-specific coordinate logic across
randomizer and builder modules by providing a single reusable
coordinate transform helper.
"""

from __future__ import annotations

import math


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle from a unit quaternion.

    Uses the standard atan2 formula for the Z-axis rotation.

    Args:
        x: X component of the quaternion.
        y: Y component of the quaternion.
        z: Z component of the quaternion.
        w: W component of the quaternion.

    Returns:
        Yaw angle in radians, in the range [-π, π].
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
