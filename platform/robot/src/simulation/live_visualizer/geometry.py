"""Geometry helpers for the live RViz visualizer's marker builders.

Pure functions: no ROS2 node state, no ``self``. Kept separate from
:mod:`src.simulation.live_visualizer.visualizer` so the marker maths does not
bloat the node class and can be unit-tested without a ``LiveScenarioVisualizer``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from geometry_msgs.msg import Quaternion

from src.navigation.utils import wrap_angle

if TYPE_CHECKING:
    from geometry_msgs.msg import Marker
    from shared.config.navigation_tuning.sensors import LidarSectorParams

    from src.config.rviz_visualization import Rgb


def _apply_color(marker: Marker, color: Rgb, alpha: float = 1.0) -> None:
    """Set a Marker's RGBA from an :class:`Rgb`, avoiding bare-tuple unpacking."""
    marker.color.r = color.r
    marker.color.g = color.g
    marker.color.b = color.b
    marker.color.a = alpha


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _pitch_to_quaternion(pitch: float) -> Quaternion:
    return Quaternion(x=0.0, y=math.sin(pitch / 2.0), z=0.0, w=math.cos(pitch / 2.0))


def _is_masked_bearing(angle_rad: float, sectors: LidarSectorParams) -> bool:
    """True where the mount occludes its own sensor, by ANGLE not by range.

    These rays self-collide as a matter of geometry, so whatever range comes
    back is meaningless regardless of how large it is -- which is exactly why
    the navigator's own rear-sector read excludes them by bearing rather than
    filtering on distance. Same two wedges, same config.
    """
    degrees = math.degrees(wrap_angle(angle_rad))
    return (
        sectors.BLIND_WEDGE_LEFT_MIN_DEG <= degrees <= sectors.BLIND_WEDGE_LEFT_MAX_DEG
        or sectors.BLIND_WEDGE_RIGHT_MIN_DEG <= degrees <= sectors.BLIND_WEDGE_RIGHT_MAX_DEG
    )


def _wheel_to_quaternion(steer: float, roll: float = 0.0) -> Quaternion:
    """Orient a marker as a road wheel, steered by ``steer`` and rolled by ``roll``.

    A Marker CYLINDER extrudes along its own z, but a wheel's axle is lateral,
    so this is Rz(steer) * Rx(90deg) * Rz(-roll): the ``rpy="${pi/2} 0 0"`` the
    URDF applies to its wheel visuals, steered about the world vertical, then
    spun about the wheel's own axle. That last rotation is post-multiplied
    because after Rx(90deg) the marker's local z IS the axle.

    ``roll`` is positive rolling forward. It is negated inside because Rx(90deg)
    lays the axle along -y, so a naive positive rotation would spin the wheel
    backwards while the robot drove forwards.

    Written out as the closed-form product rather than by composing three
    Quaternion objects: expanding it collapses to half-angle sums (s =
    sqrt(2)/2 is Rx(90deg)'s shared term), which is both shorter and cheaper
    than the general multiply, and matches the two helpers above.
    """
    s = math.sqrt(2.0) / 2.0
    steered, spun = (steer + roll) / 2.0, (steer - roll) / 2.0
    return Quaternion(
        x=s * math.cos(steered),
        y=s * math.sin(steered),
        z=s * math.sin(spun),
        w=s * math.cos(spun),
    )
