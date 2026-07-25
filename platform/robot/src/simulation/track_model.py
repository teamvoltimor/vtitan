"""Track geometry model for the headless Open Challenge simulation.

Rebuilds the exact wall geometry the Go scenario generator emits
(``platform/gazebo/generator``) from a scenario's corridor widths, then
exposes the two queries the simulated car needs:

* :meth:`TrackModel.raycast_scan` — a simulated Slamtec-C1 LIDAR sweep.
* :meth:`TrackModel.footprint_collides` — chassis-vs-wall collision.

Two wall representations are kept, mirroring the generator's split between
*visual* meshes (0.10 m thick) and *collision* meshes (0.18 m thick):

* **Visual faces** — the outer track boundary at 0/3 m and the inner block at
  the corridor-width lines. The LIDAR rangefinder hits these surfaces.
* **Collision faces** — the same walls inflated inward by half the difference
  between collision and visual thickness (``0.09 - 0.05 = 0.04 m``). The
  chassis footprint must never overlap these. This is what makes the
  ``0.18 m`` collision mesh slightly "fatter" than what the LIDAR sees.

Geometry recap (WRO 2026, bottom-left origin, 3.0 x 3.0 m track):

    south face of inner block  y = south_width
    north face of inner block  y = 3 - north_width
    west  face of inner block  x = west_width
    east  face of inner block  x = 3 - east_width
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions

from src.navigation.track_geometry import TrackWalls

if TYPE_CHECKING:
    from shared.config.enums import Section

# Wall thickness halves (metres) — straight from the generator's constants:
# WallThickness = 0.10 (visual), WallCollisionThickness = 0.18 (collision).
_WALL_VISUAL_HALF = 0.05
_WALL_COLLISION_HALF = 0.09
# How much further the collision mesh protrudes past the visual face.
_COLLISION_MARGIN = _WALL_COLLISION_HALF - _WALL_VISUAL_HALF  # 0.04 m

_TRACK_MIN = 0.0
_TRACK_MAX = TrackDimensions.MAX_COORD  # 3.0


@dataclass(frozen=True, slots=True)
class _Box:
    """An axis-aligned keep-out box (for footprint collision)."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def corners(self) -> list[tuple[float, float]]:
        """Return the four box corners (CCW from bottom-left)."""
        return [
            (self.x_min, self.y_min),
            (self.x_max, self.y_min),
            (self.x_max, self.y_max),
            (self.x_min, self.y_max),
        ]


class TrackModel:
    """Wall geometry + sensor/collision queries for one Open Challenge layout."""

    def __init__(self, corridor_widths_m: dict[Section, float]) -> None:
        """Build the track from per-side corridor widths.

        Args:
            corridor_widths_m: Navigable corridor width (metres) for each of the
                four sections, e.g. ``{Section.SOUTH: 0.6, ...}``.
        """
        self._widths = corridor_widths_m
        self._walls = TrackWalls(corridor_widths_m)

        inner = self._walls.inner_block
        self._inner_visual = _Box(inner.x_min, inner.y_min, inner.x_max, inner.y_max)
        self._inner_collision = _Box(
            inner.x_min - _COLLISION_MARGIN,
            inner.y_min - _COLLISION_MARGIN,
            inner.x_max + _COLLISION_MARGIN,
            inner.y_max + _COLLISION_MARGIN,
        )
        # Footprint must stay within this outer collision boundary.
        self._outer_collision = _Box(
            _TRACK_MIN + _COLLISION_MARGIN,
            _TRACK_MIN + _COLLISION_MARGIN,
            _TRACK_MAX - _COLLISION_MARGIN,
            _TRACK_MAX - _COLLISION_MARGIN,
        )

    # Simulated LIDAR

    def raycast_scan(
        self,
        x: float,
        y: float,
        yaw: float,
        angles_robot: np.ndarray,
        max_range: float = RobotSpecs.LIDAR_MAX_RANGE,
    ) -> np.ndarray:
        """Cast a fan of rays and return the nearest wall range per ray.

        Args:
            x: Sensor world X (metres).
            y: Sensor world Y (metres).
            yaw: Robot heading (radians).
            angles_robot: Ray bearings in the robot frame (radians, 0 = forward).
            max_range: Sensor ceiling; rays that hit nothing return this.

        Returns:
            Range (metres) for each bearing, clamped to ``[LIDAR_MIN_RANGE, max_range]``.
        """
        return self._walls.raycast(x, y, yaw, angles_robot, max_range)

    # Collision

    def footprint_collides(
        self,
        x: float,
        y: float,
        yaw: float,
        length: float = RobotSpecs.LENGTH,
        width: float = RobotSpecs.WIDTH,
    ) -> bool:
        """Return ``True`` if the oriented chassis rectangle hits any wall.

        Checks the chassis footprint against the outer collision boundary and
        the inner keep-out block, both built from the 0.18 m collision meshes.
        """
        corners = _rect_corners(x, y, yaw, length, width)

        # Outer boundary: every corner must stay inside the collision box.
        ob = self._outer_collision
        for cx, cy in corners:
            if cx < ob.x_min or cx > ob.x_max or cy < ob.y_min or cy > ob.y_max:
                return True

        # Inner block: oriented footprint must not overlap the keep-out box.
        return _convex_overlap(corners, self._inner_collision.corners(), yaw)

    # Geometry helpers exposed for tests / planners

    def point_in_free_space(self, x: float, y: float, clearance: float = 0.0) -> bool:
        """Return ``True`` if (x, y) is in the navigable ring with ``clearance`` margin.

        Used to validate that planned waypoints sit inside the corridor with at
        least ``clearance`` metres to the nearest *visual* wall.
        """
        if not (
            _TRACK_MIN + clearance <= x <= _TRACK_MAX - clearance
            and _TRACK_MIN + clearance <= y <= _TRACK_MAX - clearance
        ):
            return False
        iv = self._inner_visual
        # Inside the inner block (with clearance shrinking the safe corridor) -> blocked.
        return not (iv.x_min + clearance < x < iv.x_max - clearance and iv.y_min + clearance < y < iv.y_max - clearance)

    @property
    def inner_block_visual(self) -> tuple[float, float, float, float]:
        """Inner-block visual bounds ``(x_min, y_min, x_max, y_max)`` in metres."""
        iv = self._inner_visual
        return (iv.x_min, iv.y_min, iv.x_max, iv.y_max)


def _rect_corners(
    cx: float,
    cy: float,
    yaw: float,
    length: float,
    width: float,
) -> list[tuple[float, float]]:
    """Four corners of an oriented rectangle centred at (cx, cy)."""
    hl, hw = length / 2.0, width / 2.0
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    local = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    return [(cx + lx * cos_y - ly * sin_y, cy + lx * sin_y + ly * cos_y) for lx, ly in local]


def _convex_overlap(
    poly_a: list[tuple[float, float]],
    poly_b: list[tuple[float, float]],
    yaw: float,
) -> bool:
    """Separating-axis test between two convex polygons (rect vs AABB).

    Axes tested: the oriented rectangle's two edge normals plus the world X/Y
    axes (the AABB's normals). Exact for rectangle-vs-box overlap.
    """
    axes = (
        (np.cos(yaw), np.sin(yaw)),
        (-np.sin(yaw), np.cos(yaw)),
        (1.0, 0.0),
        (0.0, 1.0),
    )
    for ax, ay in axes:
        a_proj = [px * ax + py * ay for px, py in poly_a]
        b_proj = [px * ax + py * ay for px, py in poly_b]
        if max(a_proj) < min(b_proj) or max(b_proj) < min(a_proj):
            return False  # found a separating axis -> no overlap
    return True
