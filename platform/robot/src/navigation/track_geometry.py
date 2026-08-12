"""Shared track wall geometry — the single source of truth for where the WRO.

2026 track's walls sit, given each corridor's width. Both the headless
simulator (:class:`~src.simulation.track_model.TrackModel`, generating a
synthetic LIDAR scan from a known pose) and :class:`~src.navigation.localization.LidarLocalizer`
(inferring pose from a real scan) raycast against the exact same geometry, so
the two can never silently drift apart.

Geometry recap (WRO 2026, bottom-left origin, 3.0 x 3.0 m track):

    south face of inner block  y = south_width
    north face of inner block  y = 3 - north_width
    west  face of inner block  x = west_width
    east  face of inner block  x = 3 - east_width
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise
from typing import Any

import numpy as np
from shared.config.constants import DictKeys, RobotSpecs, TrackDimensions
from shared.domain.enums import Section
from shared.domain.models import CorridorGeometry, InnerBlock, ScenarioMetadata

from src.navigation.utils import wrap_angle

_TRACK_MIN = TrackDimensions.MIN_COORD
_TRACK_MAX = TrackDimensions.MAX_COORD  # 3.0


def corridor_geometry_from_widths(widths: dict[Section, float]) -> CorridorGeometry:
    """Build CorridorGeometry from a per-section width dict.

    Needed for blind operation where widths come from CorridorWidthEstimator
    rather than from scenario metadata.
    """
    north = widths[Section.NORTH]
    south = widths[Section.SOUTH]
    east = widths[Section.EAST]
    west = widths[Section.WEST]
    south_y = south
    north_y = _TRACK_MAX - north
    west_x = west
    east_x = _TRACK_MAX - east
    return CorridorGeometry(
        north_width_m=north,
        south_width_m=south,
        east_width_m=east,
        west_width_m=west,
        inner_block=InnerBlock(west_x, south_y, east_x, north_y),
    )


def corridor_widths_from_metadata(metadata: ScenarioMetadata | dict[str, Any]) -> CorridorGeometry:
    """Extract corridor geometry (widths + inner block) from scenario metadata.

    Shared by every consumer that needs to build a :class:`TrackWalls` from a
    scenario's metadata dict (the simulator and the real ROS2 localizer), so
    this parsing lives in exactly one place.
    """
    if isinstance(metadata, ScenarioMetadata):
        cw = metadata.corridor_widths
        north = cw.north.width_mm / 1000.0
        south = cw.south.width_mm / 1000.0
        east = cw.east.width_mm / 1000.0
        west = cw.west.width_mm / 1000.0
    else:
        raw = metadata[DictKeys.CORRIDOR_WIDTHS]
        north = raw["north"][DictKeys.WIDTH_MM] / 1000.0
        south = raw["south"][DictKeys.WIDTH_MM] / 1000.0
        east = raw["east"][DictKeys.WIDTH_MM] / 1000.0
        west = raw["west"][DictKeys.WIDTH_MM] / 1000.0

    south_y = south
    north_y = _TRACK_MAX - north
    west_x = west
    east_x = _TRACK_MAX - east
    return CorridorGeometry(
        north_width_m=north,
        south_width_m=south,
        east_width_m=east,
        west_width_m=west,
        inner_block=InnerBlock(west_x, south_y, east_x, north_y),
    )


def cross_track_error(waypoints: list[tuple[float, float]], x: float, y: float) -> float:
    """Perpendicular distance (metres) from ``(x, y)`` to the waypoint polyline.

    The minimum point-to-segment distance over every consecutive waypoint
    pair — used to measure how far off-path the robot has drifted, e.g. after
    an injected pose disturbance.
    """
    best = math.inf
    for (ax, ay), (bx, by) in pairwise(waypoints):
        abx, aby = bx - ax, by - ay
        seg_len_sq = abx * abx + aby * aby
        if seg_len_sq == 0.0:
            t = 0.0
        else:
            t = ((x - ax) * abx + (y - ay) * aby) / seg_len_sq
            t = min(1.0, max(0.0, t))
        px, py = ax + t * abx, ay + t * aby
        dist = math.hypot(x - px, y - py)
        best = min(best, dist)
    return best


_MIN_WAYPOINTS_FOR_TURN = 3
"""Two waypoints are a single segment, which has no heading change to measure."""


def path_turn_ahead(
    waypoints: list[tuple[float, float]],
    waypoint_index: int,
    preview_distance_m: float,
) -> float:
    """Unsigned heading change the path makes within ``preview_distance_m``.

    Measured forward from ``waypoint_index``. Near zero along a straight and roughly ``preview_distance / arc_radius``
    approaching a corner, so it says "a corner is coming" *before* the robot
    has begun to fall behind one. That is the distinction crosstrack error
    cannot draw: crosstrack only rises once the turn has already been missed.

    Walks the waypoint ring, so it reads correctly across the start/finish
    seam rather than reporting a straight for the last few waypoints of a lap.

    Args:
        waypoints: The closed-loop planned path.
        waypoint_index: Index the robot is currently working toward.
        preview_distance_m: How far along the path to look.

    Returns:
        Unsigned heading change in radians, or 0.0 if the path is too short
        to measure one.
    """
    count = len(waypoints)
    if count < _MIN_WAYPOINTS_FOR_TURN or preview_distance_m <= 0.0:
        return 0.0

    start = waypoint_index % count
    first_heading: float | None = None
    last_heading: float | None = None
    travelled = 0.0

    for offset in range(count):
        ax, ay = waypoints[(start + offset) % count]
        bx, by = waypoints[(start + offset + 1) % count]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len == 0.0:
            continue
        heading = math.atan2(by - ay, bx - ax)
        if first_heading is None:
            first_heading = heading
        last_heading = heading
        travelled += seg_len
        if travelled >= preview_distance_m:
            break

    if first_heading is None or last_heading is None:
        return 0.0
    return abs(wrap_angle(last_heading - first_heading))


@dataclass(frozen=True, slots=True)
class _Segment:
    """An axis-aligned wall face as a line segment (for LIDAR raycasting)."""

    x1: float
    y1: float
    x2: float
    y2: float


class TrackWalls:
    """Axis-aligned wall segments for one Open Challenge layout, and raycasting."""

    def __init__(self, geometry: CorridorGeometry | dict[Section, float]) -> None:
        """Build the wall layout from corridor geometry.

        Args:
            geometry: Complete corridor layout including widths and inner block.
                Also accepts ``dict[Section, float]`` for backward compatibility.
        """
        if isinstance(geometry, dict):
            geometry = corridor_geometry_from_widths(geometry)
        self.inner_block = geometry.inner_block

        self._segments = self._build_segments(self.inner_block)
        # Pre-stack segment endpoints for vectorised raycasting.
        self._seg_ax = np.array([s.x1 for s in self._segments])
        self._seg_ay = np.array([s.y1 for s in self._segments])
        self._seg_ex = np.array([s.x2 - s.x1 for s in self._segments])
        self._seg_ey = np.array([s.y2 - s.y1 for s in self._segments])

    @staticmethod
    def _build_segments(inner: InnerBlock) -> list[_Segment]:
        """Outer track boundary (0/3) + inner block faces — what the LIDAR sees."""
        lo, hi = _TRACK_MIN, _TRACK_MAX
        return [
            # Outer boundary (inner faces of the exterior walls).
            _Segment(lo, lo, hi, lo),  # south
            _Segment(lo, hi, hi, hi),  # north
            _Segment(lo, lo, lo, hi),  # west
            _Segment(hi, lo, hi, hi),  # east
            # Inner block (outer faces of the interior walls).
            _Segment(inner.x_min, inner.y_min, inner.x_max, inner.y_min),  # south
            _Segment(inner.x_min, inner.y_max, inner.x_max, inner.y_max),  # north
            _Segment(inner.x_min, inner.y_min, inner.x_min, inner.y_max),  # west
            _Segment(inner.x_max, inner.y_min, inner.x_max, inner.y_max),  # east
        ]

    def raycast(
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
        world_ang = yaw + angles_robot
        dx = np.cos(world_ang)
        dy = np.sin(world_ang)
        n_rays = angles_robot.shape[0]
        best = np.full(n_rays, np.inf)

        # Vectorise across rays, loop the 8 segments (cheap).
        for ax, ay, ex, ey in zip(
            self._seg_ax,
            self._seg_ay,
            self._seg_ex,
            self._seg_ey,
            strict=True,
        ):
            denom = dx * ey - dy * ex
            # Avoid divide-by-zero for parallel rays.
            safe = np.where(denom == 0.0, np.nan, denom)
            rx = ax - x
            ry = ay - y
            t = (rx * ey - ry * ex) / safe  # distance along the ray
            u = (rx * dy - ry * dx) / safe  # parameter along the segment
            hit = (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
            best = np.minimum(best, np.where(hit, t, np.inf))

        return np.clip(best, RobotSpecs.LIDAR_MIN_RANGE, max_range)

    def point_in_free_space(self, x: float, y: float, clearance: float = 0.0) -> bool:
        """Return ``True`` if (x, y) is in the navigable ring with ``clearance`` margin."""
        if not (
            _TRACK_MIN + clearance <= x <= _TRACK_MAX - clearance
            and _TRACK_MIN + clearance <= y <= _TRACK_MAX - clearance
        ):
            return False
        iv = self.inner_block
        return not (iv.x_min + clearance < x < iv.x_max - clearance and iv.y_min + clearance < y < iv.y_max - clearance)
