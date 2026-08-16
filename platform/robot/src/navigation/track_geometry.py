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
from shared.domain.models import CorridorGeometry, InnerBlock, ScenarioMetadata, Waypoint

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


@dataclass(frozen=True, slots=True)
class PathProjection:
    """Where a point sits relative to the planned path, in the path's own frame.

    The frame rotates with the path, which is the whole point: a global-axis
    lateral component is only a cross-track error while the path runs along
    that axis. Measured against a corner it also picks up the arc, so a robot
    tracking its turn perfectly reads as drifting. Anything comparing two
    points on a curving path — commanded target against chassis, sign against
    the line that was meant to clear it — has to difference them in this frame.
    """

    x: float
    """Closest point on the polyline."""

    y: float

    distance_m: float
    """Distance to the polyline itself, always non-negative.

    Clamped at the ends, so a point off the end of an open path is correctly
    far away rather than merely off to one side of the last segment's heading.
    This is the "how far off-path am I" number.
    """

    signed_offset_m: float
    """Lateral offset from the nearest segment's LINE, positive to the LEFT.

    Deliberately the infinite line and not the segment, which makes this differ
    from ``distance_m`` in exactly one place: a point outside a convex vertex,
    where the nearest point on the polyline is the vertex itself. Measured to
    the vertex, such a point picks up an along-track term and a target sweeping
    past a corner shows a spurious bulge in its offset — which is the very
    artifact this frame exists to avoid.

    Signed rather than absolute so two offsets can be differenced: unsigned
    would collapse points straddling the path onto the same value, and
    straddling is the larger error of the two.
    """

    tangent_rad: float
    """Heading of the path at the closest point."""

    segment_index: int
    """Index of the polyline segment the point projected onto.

    The handle for :func:`path_turn_ahead`, i.e. for asking whether this
    projection landed on a straight or mid-corner.
    """


def project_onto_path(waypoints: list[Waypoint], x: float, y: float) -> PathProjection:
    """Project ``(x, y)`` onto the waypoint polyline and return the path frame.

    The nearest point over every consecutive waypoint pair. Distance to the
    nearest *waypoint* would overstate the offset by up to half the waypoint
    spacing — enough to matter against a ±6.7 cm sign-pass budget.
    """
    best: PathProjection | None = None
    best_dist = math.inf
    for index, (a, b) in enumerate(pairwise(waypoints)):
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
        abx, aby = bx - ax, by - ay
        seg_len_sq = abx * abx + aby * aby
        if seg_len_sq == 0.0:
            continue
        t = ((x - ax) * abx + (y - ay) * aby) / seg_len_sq
        t = min(1.0, max(0.0, t))
        px, py = ax + t * abx, ay + t * aby
        dist = math.hypot(x - px, y - py)
        if dist >= best_dist:
            continue
        best_dist = dist
        best = PathProjection(
            x=px,
            y=py,
            distance_m=dist,
            # Left-normal component about the segment's infinite line: rotate
            # the tangent +90 deg and dot. Taken from the segment start, not
            # from the clamped projection, which would read zero for any point
            # that projected past an end.
            signed_offset_m=(-aby * (x - ax) + abx * (y - ay)) / math.sqrt(seg_len_sq),
            tangent_rad=math.atan2(aby, abx),
            segment_index=index,
        )
    if best is None:
        # Fewer than two distinct waypoints: there is no tangent to define a
        # frame, so fall back to the nearest waypoint and leave the offset
        # unsigned. Keeps ``cross_track_error`` meaningful on a degenerate path
        # rather than reporting a confident zero.
        nearest = min(waypoints, key=lambda w: math.hypot(x - w.x, y - w.y), default=None)
        if nearest is None:
            return PathProjection(x=x, y=y, distance_m=math.inf, signed_offset_m=math.inf, tangent_rad=0.0, segment_index=0)
        away = math.hypot(x - nearest.x, y - nearest.y)
        return PathProjection(x=nearest.x, y=nearest.y, distance_m=away, signed_offset_m=away, tangent_rad=0.0, segment_index=0)
    return best


def cross_track_error(waypoints: list[Waypoint], x: float, y: float) -> float:
    """Perpendicular distance (metres) from ``(x, y)`` to the waypoint polyline.

    The minimum point-to-segment distance over every consecutive waypoint
    pair — used to measure how far off-path the robot has drifted, e.g. after
    an injected pose disturbance. Unsigned; use :func:`project_onto_path` when
    the offsets are going to be differenced against each other.
    """
    return project_onto_path(waypoints, x, y).distance_m


_MIN_WAYPOINTS_FOR_TURN = 3
"""Two waypoints are a single segment, which has no heading change to measure."""


def path_turn_ahead(
    waypoints: list[Waypoint],
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
        a = waypoints[(start + offset) % count]
        b = waypoints[(start + offset + 1) % count]
        ax, ay = a.x, a.y
        bx, by = b.x, b.y
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

    def raycast_grid(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        yaw: float,
        angles_robot: np.ndarray,
        max_range: float = RobotSpecs.LIDAR_MAX_RANGE,
    ) -> np.ndarray:
        """``raycast`` batched over many candidate positions at one shared yaw.

        Same formula as :meth:`raycast`, just broadcast over ``xs``/``ys``
        instead of looped in Python -- built for :class:`~src.navigation.localization.LidarLocalizer`'s
        grid-search pose estimate, which used to call ``raycast`` once per
        candidate in a nested Python loop (``passes * grid_points**2`` calls
        per :meth:`~src.navigation.localization.LidarLocalizer.estimate_position`).
        Each per-segment ``denom``/``safe`` term depends only on ray
        direction and the segment, never on candidate position, so it is
        computed once per segment here instead of once per (segment,
        candidate) pair -- the only per-candidate work left is the vectorised
        ``rx``/``ry`` broadcast, which numpy does in one shot.

        Args:
            xs: Candidate sensor world X positions, shape ``(n_candidates,)``.
            ys: Candidate sensor world Y positions, shape ``(n_candidates,)``.
            yaw: Robot heading (radians), shared by every candidate.
            angles_robot: Ray bearings in the robot frame (radians, 0 = forward).
            max_range: Sensor ceiling; rays that hit nothing return this.

        Returns:
            Range (metres), shape ``(n_candidates, n_rays)``, clamped to
            ``[LIDAR_MIN_RANGE, max_range]``.
        """
        world_ang = yaw + angles_robot
        dx = np.cos(world_ang)
        dy = np.sin(world_ang)
        n_candidates = xs.shape[0]
        n_rays = angles_robot.shape[0]
        best = np.full((n_candidates, n_rays), np.inf)

        for ax, ay, ex, ey in zip(
            self._seg_ax,
            self._seg_ay,
            self._seg_ex,
            self._seg_ey,
            strict=True,
        ):
            denom = dx * ey - dy * ex
            safe = np.where(denom == 0.0, np.nan, denom)
            rx = ax - xs[:, None]
            ry = ay - ys[:, None]
            t = (rx * ey - ry * ex) / safe[None, :]
            u = (rx * dy[None, :] - ry * dx[None, :]) / safe[None, :]
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
