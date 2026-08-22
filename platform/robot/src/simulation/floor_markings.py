"""Floor marking geometry for the WRO 2026 mat.

Builds map-frame line geometry for the starting-square grid and corner lines.
The actual ROS2 :class:`visualization_msgs.msg.Marker` construction stays in
:mod:`src.simulation.live_visualizer` so this module has no ROS2 dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from shared.config.constants import TrackDimensions, TrackMarkings
from shared.config.starting_zone import STARTING_ZONE_LAYOUT
from shared.domain.enums import Section

_RAY_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class StartingSquareGeometry:
    """The four corners and division-line endpoints of one corridor's starting square.

    The starting square occupies the middle metre of a side, from
    ``TRACK_CENTER - 0.5`` to ``TRACK_CENTER + 0.5`` along the corridor, and
    spans from the outer wall inward for the full corridor width.
    """

    section: Section

    def corners(self, corridor_width: float) -> list[tuple[float, float]]:
        """Return the four corners of the starting square, CCW from the outer-wall start."""
        lo = TrackDimensions.CENTER_COORD - 0.5
        hi = TrackDimensions.CENTER_COORD + 0.5
        match self.section:
            case Section.SOUTH:
                return [(lo, 0.0), (hi, 0.0), (hi, corridor_width), (lo, corridor_width)]
            case Section.NORTH:
                y_outer = TrackDimensions.MAX_COORD
                y_inner = y_outer - corridor_width
                return [(lo, y_inner), (hi, y_inner), (hi, y_outer), (lo, y_outer)]
            case Section.EAST:
                x_outer = TrackDimensions.MAX_COORD
                x_inner = x_outer - corridor_width
                return [(x_inner, lo), (x_inner, hi), (x_outer, hi), (x_outer, lo)]
            case Section.WEST:
                return [(0.0, lo), (0.0, hi), (corridor_width, hi), (corridor_width, lo)]

    def across_line(self, distance_from_outer: float) -> tuple[tuple[float, float], tuple[float, float]]:
        """Endpoints of a line across the corridor at ``distance_from_outer`` from the outer wall."""
        lo = TrackDimensions.CENTER_COORD - 0.5
        hi = TrackDimensions.CENTER_COORD + 0.5
        match self.section:
            case Section.SOUTH:
                y = distance_from_outer
                return (lo, y), (hi, y)
            case Section.NORTH:
                y = TrackDimensions.MAX_COORD - distance_from_outer
                return (lo, y), (hi, y)
            case Section.EAST:
                x = TrackDimensions.MAX_COORD - distance_from_outer
                return (x, lo), (x, hi)
            case Section.WEST:
                x = distance_from_outer
                return (x, lo), (x, hi)

    def along_line(self, corridor_width: float) -> tuple[tuple[float, float], tuple[float, float]]:
        """Endpoints of the line along the corridor that splits the two cells."""
        mid = TrackDimensions.CENTER_COORD
        match self.section:
            case Section.SOUTH:
                return (mid, 0.0), (mid, corridor_width)
            case Section.NORTH:
                y_outer = TrackDimensions.MAX_COORD
                y_inner = y_outer - corridor_width
                return (mid, y_inner), (mid, y_outer)
            case Section.EAST:
                x_outer = TrackDimensions.MAX_COORD
                x_inner = x_outer - corridor_width
                return (x_inner, mid), (x_outer, mid)
            case Section.WEST:
                return (0.0, mid), (corridor_width, mid)

    def indicator_center(self, corridor_width: float) -> tuple[float, float]:
        """Centre of the direction indicator painted in this starting square."""
        mid = TrackDimensions.CENTER_COORD
        across = corridor_width / 2.0
        match self.section:
            case Section.SOUTH:
                return (mid, across)
            case Section.NORTH:
                return (mid, TrackDimensions.MAX_COORD - across)
            case Section.EAST:
                return (TrackDimensions.MAX_COORD - across, mid)
            case Section.WEST:
                return (across, mid)


def starting_square_band_divisions(corridor_width: float) -> list[float]:
    """Cumulative band-edge distances from the outer wall that fit in this corridor.

    A wide corridor has two division lines (at 0.40 m and 0.60 m) giving three
    bands and six cells. A narrow corridor has one division line (at 0.40 m)
    giving two bands and four cells.
    """
    cumulative = 0.0
    divisions: list[float] = []
    for band in STARTING_ZONE_LAYOUT.band_widths:
        cumulative += band
        if cumulative >= corridor_width - 1e-9:
            break
        divisions.append(cumulative)
    return divisions


_STARTING_SQUARE_GEOMETRY: dict[Section, StartingSquareGeometry] = {
    section: StartingSquareGeometry(section) for section in Section
}


def starting_square_geometry(section: Section) -> StartingSquareGeometry:
    """Return the geometry helper for ``section``."""
    return _STARTING_SQUARE_GEOMETRY[section]


@dataclass(frozen=True, slots=True)
class CornerLine:
    """One 30-degree corner line: start point, end point and colour."""

    start: tuple[float, float]
    end: tuple[float, float]
    color: tuple[float, float, float]


def corner_lines() -> list[CornerLine]:
    """The eight 30° corner lines (four corners × two colours).

    Each corner has two coloured lines that start at the inner-block corner,
    run into the 1 m × 1 m corner region (the square outside the inner block),
    and stop at the track's outer wall. The two lines are symmetric around the
    corner bisector, each 30° from one of the inner-block walls, so they are
    15° on either side of the bisector.
    """
    lines: list[CornerLine] = []

    def _extend(
        cx: float,
        cy: float,
        dx: float,
        dy: float,
        sx: int,
        sy: int,
    ) -> tuple[float, float]:
        """Return where the ray from (cx,cy) in direction (dx,dy) hits the corner-region edge."""
        x_max = cx + max(sx, 0)
        x_min = cx + min(sx, 0)
        y_max = cy + max(sy, 0)
        y_min = cy + min(sy, 0)
        ts: list[float] = []
        if dx > _RAY_EPSILON:
            ts.append((x_max - cx) / dx)
        elif dx < -_RAY_EPSILON:
            ts.append((x_min - cx) / dx)
        if dy > _RAY_EPSILON:
            ts.append((y_max - cy) / dy)
        elif dy < -_RAY_EPSILON:
            ts.append((y_min - cy) / dy)
        t = min(t for t in ts if t > 0)
        return cx + t * dx, cy + t * dy

    def _corner(cx: float, cy: float, sx: int, sy: int) -> None:
        # Bisector points into the corner region (diagonal).
        bisector = math.atan2(sy, sx)
        offset = math.radians(TrackMarkings.ANGLE / 2.0)  # 15°
        for color, angle in (
            (TrackMarkings.ORANGE_COLOR, bisector - offset),
            (TrackMarkings.BLUE_COLOR, bisector + offset),
        ):
            end = _extend(cx, cy, math.cos(angle), math.sin(angle), sx, sy)
            lines.append(CornerLine((cx, cy), end, color))

    _corner(TrackDimensions.CORNER_MIN, TrackDimensions.CORNER_MIN, -1, -1)  # SW
    _corner(TrackDimensions.CORNER_MAX, TrackDimensions.CORNER_MIN, 1, -1)   # SE
    _corner(TrackDimensions.CORNER_MAX, TrackDimensions.CORNER_MAX, 1, 1)    # NE
    _corner(TrackDimensions.CORNER_MIN, TrackDimensions.CORNER_MAX, -1, 1)   # NW
    return lines
