"""Per-corridor segment assembly for waypoint generation.

Builds the straight + corner-arc waypoint list for each section, stitches a
full lap loop, and produces the multi-lap sequence aligned to the robot's
start pose. Also holds the loop-end validation and consecutive-dedupe
helpers. Pure functions: data in, waypoints out.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import TrackDimensions
from shared.domain.enums import Direction, Section
from shared.domain.models import Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.waypoints.geometry import _arc_with_endpoints, _straight_waypoints

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning


def _build_all_segments(
    north_cy: float,
    south_cy: float,
    east_cx: float,
    west_cx: float,
    corner_radii: dict[str, float],
    direction: Direction,
    tuning: NavigationTuning | None = None,
) -> dict[Section, list[Waypoint]]:
    """Construct per-corridor waypoint lists (straights + corner arcs) for both directions.

    Args:
        north_cy: North corridor centreline (y), already bias-adjusted.
        south_cy: South corridor centreline (y).
        east_cx: East corridor centreline (x).
        west_cx: West corridor centreline (x).
        corner_radii: Arc radius per corner, keyed ``se``/``sw``/``nw``/``ne``.
            Sized per corner by the corner-arc-radius helper, so the four can
            differ and each straight is trimmed by the radius of the corner at
            its own end rather than by one shared value.
        direction: Travel direction; CCW reverses each segment.
        tuning: Navigation tuning instance. Defaults to loaded defaults.

    Returns:
        Per-section waypoint lists, each a straight followed by its exit arc.

    Uses tuning: waypoints.NUM_INTERMEDIATE_ARC_POINTS, STRAIGHT_WAYPOINT_COUNT
    """
    tuning = get_tuning(tuning)
    num_intermediate = tuning.waypoints.NUM_INTERMEDIATE_ARC_POINTS
    straight_count = tuning.waypoints.STRAIGHT_WAYPOINT_COUNT

    r_se, r_sw, r_nw, r_ne = (corner_radii[k] for k in ("se", "sw", "nw", "ne"))

    # Corner arc ICR positions and arc angle ranges (CW direction)
    se_icr = Waypoint(east_cx - r_se, south_cy + r_se)
    sw_icr = Waypoint(west_cx + r_sw, south_cy + r_sw)
    nw_icr = Waypoint(west_cx + r_nw, north_cy - r_nw)
    ne_icr = Waypoint(east_cx - r_ne, north_cy - r_ne)

    se_cw = _arc_with_endpoints(se_icr, r_se, 0.0, -math.pi / 2, num_intermediate)
    sw_cw = _arc_with_endpoints(sw_icr, r_sw, -math.pi / 2, -math.pi, num_intermediate)
    nw_cw = _arc_with_endpoints(nw_icr, r_nw, math.pi, math.pi / 2, num_intermediate)
    ne_cw = _arc_with_endpoints(ne_icr, r_ne, math.pi / 2, 0.0, num_intermediate)

    # CW straight segments. Each end is trimmed by the radius of the corner it
    # runs into, which is why the two bounds no longer share a value.
    east_straight = _straight_waypoints(
        east_cx,
        is_x=True,
        start=north_cy - r_ne,
        end=south_cy + r_se,
        count=straight_count,
    )
    south_straight = _straight_waypoints(
        south_cy,
        is_x=False,
        start=east_cx - r_se,
        end=west_cx + r_sw,
        count=straight_count,
    )
    west_straight = _straight_waypoints(
        west_cx,
        is_x=True,
        start=south_cy + r_sw,
        end=north_cy - r_nw,
        count=straight_count,
    )
    north_straight = _straight_waypoints(
        north_cy,
        is_x=False,
        start=west_cx + r_nw,
        end=east_cx - r_ne,
        count=straight_count,
    )

    if direction is Direction.CLOCKWISE:
        return {
            Section.EAST: east_straight + se_cw,
            Section.SOUTH: south_straight + sw_cw,
            Section.WEST: west_straight + nw_cw,
            Section.NORTH: north_straight + ne_cw,
        }
    # Counter-clockwise: reverse each segment
    return {
        Section.EAST: list(reversed(east_straight)) + list(reversed(ne_cw)),
        Section.SOUTH: list(reversed(south_straight)) + list(reversed(se_cw)),
        Section.WEST: list(reversed(west_straight)) + list(reversed(sw_cw)),
        Section.NORTH: list(reversed(north_straight)) + list(reversed(nw_cw)),
    }


def _assemble_loop(
    order: list[Section],
    segments: dict[Section, list[Waypoint]],
) -> list[Waypoint]:
    loop: list[Waypoint] = []
    for section in order:
        loop.extend(segments[section])
    return loop


def _build_waypoint_sequence(
    full_loop: list[Waypoint],
    segments: dict[Section, list[Waypoint]],
    order: list[Section],
    start_x: float,
    start_y: float,
    num_laps: int,
    tuning: NavigationTuning | None = None,
) -> list[Waypoint]:
    """Build multi-lap waypoints starting from the closest point in the first segment.

    Uses tuning: waypoints.DEDUPE_DISTANCE_M
    """
    tuning = get_tuning(tuning)
    first_seg = segments[order[0]]
    start_index = _nearest_waypoint_index(first_seg, start_x, start_y)

    partial_first = first_seg[start_index:]
    remaining_first_lap = _assemble_loop(order[1:], segments)
    waypoints = partial_first + remaining_first_lap

    for _ in range(num_laps - 1):
        waypoints.extend(full_loop)

    # Complete the skipped portion of the first segment
    if start_index > 0:
        waypoints.extend(first_seg[:start_index])

    return _deduplicate_consecutive(waypoints, tuning)


def _nearest_waypoint_index(
    waypoints: list[Waypoint],
    x: float,
    y: float,
) -> int:
    pts = np.array([(wp.x, wp.y) for wp in waypoints])
    deltas = pts - np.array([x, y])
    return int(np.argmin((deltas**2).sum(axis=1)))


def _validate_bounds(waypoints: list[Waypoint]) -> None:
    """Raise ValueError if generation produced an out-of-bounds waypoint.

    Malformed metadata (e.g. mm-vs-m width) can otherwise silently produce
    wall-crossing waypoints. Every waypoint must stay on the track and clear
    of the restricted inner square.
    """
    for wp in waypoints:
        x, y = wp.x, wp.y
        if not (TrackDimensions.MIN_COORD <= x <= TrackDimensions.MAX_COORD) or not (
            TrackDimensions.MIN_COORD <= y <= TrackDimensions.MAX_COORD
        ):
            msg = f"Generated waypoint ({x:.3f}, {y:.3f}) falls outside the track bounds"
            raise ValueError(msg)
        if (
            TrackDimensions.CORNER_MIN < x < TrackDimensions.CORNER_MAX
            and TrackDimensions.CORNER_MIN < y < TrackDimensions.CORNER_MAX
        ):
            msg = f"Generated waypoint ({x:.3f}, {y:.3f}) falls inside the restricted inner square"
            raise ValueError(msg)


def _deduplicate_consecutive(
    waypoints: list[Waypoint],
    tuning: NavigationTuning | None = None,
) -> list[Waypoint]:
    """Remove consecutive duplicate waypoints (within dedupe distance).

    Uses tuning: waypoints.DEDUPE_DISTANCE_M
    """
    tuning = get_tuning(tuning)
    if not waypoints:
        return []
    dedupe_distance_m = tuning.waypoints.DEDUPE_DISTANCE_M
    deduped = [waypoints[0]]
    for point in waypoints[1:]:
        prev = deduped[-1]
        if abs(point.x - prev.x) > dedupe_distance_m or abs(point.y - prev.y) > dedupe_distance_m:
            deduped.append(point)
    return deduped
