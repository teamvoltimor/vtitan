"""Waypoint generation for WRO 2026 track navigation.

Computes a list of (x, y) waypoints that the TrackNavigator follows.
Uses circular arc waypoints at corners to stay within the Ackermann robot's
minimum turning radius (~0.034 m, from WHEELBASE/tan(MAX_STEERING_ANGLE) with
counter-phase steering).

All functions are pure — they accept data and return results without I/O.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.config.enums import CorridorSide, Direction, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import PathPlannability, ScenarioMetadata

_INNER_MIN = TrackDimensions.CORNER_MIN  # 1.0 m
_INNER_MAX = TrackDimensions.CORNER_MAX  # 2.0 m


def validate_path_feasibility(min_corridor_width_m: float, arc_radius: float) -> PathPlannability:
    """Check whether a path fits within the available corridor width.

    Args:
        min_corridor_width_m: Minimum corridor width across all four sides.
        arc_radius: Corner arc radius (m).

    Returns:
        PathPlannability with margin and reason.
    """
    required = RobotSpecs.WIDTH / 2 + arc_radius
    margin = min_corridor_width_m - required
    if margin > 0:
        return PathPlannability(
            is_feasible=True,
            min_required_m=required,
            min_available_m=min_corridor_width_m,
            margin_m=margin,
        )
    return PathPlannability(
        is_feasible=False,
        min_required_m=required,
        min_available_m=min_corridor_width_m,
        margin_m=margin,
        reason=f"Corridor too narrow: required {required:.3f} m, got {min_corridor_width_m:.3f} m",
    )


def calculate_waypoints(
    metadata: ScenarioMetadata | dict[str, Any],
    num_laps: int,
    arc_radius: float | None = None,
    tuning: NavigationTuning | None = None,
) -> list[tuple[float, float]]:
    """Build the full multi-lap waypoint sequence for a scenario.

    Reads corridor widths and starting conditions from the scenario metadata
    and constructs arc waypoints at corners plus straight waypoints along
    each corridor centerline.

    Args:
        metadata: Scenario metadata (Pydantic model or coercible dict).
        num_laps: Total laps the robot must complete.
        arc_radius: Corner arc radius (m). Must exceed the Ackermann minimum
            turning radius (~0.034 m, from WHEELBASE/tan(MAX_STEERING_ANGLE) with
            counter-phase steering). Defaults to the tuning profile's value so a
            loaded profile actually takes effect instead of a value frozen at
            import time.
        tuning: Navigation tuning instance. Defaults to loaded defaults.

    Returns:
        Ordered list of (x, y) world-frame waypoints starting near the robot's
        spawn position, covering num_laps full loops.

    Raises:
        ValueError: If a generated or deformed waypoint would fall outside
            the track or inside the restricted inner square.

    Uses tuning: waypoints.ARC_RADIUS, CENTER_BIAS_M, CENTER_BIAS_SIDE
    """
    if tuning is None:
        tuning = NavigationTuning.load_default()
    if not isinstance(metadata, ScenarioMetadata):
        metadata = ScenarioMetadata.model_validate(metadata)
    arc_radius = arc_radius if arc_radius is not None else tuning.waypoints.ARC_RADIUS

    corridor_widths = metadata.corridor_widths
    starting = metadata.starting_conditions
    direction = Direction.from_string(starting.direction)

    cw_entries = {
        Section.NORTH: corridor_widths.north,
        Section.SOUTH: corridor_widths.south,
        Section.EAST: corridor_widths.east,
        Section.WEST: corridor_widths.west,
    }
    min_width_m = min(cw.width_mm for cw in cw_entries.values()) / 1000.0
    feasibility = validate_path_feasibility(min_width_m, arc_radius)
    if not feasibility.is_feasible:
        raise ValueError(feasibility.reason)

    widths = {section: cw.width_mm / 1000.0 for section, cw in cw_entries.items()}
    north_width = widths[Section.NORTH]
    south_width = widths[Section.SOUTH]
    east_width = widths[Section.EAST]
    west_width = widths[Section.WEST]

    # Derive center bias from tuning (positive toward inner block)
    center_bias_m = tuning.waypoints.CENTER_BIAS_M * (
        1.0 if tuning.waypoints.CENTER_BIAS_SIDE is CorridorSide.INNER else -1.0
    )

    track_max = TrackDimensions.MAX_COORD
    # Signs put the bias toward the inner block on every side: north and east
    # corridors have the block below/left of them, south and west above/right.
    north_cy = track_max - north_width / 2 - center_bias_m
    south_cy = south_width / 2 + center_bias_m
    east_cx = track_max - east_width / 2 - center_bias_m
    west_cx = west_width / 2 + center_bias_m

    segments = _build_all_segments(
        north_cy,
        south_cy,
        east_cx,
        west_cx,
        arc_radius,
        direction,
    )

    order = _build_corridor_order(direction)

    start_section = Section.from_string(starting.section)
    order = _rotate_to_start(order, start_section)

    full_loop = _assemble_loop(order, segments)
    start_x, start_y = starting.position.x, starting.position.y

    waypoints = _build_waypoint_sequence(
        full_loop,
        segments,
        order,
        start_x,
        start_y,
        num_laps,
        tuning,
    )
    _validate_bounds(waypoints)
    return waypoints


# Segment builders
def _build_all_segments(
    north_cy: float,
    south_cy: float,
    east_cx: float,
    west_cx: float,
    arc_radius: float,
    direction: Direction,
) -> dict[Section, list[tuple[float, float]]]:
    """Construct per-corridor waypoint lists (straights + corner arcs) for both directions."""
    # Corner arc ICR positions and arc angle ranges (CW direction)
    se_icr = (east_cx - arc_radius, south_cy + arc_radius)
    sw_icr = (west_cx + arc_radius, south_cy + arc_radius)
    nw_icr = (west_cx + arc_radius, north_cy - arc_radius)
    ne_icr = (east_cx - arc_radius, north_cy - arc_radius)

    se_cw = _arc_with_endpoints(se_icr, arc_radius, 0.0, -math.pi / 2)
    sw_cw = _arc_with_endpoints(sw_icr, arc_radius, -math.pi / 2, -math.pi)
    nw_cw = _arc_with_endpoints(nw_icr, arc_radius, math.pi, math.pi / 2)
    ne_cw = _arc_with_endpoints(ne_icr, arc_radius, math.pi / 2, 0.0)

    # CW straight segments
    east_straight = _straight_waypoints(east_cx, is_x=True, start=north_cy - arc_radius, end=south_cy + arc_radius)
    south_straight = _straight_waypoints(south_cy, is_x=False, start=east_cx - arc_radius, end=west_cx + arc_radius)
    west_straight = _straight_waypoints(west_cx, is_x=True, start=south_cy + arc_radius, end=north_cy - arc_radius)
    north_straight = _straight_waypoints(north_cy, is_x=False, start=west_cx + arc_radius, end=east_cx - arc_radius)

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


def _build_corridor_order(direction: Direction) -> list[Section]:
    if direction is Direction.CLOCKWISE:
        return [Section.EAST, Section.SOUTH, Section.WEST, Section.NORTH]
    return [Section.EAST, Section.NORTH, Section.WEST, Section.SOUTH]


def _rotate_to_start(order: list[Section], start_section: Section) -> list[Section]:
    """Rotate the corridor order so start_section comes first."""
    rotated = list(order)
    while rotated[0] != start_section:
        rotated.append(rotated.pop(0))
    return rotated


def _assemble_loop(
    order: list[Section],
    segments: dict[Section, list[tuple[float, float]]],
) -> list[tuple[float, float]]:
    loop: list[tuple[float, float]] = []
    for section in order:
        loop.extend(segments[section])
    return loop


def _build_waypoint_sequence(
    full_loop: list[tuple[float, float]],
    segments: dict[Section, list[tuple[float, float]]],
    order: list[Section],
    start_x: float,
    start_y: float,
    num_laps: int,
    tuning: NavigationTuning | None = None,
) -> list[tuple[float, float]]:
    """Build multi-lap waypoints starting from the closest point in the first segment.

    Uses tuning: waypoints.DEDUPE_DISTANCE_M
    """
    if tuning is None:
        tuning = NavigationTuning.load_default()
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
    waypoints: list[tuple[float, float]],
    x: float,
    y: float,
) -> int:
    pts = np.array(waypoints)
    deltas = pts - np.array([x, y])
    return int(np.argmin((deltas**2).sum(axis=1)))


def _validate_bounds(waypoints: list[tuple[float, float]]) -> None:
    """Raise ValueError if generation produced an out-of-bounds waypoint.

    Malformed metadata (e.g. mm-vs-m width) can otherwise silently produce
    wall-crossing waypoints. Every waypoint must stay on the track and clear
    of the restricted inner square.
    """
    for x, y in waypoints:
        if not (TrackDimensions.MIN_COORD <= x <= TrackDimensions.MAX_COORD) or not (
            TrackDimensions.MIN_COORD <= y <= TrackDimensions.MAX_COORD
        ):
            msg = f"Generated waypoint ({x:.3f}, {y:.3f}) falls outside the track bounds"
            raise ValueError(msg)
        if _INNER_MIN < x < _INNER_MAX and _INNER_MIN < y < _INNER_MAX:
            msg = f"Generated waypoint ({x:.3f}, {y:.3f}) falls inside the restricted inner square"
            raise ValueError(msg)


def _deduplicate_consecutive(
    waypoints: list[tuple[float, float]],
    tuning: NavigationTuning | None = None,
) -> list[tuple[float, float]]:
    """Remove consecutive duplicate waypoints (within dedupe distance).

    Uses tuning: waypoints.DEDUPE_DISTANCE_M
    """
    if tuning is None:
        tuning = NavigationTuning.load_default()
    if not waypoints:
        return []
    dedupe_distance_m = tuning.waypoints.DEDUPE_DISTANCE_M
    deduped = [waypoints[0]]
    for point in waypoints[1:]:
        prev = deduped[-1]
        if abs(point[0] - prev[0]) > dedupe_distance_m or abs(point[1] - prev[1]) > dedupe_distance_m:
            deduped.append(point)
    return deduped


# Geometry helpers
def _arc_with_endpoints(
    center: tuple[float, float],
    radius: float,
    theta_start: float,
    theta_end: float,
    num_intermediate: int = 3,  # see NavigationTuning.waypoints.NUM_INTERMEDIATE_ARC_POINTS
) -> list[tuple[float, float]]:
    """Generate arc points including entry and exit, with intermediate samples."""
    cx, cy = center
    entry = (
        round(cx + radius * math.cos(theta_start), 3),
        round(cy + radius * math.sin(theta_start), 3),
    )
    exit_pt = (
        round(cx + radius * math.cos(theta_end), 3),
        round(cy + radius * math.sin(theta_end), 3),
    )
    intermediates = _arc_intermediate_points(cx, cy, radius, theta_start, theta_end, num_intermediate)
    return [entry, *intermediates, exit_pt]


def _arc_intermediate_points(
    cx: float,
    cy: float,
    radius: float,
    theta_start: float,
    theta_end: float,
    count: int,
) -> list[tuple[float, float]]:
    """Sample count evenly-spaced interior arc points (excluding endpoints)."""
    points: list[tuple[float, float]] = []
    for step in range(1, count + 1):
        fraction = step / (count + 1)
        theta = theta_start + fraction * (theta_end - theta_start)
        points.append(
            (
                round(cx + radius * math.cos(theta), 3),
                round(cy + radius * math.sin(theta), 3),
            ),
        )
    return points


def _straight_waypoints(
    fixed_coord: float,
    is_x: bool,
    start: float,
    end: float,
    count: int = 8,  # see NavigationTuning.waypoints.STRAIGHT_WAYPOINT_COUNT
) -> list[tuple[float, float]]:
    """Generate evenly-spaced waypoints along a corridor centerline.

    Args:
        fixed_coord: The constant coordinate (X if is_x else Y).
        is_x: True if fixed_coord is the X axis (East/West corridors).
        start: Starting value of the varying coordinate.
        end: Ending value of the varying coordinate.
        count: Number of waypoints to generate.

    Returns:
        List of (x, y) waypoints.
    """
    points: list[tuple[float, float]] = []
    for step in range(count):
        fraction = step / (count - 1) if count > 1 else 0.5
        varying = start + fraction * (end - start)
        if is_x:
            points.append((round(fixed_coord, 3), round(varying, 3)))
        else:
            points.append((round(varying, 3), round(fixed_coord, 3)))
    return points


def corridor_for_position(x: float, y: float) -> Section:
    """Classify which corridor section the robot is currently in.

    Uses the fixed inner-square boundaries (1.0–2.0 in both axes) to assign
    a cardinal section. In corner zones (both x and y outside the inner square
    range simultaneously), the nearest boundary face determines the section.

    Args:
        x: Robot world X position (metres).
        y: Robot world Y position (metres).

    Returns:
        Section enum for the current corridor.
    """
    in_x = _INNER_MIN <= x <= _INNER_MAX
    in_y = _INNER_MIN <= y <= _INNER_MAX

    if y < _INNER_MIN and in_x:
        return Section.SOUTH
    if y > _INNER_MAX and in_x:
        return Section.NORTH
    if x > _INNER_MAX and in_y:
        return Section.EAST
    if x < _INNER_MIN and in_y:
        return Section.WEST

    # Corner: classify by nearest inner-boundary face. Dict insertion order
    # (S, N, E, W) preserves the original tie-break.
    face_distances = {
        Section.SOUTH: abs(y - _INNER_MIN),
        Section.NORTH: abs(y - _INNER_MAX),
        Section.EAST: abs(x - _INNER_MAX),
        Section.WEST: abs(x - _INNER_MIN),
    }
    return min(face_distances, key=lambda section: face_distances[section])
