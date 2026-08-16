"""Waypoint generation for WRO 2026 track navigation.

Computes a list of Waypoints that the TrackNavigator follows.
Uses circular arc waypoints at corners to stay within the Ackermann robot's
minimum turning radius (~0.034 m, from WHEELBASE/tan(MAX_STEERING_ANGLE) with
counter-phase steering).

All functions are pure — they accept data and return results without I/O.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.domain.enums import CorridorSide, Direction, Section
from shared.domain.models import (
    CorridorWidthEntry,
    CorridorWidths,
    PathPlannability,
    Position2D,
    ScenarioMetadata,
    Waypoint,
)

from src.config.tuning_helpers import get_tuning

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

_INNER_MIN = TrackDimensions.CORNER_MIN  # 1.0 m
_INNER_MAX = TrackDimensions.CORNER_MAX  # 2.0 m


def validate_path_feasibility(min_corridor_width_m: float, center_bias_m: float) -> PathPlannability:
    """Check whether the chassis fits the narrowest corridor once biased off centre.

    The corner arcs deliberately do not appear here. :func:`_corner_arc_radius`
    caps every arc at the clearance the straights already have, so a corner can
    never be the tightest point on the path.

    The previous form added ``arc_radius`` to a lateral half-extent, which are
    not commensurable -- one is a path curvature, the other a width -- and it
    ignored ``center_bias_m`` entirely, so it scored a centred path and a biased
    one identically while the bias was the thing actually spending the margin.

    Args:
        min_corridor_width_m: Minimum corridor width across all four sides.
        center_bias_m: Signed offset of the path from the centreline, positive
            toward the inner block. Only the magnitude matters: biasing either
            way moves the chassis toward one wall by the same amount.

    Returns:
        PathPlannability with margin and reason, in corridor-width units. The
        margin is the total slack across both walls; clearance to the nearer
        wall is half of it.
    """
    required = RobotSpecs.WIDTH + 2 * abs(center_bias_m)
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


def _corner_arc_radius(width_entry_m: float, width_exit_m: float, center_bias_m: float, max_radius: float) -> float:
    """Largest arc radius at one corner that costs no clearance to the inner block.

    A corner arc is tangent to both corridor centrelines, so its centre sits at
    ``(radius, radius)`` in from their intersection. The path's distance to the
    inner block corner is therefore set by the radius, and it stays at the value
    the straights already have -- ``width/2 - center_bias_m`` -- right up until
    the radius passes that value, after which the arc bulges past the centreline
    and starts eating margin. Clearance is flat below that point rather than
    peaked, so the largest such radius is strictly best: a wider arc means a
    shorter lap (``dP/dr = 2*pi - 8``, i.e. -1.717 m per metre of radius) and
    gentler steering, both for free until the cap binds.

    Takes the ``max`` of the two corridors, not the ``min``: the radius has to
    reach the wider corridor's centreline to be tangent to it, and forcing it
    down to the narrow side's value pulls the arc off that tangent and *into*
    the corner -- 0.05 m of clearance on a mixed corner instead of 0.25 m.
    ``max`` is also symmetric, so a corner plans the same arc whichever way the
    round is driven.

    With the checked-in 0.6/1.0 m widths and a 0.05 m inner bias this returns
    the configured 0.45 m everywhere except a narrow-to-narrow corner, where it
    returns 0.25 m and doubles that corner's clearance.

    Args:
        width_entry_m: Width of the corridor the corner is entered from.
        width_exit_m: Width of the corridor it exits into.
        center_bias_m: Signed path offset from the centreline, positive toward
            the inner block. An outward bias leaves more room, so it widens the
            arc by the same amount.
        max_radius: Ceiling from ``waypoints.ARC_RADIUS``. Only binds if a
            corridor is wider than the geometry this track can present.

    Returns:
        Arc radius in metres.
    """
    return min(max_radius, max(width_entry_m, width_exit_m) / 2 - center_bias_m)


def calculate_waypoints(
    metadata: ScenarioMetadata | dict[str, Any],
    num_laps: int,
    arc_radius: float | None = None,
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> list[Waypoint]:
    """Build the full multi-lap waypoint sequence for a scenario.

    Reads corridor widths and starting conditions from the scenario metadata
    and constructs arc waypoints at corners plus straight waypoints along
    each corridor centerline.

    Args:
        metadata: Scenario metadata (Pydantic model or coercible dict).
        num_laps: Total laps the robot must complete.
        arc_radius: Ceiling on the corner arc radius (m). Each corner picks its
            own radius from the two corridors it joins -- see
            :func:`_corner_arc_radius` -- and this only caps the result, so it
            no longer sets the geometry on its own. Must exceed the Ackermann
            minimum turning radius (~0.034 m, from
            WHEELBASE/tan(MAX_STEERING_ANGLE) with counter-phase steering).
            Defaults to the tuning profile's value so a loaded profile actually
            takes effect instead of a value frozen at import time.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: Centreline shift MAGNITUDE (m), overriding
            ``waypoints.CENTER_BIAS_M``. ``None`` keeps the tuning value, so
            every existing caller is unchanged. The Obstacles Challenge passes
            ``waypoints.OBSTACLES_CENTER_BIAS_M`` (0.0, centred) -- see that
            field for why the inner bias is an Open-Challenge-only argument.

    Returns:
        Ordered list of world-frame Waypoints starting near the robot's
        spawn position, covering num_laps full loops.

    Raises:
        ValueError: If a generated or deformed waypoint would fall outside
            the track or inside the restricted inner square.

    Uses tuning: waypoints.ARC_RADIUS, CENTER_BIAS_M, CENTER_BIAS_SIDE
    """
    tuning = get_tuning(tuning)
    if not isinstance(metadata, ScenarioMetadata):
        metadata = ScenarioMetadata.model_validate(metadata)
    arc_radius = arc_radius if arc_radius is not None else tuning.waypoints.ARC_RADIUS

    corridor_widths = metadata.corridor_widths
    starting = metadata.starting_conditions
    if starting.direction is None:
        msg = (
            "calculate_waypoints requires a resolved starting_conditions.direction; "
            "callers must infer/assign it (see TrackNavigator._plan / ScenarioSimulator._plan) "
            "before planning a path"
        )
        raise ValueError(msg)
    direction = starting.direction

    cw_entries = {
        Section.NORTH: corridor_widths.north,
        Section.SOUTH: corridor_widths.south,
        Section.EAST: corridor_widths.east,
        Section.WEST: corridor_widths.west,
    }
    # Derive center bias from tuning (positive toward inner block). An explicit
    # magnitude overrides the tuning default so the Obstacles Challenge can
    # plan down the middle without changing Open's value -- see
    # WaypointParams.OBSTACLES_CENTER_BIAS_M. The SIDE still comes from tuning:
    # only the distance differs between the challenges, and a zero magnitude
    # makes the side moot anyway.
    bias_magnitude = tuning.waypoints.CENTER_BIAS_M if center_bias_m is None else center_bias_m
    center_bias_m = bias_magnitude * (
        1.0 if tuning.waypoints.CENTER_BIAS_SIDE is CorridorSide.INNER else -1.0
    )

    min_width_m = min(cw.width_mm for cw in cw_entries.values()) / 1000.0
    feasibility = validate_path_feasibility(min_width_m, center_bias_m)
    if not feasibility.is_feasible:
        raise ValueError(feasibility.reason)

    widths = {section: cw.width_mm / 1000.0 for section, cw in cw_entries.items()}
    north_width = widths[Section.NORTH]
    south_width = widths[Section.SOUTH]
    east_width = widths[Section.EAST]
    west_width = widths[Section.WEST]

    track_max = TrackDimensions.MAX_COORD
    # Signs put the bias toward the inner block on every side: north and east
    # corridors have the block below/left of them, south and west above/right.
    north_cy = track_max - north_width / 2 - center_bias_m
    south_cy = south_width / 2 + center_bias_m
    east_cx = track_max - east_width / 2 - center_bias_m
    west_cx = west_width / 2 + center_bias_m

    # Each corner is sized by the two corridors it joins, so a narrow-to-narrow
    # corner tightens while the rest keep the configured radius.
    corner_radii = {
        corner: _corner_arc_radius(entry_w, exit_w, center_bias_m, arc_radius)
        for corner, (entry_w, exit_w) in {
            "se": (east_width, south_width),
            "sw": (south_width, west_width),
            "nw": (west_width, north_width),
            "ne": (north_width, east_width),
        }.items()
    }

    segments = _build_all_segments(
        north_cy,
        south_cy,
        east_cx,
        west_cx,
        corner_radii,
        direction,
        tuning,
    )

    order = _build_corridor_order(direction)

    order = _rotate_to_start(order, starting.section)

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


def corridor_widths_dict_to_model(widths: dict[Section, float]) -> CorridorWidths:
    """Convert a per-section width belief (metres) into the on-model mm representation."""
    return CorridorWidths(
        **{s.value: CorridorWidthEntry(width_mm=round(width * 1000)) for s, width in widths.items()},
    )


def plan_believed_path(
    metadata: ScenarioMetadata,
    widths: dict[Section, float],
    *,
    direction: Direction,
    believed_section: Section,
    believed_position: Position2D,
    believed_yaw: float,
    arc_radius: float | None,
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> list[Waypoint]:
    """Build a one-lap path for the layout the robot currently believes it is on.

    Shared by ``ScenarioSimulator._plan`` and ``TrackNavigator._plan``: both
    replan from a *believed* corridor-width estimate and a believed start pose
    that can differ from ``metadata.starting_conditions`` (the ground-truth /
    on-file record), which is why the believed section/position/yaw/direction
    are threaded in separately rather than read off ``metadata`` itself.

    ``num_laps=1`` is deliberate: :func:`calculate_waypoints` bakes the lap
    count into the returned list, but ``CoreNavigator`` already cycles one
    canonical lap the real lap count times (see its waypoint-wrap in
    ``step()``). Passing the real count here would multiply laps
    (e.g. 3 -> 9).

    Args:
        metadata: Validated scenario metadata; only its ``starting_conditions``
            and structure are used, both re-derived below with the believed
            widths/pose swapped in.
        widths: Believed corridor width per section (m).
        direction: Believed travel direction.
        believed_section: Section the robot believes it is standing in.
        believed_position: Position the robot believes it is standing at.
        believed_yaw: Heading the robot believes it is facing.
        arc_radius: Ceiling on the corner arc radius (m), forwarded to
            :func:`calculate_waypoints`.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: Centreline shift magnitude (m); forwarded to
            :func:`calculate_waypoints`. ``None`` keeps the tuning value.

    Returns:
        Single-lap ordered list of world-frame Waypoints.
    """
    new_widths = corridor_widths_dict_to_model(widths)
    new_starting = metadata.starting_conditions.replanned_at(
        direction=direction,
        section=believed_section,
        position=believed_position,
        yaw=believed_yaw,
    )
    planning_metadata = metadata.replanned_with(
        corridor_widths=new_widths,
        starting_conditions=new_starting,
    )
    return calculate_waypoints(
        planning_metadata,
        num_laps=1,
        arc_radius=arc_radius,
        tuning=tuning,
        center_bias_m=center_bias_m,
    )


# Segment builders
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
            Sized per corner by :func:`_corner_arc_radius`, so the four can
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
    se_icr = (east_cx - r_se, south_cy + r_se)
    sw_icr = (west_cx + r_sw, south_cy + r_sw)
    nw_icr = (west_cx + r_nw, north_cy - r_nw)
    ne_icr = (east_cx - r_ne, north_cy - r_ne)

    se_cw = _arc_with_endpoints(se_icr, r_se, 0.0, -math.pi / 2, num_intermediate)
    sw_cw = _arc_with_endpoints(sw_icr, r_sw, -math.pi / 2, -math.pi, num_intermediate)
    nw_cw = _arc_with_endpoints(nw_icr, r_nw, math.pi, math.pi / 2, num_intermediate)
    ne_cw = _arc_with_endpoints(ne_icr, r_ne, math.pi / 2, 0.0, num_intermediate)

    # CW straight segments. Each end is trimmed by the radius of the corner it
    # runs into, which is why the two bounds no longer share a value.
    east_straight = _straight_waypoints(
        east_cx, is_x=True, start=north_cy - r_ne, end=south_cy + r_se, count=straight_count,
    )
    south_straight = _straight_waypoints(
        south_cy, is_x=False, start=east_cx - r_se, end=west_cx + r_sw, count=straight_count,
    )
    west_straight = _straight_waypoints(
        west_cx, is_x=True, start=south_cy + r_sw, end=north_cy - r_nw, count=straight_count,
    )
    north_straight = _straight_waypoints(
        north_cy, is_x=False, start=west_cx + r_nw, end=east_cx - r_ne, count=straight_count,
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
        if _INNER_MIN < x < _INNER_MAX and _INNER_MIN < y < _INNER_MAX:
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


# Geometry helpers
def _arc_with_endpoints(
    center: tuple[float, float],
    radius: float,
    theta_start: float,
    theta_end: float,
    num_intermediate: int = 3,  # see NavigationTuning.waypoints.NUM_INTERMEDIATE_ARC_POINTS
) -> list[Waypoint]:
    """Generate arc points including entry and exit, with intermediate samples."""
    cx, cy = center
    entry = Waypoint(
        round(cx + radius * math.cos(theta_start), 3),
        round(cy + radius * math.sin(theta_start), 3),
    )
    exit_pt = Waypoint(
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
) -> list[Waypoint]:
    """Sample count evenly-spaced interior arc points (excluding endpoints)."""
    points: list[Waypoint] = []
    for step in range(1, count + 1):
        fraction = step / (count + 1)
        theta = theta_start + fraction * (theta_end - theta_start)
        points.append(
            Waypoint(
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
) -> list[Waypoint]:
    """Generate evenly-spaced waypoints along a corridor centerline.

    Args:
        fixed_coord: The constant coordinate (X if is_x else Y).
        is_x: True if fixed_coord is the X axis (East/West corridors).
        start: Starting value of the varying coordinate.
        end: Ending value of the varying coordinate.
        count: Number of waypoints to generate.

    Returns:
        List of Waypoints.
    """
    points: list[Waypoint] = []
    for step in range(count):
        fraction = step / (count - 1) if count > 1 else 0.5
        varying = start + fraction * (end - start)
        if is_x:
            points.append(Waypoint(round(fixed_coord, 3), round(varying, 3)))
        else:
            points.append(Waypoint(round(varying, 3), round(fixed_coord, 3)))
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
