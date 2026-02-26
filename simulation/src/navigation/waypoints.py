"""Waypoint generation for WRO 2026 track navigation.

Computes a list of (x, y) waypoints that the TrackNavigator follows.
Uses circular arc waypoints at corners to stay within the Ackermann robot's
minimum turning radius (~0.294 m).

All functions are pure — they accept data and return results without I/O.
"""

from __future__ import annotations

import math


def calculate_waypoints(
    metadata: dict,
    num_laps: int,
) -> list[tuple[float, float]]:
    """Build the full multi-lap waypoint sequence for a scenario.

    Reads corridor widths and starting conditions from the scenario metadata
    and constructs arc waypoints at corners plus straight waypoints along
    each corridor centerline.

    Args:
        metadata: Scenario metadata dict (from ScenarioGenerator._build_metadata).
        num_laps: Total laps the robot must complete.

    Returns:
        Ordered list of (x, y) world-frame waypoints starting near the robot's
        spawn position, covering num_laps full loops.
    """
    corridor_widths = metadata["corridor_widths"]
    direction = metadata["starting_conditions"]["direction"]

    # Convert corridor widths from mm to meters
    north_width = corridor_widths["north"]["width_mm"] / 1000.0
    south_width = corridor_widths["south"]["width_mm"] / 1000.0
    east_width = corridor_widths["east"]["width_mm"] / 1000.0
    west_width = corridor_widths["west"]["width_mm"] / 1000.0

    track_max = 3.0
    # Bias corridor centers toward outer walls for inner-wall clearance
    outer_bias = 0.05
    north_cy = track_max - north_width / 2 + outer_bias
    south_cy = south_width / 2 - outer_bias
    east_cx = track_max - east_width / 2 + outer_bias
    west_cx = west_width / 2 - outer_bias

    # Arc radius must exceed the robot's minimum turning radius (~0.294 m).
    # 0.45 m starts corners early enough to clear inner-wall junctions.
    arc_radius = 0.45

    segments = _build_all_segments(
        north_cy, south_cy, east_cx, west_cx, arc_radius, direction
    )

    order = _build_corridor_order(direction)
    start_section = metadata["starting_conditions"]["section"].lower()
    order = _rotate_to_start(order, start_section)

    full_loop = _assemble_loop(order, segments)
    start_pos = metadata["starting_conditions"]["position"]
    start_x, start_y = start_pos["x"], start_pos["y"]

    return _build_waypoint_sequence(
        full_loop, segments, order, start_x, start_y, num_laps
    )


# ── Segment builders ──────────────────────────────────────────────────────────

def _build_all_segments(
    north_cy: float,
    south_cy: float,
    east_cx: float,
    west_cx: float,
    arc_radius: float,
    direction: str,
) -> dict[str, list[tuple[float, float]]]:
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

    if direction == "clockwise":
        return {
            "east":  east_straight + se_cw,
            "south": south_straight + sw_cw,
            "west":  west_straight + nw_cw,
            "north": north_straight + ne_cw,
        }
    # Counter-clockwise: reverse each segment
    return {
        "east":  list(reversed(east_straight)) + list(reversed(ne_cw)),
        "south": list(reversed(south_straight)) + list(reversed(se_cw)),
        "west":  list(reversed(west_straight)) + list(reversed(sw_cw)),
        "north": list(reversed(north_straight)) + list(reversed(nw_cw)),
    }


def _build_corridor_order(direction: str) -> list[str]:
    if direction == "clockwise":
        return ["east", "south", "west", "north"]
    return ["east", "north", "west", "south"]


def _rotate_to_start(order: list[str], start_section: str) -> list[str]:
    """Rotate the corridor order so start_section comes first."""
    rotated = list(order)
    while rotated[0] != start_section:
        rotated.append(rotated.pop(0))
    return rotated


def _assemble_loop(
    order: list[str],
    segments: dict[str, list[tuple[float, float]]],
) -> list[tuple[float, float]]:
    loop: list[tuple[float, float]] = []
    for section in order:
        loop.extend(segments[section])
    return loop


def _build_waypoint_sequence(
    full_loop: list[tuple[float, float]],
    segments: dict[str, list[tuple[float, float]]],
    order: list[str],
    start_x: float,
    start_y: float,
    num_laps: int,
) -> list[tuple[float, float]]:
    """Build multi-lap waypoints starting from the closest point in the first segment."""
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

    return _deduplicate_consecutive(waypoints)


def _nearest_waypoint_index(
    waypoints: list[tuple[float, float]],
    x: float,
    y: float,
) -> int:
    min_dist = float("inf")
    nearest_index = 0
    for index, (wx, wy) in enumerate(waypoints):
        dist = math.sqrt((wx - x) ** 2 + (wy - y) ** 2)
        if dist < min_dist:
            min_dist = dist
            nearest_index = index
    return nearest_index


def _deduplicate_consecutive(
    waypoints: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Remove consecutive duplicate waypoints (within 1 mm)."""
    if not waypoints:
        return []
    deduped = [waypoints[0]]
    for point in waypoints[1:]:
        prev = deduped[-1]
        if abs(point[0] - prev[0]) > 0.001 or abs(point[1] - prev[1]) > 0.001:
            deduped.append(point)
    return deduped


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _arc_with_endpoints(
    center: tuple[float, float],
    radius: float,
    theta_start: float,
    theta_end: float,
    num_intermediate: int = 3,
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
    return [entry] + intermediates + [exit_pt]


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
        points.append((
            round(cx + radius * math.cos(theta), 3),
            round(cy + radius * math.sin(theta), 3),
        ))
    return points


def _straight_waypoints(
    fixed_coord: float,
    is_x: bool,
    start: float,
    end: float,
    count: int = 8,
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
