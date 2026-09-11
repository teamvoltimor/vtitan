"""Geometry primitives for waypoint generation.

Pure coordinate math: arc and straight segment sampling, and the corner-arc
radius that caps every arc at the straights' clearance. No I/O, no tuning
access beyond the radius ceiling passed in.
"""

from __future__ import annotations

import math

from shared.domain.models import Waypoint


def corner_arc_radius(
    width_entry_m: float, width_exit_m: float, center_bias_m: float, max_radius: float
) -> float:
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
        max_radius: Ceiling from the tuning profile's arc-radius value. Only
            binds if a corridor is wider than the geometry this track can present.

    Returns:
        Arc radius in metres.
    """
    return min(max_radius, max(width_entry_m, width_exit_m) / 2 - center_bias_m)


def arc_with_endpoints(
    center: Waypoint,
    radius: float,
    theta_start: float,
    theta_end: float,
    num_intermediate: int,
) -> list[Waypoint]:
    """Generate arc points including entry and exit, with intermediate samples.

    ``num_intermediate`` has no default: the production caller already sources
    it from the tuning file (``waypoints.NUM_INTERMEDIATE_ARC_POINTS``), and a
    local default here duplicated that shipped value, so a caller forgetting
    the argument would silently plan past a corner with a different sample
    count than the profile said without anything failing.
    """
    cx, cy = center.x, center.y
    entry = Waypoint(
        round(cx + radius * math.cos(theta_start), 3),
        round(cy + radius * math.sin(theta_start), 3),
    )
    exit_pt = Waypoint(
        round(cx + radius * math.cos(theta_end), 3),
        round(cy + radius * math.sin(theta_end), 3),
    )
    intermediates = arc_intermediate_points(cx, cy, radius, theta_start, theta_end, num_intermediate)
    return [entry, *intermediates, exit_pt]


def arc_intermediate_points(
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
            )
        )
    return points


def straight_waypoints(
    fixed_coord: float,
    is_x: bool,
    start: float,
    end: float,
    count: int,
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
