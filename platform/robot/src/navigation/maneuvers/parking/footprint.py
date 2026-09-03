"""Chassis-footprint containment and collision predicates for parking.

Given a pose and the lot ``ParkZone``, these decide whether the whole chassis is
inside the bay, whether its heading is wall-parallel, and whether any corner has
breached the field wall or a marker fin. Pure geometry; no ROS2, no controller
state.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs
from shared.domain.models import BBox, Waypoint

from src.navigation.maneuvers.parking.context import DEFAULT_PARKING_CONTEXT

if TYPE_CHECKING:
    from src.navigation.maneuvers.parking.zone import ParkZone


def chassis_corners(
    rx: float,
    ry: float,
    robot_yaw: float,
) -> list[tuple[float, float]]:
    """The four corners of the chassis footprint at this pose (world frame)."""
    half_l, half_w = RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    return [
        (rx + dx * cos_yaw - dy * sin_yaw, ry + dx * sin_yaw + dy * cos_yaw)
        for dx, dy in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
    ]


def is_beyond_lot_centre(coord: float, zone: ParkZone) -> bool:
    """Whether ``coord`` lies on the wall side of the lot's midline."""
    centre = zone.gap_cx if zone.wall_is_x else zone.gap_cy
    return coord > centre if zone.wall_coord > centre else coord < centre


def footprint_inside(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> bool:
    """Whether the robot's whole projection on the mat lies inside the parking lot.

    This is the rule as written ("the projection of the robot on the mat is fully
    inside the rectangle between the two markers"), not the centre-of-chassis
    approximation it replaces. The difference is not cosmetic: a centre-in-box test
    reports a successful park for a robot sitting mostly in the corridor, or with its
    nose through the outer wall, because neither the footprint nor the heading
    enters into it.
    """
    lot = BBox(zone.x_min, zone.y_min, zone.x_max, zone.y_max)
    return all(lot.contains(Waypoint(cx, cy)) for cx, cy in chassis_corners(rx, ry, robot_yaw))


def footprint_overlaps_lot(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> bool:
    """Whether ANY part of the chassis projection lies inside the parking lot.

    The partial-credit counterpart to :func:`footprint_inside`. WRO scores parking
    in two tiers -- 15 points for "completely in the parking area and parallel"
    (1.8.2) and **7 for "parking partly or not parallel"** (1.8.3) -- and until
    2026-09-03 nothing in this repo could express the second, so every sweep
    measured only the tier the chassis geometrically cannot reach.

    A true rectangle-rectangle overlap, not a corner-in-box test: at the headings
    that matter here the chassis can straddle the lot mouth with no corner of
    either rectangle inside the other, which a corner test reports as "outside".
    Separating-axis over both rectangles' edge normals, in the zone's
    ``(along, depth)`` frame where the lot is axis-aligned.
    """
    along_min, along_max = zone.bounds_along()
    depth_min, depth_max = zone.bounds_depth()
    chassis = [zone.project(cx, cy) for cx, cy in chassis_corners(rx, ry, robot_yaw)]
    lot = [
        (along_min, depth_min),
        (along_max, depth_min),
        (along_max, depth_max),
        (along_min, depth_max),
    ]
    # The lot's own normals are the frame axes; the chassis contributes two more.
    # A gap on ANY axis separates the rectangles, so overlap needs all four to
    # overlap. Two edges suffice per rectangle -- opposite edges share a normal.
    axes = [(1.0, 0.0), (0.0, 1.0)]
    axes += [
        (chassis[i][1] - chassis[i - 1][1], chassis[i - 1][0] - chassis[i][0])  # edge normal
        for i in (1, 2)
    ]
    for ax, ay in axes:
        c_lo = min(px * ax + py * ay for px, py in chassis)
        c_hi = max(px * ax + py * ay for px, py in chassis)
        l_lo = min(px * ax + py * ay for px, py in lot)
        l_hi = max(px * ax + py * ay for px, py in lot)
        if c_hi <= l_lo or l_hi <= c_lo:
            return False
    return True


def footprint_breaches_wall(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
    standoff_m: float | None = None,
) -> bool:
    """Whether any chassis corner has come within the wall standoff of the field wall.

    ENTER pure-pursues the lot centre, which controls position but not heading, so
    a robot that arrives across the lot rather than along it drives its nose at the
    wall and keeps going. Previously this was masked: the old zone extended past the
    lot's real far edge and used a centre-in-box test, so the maneuver "succeeded"
    and stopped short of the wall by accident. With the stop condition corrected to
    the actual rule, nothing stops it any more -- so the maneuver gives up here
    instead of pushing into the wall. Not colliding takes priority over completing
    the park.

    ``standoff_m`` defaults to the controller's safety margin. SCORING must pass
    ``0.0``: that margin exists to make the maneuver give up early, so scoring
    through it reports a legal park as a breach.
    """
    standoff = DEFAULT_PARKING_CONTEXT.constants.wall_standoff_m if standoff_m is None else standoff_m
    for cx, cy in chassis_corners(rx, ry, robot_yaw):
        coord = cx if zone.wall_is_x else cy
        if abs(coord - zone.wall_coord) < standoff and is_beyond_lot_centre(coord, zone):
            return True
    return False


def footprint_breaches_markers(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
    standoff_m: float | None = None,
) -> bool:
    """Whether any chassis corner has come within the marker standoff of a fin.

    The wall guard alone used to be sufficient by accident: with the steering limit
    modelled at 30 deg the chassis could not turn tightly enough to swing a corner
    into a fin before the wall stopped it. At the real ~70 deg lock (R_min 0.034 m
    rather than 0.165 m) ENTER's pure pursuit of the lot centre turns hard enough to
    reach them, so the fins need the same explicit give-up the wall has. Same
    priority as there: not colliding beats parking.

    A fin flanks the lot along the wall and spans its full depth, so a corner is in
    fin territory when it lies within the lot's depth band and at or past a fin's
    inner face.

    ``standoff_m`` defaults to the controller's safety margin; SCORING passes
    ``0.0`` -- see :func:`footprint_breaches_wall`. Touching a fin is not merely a
    collision here: WRO stops the robot and voids ALL parking points for it
    (ruled 2026-09-03), so this predicate at zero standoff is the scorer's veto.
    """
    marker_standoff = (
        DEFAULT_PARKING_CONTEXT.constants.marker_standoff_m if standoff_m is None else standoff_m
    )
    depth_min, depth_max = zone.bounds_depth()
    along_min, along_max = zone.bounds_along()
    for cx, cy in chassis_corners(rx, ry, robot_yaw):
        along, depth = zone.project(cx, cy)
        if not (depth_min - marker_standoff <= depth <= depth_max + marker_standoff):
            continue  # out in the corridor, past the fins' ends -- nothing to hit
        if along <= along_min + marker_standoff or along >= along_max - marker_standoff:
            return True
    return False
