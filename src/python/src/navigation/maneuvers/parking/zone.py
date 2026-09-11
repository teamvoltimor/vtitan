"""Parking-lot zone geometry: the bay rectangle and staging point.

The two markers are fins standing *perpendicular* to the outer wall, so the bay
is a pocket closed on three sides and open only toward the corridor. This module
builds that rectangle (and the wall-parallel target heading) from the block
positions, plus the staging point in front of the opening.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import ParkingLotSpecs, TrackDimensions
from shared.domain.enums import Direction, Section
from shared.domain.models import BBox, BlockPosition, Waypoint

if TYPE_CHECKING:
    from src.navigation.maneuvers.parking.context import ParkingContext


@dataclass(frozen=True)
class ParkZone:
    """The parking lot rectangle (WRO: "the rectangle between the two markers").

    This is the bay itself, not a tolerance box around it: bounded along the wall
    by the two fins' inner faces, and in depth by the outer wall and the fins'
    inner ends. The robot's whole projection has to fit inside it, so it is
    deliberately the *true* lot outline with no slack added -- containment margin
    belongs in the stop check, not here.
    """

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    target_yaw: float  # expected robot yaw when parked (radians), parallel to the outer wall
    gap_cx: float  # centre of the bay (world x)
    gap_cy: float  # centre of the bay (world y)
    wall_is_x: bool  # whether the field wall backing this lot runs along x (E/W sections)
    wall_coord: float  # the wall's coordinate on the axis normal to it

    def project(self, x: float, y: float) -> tuple[float, float]:
        """Split a world point into (along-wall, depth) coordinates for this lot.

        Which world axis plays which role flips between the N/S and E/W
        corridors, and getting it backwards silently swaps the bay's 0.43 m mouth
        for its 0.20 m depth. Both guards below need the same split, so it is
        derived once here rather than re-spelled at each use.
        """
        return (y, x) if self.wall_is_x else (x, y)

    def bounds_along(self) -> tuple[float, float]:
        """The lot's extent along the wall -- i.e. between the two fins' inner faces."""
        return (self.y_min, self.y_max) if self.wall_is_x else (self.x_min, self.x_max)

    def bounds_depth(self) -> tuple[float, float]:
        """The lot's extent out from the wall -- i.e. the depth the fins span."""
        return (self.x_min, self.x_max) if self.wall_is_x else (self.y_min, self.y_max)


def build_zone(
    b1: BlockPosition,
    b2: BlockPosition,
    section: Section,
    direction: Direction,
) -> ParkZone:
    """Compute the parking lot rectangle and the wall-parallel target yaw.

    The markers are fins perpendicular to the outer wall: ``ParkingLotSpecs.WIDTH``
    (20 mm) thick along the wall, ``ParkingLotSpecs.LENGTH`` (200 mm) deep out from
    it. So the lot spans, along the wall, between the fins' inner faces, and in depth
    from the wall out to the fins' inner ends.

    ``target_yaw`` is parallel to the outer wall -- the lot is only as deep as the
    chassis is wide, so a nose-in pose cannot fit and is not what the rule asks for.
    Of the two parallel headings, the one matching ``direction`` of travel is chosen,
    so the robot never has to turn around inside a bay with no room to do it.
    """
    half_fin_thickness = ParkingLotSpecs.WIDTH / 2
    cw = direction is Direction.CLOCKWISE

    if section in (Section.SOUTH, Section.NORTH):
        x1, x2 = sorted([b1.x, b2.x])
        x_min = x1 + half_fin_thickness
        x_max = x2 - half_fin_thickness
        if section is Section.SOUTH:
            y_min, y_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + ParkingLotSpecs.LENGTH
            target_yaw = math.pi if cw else 0.0
        else:
            y_min, y_max = TrackDimensions.MAX_COORD - ParkingLotSpecs.LENGTH, TrackDimensions.MAX_COORD
            target_yaw = 0.0 if cw else math.pi
    else:
        y1, y2 = sorted([b1.y, b2.y])
        y_min = y1 + half_fin_thickness
        y_max = y2 - half_fin_thickness
        if section is Section.EAST:
            x_min, x_max = TrackDimensions.MAX_COORD - ParkingLotSpecs.LENGTH, TrackDimensions.MAX_COORD
            target_yaw = math.pi / 2 if cw else -math.pi / 2
        else:
            x_min, x_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + ParkingLotSpecs.LENGTH
            target_yaw = -math.pi / 2 if cw else math.pi / 2

    wall_is_x = section in (Section.EAST, Section.WEST)
    if wall_is_x:
        wall_coord = x_max if section is Section.EAST else x_min
    else:
        wall_coord = y_max if section is Section.NORTH else y_min

    return ParkZone(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        target_yaw=target_yaw,
        gap_cx=BBox(x_min, y_min, x_max, y_max).center.x,
        gap_cy=BBox(x_min, y_min, x_max, y_max).center.y,
        wall_is_x=wall_is_x,
        wall_coord=wall_coord,
    )


def staging_pos(zone: ParkZone, section: Section, context: ParkingContext) -> Waypoint:
    """Position directly in front of the gap opening, on the track side."""
    clearance = context.constants.approach_clearance
    if section is Section.SOUTH:
        return Waypoint(zone.gap_cx, zone.y_max + clearance)
    if section is Section.NORTH:
        return Waypoint(zone.gap_cx, zone.y_min - clearance)
    if section is Section.EAST:
        return Waypoint(zone.x_min - clearance, zone.gap_cy)
    return Waypoint(zone.x_max + clearance, zone.gap_cy)  # WEST
