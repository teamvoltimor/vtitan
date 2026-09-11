"""Score a final pose the way a WRO judge scores parking.

The repo has always measured parking as a boolean -- ``footprint_inside``, the
"completely in the parking area and parallel" test. That is only the TOP tier.
The 2026 scoring table (section 10, page 21 of the official rules) pays:

===== ============================================================ ======
 id    requirement                                                  points
===== ============================================================ ======
1.8.2  Parking successfully (completely in the parking area and         15
       parallel)
1.8.3  **Parking partly or not parallel in the parking area**            7
===== ============================================================ ======

Measuring only 1.8.2 is what made parking look worthless. The chassis cannot
reach it -- 0.194 m in a 0.20 m bay is a 3 mm containment window and a 1.15
degree heading budget -- but that argument says nothing about 1.8.3, which a
perpendicular nose-in satisfies with 0.118 m of clearance on each side.

Both tiers are vetoed by contact. Ruled 2026-09-03: "The parking lot limitations
cannot be touched by the robot. When they are touched, the robot is stopped and
no points for the parking can be scored." So a run that grinds its way to a
perfect pose scores ZERO, and any sweep that does not model contact as a veto
overstates every strategy that leans on it.

Pure geometry over a final pose; no controller state, no ROS2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.navigation.maneuvers.parking.context import DEFAULT_PARKING_CONTEXT
from src.navigation.maneuvers.parking.footprint import (
    footprint_breaches_markers,
    footprint_breaches_wall,
    footprint_inside,
    footprint_overlaps_lot,
)
from src.navigation.utils import wrap_angle

if TYPE_CHECKING:
    from src.navigation.maneuvers.parking.context import ParkingContext
    from src.navigation.maneuvers.parking.zone import ParkZone

FULL_PARK_POINTS = 15
"""WRO 1.8.2 -- completely in the parking area AND parallel to the wall."""

PARTIAL_PARK_POINTS = 7
"""WRO 1.8.3 -- partly in the parking area, OR in it but not parallel."""

_SCORING_STANDOFF_M = 0.0
"""Contact for SCORING is geometric, not the controller's give-up margin.

``footprint_breaches_*`` default to a safety standoff so the maneuver abandons
before it touches. Scoring through that margin would call a legal park a breach
and hide exactly the near-wall poses partial credit depends on.
"""


@dataclass(frozen=True, slots=True)
class ParkScore:
    """What a judge would award for a final pose, and the reasons behind it."""

    points: int
    contained: bool
    """Whole footprint inside the lot rectangle (the 1.8.2 half of full credit)."""
    parallel: bool
    """Wheel-to-wall difference within tolerance (the other 1.8.2 half)."""
    overlaps: bool
    """Any part of the footprint inside the lot -- the 1.8.3 criterion."""
    touched: bool
    """Contacted a fin or the wall. Vetoes both tiers regardless of pose."""


def is_wall_parallel(
    robot_yaw: float,
    zone: ParkZone,
    context: ParkingContext | None = None,
) -> bool:
    """Whether the heading satisfies the rule's parallel test.

    The rule is about geometry, not travel: "the distances between the two wheels
    on one side and the wall do not differ by more than 2 cm", which is a
    wheelbase-scaled heading tolerance and is satisfied by BOTH headings along
    the wall. ``zone.target_yaw`` names only one of them -- the one matching the
    direction of travel -- so comparing against it alone would score a robot
    parked perfectly but facing the other way as not parallel. The controller is
    right to aim at one; the judge does not care which.
    """
    ctx = DEFAULT_PARKING_CONTEXT if context is None else context
    yaw_err = abs(wrap_angle(robot_yaw - zone.target_yaw))
    return min(yaw_err, math.pi - yaw_err) <= ctx.constants.yaw_tolerance


def score_park(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
    context: ParkingContext | None = None,
) -> ParkScore:
    """Award 15, 7, or 0 for a final pose, per the 2026 scoring table.

    Contact is checked FIRST and short-circuits: it is a veto, not a deduction.
    """
    touched = footprint_breaches_markers(
        rx, ry, robot_yaw, zone, standoff_m=_SCORING_STANDOFF_M
    ) or footprint_breaches_wall(rx, ry, robot_yaw, zone, standoff_m=_SCORING_STANDOFF_M)
    contained = footprint_inside(rx, ry, robot_yaw, zone)
    parallel = is_wall_parallel(robot_yaw, zone, context)
    overlaps = contained or footprint_overlaps_lot(rx, ry, robot_yaw, zone)

    if touched:
        points = 0
    elif contained and parallel:
        points = FULL_PARK_POINTS
    elif overlaps:
        points = PARTIAL_PARK_POINTS
    else:
        points = 0
    return ParkScore(
        points=points, contained=contained, parallel=parallel, overlaps=overlaps, touched=touched
    )
