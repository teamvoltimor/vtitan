"""Tuning-derived parking constants and their injection context.

Holds the ``ParkingContext`` (and the constants it carries) so helper
functions receive tuning without module-level globals frozen at import time.
Pure-Python, no ROS2 dependency, unit-testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.config.tuning_helpers import TuningContext

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning


@dataclass(frozen=True, slots=True)
class ParkingConstants:
    """Tuning-derived parking constants, computed on-demand instead of frozen at module level."""

    parallel_tolerance_m: float
    yaw_tolerance: float
    approach_clearance: float
    pos_reach_dist_m: float
    default_max_frames: int
    saturated_steer_threshold: float
    saturation_stuck_ticks: int
    reposition_speed: float
    reposition_steer_mag: float
    min_lookahead_dist_m: float
    wall_standoff_m: float
    marker_standoff_m: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> ParkingConstants:
        """Create from a NavigationTuning instance."""
        parking_tuning = tuning.parking
        escape_tuning = tuning.escape
        parallel_tolerance = parking_tuning.parallel_tolerance_m
        return cls(
            parallel_tolerance_m=parallel_tolerance,
            yaw_tolerance=math.atan2(parallel_tolerance, RobotSpecs.WHEELBASE),
            approach_clearance=tuning.waypoints.arc_radius,
            pos_reach_dist_m=parking_tuning.pos_reach_dist_m,
            default_max_frames=parking_tuning.default_max_frames,
            saturated_steer_threshold=parking_tuning.saturated_steer_threshold,
            saturation_stuck_ticks=parking_tuning.saturation_stuck_ticks,
            reposition_speed=escape_tuning.rev_speed,
            reposition_steer_mag=escape_tuning.rev_steer_norm(),
            min_lookahead_dist_m=parking_tuning.min_lookahead_dist_m,
            wall_standoff_m=parking_tuning.wall_standoff_m,
            marker_standoff_m=parking_tuning.marker_standoff_m,
        )


class ParkingContext(TuningContext[ParkingConstants]):
    """Context holding tuning-derived parking constants, passed to helper functions.

    Eliminates module-level constants by holding them in an instance, which is
    passed to functions that need them. Enables test-time tuning injection.
    """

    _constants_cls = ParkingConstants


DEFAULT_PARKING_CONTEXT = ParkingContext()
