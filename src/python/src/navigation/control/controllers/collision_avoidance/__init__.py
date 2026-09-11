"""Collision avoidance controller for LIDAR-based obstacle detection.

Public API for assessing collision risk from LIDAR and generating escape
maneuvers. The implementation is split into submodules -- bumper (sensor-to-
bumper conversions), sectors (pure LIDAR sector math + mapped-sign masking),
controller (the stateful CollisionAvoidanceController and its escape logic) --
and this package re-exports the names callers import from
``...controllers.collision_avoidance_controller``.
"""

from shared.domain.enums import ManeuverType, ThreatDirection

from src.navigation.control.controllers.collision_avoidance.bumper import (
    bumper_gap_ahead,
    bumper_gap_behind,
)
from src.navigation.control.controllers.collision_avoidance.controller import (
    CollisionAvoidanceController,
    EscapeManeuver,
    ParkingGate,
)
from src.navigation.control.controllers.collision_avoidance.sectors import (
    mask_mapped_obstacles,
    ranges_beyond_chassis,
    sector_ranges,
)

__all__ = [
    "CollisionAvoidanceController",
    "EscapeManeuver",
    "ManeuverType",
    "ParkingGate",
    "ThreatDirection",
    "bumper_gap_ahead",
    "bumper_gap_behind",
    "mask_mapped_obstacles",
    "ranges_beyond_chassis",
    "sector_ranges",
]
