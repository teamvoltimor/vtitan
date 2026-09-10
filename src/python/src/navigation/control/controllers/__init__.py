"""Navigation controllers extracted from monolithic TrackNavigator.

These focused controllers implement the single responsibility principle,
enabling independent testing and reuse.

Exports:
    - WaypointController: Pure pursuit steering for waypoint following
    - CollisionAvoidanceController: LIDAR-based collision detection and escape
    - StuckDetector: Stuck condition detection and recovery
"""

from shared.domain.enums import ManeuverType, ThreatDirection

from src.navigation.control.controllers.collision_avoidance.bumper import (
    bumper_gap_ahead,
    bumper_gap_behind,
)
from src.navigation.control.controllers.collision_avoidance.controller import (
    CollisionAvoidanceController,
    EscapeManeuver,
)
from src.navigation.control.controllers.collision_avoidance.sectors import mask_mapped_obstacles
from src.navigation.control.controllers.stuck_detector import StuckDetector
from src.navigation.control.controllers.waypoint_controller import WaypointController

__all__ = [
    "CollisionAvoidanceController",
    "EscapeManeuver",
    "ManeuverType",
    "StuckDetector",
    "ThreatDirection",
    "WaypointController",
    "bumper_gap_ahead",
    "bumper_gap_behind",
    "mask_mapped_obstacles",
]
