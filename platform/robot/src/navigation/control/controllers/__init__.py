"""Navigation controllers extracted from monolithic TrackNavigator.

These focused controllers implement the single responsibility principle,
enabling independent testing and reuse.

Exports:
    - WaypointController: Pure pursuit steering for waypoint following
    - CollisionAvoidanceController: LIDAR-based collision detection and escape
    - StuckDetector: Stuck condition detection and recovery
"""

from src.navigation.control.controllers.collision_avoidance_controller import (
    CollisionAvoidanceController,
    EscapeManeuver,
    ManeuverType,
    ThreatDirection,
    mask_mapped_obstacles,
)
from src.navigation.control.controllers.stuck_detector import StuckDetector
from src.navigation.control.controllers.waypoint_controller import WaypointController

__all__ = [
    "CollisionAvoidanceController",
    "EscapeManeuver",
    "ManeuverType",
    "StuckDetector",
    "ThreatDirection",
    "WaypointController",
    "mask_mapped_obstacles",
]
