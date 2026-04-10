"""Navigation controllers extracted from monolithic TrackNavigator.

These focused controllers implement the single responsibility principle,
enabling independent testing and reuse.

Exports:
    - WaypointController: Pure pursuit steering for waypoint following
    - CollisionAvoidanceController: LIDAR-based collision detection and escape
    - StuckDetector: Stuck condition detection and recovery
"""

from .collision_avoidance_controller import CollisionAvoidanceController, EscapeManeuver
from .stuck_detector import StuckDetector
from .waypoint_controller import WaypointController

__all__ = [
    "WaypointController",
    "CollisionAvoidanceController",
    "EscapeManeuver",
    "StuckDetector",
]
