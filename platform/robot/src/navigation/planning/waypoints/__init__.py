"""Waypoint generation for WRO 2026 track navigation.

Public API for building the multi-lap waypoint sequence the TrackNavigator
follows. The implementation is split into submodules (geometry primitives,
segment assembly, generation orchestration, position classification); this
package re-exports the names callers import from ``...planning.waypoints``.
"""

from src.navigation.planning.waypoints.classification import corridor_for_position
from src.navigation.planning.waypoints.generation import (
    calculate_waypoints,
    corridor_widths_dict_to_model,
    plan_believed_path,
    validate_path_feasibility,
)
from src.navigation.planning.waypoints.geometry import (
    _arc_intermediate_points,
    _arc_with_endpoints,
    _corner_arc_radius,
    _straight_waypoints,
)
from src.navigation.planning.waypoints.segments import (
    _assemble_loop,
    _build_all_segments,
    _build_waypoint_sequence,
    _deduplicate_consecutive,
    _nearest_waypoint_index,
    _validate_bounds,
)

__all__ = [
    "_arc_intermediate_points",
    "_arc_with_endpoints",
    "_assemble_loop",
    "_build_all_segments",
    "_build_waypoint_sequence",
    "_corner_arc_radius",
    "_deduplicate_consecutive",
    "_nearest_waypoint_index",
    "_straight_waypoints",
    "_validate_bounds",
    "calculate_waypoints",
    "corridor_for_position",
    "corridor_widths_dict_to_model",
    "plan_believed_path",
    "validate_path_feasibility",
]
