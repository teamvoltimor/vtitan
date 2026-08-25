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
    arc_intermediate_points,
    arc_with_endpoints,
    corner_arc_radius,
    straight_waypoints,
)
from src.navigation.planning.waypoints.segments import (
    assemble_loop,
    build_all_segments,
    build_waypoint_sequence,
    deduplicate_consecutive,
    nearest_waypoint_index,
    validate_bounds,
)

__all__ = [
    "arc_intermediate_points",
    "arc_with_endpoints",
    "assemble_loop",
    "build_all_segments",
    "build_waypoint_sequence",
    "calculate_waypoints",
    "corner_arc_radius",
    "corridor_for_position",
    "corridor_widths_dict_to_model",
    "deduplicate_consecutive",
    "nearest_waypoint_index",
    "plan_believed_path",
    "straight_waypoints",
    "validate_bounds",
    "validate_path_feasibility",
]
