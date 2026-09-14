"""Waypoint generation geometry tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.waypoint.waypoints_schema`). This
module adds only the tuning layer's shipped fallbacks, so a bare
``WaypointParams()`` still matches the checked-in ``waypoint/waypoints.toml``
without re-declaring a field.

The generated field descriptions carry the per-knob rationale that used to live
in this module's docstrings (corner arc geometry, center-bias history, the
first-lap corner caution, replan handling).
"""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.generated.navigation.waypoint.waypoints_schema import (
    NavigationWaypointWaypoints,
)
from shared.config.navigation_tuning._shared import TuningModel


class WaypointParams(TuningModel, NavigationWaypointWaypoints):
    """Waypoint generation geometry parameters.

    Attributes (inherited from the generated DTO): ``arc_radius``,
    ``dedupe_distance_m``, ``wide_center_bias_m``/``narrow_center_bias_m`` and
    their sides, ``obstacles_center_bias_m``, ``unconfirmed_width_inner_bias_m``,
    ``narrow_width_threshold_m``, the arc-point/waypoint counts, the reached
    distances, ``finish_approach_m``, ``replan_heading_tie_margin_m`` and the
    corner-caution/replan gates.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "arc_radius": 0.45,
        "corner_arc_assume_wide": True,
        "dedupe_distance_m": 0.001,
        "wide_center_bias_m": 0.10,
        "wide_center_bias_side": "inner",
        "narrow_center_bias_side": "inner",
        "narrow_center_bias_m": 0.0,
        "unconfirmed_width_inner_bias_m": 0.05,
        "defer_current_corridor_replan": True,
        "narrow_width_threshold_m": 0.8,
        "num_intermediate_arc_points": 3,
        "straight_waypoint_count": 8,
        "main_loop_reached_distance_m": 0.20,
        "controller_reached_distance_m": 0.01,
        "finish_approach_m": 0.40,
        "replan_heading_tie_margin_m": 0.15,
        "obstacles_center_bias_m": 0.15,
        "first_lap_corner_caution": True,
        "corner_caution_all_laps": False,
        "first_lap_corner_caution_narrow_only": False,
        "advance_past_passed_waypoint": False,
        "replan_blend_ticks": 0,
        "forward_only_reseek": False,
    }
