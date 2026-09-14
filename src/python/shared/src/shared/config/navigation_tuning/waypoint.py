"""Waypoint generation geometry tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.waypoint.waypoints_schema`). This
module adds only derived behaviour; every field value comes from the checked-in TOML.

The generated field descriptions carry the per-knob rationale that used to live
in this module's docstrings (corner arc geometry, center-bias history, the
first-lap corner caution, replan handling).
"""

from __future__ import annotations

from shared.config.generated.navigation.waypoint.waypoints_schema import (
    NavigationWaypointWaypoints,
)


class WaypointParams(NavigationWaypointWaypoints):
    """Waypoint generation geometry parameters.

    Attributes (inherited from the generated DTO): ``arc_radius``,
    ``dedupe_distance_m``, ``wide_center_bias_m``/``narrow_center_bias_m`` and
    their sides, ``obstacles_center_bias_m``, ``unconfirmed_width_inner_bias_m``,
    ``narrow_width_threshold_m``, the arc-point/waypoint counts, the reached
    distances, ``finish_approach_m``, ``replan_heading_tie_margin_m`` and the
    corner-caution/replan gates.
    """

