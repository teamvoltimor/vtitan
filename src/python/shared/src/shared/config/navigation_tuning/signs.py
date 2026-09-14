"""Traffic-sign routing and blind sign-discovery tuning groups.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.signs`. These classes add only the
tuning layer's shipped fallbacks plus the derived accessors (degrees to a
normalised steering command) that belong to tuning rather than the schema. The
generated field descriptions carry the measurement history that used to live
here.
"""

from __future__ import annotations

import math
from dataclasses import fields
from typing import Any, ClassVar

from shared.config.constants import RobotSpecs
from shared.config.generated.navigation.signs.sign_discovery_schema import (
    NavigationSignsSignDiscovery,
)
from shared.config.generated.navigation.signs.sign_router_schema import (
    NavigationSignsSignRouter,
)
from shared.config.navigation_tuning._shared import TuningModel
from shared.domain.steering import angle_rad_to_steering_norm

__all__ = ["SignDiscoveryParams", "SignRouterParams"]


class SignRouterParams(TuningModel, NavigationSignsSignRouter):
    """Traffic-sign avoidance routing parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "sign_clearance_margin_m": 0.10,
        "wall_clearance_margin_m": 0.04,
        "activation_dist_m": 1.40,
        "passed_dist_m": 1.60,
        "deform_depth_buffer_m": 0.5,
        "detection_match_dist_m": 0.30,
        "min_confidence": 0.25,
        "settle_ticks": 150,
        "slot_sign_map": True,
        "slot_accept_radius_m": 0.30,
        "slot_min_evidence": 0.75,
        "slot_repoint_margin": 1.5,
        "escape_mask_radius_m": 0.12,
        "escape_mask_cluster_assoc_m": 0.35,
        "escape_mask_chassis_margin_m": 0.01,
        "commit_hysteresis": True,
        "corridor_flip_ticks": 1,
        "sign_aware_speed": True,
        "sign_lane_planner": True,
        "sign_lane_offset_frac": 1.0,
        "sign_lane_ramp_m": 0.90,
        "sign_lane_hold_m": 0.25,
        "sign_lane_gap_centre_frac": 1.0,
        "sign_lane_commit_ahead_m": 0.0,
        "sign_lane_corner_entry_m": 0.50,
        "sign_lane_suppress_deform": True,
        "sign_lane_deform_fallback_m": 0.0,
        "sign_deform_sense_guard": False,
        "sign_lane_relabel_unsatisfiable": True,
        "sign_lane_skip_unsatisfiable": False,
        "sign_lane_depth_consistent_corridor": True,
        "sign_lane_split_overlap": False,
        "depth_pin": True,
        "pin_corner_guard": True,
        "pin_heading_guard": True,
        "pin_heading_guard_deg": 35.0,
        "sign_deform_speed_threshold_m": 0.02,
        "explore_lap_speed_frac": 1.0,
        "retrace_escape": False,
        "retrace_dist_m": 0.25,
        "retrace_steer_gain_deg": 55.0,
        "sign_contact_evade": False,
        "sign_contact_dist_m": 0.60,
        "sign_contact_steer_deg": 19.25,
        "sign_aware_lookahead": True,
        "sign_lidar_align": False,
        "stale_target_rescue": False,
        "sign_lidar_align_min_m": 0.40,
        "sign_lidar_align_max_m": 1.50,
        "sign_lidar_align_fov_deg": 45.0,
        "sign_lidar_align_depth_m": 0.08,
        "sign_lidar_align_max_width_m": 0.15,
        "sign_lidar_align_deadband_deg": 8.0,
        "sign_lidar_align_gain": 0.35,
        "sign_lidar_align_max_steer": 0.15,
        "sign_lidar_propose": False,
    }

    def resolve_unset(self, target: Any, *, prefix: str = "") -> None:
        """Fill every ``None`` field of a tuning-mirror dataclass from this group.

        Convention-driven so the mapping itself has no copy to keep in step: a
        target field ``activation_dist`` reads ``activation_dist_m`` and one
        named ``depth_pin`` reads ``depth_pin``; mirrors nested under a tuning
        sub-prefix pass ``prefix`` (e.g. ``"SIGN_LANE_"`` for
        ``SignLaneParams``). A ``None`` field matching neither spelling raises
        instead of being skipped, mirroring the getattr failure that made a
        renamed tunable loud rather than letting a default quietly stop tracking
        the TOML.
        """
        for field in fields(type(target)):
            if getattr(target, field.name) is not None:
                continue
            name = (prefix + field.name).lower()
            for attr in (name, f"{name}_m"):
                if hasattr(self, attr):
                    object.__setattr__(target, field.name, getattr(self, attr))
                    break
            else:
                msg = f"{type(target).__name__}.{field.name} resolved no default from {type(self).__name__}"
                raise AttributeError(msg)

    def sign_contact_steer_norm(self) -> float:
        """Sign-evade steering as the normalised command the actuator takes.

        Stored as a physical road-wheel angle, so a wider servo yields a
        SMALLER normalised command for the same 19.25 degrees rather than the
        same command meaning a wider swerve. Same rationale as
        :meth:`~shared.config.navigation_tuning.escape.EscapeManeuverParams.rev_steer_norm`.
        """
        return angle_rad_to_steering_norm(math.radians(self.sign_contact_steer_deg), RobotSpecs.MAX_STEERING_ANGLE)

    def retrace_steer_gain_norm(self, lateral_over_distance: float) -> float:
        """Reverse-pure-pursuit steering for a target ``lateral/distance`` off-axis.

        The caller passes the dimensionless bearing ratio; the gain turns it
        into a road-wheel angle, and only then does the servo's reach enter.
        Clamping happens in normalised space, exactly as the previous inline
        ``max(-1.0, min(1.0, ...))`` did.
        """
        return angle_rad_to_steering_norm(
            math.radians(self.retrace_steer_gain_deg) * lateral_over_distance, RobotSpecs.MAX_STEERING_ANGLE
        )


class SignDiscoveryParams(TuningModel, NavigationSignsSignDiscovery):
    """Blind sign-discovery (ObservedSignMap) parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "min_reliable_bbox_height_px": 5,
        "max_ingest_range_m": 1.5,
        "association_dist_m": 0.25,
        "min_hits": 3,
        "robot_corridor_flip_ticks": 5,
        "max_pillar_aspect": 1.0,
        "frame_edge_tolerance_px": 2.0,
        "range_scale": 1.95,
        "lidar_range_fusion": True,
        "vision_latency_s": 0.85,
        "snap_to_lattice_m": 0.0,
        "lidar_range_fusion_cluster": True,
        "lidar_range_fusion_agreement": 0.5,
        "max_signs_per_section": 0,
        "colour_pool_radius_m": 0.0,
    }
