"""Blind-navigation tuning groups.

Covers corridor width estimation, corridor following, direction inference,
LIDAR pose search, and odometry/IMU fusion.

Fields are inherited from the generated DTOs under
:mod:`shared.config.generated.navigation.blind_nav`. These classes add only the
tuning layer's shipped fallbacks, so a bare instance still matches the
checked-in ``blind_nav/*.toml`` without re-declaring a field. The generated
field descriptions carry the measurement history that used to live here.
"""

from __future__ import annotations

import math
from typing import Any, ClassVar

from shared.config.generated.navigation.blind_nav.corridor_estimator_schema import (
    NavigationBlindNavCorridorEstimator,
)
from shared.config.generated.navigation.blind_nav.corridor_follower_schema import (
    NavigationBlindNavCorridorFollower,
)
from shared.config.generated.navigation.blind_nav.direction_estimator_schema import (
    NavigationBlindNavDirectionEstimator,
)
from shared.config.generated.navigation.blind_nav.localization_schema import (
    NavigationBlindNavLocalization,
)
from shared.config.generated.navigation.blind_nav.state_estimator_schema import (
    NavigationBlindNavStateEstimator,
)
from shared.config.navigation_tuning._shared import TuningModel

__all__ = [
    "CorridorEstimatorParams",
    "CorridorFollowerParams",
    "DirectionEstimatorParams",
    "LocalizationParams",
    "StateEstimatorParams",
]


class CorridorEstimatorParams(TuningModel, NavigationBlindNavCorridorEstimator):
    """Blind corridor-width estimation parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "min_samples": 12,
        "plausible_width_margin_m": 0.25,
        "max_start_samples": 20,
        "decision_boundary_m": 0.8,
    }


class CorridorFollowerParams(TuningModel, NavigationBlindNavCorridorFollower):
    """Blind corridor-following, corner-turn, and parking-bay-exit parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "turn_clearance_m": 0.60,
        "narrow_turn_clearance_m": 0.40,
        "centering_gain_deg_per_m": 0.0,
        "heading_gain": 0.767945,
        "max_centering_steer_deg": 13.75,
        "max_corner_steer_deg": 21.25,
        "steer_cap_from_commit_distance": True,
        "corner_speed_scale": 0.6,
        "reverse_speed_scale": 0.6,
        "turn_arc_half_fov_deg": 15.0,
        "turn_open_range_m": 1.00,
        "corner_leak_margin_m": 0.35,
        "min_forward_clearance_m": 0.30,
        "min_reverse_clearance_m": 0.30,
        "bay_wall_clearance_m": 0.20,
        "assume_bay_start": True,
        "bay_exit_clearance_guard": True,
        "bay_exit_clearance_margin_m": 0.001,
        "bay_exit_guard_overlap_recovery": True,
        "bay_exit_guard_block_ticks": 0,
        "bay_exit_arc_steer_norm": 1.0,
        "bay_exit_speed_scale": 0.35,
        "bay_exit_cycle": True,
        "bay_exit_cycle_reverse_m": 0.09,
        "bay_exit_cycle_reverse_steer_norm": 0.0,
        "bay_exit_forward_m": 0.08,
        "bay_exit_reverse_m": 0.05,
        "bay_exit_steer_norm": 1.0,
        "bay_exit_reverse_steer_norm": 0.0,
        "bay_exit_hold_steer": True,
        "bay_exit_leg_stall_ticks": 6,
        "bay_exit_latch_direction": True,
        "bay_exit_open_side_sector_deg": 15.0,
        "bay_exit_open_side_votes": 5,
        "bay_exit_latch_reverse": False,
        "bay_exit_fallback_frames": 0,
        "bay_exit_max_frames": 900,
        "bay_exit_speed_mps": 0.15,
        "bay_exit_contact_dist_m": 0.08,
        "bay_exit_contact_recovery_ticks": 0,
        "bay_exit_target_yaw_deg": 70.0,
        "bay_exit_leg_max_s": 0.5,
        "bay_exit_guard_measured_coast": True,
        "bay_exit_clearance_tolerance_m": 0.0,
        "bay_exit_guard_mirrors_reverse": True,
        "bay_exit_dr_uses_measured_yaw": False,
    }


class DirectionEstimatorParams(TuningModel, NavigationBlindNavDirectionEstimator):
    """Blind travel-direction inference parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "alignment_tolerance_rad": math.radians(25.0),
        "corner_clearance_m": 1.00,
        "max_in_track_range_m": 4.5,
        "min_asymmetry_m": 0.20,
        "plausible_span_threshold_m": 1.25,
        "min_votes": 5,
        "gate_log_period_ticks": 5,
    }


class LocalizationParams(TuningModel, NavigationBlindNavLocalization):
    """LIDAR-based pose search (LidarLocalizer) parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "search_radius_m": 0.15,
        "passes": 4,
        "grid_points": 5,
        "residual_clip_m": 0.25,
        "max_speed_mps": 0.60,
        "jump_confirm_tolerance_m": 0.05,
        "relocalize_cost_threshold": 0.03,
        "relocalize_after_scans": 15,
        "relocalize_grid_step_m": 0.03,
        "relocalize_accept_ratio": 0.5,
    }


class StateEstimatorParams(TuningModel, NavigationBlindNavStateEstimator):
    """Odometry/IMU fusion parameters."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "yaw_correction_gain": 0.05,
    }
