"""Shared navigation helpers.

Extracted from corridor_estimator, corridor_follower, direction_estimator to
eliminate identical module-private definitions.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.config.tuning_helpers import get_tuning

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.models import Waypoint


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp ``value`` to the closed interval ``[lo, hi]``."""
    return max(lo, min(hi, value))


def axis_offset_rad(yaw: float) -> float:
    """Signed deviation from the nearest track axis: + is left of it, - is right.

    Same quantity as :func:`axis_error_rad` before the absolute value. Gates ask
    "how far off axis am I", which has no direction; a controller correcting the
    error has to know which way to steer, so it needs the sign.
    """
    quarter = math.pi / 2
    return wrap_angle(yaw - round(yaw / quarter) * quarter)


def axis_error_rad(yaw: float) -> float:
    """How far a heading sits from the nearest track axis, always positive.

    The track is a Manhattan world, so "aligned with a corridor" means "within
    tolerance of a multiple of 90 degrees" regardless of which corridor. This is
    the measure the direction gate accepts or refuses readings on
    (:func:`src.navigation.direction_estimator.infer_direction`), and the one
    ``track_navigator_node._direction_gate_verdict`` reports for the log; both
    spelled it out separately, and a bag diagnostic then spelled it a third time.
    """
    return abs(axis_offset_rad(yaw))


def _dist2d(a: Waypoint, b: Waypoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _nearest_ray(ranges_m: Sequence[float], angles_rad: Sequence[float], target: float) -> float:
    index = min(range(len(angles_rad)), key=lambda i: abs(wrap_angle(angles_rad[i] - target)))
    return ranges_m[index]


def _forward_clearance(ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning | None = None) -> float:
    """Min clearance in forward direction.

    Uses tuning: lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG, MIN_VALID_RANGE_M
    """
    tuning = get_tuning(tuning)
    arc_rad = math.radians(tuning.lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG)
    min_valid = tuning.lidar_sectors.MIN_VALID_RANGE_M
    forward = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(wrap_angle(a)) <= arc_rad and r > min_valid
    ]
    return min(forward) if forward else math.inf


def _local_frame(robot_pos: Waypoint, robot_yaw: float, target: Waypoint) -> tuple[float, float]:
    """Rotate ``target`` into the robot's local frame (x forward, y left)."""
    dx = target.x - robot_pos.x
    dy = target.y - robot_pos.y
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    x_local = dx * cos_yaw + dy * sin_yaw
    y_local = -dx * sin_yaw + dy * cos_yaw
    return x_local, y_local


def _pure_pursuit_steer(
    x_local: float, y_local: float, min_lookahead_dist: float, max_steering_angle: float = RobotSpecs.MAX_STEERING_ANGLE
) -> float:
    """Curvature-based pure pursuit steering toward a local-frame target (normalised [-1, 1]).

    Standard formulation: ``curvature = 2*y_local / L_d**2``, ``steering angle =
    atan(curvature * L_eff)``, clamped to the chassis's physical steering limit.
    Shared by :class:`~src.navigation.control.controllers.WaypointController` and
    :class:`~src.navigation.maneuvers.parking.ParkController` -- both used to carry
    their own copy of this, and ``WaypointController``'s copy was a bare
    ``steer_kp * angle_error`` P-term instead, which is what produced the
    2026-08-03 real-hardware full-lock steering oscillation (see
    ``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).

    ``L_eff`` is the wheelbase HALVED, not the wheelbase: this chassis steers both
    axles in opposite directions by the same amount (confirmed on hardware
    2026-07-25), which pivots it about its centre instead of the rear axle and
    doubles the yaw rate for a given steering angle. Using the full wheelbase asks
    for twice the steering angle each curvature actually needs.

    Only valid for a target roughly ahead (``x_local > 0``) -- the formula gives a
    plausible-looking but wrong result for a target behind the robot; callers must
    handle that case separately.

    Args:
        x_local: Forward distance to the target (metres).
        y_local: Leftward distance to the target (metres).
        min_lookahead_dist: Floor on the effective lookahead distance, avoiding a
            near-zero-distance curvature blow-up when the target is very close.
        max_steering_angle: Steering limit to clamp against and normalise by.
            Defaults to the physical robot spec, but ``WaypointController``
            allows a per-instance override (used by its tests), which must be
            honoured here rather than silently normalising against a different
            limit than the caller's own.
    """
    lookahead = max(math.hypot(x_local, y_local), min_lookahead_dist)
    curvature = 2.0 * y_local / (lookahead**2)
    steer_angle = math.atan(curvature * RobotSpecs.WHEELBASE / 2.0)
    steer_angle = max(-max_steering_angle, min(max_steering_angle, steer_angle))
    return steer_angle / max_steering_angle
