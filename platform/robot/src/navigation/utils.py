"""Shared navigation helpers.

Extracted from corridor_estimator, corridor_follower, direction_estimator to
eliminate identical module-private definitions.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.models import Distanceable, Pose, Waypoint

from src.config.tuning_helpers import get_tuning

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning


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
    """Euclidean distance between two points with ``.x``/``.y`` attributes."""
    return a.distance_to(b)


def _as_waypoint(point: Distanceable | tuple[float, float, float]) -> Waypoint:
    """Normalise a trail point to a :class:`Waypoint`.

    Accepts a :class:`Pose`/:class:`Waypoint`/:class:`Position2D` (anything
    satisfying ``Distanceable``, via ``to_waypoint``) or the legacy
    ``(x, y, yaw)`` tuple the tests still seed, so trail consumers don't care
    which shape a breadcrumb happens to be.
    """
    if isinstance(point, tuple):
        x, y, *_ = point
        return Waypoint(float(x), float(y))
    return point.to_waypoint()


def _nearest_ray(ranges_m: Sequence[float], angles_rad: Sequence[float], target: float) -> float:
    index = min(range(len(angles_rad)), key=lambda i: abs(wrap_angle(angles_rad[i] - target)))
    return ranges_m[index]


def trail_clearance_behind(
    trail: Sequence[Pose],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    half_width_m: float = RobotSpecs.WIDTH / 2.0,
) -> float | None:
    """How far the chassis may reverse over ground it has already occupied.

    This is not a sensor reading and does not pretend to be one: it is a record
    of where the chassis physically was, which is the one statement about the
    space behind it that needs no rear vision at all. The mount lost its rear
    slot (see ``lidar_sectors`` blind wedges), so a reverse gate that consults
    only LIDAR must refuse every time; this is what lets it say yes on evidence
    instead.

    Walks back from the newest breadcrumb and stops at the first one that
    leaves a corridor of the chassis' own width — the trail is only a promise
    about ground the footprint actually covered, so a trail that curves away is
    no longer describing the path a reverse would take. Points still ahead of
    the chassis are skipped rather than terminating the walk: the newest
    breadcrumbs sit within centimetres of the current pose and their sign is
    noise.

    Deliberately conservative in two ways. The trail records the chassis
    CENTRE, so ground occupied by the rear half of the footprint is not counted
    -- the true clearance behind the bumper is up to LENGTH/2 greater than this
    returns. And it says nothing about anything that MOVED into that space
    since; on a WRO mat the walls and blocks are static, but a rear estimate is
    never as strong as a rear measurement.

    Returns:
        Distance in metres the chassis centre may retrace, or ``None`` when the
        trail is empty or offers nothing usable behind. ``None`` means no
        evidence, which is not the same as no room.
    """
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    reachable = 0.0
    for point in reversed(trail):
        wp = _as_waypoint(point)
        delta_x, delta_y = wp.x - robot_x, wp.y - robot_y
        along = delta_x * cos_yaw + delta_y * sin_yaw
        if along > 0.0:
            continue
        if abs(-delta_x * sin_yaw + delta_y * cos_yaw) > half_width_m:
            break
        reachable = max(reachable, -along)
    return reachable or None


def _rear_clearance(
    ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning | None = None
) -> float | None:
    """Min clearance in the rear sector, or ``None`` when the mount cannot see it.

    ``None`` does not mean clear -- it means NO INFORMATION, and the two must
    not collapse into one number. A single raw ray straight back (what this
    replaced) cannot tell them apart: the gateway substitutes max range for a
    no-return, so an occluded bearing reads 12 m and a reverse gate comparing
    the distance alone waves it through into whatever is actually there.

    Excluded here, and each for a different reason: the mount's occlusion
    wedges (rays self-collide by geometry regardless of range, so they must go
    by angle), readings below the sensor's rated minimum (not measurements),
    and self-detection returns (the chassis and its own cabling).

    Uses tuning: lidar_sectors.THREAT_HALF_FOV_DEG, MIN_VALID_RANGE_M,
        SELF_DETECTION_THRESHOLD_M, BLIND_WEDGE_{LEFT,RIGHT}_{MIN,MAX}_DEG
    """
    tuning = get_tuning(tuning)
    sectors = tuning.lidar_sectors
    arc_rad = math.radians(sectors.THREAT_HALF_FOV_DEG)
    wedges = (
        (math.radians(sectors.BLIND_WEDGE_LEFT_MIN_DEG), math.radians(sectors.BLIND_WEDGE_LEFT_MAX_DEG)),
        (math.radians(sectors.BLIND_WEDGE_RIGHT_MIN_DEG), math.radians(sectors.BLIND_WEDGE_RIGHT_MAX_DEG)),
    )
    rear = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(wrap_angle(a - math.pi)) <= arc_rad
        and r > sectors.MIN_VALID_RANGE_M
        and r > sectors.SELF_DETECTION_THRESHOLD_M
        and not any(low <= wrap_angle(a) <= high for low, high in wedges)
    ]
    return min(rear) if rear else None


def _forward_clearance(
    ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning | None = None
) -> float:
    """Min clearance in forward direction.

    Uses tuning: lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG, MIN_VALID_RANGE_M
    """
    tuning = get_tuning(tuning)
    arc_rad = math.radians(tuning.lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG)
    min_valid = tuning.lidar_sectors.MIN_VALID_RANGE_M
    forward = [r for r, a in zip(ranges_m, angles_rad, strict=False) if abs(wrap_angle(a)) <= arc_rad and r > min_valid]
    return min(forward) if forward else math.inf


def _wedge_median(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    center_rad: float,
    half_width_rad: float,
    min_valid_range_m: float = 0.0,
    self_detection_threshold_m: float | None = None,
    max_valid_range_m: float | None = None,
) -> float | None:
    """Median valid range in a wedge about ``center_rad``, or ``None`` if none.

    Median rather than mean or minimum: a mean is dragged by the occasional
    max-range no-return, and a minimum reports whatever speck is nearest rather
    than the wall the wedge is pointed at.

    Ranges outside ``[min_valid_range_m, max_valid_range_m]`` are excluded;
    either bound may be left at its default (no lower / no upper cap).
    """
    valid = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(wrap_angle(a - center_rad)) <= half_width_rad
        and r > min_valid_range_m
        and (max_valid_range_m is None or r < max_valid_range_m)
        and (self_detection_threshold_m is None or r > self_detection_threshold_m)
    ]
    return float(np.median(valid)) if valid else None


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
