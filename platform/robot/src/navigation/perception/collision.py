"""LIDAR-based collision detection for WRO track navigation.

Pure functions — no ROS2 dependency, no global state.
All functions accept data (numpy arrays, floats) and return results.
"""

from __future__ import annotations

import math
from typing import TypedDict

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.enums import RiskLevel


class DistancesDict(TypedDict):
    """LIDAR-measured clearance in the three principal directions."""

    forward: float
    left: float
    right: float


# Minimum distance (m) before the robot contacts the obstacle.
# Kept slightly above zero so a reading at exactly LIDAR_MIN_RANGE
# (sensor floor, not a real wall) never triggers escape.
_CONTACT_BUFFER = 0.01

# Angular tolerance (radians) used for side-direction queries.
# Covers ±14° around the target bearing — wide enough to catch all
# rays in the 90° sector without overlapping the forward sector.
_SIDE_TOLERANCE = 0.25

# Robot chassis half-width at the LIDAR plane. gpu_lidar renders ALL
# visuals including the same model body; rays that clip the chassis at
# ≤ 0.08m are self-reflections, not real obstacles.
_SELF_DETECTION_THRESHOLD = 0.08

# Near-side clearance below which the robot is boxed in by a wall.
# 600 mm corridor with outer-bias puts the near wall at ~0.25 m, so 0.28 m
# is the threshold that separates a wall corner from a free-standing sign.
_BOXED_IN_NEAR_SIDE = 0.28

# Heading error (radians) above which the robot is considered to be actively
# turning rather than tracking a straight corridor (~14°).
_TURNING_HEADING_THRESHOLD = 0.25


def measure_distance_in_direction(
    lidar_ranges: np.ndarray,
    lidar_angles: np.ndarray,
    target_angle: float,
    tolerance: float = 0.25,
    filter_self_detection: bool = False,
) -> float:
    """Return the minimum LIDAR range within a bearing sector.

    Args:
        lidar_ranges: 1-D array of range values (metres), already clamped.
        lidar_angles: 1-D array of bearing angles (radians, robot frame).
        target_angle: Centre bearing of the sector (radians).
        tolerance: Half-width of the bearing sector (radians).
        filter_self_detection: Discard readings ≤ ``_SELF_DETECTION_THRESHOLD``.
            Use only for side bearings (±π/2) where the robot body occludes
            real readings at close range. Never filter the forward direction.

    Returns:
        Minimum range in the sector, or ``inf`` when no rays match.
    """
    diff = lidar_angles - target_angle
    angular_distance = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))
    sector_mask = angular_distance < tolerance

    if not np.any(sector_mask):
        return float("inf")

    sector_ranges = lidar_ranges[sector_mask]
    if filter_self_detection:
        sector_ranges = sector_ranges[sector_ranges > _SELF_DETECTION_THRESHOLD]

    if len(sector_ranges) == 0:
        return float("inf")

    return float(np.min(sector_ranges))


def assess_collision_risk(
    lidar_ranges: np.ndarray,
    lidar_angles: np.ndarray,
    critical_distance: float,
    fwd_critical_count: int,
    fwd_critical_threshold: int,
    is_open_challenge: bool,
    angle_error: float = 0.0,
    is_simulation: bool = False,
) -> tuple[RiskLevel, DistancesDict]:
    """Classify the current collision risk from LIDAR data.

    Returns a risk label and the three measured distances.

    Risk levels:
    - ``RiskLevel.SAFE``: no imminent contact.
    - ``RiskLevel.CRITICAL``: wall or corner contact — K-turn escape required.
    - ``RiskLevel.OBSTACLE``: free-standing traffic sign with room on both sides
      (obstacles challenge only).

    Args:
        lidar_ranges: Clamped LIDAR range array.
        lidar_angles: Corresponding bearing array.
        critical_distance: Forward distance threshold that triggers escape.
        fwd_critical_count: Consecutive readings already below threshold.
        fwd_critical_threshold: Minimum consecutive count before escape fires.
        is_open_challenge: ``True`` when there are no traffic signs on track.
        angle_error: Current heading error to steer target (radians).
        is_simulation: Whether running in simulation.

    Returns:
        Tuple of (risk_label, distances) where ``distances`` is a
        ``DistancesDict`` with keys ``forward``, ``left``, ``right``.
    """
    forward_dist = measure_distance_in_direction(
        lidar_ranges,
        lidar_angles,
        target_angle=0.0,
        tolerance=_SIDE_TOLERANCE,
    )
    left_dist = measure_distance_in_direction(
        lidar_ranges,
        lidar_angles,
        target_angle=math.pi / 2,
        tolerance=_SIDE_TOLERANCE,
        filter_self_detection=True,
    )
    right_dist = measure_distance_in_direction(
        lidar_ranges,
        lidar_angles,
        target_angle=-math.pi / 2,
        tolerance=_SIDE_TOLERANCE,
        filter_self_detection=True,
    )

    distances: DistancesDict = {
        "forward": forward_dist,
        "left": left_dist,
        "right": right_dist,
    }

    # Forward > 4 m is impossible inside the 3×3 m track — the ray passed
    # through a thin wall mesh into open space (GPU LIDAR wall-clipping).
    if is_simulation and forward_dist > 4.0:
        return RiskLevel.CRITICAL, distances

    debounced = fwd_critical_count >= fwd_critical_threshold

    # "obstacle" only when there is room on both sides (traffic sign, not wall).
    # 600 mm corridor with outer-bias: near side ≈ 0.25 m → boxed_in.
    # 1000 mm corridor: near side ≈ 0.45 m → can classify as obstacle.
    near_side = min(left_dist, right_dist)
    boxed_in = near_side < _BOXED_IN_NEAR_SIDE
    turning = abs(angle_error) > _TURNING_HEADING_THRESHOLD
    can_be_obstacle = not is_open_challenge and not boxed_in and not turning

    contact = forward_dist <= RobotSpecs.LIDAR_MIN_RANGE + _CONTACT_BUFFER

    if contact or forward_dist < critical_distance:
        if not debounced:
            # Single-reading spike (GPU LIDAR artifact at inner-corner junction).
            # Speed scaling still applies; wait for a second reading to confirm.
            return RiskLevel.SAFE, distances
        if can_be_obstacle:
            return RiskLevel.OBSTACLE, distances
        return RiskLevel.CRITICAL, distances

    return RiskLevel.SAFE, distances


def update_fwd_critical_count(
    lidar_ranges: np.ndarray,
    lidar_angles: np.ndarray,
    critical_distance: float,
    current_count: int,
) -> int:
    """Return the updated consecutive forward-critical counter.

    Increments when the forward distance is below ``critical_distance``,
    resets to zero otherwise.

    Args:
        lidar_ranges: Clamped LIDAR range array.
        lidar_angles: Corresponding bearing array.
        critical_distance: Threshold distance (metres).
        current_count: Counter value from the previous reading.

    Returns:
        Updated counter (non-negative integer).
    """
    forward_dist = measure_distance_in_direction(
        lidar_ranges,
        lidar_angles,
        target_angle=0.0,
        tolerance=_SIDE_TOLERANCE,
    )
    if forward_dist < critical_distance:
        return current_count + 1
    return 0


def clamp_lidar_scan(
    raw_ranges: np.ndarray,
    max_range: float,
) -> np.ndarray:
    """Clamp raw LIDAR ranges to the valid sensor interval.

    Replaces below-minimum readings with ``RobotSpecs.LIDAR_MIN_RANGE``
    and infinite / out-of-range readings with ``max_range``.

    Args:
        raw_ranges: Raw range array from the LaserScan message.
        max_range: Sensor maximum range (metres).

    Returns:
        New array with clamped values (same shape as input).
    """
    clamped = raw_ranges.copy()
    clamped[clamped < RobotSpecs.LIDAR_MIN_RANGE] = RobotSpecs.LIDAR_MIN_RANGE
    clamped[np.isinf(clamped)] = max_range
    return clamped
