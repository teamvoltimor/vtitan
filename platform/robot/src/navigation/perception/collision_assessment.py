"""Collision risk assessment and LIDAR processing.

Pure functions for analyzing LIDAR scans and computing clearance metrics.
No ROS2 dependencies. Unit testable.
"""

import math
from enum import Enum

import numpy as np

from src.navigation.config import NavigationConfig


class ClearanceBand(Enum):
    """Graded forward-clearance bands used to scale speed and trigger escape.

    Distinct from the navigation stack's ``RiskLevel`` (SAFE/OBSTACLE/CRITICAL):
    this is a finer proximity ladder owned by ``CollisionAssessor``.
    """

    CLEAR = "clear"  # No immediate risk
    CAUTION = "caution"  # Moderate risk, reduce speed
    WARNING = "warning"  # High risk, activate escape
    CRITICAL = "critical"  # Imminent collision


class CollisionAssessor:
    """Assess collision risk and compute clearance metrics from LIDAR.

    Analyzes laser scan data to detect obstacles in different directions
    (forward, left, right, reverse) and compute safe clearance distances.
    """

    def __init__(self, config: NavigationConfig):
        """Initialize collision assessor.

        Args:
            config: NavigationConfig with collision avoidance parameters.
        """
        self.config = config

    def assess_risk(self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray) -> ClearanceBand:
        """Assess overall forward-clearance band.

        Args:
            lidar_ranges: LIDAR range array (metres).
            lidar_angles: LIDAR angle array (radians, 0 = forward).

        Returns:
            ClearanceBand enum (CLEAR, CAUTION, WARNING, or CRITICAL).
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return ClearanceBand.CLEAR

        fwd_clear = self.measure_clearance(lidar_ranges, lidar_angles, direction="forward")

        if fwd_clear < self.config.collision.critical_dist:
            return ClearanceBand.CRITICAL
        if fwd_clear < self.config.collision.warning_dist:
            return ClearanceBand.WARNING
        if fwd_clear < self.config.collision.caution_dist:
            return ClearanceBand.CAUTION
        return ClearanceBand.CLEAR

    def measure_clearance(
        self,
        lidar_ranges: np.ndarray,
        lidar_angles: np.ndarray,
        direction: str = "forward",
        fov_degrees: float = 60.0,
    ) -> float:
        """Measure clearance in a specific direction.

        Args:
            lidar_ranges: LIDAR range array (metres).
            lidar_angles: LIDAR angle array (radians).
            direction: "forward", "left", "right", or "reverse".
            fov_degrees: Field of view width (degrees).

        Returns:
            Minimum range in direction (metres), or inf if no data.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return float("inf")
        return _clearance_in_direction(lidar_ranges, lidar_angles, direction, fov_degrees)

    def detect_stuck_robot(
        self,
        odometry_history: list[tuple[float, float]],
        move_threshold: float | None = None,
        min_samples: int = 3,
    ) -> bool:
        """Detect if robot is stuck based on position history.

        Args:
            odometry_history: List of (x, y) positions in order.
            move_threshold: Distance threshold to consider stuck (metres).
                Uses config if None.
            min_samples: Minimum history samples to evaluate.

        Returns:
            True if robot appears stuck (low movement).
        """
        if move_threshold is None:
            move_threshold = self.config.stuck.move_threshold

        if len(odometry_history) < min_samples:
            return False

        # Check movement between first and last samples
        x0, y0 = odometry_history[0]
        x1, y1 = odometry_history[-1]
        movement = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

        return movement < move_threshold

    def detect_close_wall(
        self,
        lidar_ranges: np.ndarray,
        lidar_angles: np.ndarray,
        threshold: float | None = None,
    ) -> str:
        """Detect if robot is near a wall.

        Args:
            lidar_ranges: LIDAR range array (metres).
            lidar_angles: LIDAR angle array (radians).
            threshold: Distance threshold (metres). Uses config if None.

        Returns:
            Direction of close wall: "left", "right", "forward", or "none".
        """
        if threshold is None:
            threshold = self.config.stuck.close_wall_dist

        fwd_clear = self.measure_clearance(lidar_ranges, lidar_angles, "forward", fov_degrees=45)
        left_clear = self.measure_clearance(lidar_ranges, lidar_angles, "left", fov_degrees=45)
        right_clear = self.measure_clearance(lidar_ranges, lidar_angles, "right", fov_degrees=45)

        # Return closest wall (priority: forward > left/right)
        if fwd_clear < threshold:
            return "forward"
        if left_clear < right_clear:
            if left_clear < threshold:
                return "left"
        elif right_clear < threshold:
            return "right"

        return "none"


def clamp_lidar_scan(
    ranges: np.ndarray,
    max_range: float = 3.0,
    min_range: float = 0.0,
) -> np.ndarray:
    """Clamp LIDAR range values to valid bounds.

    Filters out invalid readings (inf, nan, out of bounds).

    Args:
        ranges: Raw LIDAR range array.
        max_range: Maximum valid range (metres).
        min_range: Minimum valid range (metres).

    Returns:
        Clamped range array with invalid values replaced by max_range.
    """
    clamped = np.array(ranges, dtype=float)

    # Replace inf and nan
    invalid = np.isinf(clamped) | np.isnan(clamped)
    clamped[invalid] = max_range

    # Clamp to valid bounds
    return np.clip(clamped, min_range, max_range)


def compute_cost_map(
    lidar_ranges: np.ndarray,
    lidar_angles: np.ndarray,
    cost_threshold: float = 0.5,
) -> dict[str, float]:
    """Compute directional cost map from LIDAR.

    Returns cost scores for different steering directions (lower = safer).

    Args:
        lidar_ranges: LIDAR range array (metres).
        lidar_angles: LIDAR angle array (radians).
        cost_threshold: Range below which cost increases rapidly.

    Returns:
        Dict with directional costs: {"forward": float, "left": float, ...}
    """
    if lidar_ranges is None or len(lidar_ranges) == 0:
        return {"forward": 0.0, "left": 0.0, "right": 0.0}

    costs = {}

    for direction in ["forward", "left", "right"]:
        clearance = _clearance_in_direction(lidar_ranges, lidar_angles, direction, fov_degrees=45.0)

        if clearance < cost_threshold:
            costs[direction] = 1.0 - (clearance / cost_threshold) ** 2
        else:
            costs[direction] = 0.0

    return costs


def _clearance_in_direction(
    lidar_ranges: np.ndarray,
    lidar_angles: np.ndarray,
    direction: str,
    fov_degrees: float = 60.0,
) -> float:
    """Minimum LIDAR range within an angular cone centred on a cardinal direction.

    Single implementation shared by CollisionAssessor.measure_clearance and
    compute_cost_map.

    Args:
        lidar_ranges: Range array (metres).
        lidar_angles: Angle array (radians, 0 = forward).
        direction: "forward", "left", "right", or "reverse".
        fov_degrees: Full cone width in degrees.

    Returns:
        Minimum range inside cone, or inf if cone is empty.
    """
    angles = np.array(lidar_angles, dtype=float)
    angles = np.where(angles > math.pi, angles - 2 * math.pi, angles)
    angles = np.where(angles < -math.pi, angles + 2 * math.pi, angles)

    fov_rad = math.radians(fov_degrees / 2)

    if direction == "forward":
        mask = np.abs(angles) <= fov_rad
    elif direction == "left":
        mask = (angles >= (math.pi / 2 - fov_rad)) & (angles <= (math.pi / 2 + fov_rad))
    elif direction == "right":
        mask = (angles >= (-math.pi / 2 - fov_rad)) & (angles <= (-math.pi / 2 + fov_rad))
    elif direction == "reverse":
        mask = (angles >= (math.pi - fov_rad)) | (angles <= (-math.pi + fov_rad))
    else:
        return float("inf")

    if not np.any(mask):
        return float("inf")

    return float(np.min(lidar_ranges[mask]))
