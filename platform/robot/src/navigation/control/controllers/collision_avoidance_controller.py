"""Collision avoidance controller for LIDAR-based obstacle detection.

Assesses collision risk from LIDAR data and generates escape maneuvers
(K-turn, slalom) when obstacles are detected.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np
from shared.domain.enums import RiskLevel

logger = logging.getLogger(__name__)


@dataclass
class EscapeManeuver:
    """Escape maneuver command.

    Attributes:
        maneuver_type: Type of escape (k_turn, slalom, side_correction)
        steering: Steering angle [-1, 1]
        speed: Speed command [-1, 1]
        duration_frames: How long to execute this maneuver
        priority: Priority level (higher = more urgent)
    """

    maneuver_type: str
    steering: float
    speed: float
    duration_frames: int
    priority: int = 1


class CollisionAvoidanceController:
    """Manages collision detection and escape maneuver generation.

    Analyzes LIDAR data to:
    1. Assess collision risk (safe, obstacle, critical)
    2. Detect direction of threat (front, left, right)
    3. Generate appropriate escape maneuver

    Attributes:
        contact_dist: Critical collision distance (m)
        slow_dist: Begin avoiding (m)
        escape_rev_speed: Reverse speed during escape
        escape_steer_scale: Steering aggressiveness
        stuck_threshold: Distance threshold for stuck detection (m)
    """

    def __init__(
        self,
        contact_dist: float = 0.10,
        slow_dist: float = 0.25,
        fast_dist: float = 0.50,
        escape_rev_speed: float = -0.20,
        escape_steer_scale: float = 0.8,
        stuck_threshold: float = 0.03,
    ):
        """Initialize collision avoidance controller.

        Args:
            contact_dist: Critical distance threshold (m)
            slow_dist: Start avoiding distance (m)
            fast_dist: Normal speed distance (m)
            escape_rev_speed: Reverse speed for escapes
            escape_steer_scale: Steering intensity for escapes
            stuck_threshold: Distance threshold for stuck (m)
        """
        self.contact_dist = contact_dist
        self.slow_dist = slow_dist
        self.fast_dist = fast_dist
        self.escape_rev_speed = escape_rev_speed
        self.escape_steer_scale = escape_steer_scale
        self.stuck_threshold = stuck_threshold

    def assess_risk(self, lidar_ranges: np.ndarray) -> RiskLevel:
        """Assess collision risk from LIDAR ranges.

        Args:
            lidar_ranges: Array of LIDAR range measurements

        Returns:
            RiskLevel enum (SAFE, OBSTACLE, CRITICAL)
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return RiskLevel.SAFE

        # The HardwareGateway returns ranges as a plain list (see ROS2HardwareGateway
        # and SimulatedHardwareGateway); coerce as the sector helpers do.
        ranges = np.asarray(lidar_ranges, dtype=float)
        valid_ranges = ranges[ranges > 0.01]

        if len(valid_ranges) == 0:
            return RiskLevel.SAFE

        min_range = np.min(valid_ranges)

        if min_range < self.contact_dist:
            return RiskLevel.CRITICAL
        if min_range < self.slow_dist:
            return RiskLevel.OBSTACLE
        return RiskLevel.SAFE

    @staticmethod
    def _sector_ranges(
        lidar_ranges: np.ndarray,
        lidar_angles: np.ndarray | None,
        center_rad: float,
        half_fov_rad: float,
    ) -> np.ndarray:
        """Valid ranges whose bearing falls within ``center ± half_fov``.

        Bearings come from ``lidar_angles`` (0 rad = forward, +pi/2 = left,
        -pi/2 = right, +/-pi = rear). When angles are unavailable a full 360 deg
        scan indexed from ``angle_min = -pi`` is assumed, so every sector helper
        agrees on which way is forward regardless of the scan's index ordering.
        """
        ranges = np.asarray(lidar_ranges, dtype=float)
        if ranges.size == 0:
            return ranges

        if lidar_angles is None:
            angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
        else:
            angles = np.asarray(lidar_angles, dtype=float)

        # Wrapped angular distance from the sector centre, in [-pi, pi].
        delta = np.arctan2(np.sin(angles - center_rad), np.cos(angles - center_rad))
        mask = (np.abs(delta) <= half_fov_rad) & (ranges > 0.01)
        return ranges[mask]

    def compute_forward_clearance(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None = None,
    ) -> float:
        """Mean clearance in the forward +/-30 deg sector (0 rad = forward).

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            Forward clearance distance (m).
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0  # Default: far away

        forward = self._sector_ranges(lidar_ranges, lidar_angles, 0.0, math.radians(30))
        if forward.size == 0:
            return 10.0
        return float(np.mean(forward))

    def compute_rear_clearance(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None = None,
    ) -> float:
        """Minimum clearance in the rear +/-45 deg sector (+/-pi rad = rear).

        Used to gate reverse / K-turn escapes so the robot never backs into a
        wall it cannot see.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0

        rear = self._sector_ranges(lidar_ranges, lidar_angles, math.pi, math.radians(45))
        if rear.size == 0:
            return 10.0
        return float(np.min(rear))

    def detect_threat_direction(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None = None,
    ) -> str:
        """Direction of the closest obstacle: front, left, right, back, or none.

        Sectors are angular cones (+/-45 deg) about forward (0), left (+pi/2),
        right (-pi/2) and rear (+/-pi), so the result is correct regardless of
        the scan's index ordering.

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            Direction string: "front", "left", "right", "back", or "none".
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return "none"

        def sector_min(center_rad: float) -> float:
            sect = self._sector_ranges(lidar_ranges, lidar_angles, center_rad, math.radians(45))
            return float(np.min(sect)) if sect.size > 0 else 10.0

        directions = {
            "front": sector_min(0.0),
            "left": sector_min(math.pi / 2),
            "right": sector_min(-math.pi / 2),
            "back": sector_min(math.pi),
        }

        closest = min(directions, key=directions.get)
        if directions[closest] > 1.0:
            return "none"
        return closest

    def compute_escape_maneuver(self, risk: RiskLevel, threat_dir: str) -> EscapeManeuver | None:
        """Generate escape maneuver for detected threat.

        Args:
            risk: Current risk level
            threat_dir: Threat direction from detect_threat_direction()

        Returns:
            EscapeManeuver command or None if no maneuver needed
        """
        if risk == RiskLevel.SAFE:
            return None

        if threat_dir == "front":
            # K-turn: reverse while steering hard
            return EscapeManeuver(
                maneuver_type="k_turn",
                steering=self.escape_steer_scale if risk == RiskLevel.CRITICAL else 0.0,
                speed=self.escape_rev_speed,
                duration_frames=8 if risk == RiskLevel.CRITICAL else 4,
                priority=2 if risk == RiskLevel.CRITICAL else 1,
            )

        if threat_dir == "left":
            # Move right
            return EscapeManeuver(
                maneuver_type="side_correction",
                steering=0.3,
                speed=0.1,
                duration_frames=4,
                priority=1,
            )

        if threat_dir == "right":
            # Move left
            return EscapeManeuver(
                maneuver_type="side_correction",
                steering=-0.3,
                speed=0.1,
                duration_frames=4,
                priority=1,
            )

        return None
