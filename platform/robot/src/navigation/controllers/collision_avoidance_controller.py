"""Collision avoidance controller for LIDAR-based obstacle detection.

Assesses collision risk from LIDAR data and generates escape maneuvers
(K-turn, slalom) when obstacles are detected.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
from shared.config.enums import RiskLevel

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

        valid_ranges = lidar_ranges[lidar_ranges > 0.01]

        if len(valid_ranges) == 0:
            return RiskLevel.SAFE

        min_range = np.min(valid_ranges)

        if min_range < self.contact_dist:
            return RiskLevel.CRITICAL
        if min_range < self.slow_dist:
            return RiskLevel.OBSTACLE
        return RiskLevel.SAFE

    def compute_forward_clearance(self, lidar_ranges: np.ndarray) -> float:
        """Compute forward clearance from LIDAR data.

        Averages ranges in forward sector (±30 degrees).

        Args:
            lidar_ranges: Array of LIDAR measurements

        Returns:
            Forward clearance distance (m)
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0  # Default: far away

        valid_ranges = lidar_ranges[lidar_ranges > 0.01]

        if len(valid_ranges) == 0:
            return 10.0

        # Take forward sector (center 60 degrees = ±30°)
        num_rays = len(lidar_ranges)
        sector_width = max(1, num_rays // 6)  # ~60° out of 360°
        center = num_rays // 2
        start = center - sector_width // 2
        end = center + sector_width // 2

        forward_sector = lidar_ranges[max(0, start) : min(num_rays, end)]
        forward_ranges = forward_sector[forward_sector > 0.01]

        if len(forward_ranges) == 0:
            return 10.0

        return float(np.mean(forward_ranges))

    def detect_threat_direction(self, lidar_ranges: np.ndarray) -> str:
        """Detect direction of closest obstacle.

        Args:
            lidar_ranges: Array of LIDAR measurements

        Returns:
            Direction string: "front", "left", "right", or "none"
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return "none"

        num_rays = len(lidar_ranges)
        quarter = num_rays // 4

        # Divide into quadrants
        front = np.min(lidar_ranges[:quarter]) if len(lidar_ranges[:quarter]) > 0 else 10.0
        right = np.min(lidar_ranges[quarter : 2 * quarter]) if len(lidar_ranges[quarter : 2 * quarter]) > 0 else 10.0
        back = (
            np.min(lidar_ranges[2 * quarter : 3 * quarter])
            if len(lidar_ranges[2 * quarter : 3 * quarter]) > 0
            else 10.0
        )
        left = np.min(lidar_ranges[3 * quarter :]) if len(lidar_ranges[3 * quarter :]) > 0 else 10.0

        # Find closest
        min_dist = min(front, right, back, left)

        if min_dist > 1.0:
            return "none"
        if min_dist == front:
            return "front"
        if min_dist == left:
            return "left"
        if min_dist == right:
            return "right"
        return "back"

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
