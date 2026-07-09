"""Collision avoidance controller for LIDAR-based obstacle detection.

Assesses collision risk from LIDAR data and generates escape maneuvers
(K-turn, slalom) when obstacles are detected.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.enums import RiskLevel

logger = logging.getLogger(__name__)

_MIN_VALID_LIDAR_RANGE_M: float = 0.01
"""LIDAR ranges at or below this are treated as invalid (no-return) readings."""


class ThreatDirection(StrEnum):
    """Bearing of the nearest obstacle relative to the robot."""

    FRONT = "front"
    LEFT = "left"
    RIGHT = "right"
    BACK = "back"
    NONE = "none"


class ManeuverType(StrEnum):
    """Kind of escape maneuver commanded by ``compute_escape_maneuver``."""

    K_TURN = "k_turn"
    SIDE_CORRECTION = "side_correction"
    STUCK_REVERSE = "stuck_reverse"


@dataclass
class EscapeManeuver:
    """Escape maneuver command.

    Attributes:
        maneuver_type: Kind of escape (see :class:`ManeuverType`)
        steering: Steering angle [-1, 1]
        speed: Speed command [-1, 1]
        duration_frames: How long to execute this maneuver
        priority: Priority level (higher = more urgent)
    """

    maneuver_type: ManeuverType
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
        path_half_width: Half-width of the forward driving lane risk is judged
            over — the chassis half-width plus a clearance margin (m)
    """

    def __init__(
        self,
        contact_dist: float = 0.10,
        slow_dist: float = 0.25,
        fast_dist: float = 0.50,
        escape_rev_speed: float = -0.20,
        escape_steer_scale: float = 0.8,
        stuck_threshold: float = 0.03,
        path_margin: float = 0.10,
        k_turn_min_frames: int = 6,
        k_turn_max_frames: int = 12,
        side_correction_steer: float = 0.3,
        side_correction_speed: float = 0.1,
        side_correction_frames: int = 4,
    ):
        """Initialize collision avoidance controller.

        Defaults mirror ``NavigationTuning``'s ``ClearanceZones``/
        ``EscapeManeuverParams`` defaults; callers wired to a tuning profile
        (e.g. ``CoreNavigator``) should pass those values explicitly so a
        loaded profile actually takes effect.

        Args:
            contact_dist: Critical distance threshold (m)
            slow_dist: Start avoiding distance (m)
            fast_dist: Normal speed distance (m)
            escape_rev_speed: Reverse speed for escapes
            escape_steer_scale: Steering intensity for escapes
            stuck_threshold: Distance threshold for stuck (m)
            path_margin: Extra clearance beyond the chassis half-width that still
                counts as "in the robot's path" for risk assessment (m)
            k_turn_min_frames: K-turn duration for OBSTACLE risk (frames)
            k_turn_max_frames: K-turn duration for CRITICAL risk (frames)
            side_correction_steer: Steering magnitude for a side-threat correction
            side_correction_speed: Forward speed during a side-threat correction
            side_correction_frames: Duration of a side-threat correction (frames)
        """
        self.contact_dist = contact_dist
        self.slow_dist = slow_dist
        self.fast_dist = fast_dist
        self.escape_rev_speed = escape_rev_speed
        self.escape_steer_scale = escape_steer_scale
        self.stuck_threshold = stuck_threshold
        self.path_half_width = RobotSpecs.WIDTH / 2.0 + path_margin
        self.k_turn_min_frames = k_turn_min_frames
        self.k_turn_max_frames = k_turn_max_frames
        self.side_correction_steer = side_correction_steer
        self.side_correction_speed = side_correction_speed
        self.side_correction_frames = side_correction_frames

    def _forward_path_ranges(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None,
    ) -> np.ndarray:
        """Ranges of points ahead of the robot inside its driving lane.

        A point at bearing ``theta`` (0 = forward) and range ``r`` sits at lateral
        offset ``r*sin(theta)`` from the robot's centreline. Only points that are
        ahead (``cos(theta) > 0``) and within ``path_half_width`` of the
        centreline are in the robot's path — the side walls of a corridor are
        excluded, so a robot driving straight down a narrow corridor is not
        perpetually flagged just because a wall is 0.2 m off its shoulder.
        """
        ranges = np.asarray(lidar_ranges, dtype=float)
        if ranges.size == 0:
            return ranges

        if lidar_angles is None:
            angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
        else:
            angles = np.asarray(lidar_angles, dtype=float)

        lateral = np.abs(ranges * np.sin(angles))
        ahead = np.cos(angles) > 0.0
        mask = ahead & (lateral < self.path_half_width) & (ranges > _MIN_VALID_LIDAR_RANGE_M)
        return ranges[mask]

    def assess_risk(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None = None,
    ) -> RiskLevel:
        """Assess collision risk from obstacles in the robot's forward path.

        Risk is judged over the forward driving lane only (see
        :meth:`_forward_path_ranges`), not the full 360 deg sweep: a corridor's
        side walls are not obstacles the robot is about to hit, and treating them
        as such pins the speed to a crawl for an entire lap.

        Args:
            lidar_ranges: Array of LIDAR range measurements.
            lidar_angles: Per-ray bearings (radians, 0 = forward). Synthesised
                from a full ``[-pi, pi)`` sweep when omitted.

        Returns:
            RiskLevel enum (SAFE, OBSTACLE, CRITICAL).
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return RiskLevel.SAFE

        path = self._forward_path_ranges(lidar_ranges, lidar_angles)
        if path.size == 0:
            return RiskLevel.SAFE

        min_range = float(np.min(path))

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
        filter_self_detection: bool = False,
    ) -> np.ndarray:
        """Valid ranges whose bearing falls within ``center ± half_fov``.

        Bearings come from ``lidar_angles`` (0 rad = forward, +pi/2 = left,
        -pi/2 = right, +/-pi = rear). When angles are unavailable a full 360 deg
        scan indexed from ``angle_min = -pi`` is assumed, so every sector helper
        agrees on which way is forward regardless of the scan's index ordering.

        Args:
            lidar_ranges: Array of LIDAR range measurements.
            lidar_angles: Per-ray bearings (radians), or None to synthesise a
                full ``[-pi, pi)`` sweep.
            center_rad: Centre bearing of the sector (radians).
            half_fov_rad: Half-width of the sector (radians).
            filter_self_detection: Also discard rays no farther than
                ``RobotSpecs.LIDAR_SELF_DETECTION_THRESHOLD`` — mount occlusion
                or cable clutter reflecting the chassis itself, not a real
                obstacle. Only pass this for side/rear sectors: never for the
                pure-forward bearing, where a genuine near-contact inside that
                radius must still register as a threat.
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
        min_valid = RobotSpecs.LIDAR_SELF_DETECTION_THRESHOLD if filter_self_detection else 0.01
        mask = (np.abs(delta) <= half_fov_rad) & (ranges > min_valid)
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
        wall it cannot see. Self-detection filtered: a chassis/cable reflection
        directly behind the robot must not permanently read as "wall right
        there" and block every reverse escape for the rest of the run.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0

        rear = self._sector_ranges(
            lidar_ranges, lidar_angles, math.pi, math.radians(45), filter_self_detection=True,
        )
        if rear.size == 0:
            return 10.0
        return float(np.min(rear))

    def detect_threat_direction(
        self, lidar_ranges: np.ndarray, lidar_angles: np.ndarray | None = None,
    ) -> ThreatDirection:
        """Direction of the closest obstacle: front, left, right, back, or none.

        Sectors are angular cones (+/-45 deg) about forward (0), left (+pi/2),
        right (-pi/2) and rear (+/-pi), so the result is correct regardless of
        the scan's index ordering.

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            The nearest obstacle's :class:`ThreatDirection`.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return ThreatDirection.NONE

        def sector_min(center_rad: float, filter_self_detection: bool = False) -> float:
            sect = self._sector_ranges(
                lidar_ranges, lidar_angles, center_rad, math.radians(45), filter_self_detection,
            )
            return float(np.min(sect)) if sect.size > 0 else 10.0

        directions = {
            # Forward is never self-detection filtered: a genuine near-contact
            # dead ahead must still register even inside that radius.
            ThreatDirection.FRONT: sector_min(0.0),
            ThreatDirection.LEFT: sector_min(math.pi / 2, filter_self_detection=True),
            ThreatDirection.RIGHT: sector_min(-math.pi / 2, filter_self_detection=True),
            ThreatDirection.BACK: sector_min(math.pi, filter_self_detection=True),
        }

        closest = min(directions, key=directions.get)
        if directions[closest] > 1.0:
            return ThreatDirection.NONE
        return closest

    def _k_turn_steer_sign(
        self, lidar_ranges: np.ndarray | None, lidar_angles: np.ndarray | None,
    ) -> float:
        """Steering sign that swings the nose toward the clearer side in reverse.

        Ackermann reverse flips the yaw response relative to forward travel
        (``yaw_rate = (v/L)*tan(steer)`` with ``v < 0``), so a positive steering
        command swings the nose toward the robot's RIGHT while reversing, and a
        negative command swings it left. A fixed sign therefore swings the nose
        into whichever wall happens to be on that side half the time; picking
        the sign from the wider of the two side clearances swings away from the
        tighter wall instead.
        """
        if lidar_ranges is None:
            return 1.0
        left = self._sector_ranges(
            lidar_ranges, lidar_angles, math.pi / 2, math.radians(45), filter_self_detection=True,
        )
        right = self._sector_ranges(
            lidar_ranges, lidar_angles, -math.pi / 2, math.radians(45), filter_self_detection=True,
        )
        left_clear = float(np.min(left)) if left.size > 0 else 10.0
        right_clear = float(np.min(right)) if right.size > 0 else 10.0
        # Swing left (negative steering while reversing) when the left is
        # clearer; swing right (positive) when the right is clearer.
        return -1.0 if left_clear > right_clear else 1.0

    def compute_escape_maneuver(
        self,
        risk: RiskLevel,
        threat_dir: ThreatDirection,
        lidar_ranges: np.ndarray | None = None,
        lidar_angles: np.ndarray | None = None,
    ) -> EscapeManeuver | None:
        """Generate escape maneuver for detected threat.

        Args:
            risk: Current risk level
            threat_dir: Threat direction from detect_threat_direction()
            lidar_ranges: Current scan, used to pick the K-turn's steering side
                (toward the clearer side, not a fixed direction). Optional —
                omitting it keeps the K-turn steering toward the robot's right.
            lidar_angles: Per-ray bearings matching ``lidar_ranges``.

        Returns:
            EscapeManeuver command or None if no maneuver needed
        """
        if risk == RiskLevel.SAFE:
            return None

        if threat_dir == ThreatDirection.FRONT:
            # K-turn: reverse while steering hard for CRITICAL risk; a shorter,
            # straight reverse to open clearance for the milder OBSTACLE risk.
            steer_sign = self._k_turn_steer_sign(lidar_ranges, lidar_angles)
            return EscapeManeuver(
                maneuver_type=ManeuverType.K_TURN,
                steering=self.escape_steer_scale * steer_sign if risk == RiskLevel.CRITICAL else 0.0,
                speed=self.escape_rev_speed,
                duration_frames=self.k_turn_max_frames if risk == RiskLevel.CRITICAL else self.k_turn_min_frames,
                priority=2 if risk == RiskLevel.CRITICAL else 1,
            )

        if threat_dir == ThreatDirection.LEFT:
            # Threat on the left — steer right (away). Positive steering is left
            # (CCW) throughout the stack, so steering away from a left threat is
            # negative.
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=-self.side_correction_steer,
                speed=self.side_correction_speed,
                duration_frames=self.side_correction_frames,
                priority=1,
            )

        if threat_dir == ThreatDirection.RIGHT:
            # Threat on the right — steer left (away): positive steering.
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=self.side_correction_steer,
                speed=self.side_correction_speed,
                duration_frames=self.side_correction_frames,
                priority=1,
            )

        return None
