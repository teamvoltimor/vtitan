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
from shared.domain.models import SectorRanges

logger = logging.getLogger(__name__)


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
        fast_dist: float = 1.00,
        escape_rev_speed: float = -0.20,
        escape_steer_scale: float = 0.8,
        stuck_threshold: float = 0.03,
        path_margin: float = 0.10,
        k_turn_min_frames: int = 6,
        k_turn_max_frames: int = 12,
        side_correction_steer: float = 0.3,
        side_correction_speed: float = 0.1,
        side_correction_frames: int = 4,
        front_half_fov_deg: float = 30.0,
        threat_half_fov_deg: float = 45.0,
        self_detection_threshold_m: float = 0.08,
        min_valid_range_m: float = 0.01,
    ):
        """Initialize collision avoidance controller.

        Defaults mirror ``NavigationTuning``'s ``ClearanceZones``/
        ``EscapeManeuverParams``/``LidarSectorParams`` defaults; callers
        wired to a tuning profile (e.g. ``CoreNavigator``) should pass those
        values explicitly so a loaded profile actually takes effect.

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
            front_half_fov_deg: Half-width of the forward clearance cone (deg),
                used by ``compute_forward_clearance``
            threat_half_fov_deg: Half-width of the threat-detection sectors (deg),
                used by ``detect_threat_direction``/``compute_rear_clearance``/etc.
            self_detection_threshold_m: Rays no farther than this are discarded as
                chassis/cable self-reflection when a sector filters for it (m)
            min_valid_range_m: LIDAR ranges at or below this are treated as
                invalid (no-return) readings (m)
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
        self.front_half_fov_rad = math.radians(front_half_fov_deg)
        self.threat_half_fov_rad = math.radians(threat_half_fov_deg)
        self.self_detection_threshold_m = self_detection_threshold_m
        self.min_valid_range_m = min_valid_range_m

    def _forward_path_ranges(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
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
        mask = ahead & (lateral < self.path_half_width) & (ranges > self.min_valid_range_m)
        return np.asarray(ranges[mask])

    def assess_risk(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
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
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
        self_detection_threshold_m: float = 0.08,
        min_valid_range_m: float = 0.01,
    ) -> np.ndarray:
        """Valid ranges whose bearing falls within ``center ± half_fov``.

        Bearings come from ``lidar_angles`` (0 rad = forward, +pi/2 = left,
        -pi/2 = right, +/-pi = rear). When angles are unavailable a full 360 deg
        scan indexed from ``angle_min = -pi`` is assumed, so every sector helper
        agrees on which way is forward regardless of the scan's index ordering.

        A staticmethod on purpose: called both as an instance method (which
        passes its own tuning-sourced thresholds explicitly) and directly as
        ``CollisionAvoidanceController._sector_ranges(...)`` by external,
        instance-less callers (e.g. telemetry_bridge_node.py's OLED summary),
        which fall back to these keyword defaults.

        Args:
            lidar_ranges: Array of LIDAR range measurements.
            lidar_angles: Per-ray bearings (radians), or None to synthesise a
                full ``[-pi, pi)`` sweep.
            center_rad: Centre bearing of the sector (radians).
            half_fov_rad: Half-width of the sector (radians).
            filter_self_detection: Also discard rays no farther than
                ``self_detection_threshold_m`` — mount occlusion or cable
                clutter reflecting the chassis itself, not a real obstacle.
                Only pass this for side/rear sectors: never for the
                pure-forward bearing, where a genuine near-contact inside that
                radius must still register as a threat.
            self_detection_threshold_m: Threshold used when
                ``filter_self_detection`` is set (m).
            min_valid_range_m: LIDAR ranges at or below this are treated as
                invalid (no-return) readings (m), used when
                ``filter_self_detection`` is not set.
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
        min_valid = self_detection_threshold_m if filter_self_detection else min_valid_range_m
        # np.isfinite excludes no-return rays (+inf beyond LIDAR max range):
        # ranges > min_valid alone lets them through (inf > any finite
        # threshold), and a single stray inf inside a sector's window turns
        # its mean/min/max into inf for every caller -- both the OLED's
        # displayed clearance and detect_threat_direction's real
        # collision-avoidance sectors.
        mask = (np.abs(delta) <= half_fov_rad) & (ranges > min_valid) & np.isfinite(ranges)
        return np.asarray(ranges[mask])

    @staticmethod
    def _sector_to_model(
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
        self_detection_threshold_m: float = 0.08,
        min_valid_range_m: float = 0.01,
    ) -> SectorRanges:
        """Compute aggregate metrics for an angular sector as a SectorRanges."""
        ranges = CollisionAvoidanceController._sector_ranges(
            lidar_ranges,
            lidar_angles,
            center_rad,
            half_fov_rad,
            filter_self_detection,
            self_detection_threshold_m,
            min_valid_range_m,
        )
        return SectorRanges(
            bearing_rad=center_rad,
            half_fov_rad=half_fov_rad,
            mean_range_m=float(np.mean(ranges)) if ranges.size > 0 else 10.0,
            min_range_m=float(np.min(ranges)) if ranges.size > 0 else 10.0,
            max_range_m=float(np.max(ranges)) if ranges.size > 0 else 10.0,
            valid_count=int(ranges.size),
        )

    def compute_forward_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> float:
        """Mean clearance in the forward +/-30 deg sector (0 rad = forward).

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            Forward clearance distance (m).
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0

        sr = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            0.0,
            self.front_half_fov_rad,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
        )
        return sr.mean_range_m if sr.valid_count > 0 else 10.0

    def compute_rear_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> float:
        """Minimum clearance in the rear +/-45 deg sector (+/-pi rad = rear).

        Used to gate reverse / K-turn escapes so the robot never backs into a
        wall it cannot see. Self-detection filtered: a chassis/cable reflection
        directly behind the robot must not permanently read as "wall right
        there" and block every reverse escape for the rest of the run.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0

        sr = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            math.pi,
            self.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
        )
        return sr.min_range_m if sr.valid_count > 0 else 10.0

    def compute_min_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
        center_rad: float = 0.0,
        half_fov_rad: float = math.pi / 2,
    ) -> float:
        """Minimum clearance over a wide sector (default: front ±90°).

        ``compute_forward_clearance`` averages a narrow ±30° cone — fine for normal
        driving, but blind to a lateral clip (e.g. a maneuver that swings the chassis
        sideways into an obstacle that was never in front of it). Used to gate
        maneuvers where the robot's path isn't a straight line, so a side contact is
        caught before it happens instead of only checking what's dead ahead.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return 10.0

        sr = self._sector_to_model(
            lidar_ranges, lidar_angles, center_rad, half_fov_rad, min_valid_range_m=self.min_valid_range_m,
        )
        return sr.min_range_m if sr.valid_count > 0 else 10.0

    def detect_threat_direction(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
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
            sr = self._sector_to_model(
                lidar_ranges,
                lidar_angles,
                center_rad,
                self.threat_half_fov_rad,
                filter_self_detection,
                self_detection_threshold_m=self.self_detection_threshold_m,
                min_valid_range_m=self.min_valid_range_m,
            )
            return sr.min_range_m if sr.valid_count > 0 else 10.0

        directions = {
            # Forward is never self-detection filtered: a genuine near-contact
            # dead ahead must still register even inside that radius.
            ThreatDirection.FRONT: sector_min(0.0),
            ThreatDirection.LEFT: sector_min(math.pi / 2, filter_self_detection=True),
            ThreatDirection.RIGHT: sector_min(-math.pi / 2, filter_self_detection=True),
            ThreatDirection.BACK: sector_min(math.pi, filter_self_detection=True),
        }

        closest = min(directions, key=lambda direction: directions[direction])
        if directions[closest] > 1.0:
            return ThreatDirection.NONE
        return closest

    def _k_turn_steer_sign(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...] | None,
        lidar_angles: np.ndarray | tuple[float, ...] | None,
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
        left = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            math.pi / 2,
            self.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
        )
        right = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            -math.pi / 2,
            self.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
        )
        left_clear = left.min_range_m if left.valid_count > 0 else 10.0
        right_clear = right.min_range_m if right.valid_count > 0 else 10.0
        # Swing left (negative steering while reversing) when the left is
        # clearer; swing right (positive) when the right is clearer.
        return -1.0 if left_clear > right_clear else 1.0

    def compute_escape_maneuver(
        self,
        risk: RiskLevel,
        threat_dir: ThreatDirection,
        lidar_ranges: np.ndarray | tuple[float, ...] | None = None,
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
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
            already_touching = self._side_clearance(math.pi / 2, lidar_ranges, lidar_angles) < self.contact_dist
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=-self.side_correction_steer,
                speed=self.escape_rev_speed if already_touching else self.side_correction_speed,
                duration_frames=self.k_turn_min_frames if already_touching else self.side_correction_frames,
                priority=1,
            )

        if threat_dir == ThreatDirection.RIGHT:
            # Threat on the right — steer left (away): positive steering.
            already_touching = self._side_clearance(-math.pi / 2, lidar_ranges, lidar_angles) < self.contact_dist
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=self.side_correction_steer,
                speed=self.escape_rev_speed if already_touching else self.side_correction_speed,
                duration_frames=self.k_turn_min_frames if already_touching else self.side_correction_frames,
                priority=1,
            )

        return None

    def _side_clearance(
        self,
        center_rad: float,
        lidar_ranges: np.ndarray | tuple[float, ...] | None,
        lidar_angles: np.ndarray | tuple[float, ...] | None,
    ) -> float:
        """Minimum clearance in a +/-45 deg sector about ``center_rad``.

        Used to tell "obstacle getting close" from "chassis is already at the
        wall" for a side threat: the default forward-creep ``SIDE_CORRECTION``
        assumes there's still room to rotate clear before translating into the
        obstacle. When there's none left (clearance already at or below
        ``contact_dist`` — e.g. a starting position placed right at a narrow
        corridor's edge, with zero margin by construction), creeping forward
        while steering away deepens the overlap faster than the chassis can
        rotate out of it, in the same tick. Reversing instead opens real
        separation before any forward motion resumes.
        """
        sr = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            center_rad,
            self.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
        )
        return sr.min_range_m if sr.valid_count > 0 else 10.0
