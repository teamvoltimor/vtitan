"""Collision avoidance controller for LIDAR-based obstacle detection.

Assesses collision risk from LIDAR data and generates escape maneuvers
(K-turn, slalom) when obstacles are detected.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.enums import Direction, ManeuverType, RiskLevel
from shared.domain.models import SectorRanges

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.enums import Section
    from shared.domain.models import Pose

logger = logging.getLogger(__name__)


def bumper_gap_ahead(range_m: float) -> float:
    """Convert a FORWARD sensor range into the gap from the front bumper.

    Clearance thresholds are policy statements about the CHASSIS ("do not get
    within 10 cm of something"), but the LIDAR reports distance from itself, and
    the sensor is not at the chassis centre. Comparing a threshold against a raw
    range therefore measures it from wherever the sensor is mounted, so moving
    the mount silently redefines every threshold -- which is exactly what
    ``6c727c87`` did on 2026-08-21: forward readings moved 12.2 cm closer and
    the CRITICAL gate went from unreachable to firing 30811 times across the
    corpus, with ZERO of those ticks reachable in the previous frame.

    Converting here keeps the sector helpers' documented contract (they return
    raw sensor ranges) and puts the frame change in the policy comparison, where
    it belongs.

    Approximate off-axis: the offset is exact straight ahead and shortens with
    bearing, so within the +/-30 deg forward cone this is conservative by at
    most a few millimetres. Good enough for a threshold quoted to a centimetre.
    """
    return range_m - RobotSpecs.LIDAR_TO_FRONT_BUMPER


def bumper_gap_behind(range_m: float) -> float:
    """Convert a REAR sensor range into the gap from the rear bumper.

    Subtracts ~0.272 m rather than the front's ~0.028 m, because the sensor sits
    at the front. Without this a rear threshold is unreachable: an obstacle
    touching the rear bumper reports 0.272 m, so the shipped 0.10 m contact
    distance could never fire and the reverse guard could not stop a reverse
    before impact -- measured 2026-08-22, alongside wall collisions rising 1 ->
    36 as escapes went from 4 to 54 per lap.
    """
    return range_m - RobotSpecs.LIDAR_TO_REAR_BUMPER


def mask_mapped_obstacles(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    robot_pose: Pose,
    mapped_xy: Sequence[tuple[float, float, Section]],
    radius_m: float,
) -> np.ndarray:
    """Blank the LIDAR returns that land on an obstacle the planner already owns.

    The reactive layer in this module exists for what the planner does *not*
    know about: walls it is drifting into, and unmapped returns. A traffic sign
    the ``SignRouter`` is actively routing around is the opposite case — the
    planner has a deliberate plan for it, and that plan is to pass it at
    ``lateral_offset`` centre-to-centre — a gap narrower than ``contact_dist``
    once the sign's own half-width is subtracted.
    So with the raw scan the escape maneuver
    fires on every single sign pass and reverses the robot out of a gap the
    planner aimed for on purpose. Measured over the 16 obstacles fixtures, that
    decides the run before the router's aim can matter at all: every
    planning-side knob reads flat because the reactive layer overrides it (see
    ``docs/sign-avoidance-investigation.md``, "The escape layer is the gate").

    So the split is by *provenance*, not by distance: a return attributable to a
    mapped, actively-routed sign is withheld from the escape trigger, while
    walls and genuinely unknown returns keep the full guard. This is
    deliberately narrower than blanking the whole obstacle class from
    perception (``lidar_sees_obstacles=False``), which is a diagnostic only —
    the C1 really does see the signs, and an unmapped one must still stop the
    robot.

    Masked rays are set to ``inf`` rather than dropped, so the returned array
    stays index-aligned with ``lidar_angles``. ``inf`` is already this module's
    no-return sentinel: ``sector_ranges`` filters it via ``np.isfinite`` and
    ``_forward_path_ranges`` rejects it via its lateral-offset test.

    Args:
        lidar_ranges: Array of LIDAR range measurements.
        lidar_angles: Per-ray bearings (radians, 0 = forward). Synthesised from
            a full ``[-pi, pi)`` sweep when omitted, matching the rest of this
            module.
        robot_pose: Robot pose in world frame, needed to place each ray's
            endpoint on the map.
        mapped_xy: World positions of the mapped obstacles to withhold, each
            paired with its own corridor (``SignRouter.routed_sign_positions_by_corridor``).
            A ray is only attributed to a sign if its endpoint is within
            ``radius_m`` AND ``robot_pose`` itself is currently in that sign's
            corridor -- proximity alone is not trustworthy under a believed
            pose that is a wrong-but-consistent rigid rotation of the truth
            (the blind-mode rotational-lock failure), which can reproject a
            genuinely unmapped obstacle's ray onto a routed sign's
            coordinates purely by coincidence. Gated on the ROBOT's own
            corridor rather than the ray endpoint's: a ray endpoint can jitter
            across a hard corridor boundary between ticks from ordinary LIDAR
            angle quantisation even when it is legitimately close to a sign
            just inside that boundary, which would make an endpoint-keyed
            gate flap; the robot itself is normally well inside a corridor,
            not standing on its 1.0/2.0 boundary, whenever anything is close
            enough to mask. See ``_SignTrack.corridor`` for the matching
            guard on the discovery side.
        radius_m: How close a ray endpoint must be to a mapped position to count
            as that obstacle. Must cover the obstacle's own half-diagonal plus
            localisation and mapping error, but stay well under the distance to
            the nearest wall behind it — too large and a wall standing behind a
            sign is silently masked along with it.

    Returns:
        A copy of ``lidar_ranges`` with attributed rays set to ``inf``. The
        input is returned unchanged (as an array) when there is nothing to mask.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    if ranges.size == 0 or len(mapped_xy) == 0 or radius_m <= 0.0:
        return ranges

    if lidar_angles is None:
        angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
    else:
        angles = np.asarray(lidar_angles, dtype=float)

    robot_x, robot_y, robot_yaw = robot_pose.x, robot_pose.y, robot_pose.yaw
    robot_corridor = corridor_for_position(robot_x, robot_y)
    # Only finite returns have an endpoint to attribute; inf rays are already
    # no-returns and feeding them through cos/sin yields inf-inf = nan.
    finite = np.isfinite(ranges)
    bearings = angles + robot_yaw
    end_x = robot_x + ranges * np.cos(bearings)
    end_y = robot_y + ranges * np.sin(bearings)

    attributed = np.zeros(ranges.shape, dtype=bool)
    for mapped_x, mapped_y, mapped_corridor in mapped_xy:
        if mapped_corridor != robot_corridor:
            continue
        attributed |= np.hypot(end_x - mapped_x, end_y - mapped_y) < radius_m

    masked = ranges.copy()
    masked[attributed & finite] = np.inf
    return masked


class ThreatDirection(StrEnum):
    """Bearing of the nearest obstacle relative to the robot."""

    FRONT = "front"
    LEFT = "left"
    RIGHT = "right"
    BACK = "back"
    NONE = "none"


@dataclass(slots=True)
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
        min_valid_range_m: float = 0.05,
        threat_no_detection_range_m: float = 1.0,
        no_data_range_m: float = 10.0,
        blind_wedge_left_min_deg: float = -160.0,
        blind_wedge_left_max_deg: float = -115.0,
        blind_wedge_right_min_deg: float = 115.0,
        blind_wedge_right_max_deg: float = 175.0,
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
            threat_no_detection_range_m: A sector's nearest reading beyond
                this distance doesn't count as a threat at all (m)
            no_data_range_m: Fallback range when no valid LIDAR readings are
                available (m)
            blind_wedge_left_min_deg: Start bearing (deg) of the rear-left
                mount-occlusion wedge, excluded from every sector by angle
                regardless of range (see ``sector_ranges``)
            blind_wedge_left_max_deg: End bearing (deg) of the rear-left wedge
            blind_wedge_right_min_deg: Start bearing (deg) of the rear-right
                mount-occlusion wedge
            blind_wedge_right_max_deg: End bearing (deg) of the rear-right wedge
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
        self.threat_no_detection_range_m = threat_no_detection_range_m
        self.no_data_range_m = no_data_range_m
        self.blind_wedge_left_min_rad = math.radians(blind_wedge_left_min_deg)
        self.blind_wedge_left_max_rad = math.radians(blind_wedge_left_max_deg)
        self.blind_wedge_right_min_rad = math.radians(blind_wedge_right_min_deg)
        self.blind_wedge_right_max_rad = math.radians(blind_wedge_right_max_deg)

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> CollisionAvoidanceController:
        """Build controller from NavigationTuning parameters.

        This is the preferred constructor for production code; it ensures that
        any loaded tuning profile actually takes effect rather than being
        silently overridden by hardcoded defaults.

        Args:
            tuning: NavigationTuning instance (usually from load_default).

        Returns:
            CollisionAvoidanceController with values from tuning.
        """
        return cls(
            contact_dist=tuning.clearance.CONTACT_DIST,
            slow_dist=tuning.clearance.SLOW_DIST,
            fast_dist=tuning.clearance.FAST_DIST,
            escape_rev_speed=tuning.escape.REV_SPEED,
            escape_steer_scale=tuning.escape.rev_steer_norm(),
            stuck_threshold=tuning.escape.STUCK_MOVE_THRESHOLD,
            path_margin=tuning.clearance.PATH_MARGIN,
            k_turn_min_frames=tuning.escape.K_TURN_MIN_FRAMES,
            k_turn_max_frames=tuning.escape.K_TURN_MAX_FRAMES,
            side_correction_steer=tuning.escape.side_correction_steer_norm(),
            side_correction_speed=tuning.escape.SIDE_CORRECTION_SPEED,
            side_correction_frames=tuning.escape.SIDE_CORRECTION_FRAMES,
            front_half_fov_deg=tuning.lidar_sectors.FRONT_HALF_FOV_DEG,
            threat_half_fov_deg=tuning.lidar_sectors.THREAT_HALF_FOV_DEG,
            self_detection_threshold_m=tuning.lidar_sectors.SELF_DETECTION_THRESHOLD_M,
            min_valid_range_m=tuning.lidar_sectors.MIN_VALID_RANGE_M,
            threat_no_detection_range_m=tuning.lidar_sectors.THREAT_NO_DETECTION_RANGE_M,
            no_data_range_m=tuning.lidar_sectors.NO_DATA_RANGE_M,
            blind_wedge_left_min_deg=tuning.lidar_sectors.BLIND_WEDGE_LEFT_MIN_DEG,
            blind_wedge_left_max_deg=tuning.lidar_sectors.BLIND_WEDGE_LEFT_MAX_DEG,
            blind_wedge_right_min_deg=tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MIN_DEG,
            blind_wedge_right_max_deg=tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MAX_DEG,
        )

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

        # Judged as a gap from the BUMPER, not as a raw sensor range: the
        # thresholds are statements about how close the chassis may come to
        # something, and the sensor is 12.2 cm ahead of the chassis centre.
        gap = bumper_gap_ahead(float(np.min(path)))

        if gap < self.contact_dist:
            return RiskLevel.CRITICAL
        if gap < self.slow_dist:
            return RiskLevel.OBSTACLE
        return RiskLevel.SAFE

    @staticmethod
    def sector_ranges(
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
        self_detection_threshold_m: float | None = None,
        min_valid_range_m: float | None = None,
        blind_wedge_left_min_rad: float | None = None,
        blind_wedge_left_max_rad: float | None = None,
        blind_wedge_right_min_rad: float | None = None,
        blind_wedge_right_max_rad: float | None = None,
        apply_blind_wedge_mask: bool = True,
    ) -> np.ndarray:
        """Valid ranges whose bearing falls within ``center ± half_fov``.

        Bearings come from ``lidar_angles`` (0 rad = forward, +pi/2 = left,
        -pi/2 = right, +/-pi = rear). When angles are unavailable a full 360 deg
        scan indexed from ``angle_min = -pi`` is assumed, so every sector helper
        agrees on which way is forward regardless of the scan's index ordering.

        A staticmethod on purpose: called both as an instance method (which
        passes its own tuning-sourced thresholds explicitly) and directly as
        ``CollisionAvoidanceController.sector_ranges(...)`` by external,
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
            blind_wedge_left_min_rad: Start bearing of the left rear blind
                wedge (radians).
            blind_wedge_left_max_rad: End bearing of the left rear blind wedge
                (radians).
            blind_wedge_right_min_rad: Start bearing of the right rear blind
                wedge (radians).
            blind_wedge_right_max_rad: End bearing of the right rear blind
                wedge (radians). These cover the two rear-corner mount-occlusion
                wedges measured 2026-08-04, where self-collision reads as a real
                close range at every distance -- a distance threshold can't
                separate that from a genuine close obstacle at the same bearing,
                so this is filtered by angle instead. Always applied (not gated
                behind ``filter_self_detection``): the pure-forward bearing never
                overlaps these rear wedges, so there's no case where a real
                forward contact would be discarded by them.
            apply_blind_wedge_mask: Set False to skip the wedge exclusion --
                used by ``_sector_to_model`` to tell "this bearing is a known
                blind spot" apart from "genuinely nothing out there" by
                re-running the same query with the mask lifted.
        """
        ranges = np.asarray(lidar_ranges, dtype=float)
        if ranges.size == 0:
            return ranges

        if self_detection_threshold_m is None or min_valid_range_m is None or blind_wedge_left_min_rad is None:
            tuning = get_tuning(None)
            if self_detection_threshold_m is None:
                self_detection_threshold_m = tuning.lidar_sectors.SELF_DETECTION_THRESHOLD_M
            if min_valid_range_m is None:
                min_valid_range_m = tuning.lidar_sectors.MIN_VALID_RANGE_M
            if blind_wedge_left_min_rad is None:
                blind_wedge_left_min_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_LEFT_MIN_DEG)
                blind_wedge_left_max_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_LEFT_MAX_DEG)
                blind_wedge_right_min_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MIN_DEG)
                blind_wedge_right_max_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MAX_DEG)

        if lidar_angles is None:
            angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
        else:
            angles = np.asarray(lidar_angles, dtype=float)

        # Wrapped angular distance from the sector centre, in [-pi, pi].
        delta = np.arctan2(np.sin(angles - center_rad), np.cos(angles - center_rad))
        min_valid = self_detection_threshold_m if filter_self_detection else min_valid_range_m
        if apply_blind_wedge_mask:
            in_blind_wedge = ((angles >= blind_wedge_left_min_rad) & (angles <= blind_wedge_left_max_rad)) | (
                (angles >= blind_wedge_right_min_rad) & (angles <= blind_wedge_right_max_rad)
            )
        else:
            in_blind_wedge = np.zeros(angles.shape, dtype=bool)
        # np.isfinite excludes no-return rays (+inf beyond LIDAR max range):
        # ranges > min_valid alone lets them through (inf > any finite
        # threshold), and a single stray inf inside a sector's window turns
        # its mean/min/max into inf for every caller -- both the OLED's
        # displayed clearance and detect_threat_direction's real
        # collision-avoidance sectors.
        mask = (np.abs(delta) <= half_fov_rad) & (ranges > min_valid) & np.isfinite(ranges) & ~in_blind_wedge
        return np.asarray(ranges[mask])

    @staticmethod
    def _sector_to_model(
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        center_rad: float,
        half_fov_rad: float,
        filter_self_detection: bool = False,
        self_detection_threshold_m: float | None = None,
        min_valid_range_m: float | None = None,
        no_data_range_m: float | None = None,
        blind_wedge_left_min_rad: float | None = None,
        blind_wedge_left_max_rad: float | None = None,
        blind_wedge_right_min_rad: float | None = None,
        blind_wedge_right_max_rad: float | None = None,
    ) -> SectorRanges:
        """Compute aggregate metrics for an angular sector as a SectorRanges."""
        ranges = CollisionAvoidanceController.sector_ranges(
            lidar_ranges,
            lidar_angles,
            center_rad,
            half_fov_rad,
            filter_self_detection,
            self_detection_threshold_m,
            min_valid_range_m,
            blind_wedge_left_min_rad,
            blind_wedge_left_max_rad,
            blind_wedge_right_min_rad,
            blind_wedge_right_max_rad,
        )
        if no_data_range_m is None:
            no_data_range_m = get_tuning(None).lidar_sectors.NO_DATA_RANGE_M
        wedge_masked = False
        if ranges.size == 0:
            # Distinguish "this bearing is a known permanent blind spot" from
            # "nothing is out there right now": re-run the same sector query
            # with the wedge exclusion lifted -- if rays appear, every ray
            # this sector could see was inside a blind wedge, not genuinely
            # absent. Both cases still report no_data_range_m (a fully-masked
            # sector is no more "definitely clear" than a fully-empty one),
            # but callers that care (e.g. telemetry) can check wedge_masked.
            unmasked = CollisionAvoidanceController.sector_ranges(
                lidar_ranges,
                lidar_angles,
                center_rad,
                half_fov_rad,
                filter_self_detection,
                self_detection_threshold_m,
                min_valid_range_m,
                apply_blind_wedge_mask=False,
            )
            wedge_masked = unmasked.size > 0
        return SectorRanges(
            bearing_rad=center_rad,
            half_fov_rad=half_fov_rad,
            mean_range_m=float(np.mean(ranges)) if ranges.size > 0 else no_data_range_m,
            min_range_m=float(np.min(ranges)) if ranges.size > 0 else no_data_range_m,
            max_range_m=float(np.max(ranges)) if ranges.size > 0 else no_data_range_m,
            valid_count=int(ranges.size),
            wedge_masked=wedge_masked,
        )

    def sector(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
        center_rad: float = 0.0,
        half_fov_rad: float | None = None,
        *,
        filter_self_detection: bool = False,
    ) -> SectorRanges:
        """This controller's configured view of one angular sector.

        The single place the instance's sector parameters (self-detection
        threshold, minimum valid range, no-data sentinel, blind wedges) are
        wired to ``_sector_to_model``. Every clearance helper below goes
        through here, so a change to the mount geometry or the wedge angles
        lands in one place instead of four near-identical argument lists.

        Prefer this over the ``compute_*_clearance`` shorthands when the answer
        gates an action: the returned model carries ``measured`` and
        ``wedge_masked`` alongside the distance, and a bare distance cannot
        tell "clear" from "blind" (see ``SectorRanges.measured``).
        """
        return self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            center_rad,
            self.threat_half_fov_rad if half_fov_rad is None else half_fov_rad,
            filter_self_detection=filter_self_detection,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
            no_data_range_m=self.no_data_range_m,
            blind_wedge_left_min_rad=self.blind_wedge_left_min_rad,
            blind_wedge_left_max_rad=self.blind_wedge_left_max_rad,
            blind_wedge_right_min_rad=self.blind_wedge_right_min_rad,
            blind_wedge_right_max_rad=self.blind_wedge_right_max_rad,
        )

    def rear_sector(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> SectorRanges:
        """The rear +/-``threat_half_fov`` sector, self-detection filtered.

        Filtered because a chassis/cable reflection directly behind the robot
        must not permanently read as "wall right there" and block every reverse
        escape for the rest of the run.

        Callers gating a reverse want this rather than ``compute_rear_clearance``:
        on this mount the occlusion wedges leave only a ~25 deg slot straight
        back, and if that slot goes (a different mount, a cable, a smaller scan)
        the distance alone still reads as open road. ``measured`` is what tells
        them apart.
        """
        return self.sector(lidar_ranges, lidar_angles, math.pi, filter_self_detection=True)

    def compute_forward_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> float:
        """Minimum clearance in the forward +/-30 deg sector (0 rad = forward).

        Was the sector's MEAN, not its minimum -- a real near-contact dead
        ahead widens the cone's grazing-incidence edges into no-returns
        (physically expected: a flat surface a few cm away reflects rays near
        its own edge too shallowly to return a signal at all), and the LIDAR
        callback substitutes those no-returns with ``LIDAR_MAX_RANGE`` before
        this ever sees them (see ``ros2_hardware_gateway._lidar_callback``).
        Averaging genuine ~0.08m readings together with several fabricated
        12m ones reports several metres of open road during the single most
        blocked moment of a run -- confirmed against a real 2026-08-04 bag
        (``run_20260804_114500``, t=26.02s): the sector's true minimum was
        0.078m dead ahead (matching flat-wall-at-close-range raycast geometry,
        ``d/cos(theta)`` across the cone) while the old mean reported 5.15m.
        Minimum matches the pattern ``compute_rear_clearance``/
        ``compute_min_clearance`` already use, and is what a clearance number
        meant to gate speed should be: the worst case in the cone, not an
        average that a single no-return can swamp.

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            Forward clearance distance (m).
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return self.no_data_range_m

        sr = self.sector(lidar_ranges, lidar_angles, 0.0, self.front_half_fov_rad)
        return sr.min_range_m if sr.measured else self.no_data_range_m

    def compute_rear_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> float:
        """Minimum clearance in the rear +/-45 deg sector (+/-pi rad = rear).

        Reports ``no_data_range_m`` when the rear sector saw nothing, which
        reads identically to open road -- a gate that acts on this number alone
        fails open. Use ``rear_sector`` and check ``measured`` when the answer
        authorises a reverse; this shorthand is for display and for callers
        that only want a number.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return self.no_data_range_m

        sr = self.rear_sector(lidar_ranges, lidar_angles)
        return sr.min_range_m if sr.measured else self.no_data_range_m

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
            return self.no_data_range_m

        sr = self.sector(lidar_ranges, lidar_angles, center_rad, half_fov_rad)
        return sr.min_range_m if sr.measured else self.no_data_range_m

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
                no_data_range_m=self.no_data_range_m,
                blind_wedge_left_min_rad=self.blind_wedge_left_min_rad,
                blind_wedge_left_max_rad=self.blind_wedge_left_max_rad,
                blind_wedge_right_min_rad=self.blind_wedge_right_min_rad,
                blind_wedge_right_max_rad=self.blind_wedge_right_max_rad,
            )
            return sr.min_range_m if sr.valid_count > 0 else self.no_data_range_m

        directions = {
            # Forward is never self-detection filtered: a genuine near-contact
            # dead ahead must still register even inside that radius.
            ThreatDirection.FRONT: sector_min(0.0),
            ThreatDirection.LEFT: sector_min(math.pi / 2, filter_self_detection=True),
            ThreatDirection.RIGHT: sector_min(-math.pi / 2, filter_self_detection=True),
            ThreatDirection.BACK: sector_min(math.pi, filter_self_detection=True),
        }

        closest = min(directions, key=lambda direction: directions[direction])
        if directions[closest] > self.threat_no_detection_range_m:
            return ThreatDirection.NONE
        return closest

    def _k_turn_steer_sign(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...] | None,
        lidar_angles: np.ndarray | tuple[float, ...] | None,
        direction: Direction | None = None,
    ) -> float:
        """Steering sign that swings the nose toward the clearer side in reverse.

        Ackermann reverse flips the yaw response relative to forward travel
        (``yaw_rate = (v/L)*tan(steer)`` with ``v < 0``), so a positive steering
        command swings the nose toward the robot's RIGHT while reversing, and a
        negative command swings it left. A fixed sign therefore swings the nose
        into whichever wall happens to be on that side half the time; picking
        the sign from the wider of the two side clearances swings away from the
        tighter wall instead.

        Neither side clearance is usable exactly when this matters most: a
        FRONT threat fires at every corner, where both side sectors legitimately
        see past the inner block into open track and read as no-return (or as a
        near-tie), not as a wall. A fixed fallback there is not a rare edge
        case, it is every corner of every lap, and always resolving it toward
        the same side is a standing bias that never shows up in a sim whose
        LIDAR sees the true walls cleanly enough to rarely tie at all — no
        symmetry check catches it because it does not depend on which way
        around the loop the round travels; it depends only on which body side
        the fallback happens to prefer.

        The loop geometry itself resolves the *remaining* tie without guessing:
        going clockwise around the island keeps it on the robot's right for the
        entire lap, counterclockwise keeps it on the left, so the side away
        from the island is the structurally safer one to swing the nose toward
        when LIDAR gives no side any edge at all. ``direction`` is the inferred
        travel direction, the same source ``LapDetector`` and the planned path
        already trust; ``None`` (only possible in the sliver before inference
        settles, a few corridor widths into the round) falls back to the old
        fixed side rather than stall the maneuver.

        A side with no valid ray is not the same as a tie: it means nothing
        registered within sensor range on that side at all, which is itself
        the clearest possible "open" reading -- most sharply so pinned against
        a wall, where the jammed side reads a real, close, valid return and
        the free side legitimately has nothing to reflect off within range.
        Requiring both sides to have a valid ray before trusting the
        comparison (an earlier version of this method did) throws away exactly
        that reading and falls through to the direction-based guess instead,
        which reasons about the island and has nothing to say about a chassis
        pinned against the *outer* wall -- measured pinning the right side at
        4.5 cm for the remainder of a run that never recovered. Substituting
        ``no_data_range_m`` for a missing side keeps that signal instead of
        discarding it; the direction fallback below now only fires when
        neither side has anything to say.
        """
        if lidar_ranges is not None:
            left = self._sector_to_model(
                lidar_ranges,
                lidar_angles,
                math.pi / 2,
                self.threat_half_fov_rad,
                filter_self_detection=True,
                self_detection_threshold_m=self.self_detection_threshold_m,
                min_valid_range_m=self.min_valid_range_m,
                no_data_range_m=self.no_data_range_m,
                blind_wedge_left_min_rad=self.blind_wedge_left_min_rad,
                blind_wedge_left_max_rad=self.blind_wedge_left_max_rad,
                blind_wedge_right_min_rad=self.blind_wedge_right_min_rad,
                blind_wedge_right_max_rad=self.blind_wedge_right_max_rad,
            )
            right = self._sector_to_model(
                lidar_ranges,
                lidar_angles,
                -math.pi / 2,
                self.threat_half_fov_rad,
                filter_self_detection=True,
                self_detection_threshold_m=self.self_detection_threshold_m,
                min_valid_range_m=self.min_valid_range_m,
                no_data_range_m=self.no_data_range_m,
                blind_wedge_left_min_rad=self.blind_wedge_left_min_rad,
                blind_wedge_left_max_rad=self.blind_wedge_left_max_rad,
                blind_wedge_right_min_rad=self.blind_wedge_right_min_rad,
                blind_wedge_right_max_rad=self.blind_wedge_right_max_rad,
            )
            if left.valid_count > 0 or right.valid_count > 0:
                left_clear = left.min_range_m if left.valid_count > 0 else self.no_data_range_m
                right_clear = right.min_range_m if right.valid_count > 0 else self.no_data_range_m
                if left_clear != right_clear:
                    # Swing left (negative steering while reversing) when the left is
                    # clearer; swing right (positive) when the right is clearer.
                    return -1.0 if left_clear > right_clear else 1.0
        if direction is Direction.CLOCKWISE:
            return -1.0
        if direction is Direction.COUNTERCLOCKWISE:
            return 1.0
        return 1.0

    def compute_escape_maneuver(
        self,
        risk: RiskLevel,
        threat_dir: ThreatDirection,
        lidar_ranges: np.ndarray | tuple[float, ...] | None = None,
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
        direction: Direction | None = None,
    ) -> EscapeManeuver | None:
        """Generate escape maneuver for detected threat.

        Args:
            risk: Current risk level
            threat_dir: Threat direction from detect_threat_direction()
            lidar_ranges: Current scan, used to pick the K-turn's steering side
                (toward the clearer side, not a fixed direction).
            lidar_angles: Per-ray bearings matching ``lidar_ranges``.
            direction: Inferred travel direction, the fallback for the K-turn's
                side when LIDAR alone cannot tell (see ``_k_turn_steer_sign``).

        Returns:
            EscapeManeuver command or None if no maneuver needed
        """
        if risk == RiskLevel.SAFE:
            return None

        if threat_dir == ThreatDirection.FRONT:
            # K-turn: reverse while steering hard for CRITICAL risk; a shorter,
            # straight reverse to open clearance for the milder OBSTACLE risk.
            steer_sign = self._k_turn_steer_sign(lidar_ranges, lidar_angles, direction)
            return EscapeManeuver(
                maneuver_type=ManeuverType.K_TURN,
                steering=self.escape_steer_scale * steer_sign if risk == RiskLevel.CRITICAL else 0.0,
                speed=self.escape_rev_speed,
                duration_frames=self.k_turn_max_frames if risk == RiskLevel.CRITICAL else self.k_turn_min_frames,
                priority=2 if risk == RiskLevel.CRITICAL else 1,
            )

        if threat_dir == ThreatDirection.LEFT:
            # Threat on the left — steer right (away). Positive steering is left
            # (CCW) throughout the stack for FORWARD travel, so the creeping
            # (non-touching) correction is negative.
            #
            # Already touching switches speed to reverse, and Ackermann reverse
            # flips the yaw response relative to forward (see
            # _k_turn_steer_sign's docstring for the physics), so the sign has
            # to flip with it. A fixed negative sign here drove the nose
            # further into the wall it was already touching instead of away
            # from it whenever this branch reversed -- measured pinning a side
            # at 4.5cm clearance for the rest of a run that never recovered.
            already_touching = self._side_clearance(math.pi / 2, lidar_ranges, lidar_angles) < self.contact_dist
            steer_sign = 1.0 if already_touching else -1.0
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=steer_sign * self.side_correction_steer,
                speed=self.escape_rev_speed if already_touching else self.side_correction_speed,
                duration_frames=self.k_turn_min_frames if already_touching else self.side_correction_frames,
                priority=1,
            )

        if threat_dir == ThreatDirection.RIGHT:
            # Threat on the right — steer left (away): positive steering while
            # creeping forward, negative once already touching and reversing,
            # for the same reverse-flips-yaw reason as the LEFT branch above.
            already_touching = self._side_clearance(-math.pi / 2, lidar_ranges, lidar_angles) < self.contact_dist
            steer_sign = -1.0 if already_touching else 1.0
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=steer_sign * self.side_correction_steer,
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
        if lidar_ranges is None:
            # Matches _k_turn_steer_sign's fallback: no lidar data means no
            # basis to claim the chassis is already touching a wall, so treat
            # it as clear rather than crashing _sector_to_model on None.
            # Use the configured no-data sentinel (lidar_sectors.NO_DATA_RANGE_M),
            # not a second hardcoded 10.0.
            return self.no_data_range_m
        sr = self._sector_to_model(
            lidar_ranges,
            lidar_angles,
            center_rad,
            self.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=self.self_detection_threshold_m,
            min_valid_range_m=self.min_valid_range_m,
            no_data_range_m=self.no_data_range_m,
            blind_wedge_left_min_rad=self.blind_wedge_left_min_rad,
            blind_wedge_left_max_rad=self.blind_wedge_left_max_rad,
            blind_wedge_right_min_rad=self.blind_wedge_right_min_rad,
            blind_wedge_right_max_rad=self.blind_wedge_right_max_rad,
        )
        return sr.min_range_m if sr.valid_count > 0 else self.no_data_range_m
