"""Collision avoidance controller for LIDAR-based obstacle detection.

Assesses collision risk from LIDAR data and generates escape maneuvers (K-turn,
slalom) when obstacles are detected. The class wraps the pure sector math in
``sectors`` and the bumper conversions in ``bumper``; this module owns the
controller state, tuning wiring, risk/threat assessment, and escape generation.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.enums import Direction, ManeuverType, RiskLevel, ThreatDirection

from src.navigation.control.controllers.collision_avoidance.bumper import bumper_gap_ahead
from src.navigation.control.controllers.collision_avoidance.sectors import (
    _forward_path_has_rays,
    _forward_path_ranges,
    _sector_to_model,
    forward_path_nearest_ray,
    sector_ranges,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.config.navigation_tuning.motion import ClearanceZones
    from shared.domain.models import SectorRanges

logger = logging.getLogger(__name__)


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


@dataclass(frozen=True, slots=True)
class ParkingGate:
    """The two clearance numbers the parking maneuver's stop-check needs.

    The parking controller drives laterally into a bay, so a single forward cone
    is not enough: it must also be stopped before any sideways or slightly-
    rearward clip (a wall or block edge the narrow forward cone would never see).
    Those are two different LIDAR queries -- a narrow forward min and a full 360-
    degree min -- kept together here so the navigator asks for both in one call
    rather than re-sweeping the same scan twice.
    """

    forward_m: float
    sweep_m: float


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
            over -- the chassis half-width plus a clearance margin (m)
    """

    def __init__(
        self,
        contact_dist: float,
        slow_dist: float,
        fast_dist: float,
        escape_rev_speed: float,
        escape_steer_scale: float,
        stuck_threshold: float,
        path_margin: float,
        k_turn_min_frames: int,
        k_turn_max_frames: int,
        side_correction_steer: float,
        side_correction_speed: float,
        side_correction_frames: int,
        front_half_fov_deg: float,
        threat_half_fov_deg: float,
        self_detection_threshold_m: float,
        min_valid_range_m: float,
        threat_no_detection_range_m: float,
        no_data_range_m: float,
        blind_wedge_left_min_deg: float,
        blind_wedge_left_max_deg: float,
        blind_wedge_right_min_deg: float,
        blind_wedge_right_max_deg: float,
        ahead_of_bumper: bool = False,
    ):
        """Initialize collision avoidance controller.

        Construction is via :meth:`from_tuning` (from a ``NavigationTuning``),
        which is the single source of truth for these values -- the LIDAR sector
        parameters in particular reflect the current chassis, whose rear mount
        no longer leaves a rear sensing slot. Constructing directly is for tests
        that need to pin a specific value.

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
            threat_no_detection_range_m: A sector's nearest reading beyond this
                distance doesn't count as a threat at all (m)
            no_data_range_m: Fallback range when no valid LIDAR readings are
                available (m)
            blind_wedge_left_min_deg: Start bearing (deg) of the rear-left
                mount-occlusion wedge, excluded from every sector by angle
                regardless of range (see ``sector_ranges``)
            blind_wedge_left_max_deg: End bearing (deg) of the rear-left wedge
            blind_wedge_right_min_deg: Start bearing (deg) of the rear-right wedge
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
        self.ahead_of_bumper = ahead_of_bumper

    @classmethod
    def from_tuning(
        cls, tuning: NavigationTuning, clearance: ClearanceZones | None = None
    ) -> CollisionAvoidanceController:
        """Build controller from NavigationTuning parameters.

        This is the preferred constructor for production code; it ensures that
        any loaded tuning profile actually takes effect rather than being
        silently overridden by hardcoded defaults.

        Args:
            tuning: NavigationTuning instance (usually from load_default).
            clearance: Zones to use in place of ``tuning.clearance``, for a
                caller that has already resolved the per-challenge overrides
                (``ClearanceZones.for_obstacles_challenge``). Defaults to
                ``tuning.clearance``, so every existing call is unchanged.

        Returns:
            CollisionAvoidanceController with values from tuning.
        """
        clearance = clearance if clearance is not None else tuning.clearance
        return cls(
            contact_dist=clearance.CONTACT_DIST,
            slow_dist=clearance.SLOW_DIST,
            fast_dist=clearance.FAST_DIST,
            escape_rev_speed=tuning.escape.REV_SPEED,
            escape_steer_scale=tuning.escape.rev_steer_norm(),
            stuck_threshold=tuning.escape.STUCK_MOVE_THRESHOLD,
            path_margin=clearance.PATH_MARGIN,
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
            ahead_of_bumper=clearance.FORWARD_PATH_AHEAD_OF_BUMPER,
        )

    def _forward_path_ranges(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None,
    ) -> np.ndarray:
        """Ranges of points ahead of the robot inside its driving lane.

        See ``sectors._forward_path_ranges`` for the geometry; this binds the
        instance's ``path_half_width`` and ``min_valid_range_m``.
        """
        return _forward_path_ranges(
            lidar_ranges, lidar_angles, self.path_half_width, self.min_valid_range_m,
            self.ahead_of_bumper,
        )

    def nearest_path_ray(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> tuple[float, float] | None:
        """Bearing and range of the ray :meth:`assess_risk` is deciding on.

        Same lane, same validity filter, same instance parameters -- so a
        recorded bearing is the one the risk verdict came from and not a
        re-derivation that might disagree. See
        ``sectors.forward_path_nearest_ray``.
        """
        return forward_path_nearest_ray(
            lidar_ranges, lidar_angles, self.path_half_width, self.min_valid_range_m,
            self.ahead_of_bumper,
        )

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
            # Two different causes read the same here: no scan rays fell inside
            # the forward lane at all (a near-empty scan -- nothing to judge,
            # safe by construction), or the lane DID have rays but every one of
            # them was a no-return -- the hardware gateway's fabricated
            # LIDAR_MAX_RANGE substitute for a real grazing-incidence echo,
            # which is the signature of something very close spanning the
            # WHOLE cone, not of open road (see _forward_path_ranges's
            # no-return exclusion). Only the first case is actually safe; the
            # second must not default to SAFE, or a corner an obstacle sits
            # flush against becomes invisible to the very check meant to catch
            # it -- measured on hardware 2026-08-28, a 60cm-corridor run with
            # an enlarged centre wall that never turned.
            if _forward_path_has_rays(lidar_ranges, lidar_angles, self.path_half_width):
                return RiskLevel.CRITICAL
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
        threshold, minimum valid range, no-data sentinel, blind wedges) are wired
        to ``_sector_to_model``. Every clearance helper below goes through here,
        so a change to the mount geometry or the wedge angles lands in one place
        instead of four near-identical argument lists.

        Prefer this over the ``compute_*_clearance`` shorthands when the answer
        gates an action: the returned model carries ``measured`` and
        ``wedge_masked`` alongside the distance, and a bare distance cannot tell
        "clear" from "blind" (see ``SectorRanges.measured``).
        """
        return _sector_to_model(
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

    def _sector_to_model(
        self,
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
        """Instance accessor for the pure sector-model helper (see ``sectors._sector_to_model``)."""
        return _sector_to_model(
            lidar_ranges,
            lidar_angles,
            center_rad,
            half_fov_rad,
            filter_self_detection,
            self_detection_threshold_m,
            min_valid_range_m,
            no_data_range_m,
            blind_wedge_left_min_rad,
            blind_wedge_left_max_rad,
            blind_wedge_right_min_rad,
            blind_wedge_right_max_rad,
        )

    def rear_sector(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> SectorRanges:
        """The rear +/-``threat_half_fov`` sector, self-detection filtered.

        Filtered because a chassis/cable reflection directly behind the robot must
        not permanently read as "wall right there" and block every reverse escape
        for the rest of the run.

        Callers gating a reverse want this rather than ``compute_rear_clearance``:
        on this mount the occlusion wedges leave only a ~25 deg slot straight
        back, and if that slot goes (a different mount, a cable, a smaller scan)
        the distance alone still reads as open road. ``measured`` is what tells
        them apart.
        """
        return self.sector(lidar_ranges, lidar_angles, math.pi, filter_self_detection=True)

    def front_sector(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> SectorRanges:
        """The forward +/-``front_half_fov`` sector, with ``measured`` retained.

        The counterpart to ``rear_sector``, and it exists for the same reason:
        ``compute_forward_clearance`` collapses an unreadable sector into
        ``no_data_range_m``, which reads identically to open road, so a gate
        acting on that number alone fails OPEN. The rear has had this
        distinction since the reverse guard was found failing open; the front
        did not, and drove into the wall it could no longer see.

        Measured on hardware 2026-08-31 (run_20260831_205208): pressed against a
        wall and physically immobile, every ray in the forward cone fell below
        ``min_valid_range_m`` -- a flat surface centimetres away reflects too
        shallowly to return a signal -- so the sector reported ~10 m. The
        navigator resumed 0.24 m/s into the wall, and the ``stuck_forward``
        escape fired once and immediately stood down, because by this number
        the road ahead was clear. It never recovered.

        Callers deciding whether it is safe to DRIVE FORWARD want this and must
        check ``measured``; ``compute_forward_clearance`` remains the shorthand
        for display and for callers that only want a number.

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            The sector's ranges, with ``measured`` False when nothing in the
            cone was a valid reading.
        """
        return self.sector(lidar_ranges, lidar_angles, 0.0, self.front_half_fov_rad)

    def compute_forward_clearance(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> float:
        """Minimum clearance in the forward +/-30 deg sector (0 rad = forward).

        Was the sector's MEAN, not its minimum -- a real near-contact dead ahead
        widens the cone's grazing-incidence edges into no-returns (physically
        expected: a flat surface a few cm away reflects rays near its own edge too
        shallowly to return a signal at all), and the LIDAR callback substitutes
        those no-returns with ``LIDAR_MAX_RANGE`` before this ever sees them (see
        ``ros2_hardware_gateway._lidar_callback``). Averaging genuine ~0.08m
        readings together with several fabricated 12m ones reports several metres
        of open road during the single most blocked moment of a run -- confirmed
        against a real 2026-08-04 bag (``run_20260804_114500``, t=26.02s): the
        sector's true minimum was 0.078m dead ahead (matching flat-wall-at-close-
        range raycast geometry, ``d/cos(theta)`` across the cone) while the old
        mean reported 5.15m. Minimum matches the pattern
        ``compute_rear_clearance``/``compute_min_clearance`` already use, and is
        what a clearance number meant to gate speed should be: the worst case in
        the cone, not an average that a single no-return can swamp.

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

        Reports ``no_data_range_m`` when the rear sector saw nothing, which reads
        identically to open road -- a gate that acts on this number alone fails
        open. Use ``rear_sector`` and check ``measured`` when the answer
        authorises a reverse; this shorthand is for display and for callers that
        only want a number.
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

        ``compute_forward_clearance`` averages a narrow ±30° cone -- fine for
        normal driving, but blind to a lateral clip (e.g. a maneuver that swings
        the chassis sideways into an obstacle that was never in front of it).
        Used to gate maneuvers where the robot's path isn't a straight line, so a
        side contact is caught before it happens instead of only checking what's
        dead ahead.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return self.no_data_range_m

        sr = self.sector(lidar_ranges, lidar_angles, center_rad, half_fov_rad)
        return sr.min_range_m if sr.measured else self.no_data_range_m

    def parking_clearances(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> ParkingGate:
        """Forward and full-sweep clearances for the parking stop-check.

        Returns a :class:`ParkingLot`-adjacent :class:`ParkingGate` with:

        * ``forward_m`` -- min over the narrow forward cone
          (``compute_forward_clearance``), so the staging vector never drives
          head-on into a wall.
        * ``sweep_m`` -- min over the entire 360 deg sweep
          (``compute_min_clearance(half_fov_rad=pi)``), catching any sideways or
          slightly-rearward clip the narrow cone misses -- parking geometry can
          strike a wall or block edge from the side, not just in front.

        Both are computed here so the navigator makes one call instead of
        re-sweeping the same scan for each gate. The two thresholds the caller
        applies (``CONTACT_DIST`` for forward, ``CONTACT_DIST + WIDTH/2`` for the
        sweep) stay with the caller, since they belong to the parking maneuver,
        not to clearance measurement.
        """
        return ParkingGate(
            forward_m=self.compute_forward_clearance(lidar_ranges, lidar_angles),
            sweep_m=self.compute_min_clearance(lidar_ranges, lidar_angles, half_fov_rad=math.pi),
        )

    def detect_threat_direction(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...],
        lidar_angles: np.ndarray | tuple[float, ...] | None = None,
    ) -> ThreatDirection:
        """Direction of the closest obstacle: front, left, right, back, or none.

        Sectors are angular cones (+/-45 deg) about forward (0), left (+pi/2),
        right (-pi/2) and rear (+/-pi), so the result is correct regardless of the
        scan's index ordering.

        Args:
            lidar_ranges: Array of LIDAR measurements.
            lidar_angles: Per-ray bearings (radians). Synthesised if omitted.

        Returns:
            The nearest obstacle's :class:`ThreatDirection`.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return ThreatDirection.NONE

        def sector_min(center_rad: float, filter_self_detection: bool = False) -> float:
            sr = _sector_to_model(
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
        into whichever wall happens to be on that side half the time; picking the
        sign from the wider of the two side clearances swings away from the
        tighter wall instead.

        Neither side clearance is usable exactly when this matters most: a FRONT
        threat fires at every corner, where both side sectors legitimately see
        past the inner block into open track and read as no-return (or as a near-
        tie), not as a wall. A fixed fallback there is not a rare edge case, it
        is every corner of every lap, and always resolving it toward the same
        side is a standing bias that never shows up in a sim whose LIDAR sees the
        true walls cleanly enough to rarely tie at all -- no symmetry check
        catches it because it does not depend on which way around the loop the
        round travels; it depends only on which body side the fallback happens to
        prefer.

        The loop geometry itself resolves the *remaining* tie without guessing:
        going clockwise around the island keeps it on the robot's right for the
        entire lap, counterclockwise keeps it on the left, so the side away from
        the island is the structurally safer one to swing the nose toward when
        LIDAR gives no side any edge at all. ``direction`` is the inferred travel
        direction, the same source ``LapDetector`` and the planned path already
        trust; ``None`` (only possible in the sliver before inference settles, a
        few corridor widths into the round) falls back to the old fixed side
        rather than stall the maneuver.

        A side with no valid ray is not the same as a tie: it means nothing
        registered within sensor range on that side at all, which is itself the
        clearest possible "open" reading -- most sharply so pinned against a wall,
        where the jammed side reads a real, close, valid return and the free side
        legitimately has nothing to reflect off within range. Requiring both
        sides to have a valid ray before trusting the comparison (an earlier
        version of this method did) throws away exactly that reading and falls
        through to the direction-based guess instead, which reasons about the
        island and has nothing to say about a chassis pinned against the *outer*
        wall -- measured pinning the right side at 4.5 cm for the remainder of a
        run that never recovered. Substituting ``no_data_range_m`` for a missing
        side keeps that signal instead of discarding it; the direction fallback
        below now only fires when neither side has anything to say.
        """
        if lidar_ranges is not None:
            left = _sector_to_model(
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
            right = _sector_to_model(
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
                    # Swing left (negative steering while reversing) when the left
                    # is clearer; swing right (positive) when the right is clearer.
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
            # flips the yaw response relative to forward (see _k_turn_steer_sign's
            # docstring for the physics), so the sign has to flip with it. A fixed
            # negative sign here drove the nose further into the wall it was
            # already touching instead of away from it whenever this branch
            # reversed -- measured pinning a side at 4.5cm clearance for the rest
            # of a run that never recovered.
            already_touching = self._side_clearance(
                math.pi / 2, lidar_ranges, lidar_angles
            ) < self.contact_dist or self._forward_touching(lidar_ranges, lidar_angles)
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
            # creeping forward, negative once already touching and reversing, for
            # the same reverse-flips-yaw reason as the LEFT branch above.
            already_touching = self._side_clearance(
                -math.pi / 2, lidar_ranges, lidar_angles
            ) < self.contact_dist or self._forward_touching(lidar_ranges, lidar_angles)
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
        ``contact_dist`` -- e.g. a starting position placed right at a narrow
        corridor's edge, with zero margin by construction), creeping forward while
        steering away deepens the overlap faster than the chassis can rotate out
        of it, in the same tick. Reversing instead opens real separation before
        any forward motion resumes.
        """
        if lidar_ranges is None:
            # Matches _k_turn_steer_sign's fallback: no lidar data means no basis
            # to claim the chassis is already touching a wall, so treat it as
            # clear rather than crashing _sector_to_model on None. Use the
            # configured no-data sentinel (lidar_sectors.NO_DATA_RANGE_M), not a
            # second hardcoded 10.0.
            return self.no_data_range_m
        sr = _sector_to_model(
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

    def _forward_touching(
        self,
        lidar_ranges: np.ndarray | tuple[float, ...] | None,
        lidar_angles: np.ndarray | tuple[float, ...] | None,
    ) -> bool:
        """Whether the chassis is already at ``contact_dist`` or closer straight ahead.

        ``_side_clearance`` alone cannot see this: a robot pinned at an angle
        into a corner can be touching in front while its side sector still
        reads clear, so a SIDE_CORRECTION whose "already touching" check only
        looks sideways keeps creeping forward into the wall it is already
        touching instead of reversing -- measured on hardware pinning a robot
        nose-first for up to 23s across three runs that never recovered
        (2026-08-28). Reuses ``assess_risk``'s own forward-path geometry (the
        chassis-width lane, not ``detect_threat_direction``'s narrower angular
        cone) so this agrees with whatever risk level triggered the escape in
        the first place.
        """
        if lidar_ranges is None or len(lidar_ranges) == 0:
            return False
        path = self._forward_path_ranges(lidar_ranges, lidar_angles)
        if path.size == 0:
            return False
        return bumper_gap_ahead(float(np.min(path))) < self.contact_dist

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
        """Instance-less sector-range query (see ``sectors.sector_ranges``).

        Kept as a staticmethod so external, instance-less callers (e.g.
        ``telemetry_bridge_node``'s OLED summary) can call
        ``CollisionAvoidanceController.sector_ranges(...)`` directly and fall
        back to tuning-sourced keyword defaults, exactly as before the split.
        """
        return sector_ranges(
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
            apply_blind_wedge_mask,
        )
