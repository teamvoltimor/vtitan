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
    robust_min_range,
    _sector_to_model,
    chassis_exit_range_m,
    forward_path_nearest_ray,
    sector_ranges,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.config.navigation_tuning.escape import EscapeManeuverParams
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
        risk_ray_window: Adjacent lane rays that must corroborate a short
            return before it counts -- see sectors.robust_min_range
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
        risk_ray_window: int,
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
        rear_self_detection_from_chassis: bool = True,
        escape_side_follows_committed_sign: bool = False,
        side_correction_follows_committed_sign: bool = False,
        escape_side_override_min_clearance_m: float = 0.12,
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
            ahead_of_bumper: Measure the forward driving lane from the front
                bumper face rather than from the LIDAR, so clearance means
                distance to contact rather than distance to the sensor
            rear_self_detection_from_chassis: Filter the rear sector's
                self-detection by chassis geometry at each bearing instead of by
                the single ``self_detection_threshold_m`` scalar, which sits far
                inside the body. See
                ``LidarSectorParams.rear_self_detection_from_chassis``.
        """
        self.contact_dist = contact_dist
        self.risk_ray_window = risk_ray_window
        self.slow_dist = slow_dist
        self.fast_dist = fast_dist
        self.escape_rev_speed = escape_rev_speed
        self.escape_steer_scale = escape_steer_scale
        self.escape_side_follows_committed_sign = escape_side_follows_committed_sign
        self.side_correction_follows_committed_sign = side_correction_follows_committed_sign
        self.escape_side_override_min_clearance_m = escape_side_override_min_clearance_m
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
        self.rear_self_detection_from_chassis = rear_self_detection_from_chassis
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
        cls,
        tuning: NavigationTuning,
        clearance: ClearanceZones | None = None,
        escape: EscapeManeuverParams | None = None,
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
            escape: Escape parameters in place of ``tuning.escape``, for the
                same reason and by the same rule. Without this the Obstacles
                overrides on this group are INERT here: the navigator resolves
                them into its own ``_escape`` and passes the raw tuning in, so
                reading ``tuning.escape`` directly reads the shared field and a
                flag set only under ``obstacles_`` never reaches the chassis.
                That is the exact failure OBSTACLES_CONTACT_DIST had, which is
                why ``clearance`` above exists at all.

        Returns:
            CollisionAvoidanceController with values from tuning.
        """
        clearance = clearance if clearance is not None else tuning.clearance
        escape = escape if escape is not None else tuning.escape
        # Escape durations are stored in seconds and converted here: a frame
        # count would mean a different duration if CONTROL_HZ ever moved.
        hz = tuning.control.control_hz
        return cls(
            contact_dist=clearance.contact_dist,
            risk_ray_window=clearance.risk_ray_window,
            slow_dist=clearance.slow_dist,
            fast_dist=clearance.fast_dist,
            escape_rev_speed=escape.rev_speed,
            escape_steer_scale=escape.rev_steer_norm(),
            escape_side_follows_committed_sign=escape.escape_side_follows_committed_sign,
            side_correction_follows_committed_sign=escape.side_correction_follows_committed_sign,
            escape_side_override_min_clearance_m=escape.escape_side_override_min_clearance_m,
            stuck_threshold=escape.stuck_move_threshold,
            path_margin=clearance.path_margin,
            k_turn_min_frames=escape.k_turn_min_frames(hz),
            k_turn_max_frames=escape.k_turn_max_frames(hz),
            side_correction_steer=escape.side_correction_steer_norm(),
            side_correction_speed=escape.side_correction_speed,
            side_correction_frames=escape.side_correction_frames(hz),
            front_half_fov_deg=tuning.lidar_sectors.front_half_fov_deg,
            threat_half_fov_deg=tuning.lidar_sectors.threat_half_fov_deg,
            self_detection_threshold_m=tuning.lidar_sectors.self_detection_threshold_m,
            rear_self_detection_from_chassis=tuning.lidar_sectors.rear_self_detection_from_chassis,
            min_valid_range_m=tuning.lidar_sectors.min_valid_range_m,
            threat_no_detection_range_m=tuning.lidar_sectors.threat_no_detection_range_m,
            no_data_range_m=tuning.lidar_sectors.no_data_range_m,
            blind_wedge_left_min_deg=tuning.lidar_sectors.blind_wedge_left_min_deg,
            blind_wedge_left_max_deg=tuning.lidar_sectors.blind_wedge_left_max_deg,
            blind_wedge_right_min_deg=tuning.lidar_sectors.blind_wedge_right_min_deg,
            blind_wedge_right_max_deg=tuning.lidar_sectors.blind_wedge_right_max_deg,
            ahead_of_bumper=clearance.forward_path_ahead_of_bumper,
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
            lidar_ranges,
            lidar_angles,
            self.path_half_width,
            self.min_valid_range_m,
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
            lidar_ranges,
            lidar_angles,
            self.path_half_width,
            self.min_valid_range_m,
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
            # the forward lane at all (a near-empty scan, safe by construction),
            # or the lane DID have rays but every one was a no-return, the
            # hardware gateway's fabricated LIDAR_MAX_RANGE substitute for a
            # real grazing-incidence echo. That is the signature of something
            # very close spanning the WHOLE cone, not of open road (see
            # _forward_path_ranges's no-return exclusion). Only the first case
            # is actually safe; the second must not default to SAFE.
            # adr:0056-raw-and-masked-scan
            if _forward_path_has_rays(lidar_ranges, lidar_angles, self.path_half_width):
                return RiskLevel.CRITICAL
            return RiskLevel.SAFE

        # Judged as a gap from the BUMPER, not as a raw sensor range: the
        # thresholds are statements about how close the chassis may come to
        # something, and the sensor is 12.2 cm ahead of the chassis centre.
        # Corroborated by adjacent rays rather than the bare minimum: see
        # sectors.robust_min_range for why a noisy sweep makes the raw
        # minimum a phantom obstacle.
        gap = bumper_gap_ahead(robust_min_range(path, self.risk_ray_window))

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
        on this mount the occlusion wedges leave only a narrow slot straight
        back, and if that slot goes (a different mount, a cable, a smaller scan)
        the distance alone still reads as open road. ``measured`` is what tells
        them apart. See ``adr:0056-raw-and-masked-scan`` for the wedge geometry.
        """
        # Self-detection gated by CHASSIS GEOMETRY rather than the global scalar,
        # and kept in the SENSOR frame like every other sector so
        # `most_constrained_side` still compares like with like.
        #
        # The scalar (0.08 m) sits far inside the body: the chassis rear face is
        # 0.2722 m behind the sensor and this sector's boundary runs to 0.137 m
        # at its edges, so the robot's own structure survives the filter.
        # See ``adr:0056-raw-and-masked-scan``.
        #
        # Still the SENSOR frame, so callers judging a reverse keep converting
        # with `bumper_gap_behind` -- an obstacle touching the rear bumper reads
        # 0.2722 m here, and that conversion is what makes CONTACT_DIST reachable.
        if not self.rear_self_detection_from_chassis:
            return self.sector(lidar_ranges, lidar_angles, math.pi, filter_self_detection=True)
        ranges = np.asarray(lidar_ranges, dtype=float)
        if ranges.size == 0:
            return self._sector_to_model(lidar_ranges, lidar_angles, math.pi, self.threat_half_fov_rad)
        angles = (
            np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
            if lidar_angles is None
            else np.asarray(lidar_angles, dtype=float)
        )
        inside_body = np.isfinite(ranges) & (ranges <= chassis_exit_range_m(angles))
        kept = np.where(inside_body, float("inf"), ranges)
        return self._sector_to_model(kept, angles, math.pi, self.threat_half_fov_rad)

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

        A flat surface centimetres away reflects too shallowly to return a
        signal, so every ray in the forward cone can fall below
        ``min_valid_range_m`` at once; the sector then reports a range that reads
        as open road while the chassis is in fact immobile against a wall. See
        ``adr:0056-raw-and-masked-scan``.

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

        Minimum, not the sector's mean. A real near-contact dead ahead widens
        the cone's grazing-incidence edges into no-returns (a flat surface a few
        cm away reflects rays near its own edge too shallowly to return a signal),
        and the LIDAR callback substitutes those no-returns with
        ``LIDAR_MAX_RANGE`` before this ever sees them (see
        ``ros2_hardware_gateway._lidar_callback``). The mean is therefore swamped
        by fabricated max-range readings during the single most blocked moment of
        a run, while the minimum is the worst case in the cone. Minimum matches
        the pattern ``compute_rear_clearance``/``compute_min_clearance`` already
        use. See ``adr:0056-raw-and-masked-scan``.

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
        preferred_sign: float | None = None,
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
        clearest possible "open" reading, most sharply so pinned against a wall,
        where the jammed side reads a real, close, valid return and the free side
        legitimately has nothing to reflect off within range. Substituting
        ``no_data_range_m`` for a missing side keeps that signal instead of
        discarding it; the direction fallback below now only fires when neither
        side has anything to say.

        That substitution also leaves the direction fallback effectively dead:
        once a missing side gets a number, exact float equality of
        ``left_clear`` and ``right_clear`` rarely fires, so the fallback runs
        almost only when neither side has a valid ray. The island reasoning is
        correct design intent but cannot explain observed behaviour -- the
        comparison at a corner answers a different question from the router's
        pass-side choice. See
        ``adr:0050-escape-steering-degrees-and-committed-side``.
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
                # The router's side outranks the clearance comparison when the
                # side it wants is not physically shut, because the comparison
                # is answering a different question. The floor is ABSOLUTE
                # rather than a margin between the two sides, because a relative
                # band wide enough to catch the failures also overrules episodes
                # that choose correctly. See
                # ``adr:0050-escape-steering-degrees-and-committed-side``.
                if preferred_sign is not None and self.escape_side_follows_committed_sign:
                    wanted_clear = left_clear if preferred_sign < 0 else right_clear
                    if wanted_clear >= self.escape_side_override_min_clearance_m:
                        return preferred_sign
                    # The wanted side is physically shut. Neither committing to
                    # it (which would push the pillar over) nor taking the
                    # OTHER side (a wrong-side pass, which ENDS THE ROUND
                    # rather than costing points) is right, so do neither:
                    # reverse straight and commit to no side at all.
                    #
                    # 0.0 is not a sentinel. compute_escape_maneuver multiplies
                    # it by escape_steer_scale, so it lands as steering=0.0 --
                    # the same straight reverse the non-CRITICAL branch there
                    # already emits. It buys the room the manoeuvre exists for
                    # and leaves the pass side to the planner on re-approach,
                    # which is the only actor that knows which side is correct.
                    #
                    # `sign_router.retrace_escape` is NOT this: it only re-aims
                    # steering inside an already-decided reverse leg. See
                    # ``adr:0055-escape-maneuver-selection``.
                    return 0.0
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
        preferred_sign: float | None = None,
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
            preferred_sign: Steering sign the sign router wants, negative for
                left, derived from the side its committed pillar must be passed
                on. Honoured only when ``escape_side_follows_committed_sign`` is
                set AND that side has ``escape_side_override_min_clearance_m``
                of room. ``None`` whenever the rule is unavailable, which is
                every tick of the Open Challenge.

        Returns:
            EscapeManeuver command or None if no maneuver needed
        """
        if risk == RiskLevel.SAFE:
            return None

        if threat_dir == ThreatDirection.FRONT:
            # K-turn: reverse while steering hard for CRITICAL risk; a shorter,
            # straight reverse to open clearance for the milder OBSTACLE risk.
            steer_sign = self._k_turn_steer_sign(lidar_ranges, lidar_angles, direction, preferred_sign)
            return EscapeManeuver(
                maneuver_type=ManeuverType.K_TURN,
                steering=self.escape_steer_scale * steer_sign if risk == RiskLevel.CRITICAL else 0.0,
                speed=self.escape_rev_speed,
                duration_frames=self.k_turn_max_frames if risk == RiskLevel.CRITICAL else self.k_turn_min_frames,
                priority=2 if risk == RiskLevel.CRITICAL else 1,
            )

        if threat_dir == ThreatDirection.LEFT:
            # Threat on the left - steer right (away). Positive steering is left
            # (CCW) throughout the stack for FORWARD travel, so the creeping
            # (non-touching) correction is negative.
            #
            # Already touching switches speed to reverse, and Ackermann reverse
            # flips the yaw response relative to forward (see _k_turn_steer_sign's
            # docstring for the physics), so the sign has to flip with it. A fixed
            # negative sign here would drive the nose further into the wall it is
            # already touching instead of away from it whenever this branch
            # reverses.
            already_touching = self._side_clearance(
                math.pi / 2, lidar_ranges, lidar_angles
            ) < self.contact_dist or self._forward_touching(lidar_ranges, lidar_angles)
            steer_sign, refused = self._side_correction_steer_sign(
                1.0 if already_touching else -1.0,
                threat_is_left=True,
                preferred_sign=preferred_sign,
            )
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=steer_sign * self.side_correction_steer,
                speed=self.escape_rev_speed if already_touching or refused else self.side_correction_speed,
                duration_frames=(
                    self.k_turn_min_frames if already_touching or refused else self.side_correction_frames
                ),
                priority=1,
            )

        if threat_dir == ThreatDirection.RIGHT:
            # Threat on the right - steer left (away): positive steering while
            # creeping forward, negative once already touching and reversing, for
            # the same reverse-flips-yaw reason as the LEFT branch above.
            already_touching = self._side_clearance(
                -math.pi / 2, lidar_ranges, lidar_angles
            ) < self.contact_dist or self._forward_touching(lidar_ranges, lidar_angles)
            steer_sign, refused = self._side_correction_steer_sign(
                -1.0 if already_touching else 1.0,
                threat_is_left=False,
                preferred_sign=preferred_sign,
            )
            return EscapeManeuver(
                maneuver_type=ManeuverType.SIDE_CORRECTION,
                steering=steer_sign * self.side_correction_steer,
                speed=self.escape_rev_speed if already_touching or refused else self.side_correction_speed,
                duration_frames=(
                    self.k_turn_min_frames if already_touching or refused else self.side_correction_frames
                ),
                priority=1,
            )

        return None

    def _side_correction_steer_sign(
        self,
        away_sign: float,
        threat_is_left: bool,
        preferred_sign: float | None,
    ) -> tuple[float, bool]:
        """Let the router's committed pass side outrank "steer away from the threat".

        The FRONT branch consults ``preferred_sign`` when
        ``escape_side_follows_committed_sign`` is set, and the two SIDE branches
        must as well: picking from the threat side and ``already_touching`` alone
        can steer against the side the router needs, and a wrong-side pass ends
        the round. See ``adr:0050-escape-steering-degrees-and-committed-side``.

        REFUSES, never redirects. When the router's side and the threat are on
        OPPOSITE flanks there is no conflict -- steering away from the threat
        already goes where the router wants -- so the manoeuvre is untouched.
        When they are on the SAME flank, this reverses straight instead of
        shoving the chassis to the wrong side of the pillar.

        The second return value is that refusal, and the caller needs it: a
        refusal must also switch the tick to the reverse speed, not merely zero
        the steering. The caller applies this; zeroing steering alone would
        creep the chassis forward at the threat with no steer-away reflex.

        It does not steer toward the wanted side: the override only refuses to
        push AWAY from the side the router needs. Steering toward it would drive
        into a flank that merely clears ``escape_side_override_min_clearance_m``,
        which is not room to rotate the chassis. Refusing keeps the benefit that
        matters -- the escape stops pushing to the wrong side -- and leaves the
        pass to the planner on re-approach, the only actor that knows which side
        is correct. That is the K-turn's own answer to the same conflict.

        ``preferred_sign`` is negative for LEFT (see
        ``compute_escape_maneuver``'s docstring). ``threat_is_left`` is passed
        by the branch rather than re-derived from ``away_sign``, whose meaning
        inverts between the creeping (forward) and touching (reverse) cases --
        deriving it here got that backwards once already.
        """
        if preferred_sign is None or not self.side_correction_follows_committed_sign:
            return away_sign, False
        # NEVER steer toward the side the threat is on. The override never
        # pushes toward a threat; it only refuses to push AWAY from the side the
        # router needs, because ``escape_side_override_min_clearance_m`` is not
        # room to rotate the chassis through. That is the K-turn's own answer to
        # the same conflict: commit to neither side, reverse straight, and leave
        # the pass to the planner on re-approach, which is the only actor that
        # knows which side is correct. See
        # ``adr:0050-escape-steering-degrees-and-committed-side``.
        if (preferred_sign < 0) is not threat_is_left:
            # Steering away from the threat already goes where the router
            # wants, so there is nothing to arbitrate.
            return away_sign, False
        return 0.0, True

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
            # configured no-data sentinel (lidar_sectors.no_data_range_m), not a
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
