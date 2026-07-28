"""Navigation tuning parameters for WRO 2026 track following.

This module centralizes all robot navigation tuning constants to enable:
- Runtime configuration without code edits
- Multiple tuning profiles for A/B testing
- Consistent parameter sharing between robot and simulation
- Easy parameter experimentation during competition

All parameters have been extracted from robot/src/navigation/ modules
and consolidated into a single source of truth.

Example usage:
    from shared.config.navigation_tuning import NavigationTuning

    # Use defaults
    tuning = NavigationTuning()
    print(tuning.pursuit.LOOKAHEAD_SHORT)  # 0.20

    # Load custom profile from YAML
    tuning = NavigationTuning.load_from_yaml("tuning_profiles/aggressive.yaml")
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, ClassVar

DEFAULT_CONFIG_DIR: Path = Path(__file__).resolve().parents[3] / "config" / "navigation"
"""platform/shared/config/navigation -- the checked-in per-group TOML tree.

Resolved relative to this module's own location (platform/shared/src/shared/
config/navigation_tuning.py) rather than the caller's, since this package is
the one that actually knows where its own config lives -- callers (e.g.
CoreNavigator) shouldn't have to know or assume the two are siblings under
the same platform/ root."""


@dataclass(frozen=True)
class ClearanceZones:
    """LIDAR clearance thresholds for speed control.

    These distances define zones around the robot where speed is controlled
    based on obstacle proximity. Values are in meters.

    Attributes:
        CONTACT_DIST: Robot creeps forward (< 0.10m) - immediate danger
        SLOW_DIST: Robot enters slow zone (0.10-0.25m)
        MEDIUM_DIST: Robot enters medium speed zone (0.25-0.50m)
        FAST_DIST: Robot can go full speed (> 0.50m)
        PATH_MARGIN: Extra clearance beyond the chassis half-width still
            counted as "in the robot's forward path" for risk assessment (m)
    """

    CONTACT_DIST: float = 0.10  # Creep forward zone
    SLOW_DIST: float = 0.25  # Reduced speed
    MEDIUM_DIST: float = 0.50  # Normal speed
    FAST_DIST: float = 1.00  # Full speed capability
    PATH_MARGIN: float = 0.10  # Forward-path half-width margin beyond chassis


@dataclass(frozen=True)
class HeadingErrorZones:
    """Heading error thresholds for speed modulation.

    These thresholds define how much heading error reduces speed. Radians.

    Attributes:
        CRAWL: Severe misalignment (> 1.0 rad) - crawl speed
        SLOW: Large error (0.7-1.0 rad) - slow speed
        MEDIUM: Moderate error (0.4-0.7 rad) - medium speed
        NORMAL: Small error (< 0.4 rad) - normal speed
    """

    CRAWL: float = 1.0  # ~57° - worst case
    SLOW: float = 0.7  # ~40°
    MEDIUM: float = 0.4  # ~23°
    NORMAL: float = 0.2  # ~11°


@dataclass(frozen=True)
class PurePursuitParams:
    """Pure pursuit controller parameters for waypoint following.

    Implements lookahead-based steering to follow waypoints with
    crosstrack error minimization.

    Attributes:
        LOOKAHEAD_SHORT: Lookahead distance for sharp corners (m)
        LOOKAHEAD_LONG: Lookahead distance for straights (m)
        LOOKAHEAD_TRANSITION: Crosstrack error threshold to switch modes (m)
        STEER_KP: Proportional gain for steering P-controller
        MAX_STEERING_RATE: Maximum steering command rate (rad/s)
    """

    LOOKAHEAD_SHORT: float = 0.20  # Close to corner
    LOOKAHEAD_LONG: float = 0.40  # Normal straight
    LOOKAHEAD_TRANSITION: float = 0.30  # Crosstrack threshold
    STEER_KP: float = 1.2  # Steering P-gain
    MAX_STEERING_RATE: float = 2.0  # rad/s


@dataclass(frozen=True)
class SpeedControlParams:
    """Speed control parameters for different zones.

    Maps clearance zones and heading errors to commanded motor speeds.
    Values are normalized to [-1.0, 1.0] motor command range.

    Attributes:
        MIN_SPEED: Minimum forward speed to overcome friction
        MAX_SPEED: Maximum safe forward speed
        CREEP_SPEED: Speed in contact zone
        SLOW_SPEED: Speed in slow zone
        MEDIUM_SPEED: Speed in medium zone
        FAST_SPEED: Speed in fast/open zone
    """

    MIN_SPEED: float = 0.05  # Minimum to move
    MAX_SPEED: float = 0.50  # Maximum safe speed
    CREEP_SPEED: float = 0.05  # Contact zone
    SLOW_SPEED: float = 0.15  # Near obstacles
    MEDIUM_SPEED: float = 0.30  # Moderate clearance
    FAST_SPEED: float = 0.50  # Open track


@dataclass(frozen=True)
class EscapeManeuverParams:
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Attributes:
        REV_SPEED: Reverse speed during escapes
        REV_STEERING_SCALE: Steering aggressiveness while reversing
        K_TURN_MIN_FRAMES: Minimum frames for K-turn maneuver (OBSTACLE risk)
        K_TURN_MAX_FRAMES: Maximum frames for K-turn maneuver (CRITICAL risk)
        SLALOM_REVERSE_FRAMES: Frames spent reversing during slalom
        SLALOM_FORWARD_FRAMES: Frames spent forward turning during slalom
        STUCK_MOVE_THRESHOLD: Distance threshold to detect stuck (m)
        STUCK_TIMEOUT_FRAMES: Frames without movement before stuck (20Hz)
        SIDE_CORRECTION_STEER: Steering magnitude for a side-threat correction
        SIDE_CORRECTION_SPEED: Forward speed during a side-threat correction
        SIDE_CORRECTION_FRAMES: Duration of a side-threat correction (frames)
        ESCALATE_AFTER_ATTEMPTS: Consecutive escapes before escalating (longer
            duration, opposite side) instead of repeating an identical pulse
        MAX_ESCAPE_FRAMES: Hard cap on any single escalated escape duration
    """

    REV_SPEED: float = -0.20  # Reverse speed
    REV_STEERING_SCALE: float = 0.8  # Steering while reversing
    K_TURN_MIN_FRAMES: int = 6  # Minimum K-turn duration
    K_TURN_MAX_FRAMES: int = 12  # Maximum K-turn duration
    SLALOM_REVERSE_FRAMES: int = 8  # Reverse duration in slalom
    SLALOM_FORWARD_FRAMES: int = 10  # Forward turn duration
    STUCK_MOVE_THRESHOLD: float = 0.03  # 3cm movement threshold
    STUCK_TIMEOUT_FRAMES: int = 40  # ~2 seconds at 20Hz
    SIDE_CORRECTION_STEER: float = 0.3
    SIDE_CORRECTION_SPEED: float = 0.1
    SIDE_CORRECTION_FRAMES: int = 4
    ESCALATE_AFTER_ATTEMPTS: int = 3
    MAX_ESCAPE_FRAMES: int = 20


@dataclass(frozen=True)
class WaypointParams:
    """Waypoint generation geometry parameters.

    Attributes:
        ARC_RADIUS: Corner arc radius (m). Must exceed the Ackermann minimum
            turning radius (~0.329 m, from the measured WHEELBASE=0.19/
            MAX_STEERING_ANGLE=0.5236) — enforced by a fail-fast width
            assertion in ``calculate_waypoints``.
        DEDUPE_DISTANCE_M: Distance below which consecutive generated
            waypoints are treated as duplicates and merged.
        OUTER_WALL_BIAS: Bias (m) added to corridor centerline waypoints
            toward the outer wall, to compensate for chassis width.
        NUM_INTERMEDIATE_ARC_POINTS: Number of intermediate sample points
            per corner arc.
        STRAIGHT_WAYPOINT_COUNT: Number of evenly spaced waypoints generated
            along a straight corridor segment.
        MAIN_LOOP_REACHED_DISTANCE_M: Distance within which CoreNavigator's
            own main loop counts a waypoint as reached. Deliberately a
            different (coarser) value than CONTROLLER_REACHED_DISTANCE_M —
            the two serve different layers, not a single duplicated concept.
        CONTROLLER_REACHED_DISTANCE_M: Distance within which
            WaypointController's own internal pure-pursuit logic counts a
            waypoint as reached.
    """

    ARC_RADIUS: float = 0.45
    DEDUPE_DISTANCE_M: float = 0.001
    OUTER_WALL_BIAS: float = 0.05
    NUM_INTERMEDIATE_ARC_POINTS: int = 3
    STRAIGHT_WAYPOINT_COUNT: int = 8
    MAIN_LOOP_REACHED_DISTANCE_M: float = 0.20
    CONTROLLER_REACHED_DISTANCE_M: float = 0.01


@dataclass(frozen=True)
class SensorHealthParams:
    """Sensor dropout / staleness detection for the hardware gateway.

    Attributes:
        STALE_TIMEOUT_SEC: A cached sensor reading older than this is treated as
            a dropout — the gateway reports it as unavailable so the navigator
            degrades safely instead of acting on frozen data. Derived as 5x the
            LIDAR scan period (RobotSpecs.LIDAR_UPDATE_RATE), the slowest sensor
            feed the control loop depends on.
    """

    STALE_TIMEOUT_SEC: float = 0.5


@dataclass(frozen=True)
class LidarSectorParams:
    """LIDAR angular-sector definitions shared by collision avoidance and the OLED.

    Attributes:
        FRONT_HALF_FOV_DEG: Half-width (deg) of the forward clearance cone,
            used by CollisionAvoidanceController.compute_forward_clearance.
        THREAT_HALF_FOV_DEG: Half-width (deg) of the threat-detection sectors
            (front/left/right/back), used by detect_threat_direction and
            related methods -- a narrower, min-based cone for "is something
            about to hit me," distinct from the wider, mean-based forward
            clearance cone above.
        SELF_DETECTION_THRESHOLD_M: Rays no farther than this are discarded
            as chassis/cable self-reflection when a sector filters for it.
        MIN_VALID_RANGE_M: LIDAR ranges at or below this are treated as
            invalid (no-return) readings.
    """

    FRONT_HALF_FOV_DEG: float = 30.0
    THREAT_HALF_FOV_DEG: float = 45.0
    SELF_DETECTION_THRESHOLD_M: float = 0.08
    MIN_VALID_RANGE_M: float = 0.01


@dataclass(frozen=True)
class SignRouterParams:
    """Traffic-sign avoidance routing parameters.

    Attributes:
        SIGN_CLEARANCE_MARGIN_M: Extra safety margin (m) added to a sign's
            lateral avoidance offset, beyond chassis and sign half-widths.
        DEFORM_DEPTH_BUFFER_M: Depth-axis slack (m) beyond the inner-square
            span for a waypoint to still count as "in corridor" for a sign
            deformation.
        WALL_CLEARANCE_MARGIN_M: Margin (m) beyond the chassis half-diagonal
            that a deformed waypoint must still stay clear of a wall by.
        ACTIVATION_DIST_M: Distance (m) at which sign-avoidance deformation
            activates for a nearby sign.
        PASSED_DIST_M: Distance (m) beyond which a sign is marked "passed"
            and its deformation taper reaches zero.
        DETECTION_MATCH_DIST_M: Max distance (m) to associate a camera
            detection with an expected sign.
        MIN_CONFIDENCE: Minimum detection confidence to accept a camera
            color update for a sign.
        SETTLE_TICKS: Ticks after lap start before sign engage/pass
            bookkeeping activates (~7.5s @ 20Hz by default).
    """

    SIGN_CLEARANCE_MARGIN_M: float = 0.075
    DEFORM_DEPTH_BUFFER_M: float = 0.3
    WALL_CLEARANCE_MARGIN_M: float = 0.04
    ACTIVATION_DIST_M: float = 0.80
    PASSED_DIST_M: float = 1.20
    DETECTION_MATCH_DIST_M: float = 0.30
    MIN_CONFIDENCE: float = 0.25
    SETTLE_TICKS: int = 150


@dataclass(frozen=True)
class SignDiscoveryParams:
    """Blind sign-discovery (ObservedSignMap) parameters.

    Attributes:
        MIN_RELIABLE_BBOX_HEIGHT_PX: Minimum detection bbox height (px) for
            a reliable pinhole distance estimate.
        MAX_INGEST_RANGE_M: Max distance (m) to accept a sign observation
            for discovery at all.
        ASSOCIATION_DIST_M: Max distance (m) between two observations to be
            considered the same sign.
        MIN_HITS: Number of confirming observations before a discovered
            sign is published.
    """

    MIN_RELIABLE_BBOX_HEIGHT_PX: int = 5
    MAX_INGEST_RANGE_M: float = 2.0
    ASSOCIATION_DIST_M: float = 0.25
    MIN_HITS: int = 3


@dataclass(frozen=True)
class ParkingParams:
    """Parallel-parking maneuver parameters.

    Attributes:
        PARALLEL_TOLERANCE_M: WRO rule max allowed wheel-to-wall distance
            difference (m) for "parallel" parking.
        POS_REACH_DIST_M: Distance (m) threshold for "reached staging
            position."
        DEFAULT_MAX_FRAMES: Max control ticks before the parking maneuver
            gives up (20s @ 20Hz by default).
        SATURATED_STEER_THRESHOLD: Normalized steering magnitude counted as
            "at physical lock."
        SATURATION_STUCK_TICKS: Consecutive ticks of saturated steering
            before triggering a reverse-reorient.
        SPEED: Constant driving speed (m/s) during the parking maneuver.
        MIN_LOOKAHEAD_DIST_M: Floor distance (m) to avoid near-zero-distance
            curvature blow-up in pure pursuit.
        WALL_STANDOFF_M: Closest the chassis footprint may approach the
            field wall backing the parking lot.
        MARKER_STANDOFF_M: Closest the chassis footprint may approach a
            parking-bay marker fin.
    """

    PARALLEL_TOLERANCE_M: float = 0.02
    POS_REACH_DIST_M: float = 0.04
    DEFAULT_MAX_FRAMES: int = 400
    SATURATED_STEER_THRESHOLD: float = 0.999
    SATURATION_STUCK_TICKS: int = 20
    SPEED: float = 0.12
    MIN_LOOKAHEAD_DIST_M: float = 0.02
    WALL_STANDOFF_M: float = 0.05
    MARKER_STANDOFF_M: float = 0.01


@dataclass(frozen=True)
class LocalizationParams:
    """LIDAR-based pose search (LidarLocalizer) parameters.

    Attributes:
        SEARCH_RADIUS_M: Half-width (m) of the initial pose search window.
        PASSES: Number of coarse-to-fine grid-search passes.
        GRID_POINTS: Candidates per axis per search pass.
        RESIDUAL_CLIP_M: Per-ray residual clipping distance (m) for the cost
            function (outlier rejection).
    """

    SEARCH_RADIUS_M: float = 0.15
    PASSES: int = 4
    GRID_POINTS: int = 5
    RESIDUAL_CLIP_M: float = 0.25


@dataclass(frozen=True)
class NavigationTuning:
    """Complete navigation tuning configuration.

    Aggregates all tuning parameters into a single frozen dataclass
    for immutability and type safety.

    Can be instantiated with defaults or loaded from YAML files
    to support multiple tuning profiles.

    Example:
        # Use hardcoded defaults
        tuning = NavigationTuning()

        # Load from YAML for custom tuning
        tuning = NavigationTuning.load_from_yaml("aggressive.yaml")

        # Access parameters
        print(tuning.pursuit.LOOKAHEAD_SHORT)
        print(tuning.clearance.SLOW_DIST)
    """

    clearance: ClearanceZones = ClearanceZones()
    heading: HeadingErrorZones = HeadingErrorZones()
    pursuit: PurePursuitParams = PurePursuitParams()
    speed: SpeedControlParams = SpeedControlParams()
    escape: EscapeManeuverParams = EscapeManeuverParams()
    sensor: SensorHealthParams = SensorHealthParams()
    waypoints: WaypointParams = WaypointParams()
    lidar_sectors: LidarSectorParams = LidarSectorParams()
    sign_router: SignRouterParams = SignRouterParams()
    sign_discovery: SignDiscoveryParams = SignDiscoveryParams()
    parking: ParkingParams = ParkingParams()
    localization: LocalizationParams = LocalizationParams()

    # (group key, dataclass) pairs — the single source of truth for which
    # sections load_from_yaml/load_from_json/to_dict handle, so adding a new
    # tuning group never requires touching more than this tuple.
    _GROUPS: ClassVar[tuple[tuple[str, type], ...]] = (
        ("clearance", ClearanceZones),
        ("heading", HeadingErrorZones),
        ("pursuit", PurePursuitParams),
        ("speed", SpeedControlParams),
        ("escape", EscapeManeuverParams),
        ("sensor", SensorHealthParams),
        ("waypoints", WaypointParams),
        ("lidar_sectors", LidarSectorParams),
        ("sign_router", SignRouterParams),
        ("sign_discovery", SignDiscoveryParams),
        ("parking", ParkingParams),
        ("localization", LocalizationParams),
    )

    # No ``for_obstacles()`` profile. One existed (lookahead 0.12/0.24 +
    # FAST_SPEED 0.30) and was removed after re-measurement against the
    # corrected four-wheel-steer kinematics (8eb3c38) and the closed drive loop
    # (668e40a) showed both halves of it were inert:
    #
    # * The speed cap cannot do anything. ``AckermannKinematics`` clamps to the
    #   measured 0.156 m/s drivetrain ceiling, so FAST_SPEED 0.30 and 0.50 both
    #   saturate to the same 0.156 m/s. The profile's own justification — that a
    #   lower top speed buys steering travel per metre — never applied.
    # * The lookahead change does not help. Swept over the 16 obstacles
    #   fixtures, collisions are 16/16 at every value from 0.10 to 0.40. It is
    #   not inert — 0.12/0.24 cuts cross-track error from p90 12.9 cm to
    #   5.1 cm — but that is a path-quality result, not the sign-avoidance one
    #   the profile claimed, and buying that accuracy changes no outcome.
    #
    # See ``platform/robot/docs/sign-avoidance-investigation.md``. Re-add a
    # profile here only with a measurement that survives the current model.

    @classmethod
    def _from_mapping(cls, data: dict[str, Any]) -> NavigationTuning:
        """Reconstruct nested tuning dataclasses from a parsed mapping.

        Shared by :meth:`load_from_yaml` and :meth:`load_from_json` so both
        formats stay in lockstep with ``_GROUPS`` instead of duplicating the
        per-group reconstruction. Missing groups fall back to their defaults,
        allowing partial config files.
        """
        return cls(**{key: dataclass_type(**data.get(key, {})) for key, dataclass_type in cls._GROUPS})

    @classmethod
    def load_from_yaml(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from YAML file.

        Enables runtime configuration without code recompilation.
        Supports multiple tuning profiles for experimentation.

        Requires the ``yaml`` extra: ``uv add "voldemorbot-shared[yaml]"``.

        Args:
            path: Path to YAML file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If YAML file not found
            yaml.YAMLError: If YAML parsing fails
            ValueError: If YAML structure invalid

        Example YAML structure:
            clearance:
              CONTACT_DIST: 0.05
              SLOW_DIST: 0.20
              MEDIUM_DIST: 0.45
              FAST_DIST: 0.90
            pursuit:
              LOOKAHEAD_SHORT: 0.15
              STEER_KP: 2.0
            # ... etc
        """
        try:
            import yaml
        except ImportError as err:
            raise ImportError("PyYAML required for loading YAML tuning files") from err

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Tuning file not found: {path}")

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError("YAML must contain a mapping (dict)")

        return cls._from_mapping(data)

    @classmethod
    def load_from_toml_dir(cls, directory: Path | str) -> NavigationTuning:
        """Load tuning configuration from a directory of per-group TOML files.

        One ``<group>.toml`` per ``_GROUPS`` entry (e.g. ``clearance.toml``,
        ``pursuit.toml``) -- that file's own top-level fields ARE the group,
        no wrapper table needed since the filename already disambiguates
        which group it is. A missing file falls back to that group's
        defaults, same as a missing key in ``load_from_yaml``/
        ``load_from_json``'s single-file mapping. A missing directory
        returns all-defaults outright, so constructing a navigator in a
        test/sim context with no config tree on disk still works.

        Args:
            directory: Directory containing the per-group TOML files.

        Returns:
            NavigationTuning instance with loaded parameters.
        """
        import tomllib

        directory = Path(directory)
        data: dict[str, Any] = {}
        if directory.is_dir():
            for key, _ in cls._GROUPS:
                toml_path = directory / f"{key}.toml"
                if toml_path.exists():
                    with open(toml_path, "rb") as f:
                        data[key] = tomllib.load(f)

        return cls._from_mapping(data)

    @classmethod
    def load_default(cls) -> NavigationTuning:
        """Load from the checked-in DEFAULT_CONFIG_DIR TOML tree.

        The normal way to construct a NavigationTuning in production code --
        falls back to hardcoded per-group defaults for any file (or the
        whole directory) that isn't present, so it's also safe to call from
        a test/sim context that doesn't have the full repo checked out.
        """
        return cls.load_from_toml_dir(DEFAULT_CONFIG_DIR)

    @classmethod
    def load_from_json(cls, path: Path | str) -> NavigationTuning:
        """Load tuning configuration from JSON file.

        Similar to load_from_yaml but uses JSON format instead.

        Args:
            path: Path to JSON file with tuning parameters

        Returns:
            NavigationTuning instance with loaded parameters

        Raises:
            FileNotFoundError: If JSON file not found
            json.JSONDecodeError: If JSON parsing fails
        """
        import json

        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Tuning file not found: {path}")

        with open(path, encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict):
            raise ValueError("JSON must contain a mapping (dict)")

        return cls._from_mapping(data)

    def to_dict(self) -> dict[str, Any]:
        """Export configuration as nested dictionary.

        Useful for serialization or debugging.

        Returns:
            Dictionary representation of all parameters
        """
        import dataclasses

        return {key: dataclasses.asdict(getattr(self, key)) for key, _ in self._GROUPS}

    def to_json(self) -> str:
        """Export configuration as JSON string.

        Returns:
            JSON string representation of all parameters
        """
        import json

        return json.dumps(self.to_dict(), indent=2)

    def to_yaml(self) -> str:
        """Export configuration as YAML string.

        Requires the ``yaml`` extra: ``uv add "voldemorbot-shared[yaml]"``.

        Returns:
            YAML string representation of all parameters

        Raises:
            ImportError: If PyYAML not available
        """
        try:
            import yaml
        except ImportError as err:
            raise ImportError("PyYAML required for YAML export") from err

        return yaml.dump(self.to_dict(), default_flow_style=False)
