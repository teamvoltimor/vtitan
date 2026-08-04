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

from pydantic import AliasChoices, BaseModel, ConfigDict, Field

DEFAULT_CONFIG_DIR: Path = Path(__file__).resolve().parents[3] / "config" / "navigation"
"""platform/shared/config/navigation -- the checked-in per-group TOML tree.

Resolved relative to this module's own location (platform/shared/src/shared/
config/navigation_tuning.py) rather than the caller's, since this package is
the one that actually knows where its own config lives -- callers (e.g.
CoreNavigator) shouldn't have to know or assume the two are siblings under
the same platform/ root."""


def _alias(name: str) -> AliasChoices:
    """Accept both the SHOUT_CASE field name (YAML/JSON profiles, direct
    kwargs) and its lowercase TOML-file spelling, so nested tuning groups
    keep their existing SHOUT_CASE attribute names everywhere they're read
    (``tuning.pursuit.LOOKAHEAD_SHORT``) while the checked-in per-group TOML
    files under DEFAULT_CONFIG_DIR use lowercase keys."""
    return AliasChoices(name, name.lower())


class ClearanceZones(BaseModel):
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

    model_config = ConfigDict(frozen=True, extra="forbid")

    CONTACT_DIST: float = Field(default=0.10, validation_alias=_alias("CONTACT_DIST"))  # Creep forward zone
    SLOW_DIST: float = Field(default=0.25, validation_alias=_alias("SLOW_DIST"))  # Reduced speed
    MEDIUM_DIST: float = Field(default=0.50, validation_alias=_alias("MEDIUM_DIST"))  # Normal speed
    FAST_DIST: float = Field(default=1.00, validation_alias=_alias("FAST_DIST"))  # Full speed capability
    PATH_MARGIN: float = Field(default=0.10, validation_alias=_alias("PATH_MARGIN"))  # Forward-path margin


class HeadingErrorZones(BaseModel):
    """Heading error thresholds for speed modulation.

    These thresholds define how much heading error reduces speed. Radians.

    Attributes:
        CRAWL: Severe misalignment (> 1.0 rad) - crawl speed
        SLOW: Large error (0.7-1.0 rad) - slow speed
        MEDIUM: Moderate error (0.4-0.7 rad) - medium speed
        NORMAL: Small error (< 0.4 rad) - normal speed
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    CRAWL: float = Field(default=1.0, validation_alias=_alias("CRAWL"))  # ~57° - worst case
    SLOW: float = Field(default=0.7, validation_alias=_alias("SLOW"))  # ~40°
    MEDIUM: float = Field(default=0.4, validation_alias=_alias("MEDIUM"))  # ~23°
    NORMAL: float = Field(default=0.2, validation_alias=_alias("NORMAL"))  # ~11°


class PurePursuitParams(BaseModel):
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

    model_config = ConfigDict(frozen=True, extra="forbid")

    LOOKAHEAD_SHORT: float = Field(default=0.20, validation_alias=_alias("LOOKAHEAD_SHORT"))  # Close to corner
    LOOKAHEAD_LONG: float = Field(default=0.40, validation_alias=_alias("LOOKAHEAD_LONG"))  # Normal straight
    LOOKAHEAD_TRANSITION: float = Field(
        default=0.30, validation_alias=_alias("LOOKAHEAD_TRANSITION")
    )  # Crosstrack threshold
    STEER_KP: float = Field(default=1.2, validation_alias=_alias("STEER_KP"))  # Steering P-gain
    MAX_STEERING_RATE: float = Field(default=2.0, validation_alias=_alias("MAX_STEERING_RATE"))  # rad/s


class SpeedControlParams(BaseModel):
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

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_SPEED: float = Field(default=0.05, validation_alias=_alias("MIN_SPEED"))  # Minimum to move
    MAX_SPEED: float = Field(default=0.50, validation_alias=_alias("MAX_SPEED"))  # Maximum safe speed
    CREEP_SPEED: float = Field(default=0.05, validation_alias=_alias("CREEP_SPEED"))  # Contact zone
    SLOW_SPEED: float = Field(default=0.15, validation_alias=_alias("SLOW_SPEED"))  # Near obstacles
    MEDIUM_SPEED: float = Field(default=0.30, validation_alias=_alias("MEDIUM_SPEED"))  # Moderate clearance
    FAST_SPEED: float = Field(default=0.50, validation_alias=_alias("FAST_SPEED"))  # Open track


class EscapeManeuverParams(BaseModel):
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
        STUCK_CONFIRMATION_CHECKS: Consecutive below-threshold stuck checks
            required before StuckDetector declares the robot stuck
        STUCK_ESCALATION_FRAMES_PER_ATTEMPT: Frames added to a stuck-reverse
            maneuver's duration per repeated stuck-escape attempt
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    REV_SPEED: float = Field(default=-0.20, validation_alias=_alias("REV_SPEED"))  # Reverse speed
    REV_STEERING_SCALE: float = Field(
        default=0.8, validation_alias=_alias("REV_STEERING_SCALE")
    )  # Steering while reversing
    K_TURN_MIN_FRAMES: int = Field(default=6, validation_alias=_alias("K_TURN_MIN_FRAMES"))  # Minimum K-turn
    K_TURN_MAX_FRAMES: int = Field(default=12, validation_alias=_alias("K_TURN_MAX_FRAMES"))  # Maximum K-turn
    SLALOM_REVERSE_FRAMES: int = Field(
        default=8, validation_alias=_alias("SLALOM_REVERSE_FRAMES")
    )  # Reverse duration in slalom
    SLALOM_FORWARD_FRAMES: int = Field(
        default=10, validation_alias=_alias("SLALOM_FORWARD_FRAMES")
    )  # Forward turn duration
    STUCK_MOVE_THRESHOLD: float = Field(
        default=0.03, validation_alias=_alias("STUCK_MOVE_THRESHOLD")
    )  # 3cm movement threshold
    STUCK_TIMEOUT_FRAMES: int = Field(
        default=40, validation_alias=_alias("STUCK_TIMEOUT_FRAMES")
    )  # ~2 seconds at 20Hz
    SIDE_CORRECTION_STEER: float = Field(default=0.3, validation_alias=_alias("SIDE_CORRECTION_STEER"))
    SIDE_CORRECTION_SPEED: float = Field(default=0.1, validation_alias=_alias("SIDE_CORRECTION_SPEED"))
    SIDE_CORRECTION_FRAMES: int = Field(default=4, validation_alias=_alias("SIDE_CORRECTION_FRAMES"))
    ESCALATE_AFTER_ATTEMPTS: int = Field(default=3, validation_alias=_alias("ESCALATE_AFTER_ATTEMPTS"))
    MAX_ESCAPE_FRAMES: int = Field(default=20, validation_alias=_alias("MAX_ESCAPE_FRAMES"))
    STUCK_CONFIRMATION_CHECKS: int = Field(default=3, validation_alias=_alias("STUCK_CONFIRMATION_CHECKS"))
    STUCK_ESCALATION_FRAMES_PER_ATTEMPT: int = Field(
        default=2, validation_alias=_alias("STUCK_ESCALATION_FRAMES_PER_ATTEMPT")
    )
    MIN_HISTORY_FOR_DISTANCE: int = Field(
        default=2, validation_alias=_alias("MIN_HISTORY_FOR_DISTANCE")
    )  # Poses needed before StuckDetector can measure distance travelled


class WaypointParams(BaseModel):
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
        REPLAN_HEADING_TIE_MARGIN_M: When ``CoreNavigator.replace_path`` is
            given the robot's current heading, candidate waypoints within
            this much of the nearest one's distance are re-ranked by heading
            agreement instead of taking the nearest purely by position. Near
            a corner, several waypoints can sit at almost the same distance
            from the robot while pointing in very different directions --
            picking purely by position there can hand the pursuit controller
            a point past the turn, demanding a correction far larger than
            finishing the corner needs. Measured on real hardware: a ~193 deg
            swing where completing the corner only needed ~90 deg, right
            after a blind round's direction inference committed. Deliberately
            a tie-margin rather than a blended cost -- it only overrides the
            nearest-position pick when a comparably-close, better-aligned
            alternative actually exists, so the normal small-adjustment
            reseek (``_update_layout_belief``, following an already-similar
            path) is unaffected.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    ARC_RADIUS: float = Field(default=0.45, validation_alias=_alias("ARC_RADIUS"))
    DEDUPE_DISTANCE_M: float = Field(default=0.001, validation_alias=_alias("DEDUPE_DISTANCE_M"))
    OUTER_WALL_BIAS: float = Field(default=0.05, validation_alias=_alias("OUTER_WALL_BIAS"))
    NUM_INTERMEDIATE_ARC_POINTS: int = Field(default=3, validation_alias=_alias("NUM_INTERMEDIATE_ARC_POINTS"))
    STRAIGHT_WAYPOINT_COUNT: int = Field(default=8, validation_alias=_alias("STRAIGHT_WAYPOINT_COUNT"))
    MAIN_LOOP_REACHED_DISTANCE_M: float = Field(
        default=0.20, validation_alias=_alias("MAIN_LOOP_REACHED_DISTANCE_M")
    )
    CONTROLLER_REACHED_DISTANCE_M: float = Field(
        default=0.01, validation_alias=_alias("CONTROLLER_REACHED_DISTANCE_M")
    )
    REPLAN_HEADING_TIE_MARGIN_M: float = Field(
        default=0.15, validation_alias=_alias("REPLAN_HEADING_TIE_MARGIN_M")
    )


class SensorHealthParams(BaseModel):
    """Sensor dropout / staleness detection for the hardware gateway.

    Attributes:
        STALE_TIMEOUT_SEC: A cached sensor reading older than this is treated as
            a dropout — the gateway reports it as unavailable so the navigator
            degrades safely instead of acting on frozen data. Derived as 5x the
            LIDAR scan period (RobotSpecs.LIDAR_UPDATE_RATE), the slowest sensor
            feed the control loop depends on.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    STALE_TIMEOUT_SEC: float = Field(default=0.5, validation_alias=_alias("STALE_TIMEOUT_SEC"))


class LidarSectorParams(BaseModel):
    """LIDAR angular-sector definitions shared by collision avoidance and the OLED.

    Attributes:
        FRONT_HALF_FOV_DEG: Half-width (deg) of the forward clearance cone,
            used by CollisionAvoidanceController.compute_forward_clearance.
        THREAT_HALF_FOV_DEG: Half-width (deg) of the threat-detection sectors
            (front/left/right/back), used by detect_threat_direction and
            related methods -- a narrower cone than the forward clearance
            cone above; both are min-based (see compute_forward_clearance's
            docstring for why it switched from mean 2026-08-04).
        SELF_DETECTION_THRESHOLD_M: Rays no farther than this are discarded
            as chassis/cable self-reflection when a sector filters for it.
        MIN_VALID_RANGE_M: LIDAR ranges at or below this are treated as
            invalid (no-return) readings. Matches RobotSpecs.LIDAR_MIN_RANGE
            (the C1's real rated minimum, 0.05m) -- was 0.01m, five times
            below what the sensor can physically report.
        THREAT_NO_DETECTION_RANGE_M: A sector's nearest reading beyond this
            distance doesn't count as a threat at all -- used by
            detect_threat_direction to return ThreatDirection.NONE instead
            of the nearest-but-still-far sector.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    FRONT_HALF_FOV_DEG: float = Field(default=30.0, validation_alias=_alias("FRONT_HALF_FOV_DEG"))
    THREAT_HALF_FOV_DEG: float = Field(default=45.0, validation_alias=_alias("THREAT_HALF_FOV_DEG"))
    SELF_DETECTION_THRESHOLD_M: float = Field(
        default=0.08, validation_alias=_alias("SELF_DETECTION_THRESHOLD_M")
    )
    MIN_VALID_RANGE_M: float = Field(default=0.05, validation_alias=_alias("MIN_VALID_RANGE_M"))
    THREAT_NO_DETECTION_RANGE_M: float = Field(
        default=1.0, validation_alias=_alias("THREAT_NO_DETECTION_RANGE_M")
    )
    NO_DATA_RANGE_M: float = Field(
        default=10.0, validation_alias=_alias("NO_DATA_RANGE_M")
    )
    """Fallback range (m) when no valid LIDAR readings are available.

    Used as sentinel value in sector computations when all rays are invalid.
    Conservative estimate between min (0.05m) and max (12m) sensor range."""

    DIRECTION_ARC_HALF_FOV_DEG: float = Field(
        default=8.0, validation_alias=_alias("DIRECTION_ARC_HALF_FOV_DEG")
    )
    """Half-width (deg) of the narrow forward cone src.navigation.utils'
    _forward_clearance uses, consumed by corridor_follower's turn-start gate
    and direction_estimator's corner-detection gate.

    Deliberately its own field, not FRONT_HALF_FOV_DEG: the two look
    interchangeable (both "how wide is forward") but are not -- FRONT_HALF_FOV_DEG
    (30 deg) is collision-avoidance's braking cone, sized to catch an obstacle
    with margin. This one gates *when a corridor counts as ending* for corner
    detection, where direction_estimator's own docstring warns the gap between
    this and TURN_CLEARANCE_M is a fragile, measured window: three fixtures
    never settled and two settled wrong when it was mistuned by less than this
    field's difference from FRONT_HALF_FOV_DEG alone. A prior refactor pointed
    _forward_clearance at FRONT_HALF_FOV_DEG to remove a hardcoded 8.0, on the
    reasonable-looking assumption that one "forward cone" tuning number should
    serve both -- widening the corner-detection cone by 3.75x broke the
    timing outright, reproduced as every clockwise narrow-corridor run timing
    out mid-lap while every counterclockwise one passed clean (an unrelated
    starting section makes the two directions meet their first corner at a
    different point in this timing window, so the same corruption did not
    fail identically)."""


class SignRouterParams(BaseModel):
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
        DEPTH_PIN: Hold the deformed waypoint abeam the sign while the sign lies
            between the chassis and the lookahead point, instead of letting the
            commanded point recede a lookahead per tick. ``False`` restores the
            plain lookahead depth.
        PASSED_DIST_M: Distance (m) beyond which a sign is marked "passed"
            and its deformation taper reaches zero.
        DETECTION_MATCH_DIST_M: Max distance (m) to associate a camera
            detection with an expected sign.
        MIN_CONFIDENCE: Minimum detection confidence to accept a camera
            color update for a sign.
        SETTLE_TICKS: Ticks after lap start before sign engage/pass
            bookkeeping activates (~7.5s @ 20Hz by default).
        COMMIT_HYSTERESIS: Keep routing around the sign already engaged
            instead of re-running the nearest-wins race every tick. Prevents
            the commanded lateral line jumping between two legal values while
            the chassis is committed. See SignRouter._prefer_committed.
            Defaults OFF: measured flat sighted and marginally worse blind
            (see the note in sign_router.toml).
        ESCAPE_MASK_RADIUS_M: How close a LIDAR return must land to a routed
            sign to be attributed to it and withheld from the reactive escape
            trigger. Zero disables the mapped/unmapped split entirely, which
            restores the pre-fix behaviour where the escape maneuver fires on
            every sign pass.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SIGN_CLEARANCE_MARGIN_M: float = Field(default=0.075, validation_alias=_alias("SIGN_CLEARANCE_MARGIN_M"))
    DEFORM_DEPTH_BUFFER_M: float = Field(default=0.5, validation_alias=_alias("DEFORM_DEPTH_BUFFER_M"))
    WALL_CLEARANCE_MARGIN_M: float = Field(default=0.04, validation_alias=_alias("WALL_CLEARANCE_MARGIN_M"))
    ACTIVATION_DIST_M: float = Field(default=1.40, validation_alias=_alias("ACTIVATION_DIST_M"))
    PASSED_DIST_M: float = Field(default=1.60, validation_alias=_alias("PASSED_DIST_M"))
    DEPTH_PIN: bool = Field(default=True, validation_alias=_alias("DEPTH_PIN"))
    DETECTION_MATCH_DIST_M: float = Field(default=0.30, validation_alias=_alias("DETECTION_MATCH_DIST_M"))
    MIN_CONFIDENCE: float = Field(default=0.25, validation_alias=_alias("MIN_CONFIDENCE"))
    SETTLE_TICKS: int = Field(default=150, validation_alias=_alias("SETTLE_TICKS"))
    ESCAPE_MASK_RADIUS_M: float = Field(default=0.12, validation_alias=_alias("ESCAPE_MASK_RADIUS_M"))
    COMMIT_HYSTERESIS: bool = Field(default=False, validation_alias=_alias("COMMIT_HYSTERESIS"))


class CorridorEstimatorParams(BaseModel):
    """Blind corridor-width estimation parameters.

    Attributes:
        MIN_SAMPLES: Width readings a corridor must accumulate before its
            estimate is trusted. Readings are attributed to a corridor by
            heading, so a handful taken while the chassis is still swinging
            through a corner can land in the wrong one.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_SAMPLES: int = Field(default=12, validation_alias=_alias("MIN_SAMPLES"))


class StateEstimatorParams(BaseModel):
    """Odometry/IMU fusion parameters.

    Attributes:
        YAW_CORRECTION_GAIN: Fraction of the observed yaw discrepancy folded
            into the estimate per update. Distinct from LocalizationParams,
            which tunes the LIDAR pose *search*; this is the dead-reckoning
            blend that runs whether or not localization is enabled.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    YAW_CORRECTION_GAIN: float = Field(default=0.05, validation_alias=_alias("YAW_CORRECTION_GAIN"))


class SimulationParams(BaseModel):
    """Knobs that exist only in the headless simulator.

    These do not describe the robot or the mat, so they belong neither in
    robot.toml nor track.toml -- but they decide what a simulated run scores,
    which makes them exactly the kind of value that must not be a literal
    buried in a module. The contact policy in particular governs whether a
    legal start already touching a wall is a failure or a recoverable state.

    Attributes:
        START_COLLISION_WINDOW_S: Opening seconds during which contact is
            treated as a start-position artefact rather than a crash, because
            a legal starting cell may already sit against a wall.
        START_COLLISION_GRACE_S: How long such an opening contact may persist
            before it counts as a real failure.
        LIDAR_INVALID_RAY_RATE: Fraction of rays returning no measurement.
            A placeholder: the real rate depends on the mat's surface and is
            worth measuring from a recorded bag rather than guessed at.
        DETECTION_CONFIDENCE: Confidence stamped on emulated camera
            detections.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    START_COLLISION_WINDOW_S: float = Field(
        default=2.0, validation_alias=_alias("START_COLLISION_WINDOW_S")
    )
    START_COLLISION_GRACE_S: float = Field(
        default=15.0, validation_alias=_alias("START_COLLISION_GRACE_S")
    )
    LIDAR_INVALID_RAY_RATE: float = Field(
        default=0.01, validation_alias=_alias("LIDAR_INVALID_RAY_RATE")
    )
    DETECTION_CONFIDENCE: float = Field(default=0.9, validation_alias=_alias("DETECTION_CONFIDENCE"))


class CorridorFollowerParams(BaseModel):
    """Blind corridor-following and corner-turn parameters.

    Attributes:
        TURN_CLEARANCE_M: Forward clearance (m) at which the corner turn
            begins. Must stay strictly below
            DirectionEstimatorParams.CORNER_CLEARANCE_M -- see the
            cross-group check on NavigationTuning.
        CENTERING_GAIN: Steering per metre of lateral offset from the
            corridor centreline.
        MAX_CENTERING_STEER: Hard cap on the steering that gain may
            produce. Both are deliberately timid: the counter-phase
            four-wheel chassis responds violently, and oscillation swings
            the heading past the direction estimator's alignment gate,
            which then refuses every reading.
        CORNER_SPEED_SCALE: Fraction of creep speed while turning a corner
            blind, which is committed on one comparison rather than a plan.
        REVERSE_SPEED_SCALE: Fraction of creep speed while backing off.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    TURN_CLEARANCE_M: float = Field(default=0.60, validation_alias=_alias("TURN_CLEARANCE_M"))
    CENTERING_GAIN: float = Field(default=0.8, validation_alias=_alias("CENTERING_GAIN"))
    MAX_CENTERING_STEER: float = Field(default=0.25, validation_alias=_alias("MAX_CENTERING_STEER"))
    CORNER_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("CORNER_SPEED_SCALE"))
    REVERSE_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("REVERSE_SPEED_SCALE"))


class ControlLoopParams(BaseModel):
    """The rate the navigation control loop runs at.

    One number, previously written twice: ``waypoint_controller`` carried
    ``_DEFAULT_CONTROL_DT_S = 0.05`` and the simulation gateway carried
    ``CONTROL_HZ = 20.0``. They agreed only because someone kept them agreeing.
    The controller uses ``dt`` to rate-limit steering, so a divergence would not
    fail -- it would quietly tune the real robot against a cadence the simulator
    never ran at.

    Attributes:
        CONTROL_HZ: Control loop frequency (Hz). Derive periods from
            ``NavigationTuning.control_dt_s`` rather than restating 0.05.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    CONTROL_HZ: float = Field(default=20.0, validation_alias=_alias("CONTROL_HZ"))


class WallHeadingParams(BaseModel):
    """LIDAR wall-direction estimation for the blind heading reference.

    Fits short segments across the LIDAR returns and takes their common
    direction as the corridor's. Every threshold here decides whether a pair of
    returns describes one flat surface or two different things.

    Attributes:
        MIN_CONCENTRATION: How aligned the segment directions must be before
            the estimate is trusted at all. Low concentration means the
            returns disagree about where the wall runs, which is what a
            corner, a sign or a doorway looks like.
        BASELINE_RAYS: How far apart (in rays) the two returns forming one
            segment are taken. Wider is less noise-sensitive but blurs
            genuine corners.
        MAX_SEGMENT_JUMP_M: Range step above which two returns are treated as
            different surfaces rather than one wall.
        MIN_SEGMENT_M: Segments shorter than this are dominated by range
            noise rather than wall direction.
        NEAR_MAX_RANGE_M: Returns at or beyond this are no-return rays
            sanitised to max range, not real surfaces.
        MIN_RETURNS: Fewer usable returns than this cannot form a segment.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_CONCENTRATION: float = Field(default=0.55, validation_alias=_alias("MIN_CONCENTRATION"))
    BASELINE_RAYS: int = Field(default=15, validation_alias=_alias("BASELINE_RAYS"))
    MAX_SEGMENT_JUMP_M: float = Field(default=0.30, validation_alias=_alias("MAX_SEGMENT_JUMP_M"))
    MIN_SEGMENT_M: float = Field(default=0.02, validation_alias=_alias("MIN_SEGMENT_M"))
    NEAR_MAX_RANGE_M: float = Field(default=11.0, validation_alias=_alias("NEAR_MAX_RANGE_M"))
    MIN_RETURNS: int = Field(default=3, validation_alias=_alias("MIN_RETURNS"))


class DirectionEstimatorParams(BaseModel):
    """Blind travel-direction inference parameters.

    Attributes:
        CORNER_CLEARANCE_M: Forward clearance (m) below which the corridor
            counts as ending, opening the window in which the robot reads
            which side is open. Deliberately larger than
            CorridorFollowerParams.TURN_CLEARANCE_M: turning swings the
            heading past the alignment gate, so a robot that begins its
            turn the instant the comparison becomes decisive rotates
            straight through its only measurement window.
        MAX_IN_TRACK_RANGE_M: Side rays longer than this (m) cannot be a
            wall of this track and are rejected. A LIDAR dropout is
            reported as max range, which reads as "this side is open" --
            exactly the signal the estimator looks for.
        MIN_ASYMMETRY_M: Minimum left/right difference (m) for a sweep to
            count as evidence rather than noise.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    CORNER_CLEARANCE_M: float = Field(default=1.00, validation_alias=_alias("CORNER_CLEARANCE_M"))
    MAX_IN_TRACK_RANGE_M: float = Field(default=4.5, validation_alias=_alias("MAX_IN_TRACK_RANGE_M"))
    MIN_ASYMMETRY_M: float = Field(default=0.20, validation_alias=_alias("MIN_ASYMMETRY_M"))


class SignDiscoveryParams(BaseModel):
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

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_RELIABLE_BBOX_HEIGHT_PX: int = Field(default=5, validation_alias=_alias("MIN_RELIABLE_BBOX_HEIGHT_PX"))
    MAX_INGEST_RANGE_M: float = Field(default=2.0, validation_alias=_alias("MAX_INGEST_RANGE_M"))
    ASSOCIATION_DIST_M: float = Field(default=0.25, validation_alias=_alias("ASSOCIATION_DIST_M"))
    MIN_HITS: int = Field(default=3, validation_alias=_alias("MIN_HITS"))


class ParkingParams(BaseModel):
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

    model_config = ConfigDict(frozen=True, extra="forbid")

    PARALLEL_TOLERANCE_M: float = Field(default=0.02, validation_alias=_alias("PARALLEL_TOLERANCE_M"))
    POS_REACH_DIST_M: float = Field(default=0.04, validation_alias=_alias("POS_REACH_DIST_M"))
    DEFAULT_MAX_FRAMES: int = Field(default=400, validation_alias=_alias("DEFAULT_MAX_FRAMES"))
    SATURATED_STEER_THRESHOLD: float = Field(
        default=0.999, validation_alias=_alias("SATURATED_STEER_THRESHOLD")
    )
    SATURATION_STUCK_TICKS: int = Field(default=20, validation_alias=_alias("SATURATION_STUCK_TICKS"))
    SPEED: float = Field(default=0.12, validation_alias=_alias("SPEED"))
    MIN_LOOKAHEAD_DIST_M: float = Field(default=0.02, validation_alias=_alias("MIN_LOOKAHEAD_DIST_M"))
    WALL_STANDOFF_M: float = Field(default=0.05, validation_alias=_alias("WALL_STANDOFF_M"))
    MARKER_STANDOFF_M: float = Field(default=0.01, validation_alias=_alias("MARKER_STANDOFF_M"))


class LocalizationParams(BaseModel):
    """LIDAR-based pose search (LidarLocalizer) parameters.

    Attributes:
        SEARCH_RADIUS_M: Half-width (m) of the initial pose search window.
        PASSES: Number of coarse-to-fine grid-search passes.
        GRID_POINTS: Candidates per axis per search pass.
        RESIDUAL_CLIP_M: Per-ray residual clipping distance (m) for the cost
            function (outlier rejection).
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SEARCH_RADIUS_M: float = Field(default=0.15, validation_alias=_alias("SEARCH_RADIUS_M"))
    PASSES: int = Field(default=4, validation_alias=_alias("PASSES"))
    GRID_POINTS: int = Field(default=5, validation_alias=_alias("GRID_POINTS"))
    RESIDUAL_CLIP_M: float = Field(default=0.25, validation_alias=_alias("RESIDUAL_CLIP_M"))


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
    corridor_estimator: CorridorEstimatorParams = CorridorEstimatorParams()
    corridor_follower: CorridorFollowerParams = CorridorFollowerParams()
    direction_estimator: DirectionEstimatorParams = DirectionEstimatorParams()
    wall_heading: WallHeadingParams = WallHeadingParams()
    control: ControlLoopParams = ControlLoopParams()
    sign_router: SignRouterParams = SignRouterParams()
    sign_discovery: SignDiscoveryParams = SignDiscoveryParams()
    parking: ParkingParams = ParkingParams()
    localization: LocalizationParams = LocalizationParams()
    state_estimator: StateEstimatorParams = StateEstimatorParams()
    simulation: SimulationParams = SimulationParams()

    def __post_init__(self) -> None:
        """Check invariants that span two tuning groups.

        This is a frozen dataclass aggregating pydantic groups, so per-group
        rules live on the groups as validators and only cross-group ones belong
        here. A ``@model_validator`` would be inert on a dataclass -- it is
        silently ignored, which is worse than no check at all.

        The turn must begin strictly after the direction window opens.

        Turning swings the heading past the direction estimator's alignment
        gate, which then refuses every reading. If the robot began its turn at
        the same clearance that makes the left/right comparison decisive, it
        would rotate straight through the only window in which it can read
        which side is open, and come out the far side with a wall on both
        sides and nothing learned. Measured with both at 0.75 m: three fixtures
        never settled at all and two settled wrong after 20-plus seconds.

        The two values live in different groups and so in different TOML files,
        which is exactly why this belongs here -- editing one file cannot see
        the other.
        """
        turn = self.corridor_follower.TURN_CLEARANCE_M
        corner = self.direction_estimator.CORNER_CLEARANCE_M
        if turn >= corner:
            msg = (
                f"corridor_follower.TURN_CLEARANCE_M ({turn}) must be strictly below "
                f"direction_estimator.CORNER_CLEARANCE_M ({corner}); the gap is the window "
                "in which the robot is still square to the corridor and can read which side is open"
            )
            raise ValueError(msg)

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
        ("corridor_estimator", CorridorEstimatorParams),
        ("corridor_follower", CorridorFollowerParams),
        ("direction_estimator", DirectionEstimatorParams),
        ("wall_heading", WallHeadingParams),
        ("control", ControlLoopParams),
        ("sign_router", SignRouterParams),
        ("sign_discovery", SignDiscoveryParams),
        ("parking", ParkingParams),
        ("localization", LocalizationParams),
        ("state_estimator", StateEstimatorParams),
        ("simulation", SimulationParams),
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
        return {key: getattr(self, key).model_dump() for key, _ in self._GROUPS}

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
