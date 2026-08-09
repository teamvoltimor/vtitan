"""Speed/steering control tuning groups.

Covers clearance zones, heading zones, pure pursuit, speed control, and the
control loop rate they all run at.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning._shared import _alias


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
        LOOKAHEAD_TRANSITION: Crosstrack error threshold to switch modes (m).
            A ceiling, not the operative value -- see WALL_MARGIN_SAFETY_M.
        STEER_KP: Proportional gain for steering P-controller
        MAX_STEERING_RATE: Maximum steering command rate (rad/s)
        WALL_MARGIN_SAFETY_M: Clearance (m) to leave between the chassis and an
            outer wall when deciding how much crosstrack error the robot can
            afford before the short lookahead must engage.

            LOOKAHEAD_TRANSITION alone is a fixed 0.30 m, which silently
            assumes the path has at least that much room to drift into. Under
            the blind narrow prior it does not: a corridor believed 0.60 puts
            the path ~0.25-0.30 m from the outer wall, and the chassis
            half-width takes 0.097 of that, leaving 0.15-0.20 m of real
            budget. The correction was therefore armed to fire only after the
            wall had already been reached -- measured on hardware 2026-08-06 as
            crosstrack running 0.09 -> 0.15 through a corner while never
            crossing 0.30, with the robot ending up 0.10 m from the wall.
        MIN_LOOKAHEAD_TRANSITION_M: Floor (m) for that derived threshold.
            Without it a path planned very close to a wall would pin the
            controller to the short lookahead permanently, which is twitchy on
            straights -- trading one failure for another.
        CORNER_PREVIEW_DISTANCE_M: How far along the planned path to look for
            an upcoming turn. Crosstrack error is a lagging signal -- it cannot
            rise until the corner has already been missed -- so gating the
            lookahead on it alone means the sharp correction always arrives
            after the corner. Measured on hardware 2026-08-06: the robot held
            0.9 rad of heading error for three seconds at 0.23 of full lock,
            and only once crosstrack reached 0.13 did the short lookahead arm
            and steering jump to 0.52 -- the right magnitude, ~1.5 s late.
        CORNER_TURN_THRESHOLD_RAD: Heading change within the preview distance
            above which the corner is treated as imminent and the short
            lookahead engages. A straight reads ~0; a corner on the default
            0.45 m arc turns preview/0.45 rad, so 0.40 m of preview reads
            ~0.89 rad. The default sits well clear of both.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    LOOKAHEAD_SHORT: float = Field(default=0.20, validation_alias=_alias("LOOKAHEAD_SHORT"))  # Close to corner
    LOOKAHEAD_LONG: float = Field(default=0.40, validation_alias=_alias("LOOKAHEAD_LONG"))  # Normal straight
    LOOKAHEAD_TRANSITION: float = Field(
        default=0.30, validation_alias=_alias("LOOKAHEAD_TRANSITION")
    )  # Crosstrack threshold
    STEER_KP: float = Field(default=1.2, validation_alias=_alias("STEER_KP"))  # Steering P-gain
    MAX_STEERING_RATE: float = Field(default=2.0, validation_alias=_alias("MAX_STEERING_RATE"))  # rad/s
    WALL_MARGIN_SAFETY_M: float = Field(
        default=0.03, validation_alias=_alias("WALL_MARGIN_SAFETY_M")
    )  # Kept clear of an outer wall
    MIN_LOOKAHEAD_TRANSITION_M: float = Field(
        default=0.10, validation_alias=_alias("MIN_LOOKAHEAD_TRANSITION_M")
    )  # Floor for the derived threshold
    CORNER_PREVIEW_DISTANCE_M: float = Field(
        default=0.40, validation_alias=_alias("CORNER_PREVIEW_DISTANCE_M")
    )  # Path distance previewed for an upcoming turn
    CORNER_TURN_THRESHOLD_RAD: float = Field(
        default=0.35, validation_alias=_alias("CORNER_TURN_THRESHOLD_RAD")
    )  # Heading change over that preview that counts as a corner


class SpeedControlParams(BaseModel):
    """Speed control parameters for different zones, as fractions of the ceiling.

    Every tier is a fraction of ``RobotSpecs.MAX_SPEED_MPS`` -- the measured
    0.156 m/s drivetrain ceiling -- and is read through the ``*_mps()``
    accessors, never directly.

    This group used to hold absolute m/s values on a 0-0.50 scale that the
    drivetrain does not have. Mapped onto the real ceiling, the four rungs
    0.05 / 0.15 / 0.30 / 0.50 came out as 0.05 / 0.15 / 0.156 / 0.156: two
    distinguishable speeds wearing four names, with MEDIUM and FAST identical
    and SLOW within 4% of both. That made the ladder unfalsifiable -- a sweep
    over MEDIUM_SPEED could not move the robot no matter what it was set to --
    and it hid the fact that the only real transition was a 3x cliff at the
    bottom.

    The shipped fractions 0.65 / 0.75 / 0.85 / 1.0 are four speeds the
    drivetrain can actually tell apart: 0.101 / 0.117 / 0.133 / 0.156 m/s.

    The usable band is narrow. MIN_FRAC is the friction floor, so the whole
    ladder lives inside a 3.1x range between "barely moves" and "flat out";
    there is not room in it for four meaningfully distinct rungs.

    Attributes:
        MIN_FRAC: Least fraction that overcomes friction and actually moves
            the robot. A floor on the others, not a tier in its own right.
        MAX_FRAC: Upper bound on any tier. 1.0 is the drivetrain ceiling;
            above that the gateway clamps and the number is fiction.
        CREEP_FRAC: Contact zone, and the heading limiter's floor.
        SLOW_FRAC: Near obstacles.
        MEDIUM_FRAC: Moderate clearance.
        FAST_FRAC: Open track. 1.0 is the drivetrain ceiling.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_FRAC: float = Field(default=0.32, gt=0.0, le=1.0, validation_alias=_alias("MIN_FRAC"))
    MAX_FRAC: float = Field(default=1.0, gt=0.0, le=1.0, validation_alias=_alias("MAX_FRAC"))
    CREEP_FRAC: float = Field(default=0.65, gt=0.0, le=1.0, validation_alias=_alias("CREEP_FRAC"))
    SLOW_FRAC: float = Field(default=0.75, gt=0.0, le=1.0, validation_alias=_alias("SLOW_FRAC"))
    MEDIUM_FRAC: float = Field(default=0.85, gt=0.0, le=1.0, validation_alias=_alias("MEDIUM_FRAC"))
    FAST_FRAC: float = Field(default=1.0, gt=0.0, le=1.0, validation_alias=_alias("FAST_FRAC"))

    @model_validator(mode="after")
    def _floor_below_creep(self) -> SpeedControlParams:
        """The friction floor must not sit above the slowest commanded tier.

        ``core_navigator`` clamps the selected zone speed up to ``min_mps()``.
        When that floor equals or exceeds ``creep_mps()`` the clamp silently
        swallows the creep tier, and any attempt to tune the floor downward
        produces a clean no-change result that looks like evidence and is not.
        The shipped config had exactly this: MIN_SPEED and CREEP_SPEED were
        both 0.05.
        """
        if self.MIN_FRAC > self.CREEP_FRAC:
            msg = (
                f"speed.MIN_FRAC ({self.MIN_FRAC}) must not exceed speed.CREEP_FRAC "
                f"({self.CREEP_FRAC}); the envelope clamp would swallow the creep tier "
                "and mask any change made to it"
            )
            raise ValueError(msg)
        return self

    def min_mps(self) -> float:
        """Friction floor in m/s."""
        return self.MIN_FRAC * RobotSpecs.MAX_SPEED_MPS

    def max_mps(self) -> float:
        """Upper speed bound in m/s."""
        return self.MAX_FRAC * RobotSpecs.MAX_SPEED_MPS

    def creep_mps(self) -> float:
        """Contact-zone / heading-floor speed in m/s."""
        return self.CREEP_FRAC * RobotSpecs.MAX_SPEED_MPS

    def slow_mps(self) -> float:
        """Slow-zone speed in m/s."""
        return self.SLOW_FRAC * RobotSpecs.MAX_SPEED_MPS

    def medium_mps(self) -> float:
        """Medium-zone speed in m/s."""
        return self.MEDIUM_FRAC * RobotSpecs.MAX_SPEED_MPS

    def fast_mps(self) -> float:
        """Open-track speed in m/s."""
        return self.FAST_FRAC * RobotSpecs.MAX_SPEED_MPS


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
