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
    """The heading error above which speed is cut. Radians.

    One threshold, not a ladder. This held four (CRAWL/SLOW/MEDIUM/NORMAL) and
    the speed ladder stepped down through them as heading error grew. Measured
    on hardware 2026-08-09 that cost 33% of lap time -- CW 134.9 s -> 179.3 s,
    CCW 161.7 s -> 200.9 s, both past the 180 s round limit -- because ordinary
    cornering sits at 23-45 deg, so the middle rungs taxed every corner on the
    track rather than catching a dangerous case. Corner speed was 0.117 m/s
    against a 0.156 ceiling with 0.34-0.50 m of clearance and risk reading safe.

    CRAWL is the one that describes something real: past ~57 deg the steering
    servo's fixed slew rate cannot track the demand (2026-08-03), so speed has
    to come down. Below it, it does not.

    The other three were deleted rather than left in place. Config nothing reads
    advertises control it does not have, and these would have been read as live
    speed tuning. Re-adding them is a two-line change if a measurement ever
    beats the times above.

    Attributes:
        CRAWL: Severe misalignment (> 1.0 rad, ~57 deg) - drop to the creep floor
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    CRAWL: float = Field(default=1.0, validation_alias=_alias("CRAWL"))  # ~57° - worst case


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

            Was 0.40, which gave the leading signal NO LEAD AT ALL. Computed
            2026-08-29 by running the real planner and ``path_turn_ahead``
            against the geometric arc entry, on the all-narrow blind prior:

              preview  lead before the arc   armed over the lap (narrow/wide)
                0.40      0.000 m  (0.00 s)          27% / 38%
                0.60      0.000 m  (0.00 s)            -
                0.80      0.257 m  (0.73 s)          38% / 64%
                1.00      0.514 m  (1.47 s)          49% / 73%
                1.20      0.772 m  (2.21 s)          61% / 100%

            (seconds at 0.35 m/s.) The preview has to span the straight
            remainder AND reach into the arc before any heading change
            registers, and waypoint spacing is 0.117-0.258 m (mean 0.205), so
            0.40 m simply never got there -- the short lookahead armed exactly
            AT the arc, which is the "a corner late" failure this field exists
            to prevent, still present with the field in place.

            0.80 is the SMALLEST value that leads at all. Not larger, because
            the cost is the fraction of the lap spent on the short lookahead,
            and curvature is quadratic in it (2y/L**2): 1.20 arms it over an
            entire wide lap, which is no longer "corner mode" but a permanently
            higher gain, and hardware run_20260829_100947 already showed a
            +-0.18 m weave in a corridor whose total margin is 0.203 m.
            Geometry, not a track measurement -- the sim cannot show tracking.
        CORNER_TURN_THRESHOLD_RAD: Heading change within the preview distance
            above which the corner is treated as imminent and the short
            lookahead engages. A straight reads ~0; a corner reads roughly
            preview/arc_radius, and the arc radius is set per corner by the
            corridors it joins (ARC_RADIUS is only a cap and does not bind on
            this track), so 0.30-0.40 m radii put a corner well above this.

            Insensitive over a wide band, so do not reach for it to change
            WHEN the turn arms. The signal is quantised by waypoint spacing:
            measured 2026-08-29 on the all-narrow prior, one corner reads
            ``0.00 0.00 0.59 1.18 0.98 0.59 0.20 0.00``, jumping 0.00 -> 0.59
            in a single step, so every threshold in 0.21-0.58 arms at the
            identical waypoint (16/44 either way). Dropping to 0.15 only
            catches the trailing 0.20, holding the short lookahead longer on
            corner EXIT -- it does nothing at entry. Entry timing is set by
            CORNER_PREVIEW_DISTANCE_M; see its note.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    # 0.16/0.32, not the 0.20/0.40 these read until 2026-08-21: the shorter pair
    # is what pursuit.toml ships and what was measured (corpus collisions
    # 202 -> 196, laps>=3 -> 64). The bare defaults had been left behind, so any
    # caller constructing NavigationTuning() without the TOML silently drove a
    # configuration nobody chose -- which is what TestFieldDefaultsMatchShippedToml
    # exists to catch, and had been failing on.
    LOOKAHEAD_SHORT: float = Field(default=0.16, validation_alias=_alias("LOOKAHEAD_SHORT"))  # Close to corner
    LOOKAHEAD_LONG: float = Field(default=0.32, validation_alias=_alias("LOOKAHEAD_LONG"))  # Normal straight
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
        default=0.80, validation_alias=_alias("CORNER_PREVIEW_DISTANCE_M")
    )  # Path distance previewed for an upcoming turn
    CORNER_TURN_THRESHOLD_RAD: float = Field(
        default=0.35, validation_alias=_alias("CORNER_TURN_THRESHOLD_RAD")
    )  # Heading change over that preview that counts as a corner


class SpeedControlParams(BaseModel):
    """Speed control parameters for different zones, in ABSOLUTE m/s.

    Every tier is a real speed. The drivetrain ceiling
    (``RobotSpecs.MAX_SPEED_MPS``) is applied as a CLAMP by the ``*_mps()``
    accessors, never as a multiplier.

    Why absolute (2026-08-21)
    -------------------------
    These were fractions of the ceiling from 2026-08-09 until a faster motor
    made that representation actively wrong. A fraction ladder re-scales every
    tier the moment the ceiling is recalibrated, so swapping the motor silently
    edits the navigation policy instead of just the hardware description:

    * ``MIN`` is the FRICTION FLOOR -- the least speed that overcomes stiction
      and actually moves the robot. That is a property of the motor and tyres,
      ~0.05 m/s, and it does not rise because the top speed did. As a fraction
      it would have become 0.075 m/s on a 0.234 m/s drivetrain: the robot would
      believe it cannot move slower than 7.5 cm/s when it demonstrably can.
    * ``CREEP`` is argued in CENTIMETRES OF TRAVEL -- the servo slews at a fixed
      2.0 rad/s, so full lock from centre takes 0.61 s, during which the chassis
      covers 6.2 cm against 0.103 m of lateral margin. Re-scaling the speed
      invalidates that budget without touching the sentence that justifies it.

    Absolute values make a hardware change pure calibration: edit
    ``robot.toml``'s ceiling and nothing here moves. Raising the tiers to
    exploit a faster motor is then a separate, deliberate, reviewable edit
    rather than a side effect.

    History
    -------
    Before 2026-08-09 these were absolute m/s on a 0-0.50 scale the drivetrain
    does not have. Mapped onto the real ceiling, the rungs 0.05 / 0.15 / 0.30 /
    0.50 came out as 0.05 / 0.15 / 0.156 / 0.156 -- two distinguishable speeds
    wearing four names, MEDIUM and FAST identical, and a sweep over MEDIUM
    unable to move the robot at all. The fix then was fractions; the fix now is
    absolute values that are CLAMPED rather than free, which keeps that failure
    from returning: a tier above the ceiling is inert, and ``mps_ceiling()``
    reports it.

    The shipped ladder is 0.0499 / 0.1014 / 0.117 / 0.1326 / 0.156 m/s -- the
    exact values the old fractions resolved to, so this conversion changed no
    behaviour.

    These are what a zone is WORTH, not which zone applies. Deployed 2026-08-09
    the ladder cost 33% of lap time (CW 134.9 s -> 179.3 s, CCW 161.7 s ->
    200.9 s, both past the 180 s limit) -- but the cause was the heading ladder
    routing ordinary cornering through SLOW/MEDIUM, not these values. Fixed
    where the zone is chosen, in CoreNavigator.

    The FLOOR is measured good: at creep 0.101 m/s the heading limiter bound 0%
    of ticks and CCW went from 1 escape and 4 stucks to none, against 16-21% of
    ticks pinned at the old 0.05 m/s crawl.

    Attributes:
        MIN_MPS: Friction floor. A clamp on the others, not a tier itself.
        MAX_MPS: Upper bound on any tier.
        CREEP_MPS: Contact zone, and the heading limiter's floor.
        SLOW_MPS: Near obstacles.
        MEDIUM_MPS: Moderate clearance.
        FAST_MPS: Open track.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_MPS: float = Field(default=0.0499, gt=0.0, validation_alias=_alias("MIN_MPS"))
    MAX_MPS: float = Field(default=0.156, gt=0.0, validation_alias=_alias("MAX_MPS"))
    CREEP_MPS: float = Field(default=0.1014, gt=0.0, validation_alias=_alias("CREEP_MPS"))
    SLOW_MPS: float = Field(default=0.117, gt=0.0, validation_alias=_alias("SLOW_MPS"))
    MEDIUM_MPS: float = Field(default=0.1326, gt=0.0, validation_alias=_alias("MEDIUM_MPS"))
    FAST_MPS: float = Field(default=0.156, gt=0.0, validation_alias=_alias("FAST_MPS"))

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
        if self.MIN_MPS > self.CREEP_MPS:
            msg = (
                f"speed.MIN_MPS ({self.MIN_MPS}) must not exceed speed.CREEP_MPS "
                f"({self.CREEP_MPS}); the envelope clamp would swallow the creep tier "
                "and mask any change made to it"
            )
            raise ValueError(msg)
        return self

    def mps_ceiling(self) -> float:
        """The drivetrain ceiling every accessor clamps to.

        Exposed so a caller can tell "this tier is inert because the drivetrain
        cannot reach it" from "this tier was tuned to that value" -- the
        distinction the pre-2026-08-09 ladder lost when MEDIUM and FAST both
        silently resolved to the ceiling.
        """
        return RobotSpecs.MAX_SPEED_MPS

    def min_mps(self) -> float:
        """Friction floor in m/s."""
        return min(self.MIN_MPS, RobotSpecs.MAX_SPEED_MPS)

    def max_mps(self) -> float:
        """Upper speed bound in m/s."""
        return min(self.MAX_MPS, RobotSpecs.MAX_SPEED_MPS)

    def creep_mps(self) -> float:
        """Contact-zone / heading-floor speed in m/s."""
        return min(self.CREEP_MPS, RobotSpecs.MAX_SPEED_MPS)

    def slow_mps(self) -> float:
        """Slow-zone speed in m/s."""
        return min(self.SLOW_MPS, RobotSpecs.MAX_SPEED_MPS)

    def medium_mps(self) -> float:
        """Medium-zone speed in m/s."""
        return min(self.MEDIUM_MPS, RobotSpecs.MAX_SPEED_MPS)

    def fast_mps(self) -> float:
        """Open-track speed in m/s."""
        return min(self.FAST_MPS, RobotSpecs.MAX_SPEED_MPS)


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
