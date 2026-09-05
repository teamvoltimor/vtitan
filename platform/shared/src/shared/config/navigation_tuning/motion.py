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
        OBSTACLES_CONTACT_DIST: Obstacles-Challenge CONTACT_DIST. ``None`` ->
            use CONTACT_DIST. See the field for why only this zone is
            per-challenge, and why only Obstacles has an override.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    CONTACT_DIST: float = Field(default=0.10, validation_alias=_alias("CONTACT_DIST"))  # Creep forward zone
    SLOW_DIST: float = Field(default=0.25, validation_alias=_alias("SLOW_DIST"))  # Reduced speed
    MEDIUM_DIST: float = Field(default=0.50, validation_alias=_alias("MEDIUM_DIST"))  # Normal speed
    FAST_DIST: float = Field(default=1.00, validation_alias=_alias("FAST_DIST"))  # Full speed capability
    PATH_MARGIN: float = Field(default=0.10, validation_alias=_alias("PATH_MARGIN"))  # Forward-path margin

    FORWARD_PATH_AHEAD_OF_BUMPER: bool = Field(
        default=False, validation_alias=_alias("FORWARD_PATH_AHEAD_OF_BUMPER")
    )
    """Require a forward-path return to be ahead of the FRONT BUMPER, not the LIDAR.

    ``_forward_path_ranges`` admits a ray on ``cos(theta) > 0`` -- ahead of the
    SENSOR. The sensor sits ``RobotSpecs.LIDAR_TO_FRONT_BUMPER`` (2.78 cm)
    behind the bumper plane, so "ahead of the sensor" includes a band that is
    physically ALONGSIDE the chassis.

    It only matters at contact range, and there it decides the escape. A ray
    trips the 0.05 m Obstacles gate at a range below 7.78 cm, and at that range
    the lateral test (``|r*sin(theta)| < 0.197``) cannot exclude ANY bearing --
    the largest lateral offset reachable is 7.78 cm. So the forward-path
    corridor degenerates into the whole forward half-plane exactly where it
    fires, and a return at 85 deg -- 0.6 cm along-track, i.e. 2.2 cm BEHIND the
    bumper -- reads as an obstacle dead ahead and triggers a reversing escape
    at something the robot is not driving into. The cutoff is
    ``acos(2.78/7.78) = 69 deg``.

    Measured on the 256 corpus 2026-09-02: 82.1% of escape engagements come
    from this gate, and in every FAILING verdict bucket ~92% of them fire
    outside +/-10 deg with a median |bearing| of 51-57 deg, while the front
    cone reads 33-55 cm clear. ``laps-done`` is the only bucket that triggers
    head-on (16% inside +/-10 deg).

    REFUTED 2026-09-02 on the full 256 corpus, and kept OFF. Do not turn this
    on. It does exactly what it was designed to -- escapes 19,276 -> 7,730,
    timeouts 48 -> 20, and the target metric pass-side 122 -> 80 -- and it is
    still a large net loss:

        pass-side        122 -> 80     clean laps>=3   76 -> 62
        SIGN collisions    4 -> 97     in-time         47 -> 40
        wall collisions   17 -> 13     total          22 -> 111

    All 97 sign collisions came back ``masked``, colour ``right``: the router
    knew about every one of those signs and had its colour correct, and the
    robot drove into it anyway. So the lateral trigger is LOAD-BEARING. The
    escape firing at 70-90 deg is the last line of defence against clipping a
    sign the planner is deliberately passing within ~0.175 m of; suppressing it
    trades 42 disqualifications for 93 extra collisions.

    The conclusion to carry forward is that this gate is not misfiring. Given
    that the robot arrives 6.8 cm from a sign at 55-85 deg, reversing is the
    correct response, and the defect is that it arrives there at all --
    cross-track error is 4.63 cm median against 5.6 cm of plan margin
    (2026-09-01). Pass-side is a TRACKING problem and cannot be reduced through
    the reactive layer.

    Kept rather than deleted so that refutation stays reproducible in-tree:
    ``diag_sign_sweep.py ahead-of-bumper --corpus``.

    Deliberately a bearing/geometry gate rather than a smaller PATH_MARGIN:
    the margin is irrelevant at contact range (nothing can reach it), so
    tightening it would read as a flat sweep.
    """

    FORWARD_NO_DATA_IS_DEGRADED: bool = Field(
        default=True, validation_alias=_alias("FORWARD_NO_DATA_IS_DEGRADED")
    )
    """Treat an UNREADABLE forward cone as a degraded sensor, not as open road.

    ``compute_forward_clearance`` reports ``NO_DATA_RANGE_M`` (10 m) when no ray
    in the forward cone is valid, which is indistinguishable from clear road, so
    every ``clearance < threshold`` gate downstream fails OPEN.

    That is not a hypothetical. Measured on hardware 2026-08-31
    (run_20260831_205208 / _205235): pressed against a wall and physically
    immobile, every forward ray fell below ``min_valid_range_m`` -- a flat
    surface centimetres away reflects too shallowly to return a signal -- so
    forward clearance read **9.97 m for the rest of the run**. The navigator
    held 0.24 m/s into the wall, and the ``stuck_forward`` escape fired for six
    ticks and stood down, because by that number nothing was wrong. Neither run
    recovered.

    The no-LIDAR branch in ``CoreNavigator.step`` already argues the correct
    policy -- "a degraded sensor is not open road", answered with ``SLOW_DIST``
    and a non-SAFE risk. This applies the SAME policy to the case where a scan
    arrives but its forward cone is unreadable, which is strictly more
    dangerous: no scan at all is obvious, an all-invalid cone masquerades as
    12 m of open track.

    Deliberately reuses that branch's response rather than forcing clearance to
    zero. Zero would assert an obstacle at the bumper, which is a different
    claim from "I cannot see", and would fire contact handling on any transient
    dropout burst.

    The rear has had this distinction since the reverse guard was found failing
    open (``rear_sector().measured``); this is the same fix for the front, which
    never had it. See ``CollisionAvoidanceController.front_sector``.

    **Defaults True, on a measured false-positive rate of zero.** Replayed over
    the recorded bags, the forward cone is unmeasured in **0 of 847 scans** of
    the clean 3-lap run (run_20260830_182505) and in **62.7% / 66.1%** of the
    two failing runs. So it never fires on a healthy round and fires on two
    thirds of a wedged one -- there is no crawl-the-whole-race cost to weigh
    against it.

    Covers BOTH consumers of forward clearance. ``CoreNavigator.step`` treats an
    unmeasured cone as the degraded sensor it is; ``EscapeRecovery`` gates
    ``forward_open`` on it, because without that the escape reads 10 m, passes
    ``forward_clear >= CONTACT_DIST``, and chooses STUCK_FORWARD *into* the wall
    it is already touching -- which is what both failing runs recorded. Fixing
    only the navigator yields a slower crash, not a recovery.

    The simulator cannot validate any of this: contact is absorbing, so a run
    ends before the robot can be wedged, and the state is unreachable there. An
    A/B returns identical arms -- the dead end that cost two 256-scenario runs
    on MAX_CORNER_STEER_DEG. Evidence is the unit tests plus the bag replay
    above. NOT track-validated.

    Fixes RECOVERABILITY, not cause. The robot drives into the corner because
    longitudinal pose is unobservable in a corridor (pose_x wanders 0.24-0.38 m
    while the chassis is stationary, pose_y 0.01 m). This stops it grinding
    there at 0.24 m/s afterwards believing the road is clear.
    """

    OBSTACLES_CONTACT_DIST: float | None = Field(
        default=None, gt=0.0, validation_alias=_alias("OBSTACLES_CONTACT_DIST")
    )
    """Obstacles-Challenge ``CONTACT_DIST``. ``None`` -> use ``CONTACT_DIST``.

    Resolved by :meth:`for_obstacles_challenge`, which ``CoreNavigator`` calls
    once at construction -- the same shape, and the same discriminator, as the
    speed ladder's ``OBSTACLES_*`` tiers below.

    Why the contact zone wants to differ by challenge: it is the threshold that
    fires the reversing escape (``assess_risk`` returns CRITICAL below it), and
    the two challenges present completely different things to escape FROM. Open
    has walls only, and a wall at 0.10 m ahead is a genuine emergency. Obstacles
    additionally has signs the router deliberately routes PAST at ~0.175 m from
    their surface, so the same 0.10 m threshold fires on geometry the planner
    chose on purpose. Measured 2026-08-31 on subset128: with signs physical the
    robot logs ~180 escapes per run while colliding with a sign only 4-5 times
    in 128 runs, and 66/128 runs time out -- it is escaping from clearances it
    was aimed at, not from danger.

    Asymmetric on purpose, unlike the speed tiers: there is no
    ``OPEN_CONTACT_DIST``, because nothing has been measured that wants Open to
    differ from the shared value, and an unset knob that nothing has ever moved
    reads as tuning that exists. Add the Open half when a measurement asks for
    it.

    NOT track-validated. Lowering this shortens the distance in which the
    chassis must actually halt, which is a hardware question the simulator
    cannot answer -- see the stopping-distance bench check.
    """

    def for_obstacles_challenge(self) -> ClearanceZones:
        """These zones as the Obstacles Challenge should run them.

        Returns ``self`` unchanged when no Obstacles override is set, so the
        Open path and the un-overridden Obstacles path stay byte-identical.
        """
        if self.OBSTACLES_CONTACT_DIST is None:
            return self
        return self.model_copy(update={"CONTACT_DIST": self.OBSTACLES_CONTACT_DIST})

    @model_validator(mode="after")
    def _contact_below_slow(self) -> ClearanceZones:
        """The contact zone must stay below the slow zone, override included.

        The ladder in ``core_navigator`` is an if/elif chain ordered
        contact < slow < medium, so a contact threshold at or above SLOW_DIST
        does not widen the contact zone -- it makes the slow rung unreachable
        and silently deletes a tier. That is the same class of failure the
        speed ladder's ``_challenge_tiers_below_challenge_cap`` exists to
        catch, and it reads as tuning while measuring nothing.
        """
        contact = self.OBSTACLES_CONTACT_DIST
        if contact is not None and contact >= self.SLOW_DIST:
            msg = (
                f"clearance.OBSTACLES_CONTACT_DIST ({contact}) must stay below "
                f"clearance.SLOW_DIST ({self.SLOW_DIST}); the ladder is an ordered "
                "if/elif chain, so an equal or larger contact zone makes the slow "
                "tier unreachable instead of widening the contact one."
            )
            raise ValueError(msg)
        return self


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
        YAW_GAIN_COMPENSATION: Fraction of the geometrically predicted yaw the
            chassis actually delivers, divided out of the pure-pursuit steering
            demand. Pure pursuit is a bicycle model and assumes the plant turns
            exactly as predicted; ``RobotSpecs.YAW_GAIN`` says it delivers 0.55
            of that, so every demanded curvature comes out ~1.8x too wide. That
            is the leading candidate for the systematic OUTWARD displacement at
            sign passes (+7.53 cm median, 84% outward, against 5.6 cm of plan
            margin), which is concentrated on boundary signs near corners where
            the curvature demand is largest.

            ``bay_exit`` already applies ``YAW_GAIN`` in its dead reckoning and
            ``corridor_follower`` sizes its corner arc with it; pure pursuit was
            the one consumer ignoring it.

            Defaults 1.0 = OFF, the uncompensated geometric answer, because
            asking for 1.8x more steering everywhere is not a free change: it
            saturates against the steering limit sooner. Measured 2026-09-05,
            it helps one challenge and hurts the other, so it is applied PER
            CHALLENGE -- see ``OBSTACLES_YAW_GAIN_COMPENSATION``. Score it on
            SIGNED RADIAL at passes, not |error| and not pass-side counts,
            which are too coarse to screen on.
        OBSTACLES_YAW_GAIN_COMPENSATION: ``YAW_GAIN_COMPENSATION`` for the
            Obstacles Challenge only. ``None`` keeps the base value.
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
        LOOKAHEAD_BLEND_START: Fraction of either arming threshold at which the
            lookahead begins sliding from LOOKAHEAD_LONG toward
            LOOKAHEAD_SHORT, reaching short at the threshold itself. 1.0
            restores the original hard switch.

            The switch used to be a bare ``value > threshold`` step on both
            signals. In normal driving both hover near their thresholds, so the
            lookahead flipped long/short on consecutive ticks -- hardware
            run_20260829_104641 shows ``0.320, 0.160, 0.320, 0.160`` at
            ~2.5 Hz. Curvature is ``2y/L**2``, quadratic in the lookahead, so
            every flip swung the commanded curvature by 4x and the chassis
            drew a visible zigzag down the corridor. Steering effort rose with
            it: mean peak |steer| through corners 0.306 -> 0.398 against the
            pre-change baseline, while crosstrack did not improve.

            Ramping instead of switching removes the discontinuity rather than
            debouncing it -- there is no 4x jump left for hysteresis to
            suppress, and the response becomes proportional to how far off the
            robot actually is.

            0.7 keeps a deadband: below 70% of the threshold the long lookahead
            is untouched, so straights stay exactly as calm as before and only
            the approach to the limit tightens. It is also the largest value
            that leaves every pre-existing lookahead assertion true (the two
            "must stay long" cases sit at 0.50 and 0.571 of their thresholds),
            which is deliberate -- this refines the behaviour without
            re-baselining the tests that pinned it.
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
    YAW_GAIN_COMPENSATION: float = Field(default=1.0, validation_alias=_alias("YAW_GAIN_COMPENSATION"))

    OBSTACLES_YAW_GAIN_COMPENSATION: float | None = Field(
        default=None, gt=0.0, validation_alias=_alias("OBSTACLES_YAW_GAIN_COMPENSATION")
    )
    """Compensation for the Obstacles Challenge only. ``None`` keeps the base value.

    The two challenges want OPPOSITE values, so one number costs whichever
    challenge does not get it. Measured 2026-09-05, both arms paired against
    the same seeded cases, full compensation (0.55) against the base 1.0:

    * **Obstacles, 256 corpus, 64 scenarios:** real wrong-side passes
      **46 -> 6** (of 301/311 signs actually passed), runs with a violation
      **33 -> 6** of 64, rule 9.21 terminations **9 -> 0**, collisions flat
      (14 vs 15), laps credited 131 -> 196. Pass-side is the dominant Obstacles
      failure mode and this is by far the largest move anything has made on it.
    * **Open, all 640 cases:** WORSE -- **638 -> 615**. Rule 9.21 goes 1 -> 15
      and nine more runs time out at 200 s. The 128-case screen showed only
      127 -> 124, understating the cost EIGHTFOLD; this is why Open decides on
      640, not on the screen.

    Rule 9.21 therefore moves in opposite directions by challenge, so
    "compensation causes reverse-runs" is not supported in either direction --
    something challenge-specific mediates it, and that is not yet understood.

    On the OBSTACLES side, unlike ``OPEN_LOOKAHEAD_LONG``: Open is the arm that
    must not regress (638/640), and leaving its resolution path on the base
    value keeps it bit-identical. That does mean this CAN shadow the base
    constant on an Obstacles sweep, the way ``OBSTACLES_CONTACT_DIST`` did --
    ``diag_base.load_tuning`` sets both fields together for that reason.

    HARDWARE TRANSFERABILITY IS UNSETTLED: in the sim the compensation is
    exactly correct because the plant IS ``yaw_gain = 0.55``. On the real robot
    the same 1.8x is unresolved between ``rear_steer_ratio`` and
    ``linkage_ratio`` and needs the bench test.
    """
    LOOKAHEAD_SHORT: float = Field(default=0.16, validation_alias=_alias("LOOKAHEAD_SHORT"))  # Close to corner
    LOOKAHEAD_LONG: float = Field(default=0.32, validation_alias=_alias("LOOKAHEAD_LONG"))  # Normal straight
    LOOKAHEAD_TRANSITION: float = Field(
        default=0.30, validation_alias=_alias("LOOKAHEAD_TRANSITION")
    )  # Crosstrack threshold
    STEER_KP: float = Field(default=1.2, validation_alias=_alias("STEER_KP"))  # Steering P-gain
    # 1.2 to match motion/pursuit.toml, not the 2.0 this carried before: bare
    # construction was running a rate 67% higher than anything the robot ships
    # with, so a weave measured on bare tuning was not measuring the robot.
    MAX_STEERING_RATE: float = Field(default=1.2, validation_alias=_alias("MAX_STEERING_RATE"))  # rad/s
    WALL_MARGIN_SAFETY_M: float = Field(
        default=0.03, validation_alias=_alias("WALL_MARGIN_SAFETY_M")
    )  # Kept clear of an outer wall
    MIN_LOOKAHEAD_TRANSITION_M: float = Field(
        default=0.10, validation_alias=_alias("MIN_LOOKAHEAD_TRANSITION_M")
    )  # Floor for the derived threshold
    LOOKAHEAD_BLEND_START: float = Field(
        default=0.70, validation_alias=_alias("LOOKAHEAD_BLEND_START")
    )
    CORNER_PREVIEW_DISTANCE_M: float = Field(
        default=0.80, validation_alias=_alias("CORNER_PREVIEW_DISTANCE_M")
    )  # Path distance previewed for an upcoming turn
    CORNER_TURN_THRESHOLD_RAD: float = Field(
        default=0.35, validation_alias=_alias("CORNER_TURN_THRESHOLD_RAD")
    )  # Heading change over that preview that counts as a corner

    OPEN_LOOKAHEAD_LONG: float | None = Field(
        default=None, validation_alias=_alias("OPEN_LOOKAHEAD_LONG")
    )
    """``LOOKAHEAD_LONG`` for the Open Challenge only. ``None`` keeps the base value.

    The two challenges want different values and the constant is otherwise
    shared, so shipping one number costs the other challenge. Measured
    2026-09-03 at 0.24 against the base 0.32:

    * **Open, 640 cases, paired:** mean sim time **-5.39 s/case** (-3440 s
      total; 530 faster, 94 slower, 14 identical) for **one** verdict, case 590
      ``ok -> incomplete``. Case 300 fails in both arms -- it is the known
      free-space creep deadlock, not this.
    * **Obstacles, 256 corpus, paired:** WORSE -- clean 21 -> 19, timeouts
      35 -> 40, laps-driven 321 -> 315, escapes/lap 59.6 -> 66.4. Sign-pass
      cross-track was IDENTICAL at 10.32 cm median in both arms, so the Open
      tracking gain does not reproduce here at all.

    Hence the override sits on the OPEN side: Obstacles keeps the shipped 0.32
    and its resolution path is untouched, so unlike ``OBSTACLES_CONTACT_DIST``
    this cannot shadow the base constant on an Obstacles sweep -- the failure
    that silently made ``diag_sign_sweep``'s ``escape-gate`` mode measure
    nothing. Only Open sweeps need to clear it.
    """

    def for_open_challenge(self) -> PurePursuitParams:
        """These parameters as the Open Challenge should run them.

        Returns ``self`` unchanged when no Open override is set, so the
        Obstacles path and the un-overridden Open path stay byte-identical.
        """
        if self.OPEN_LOOKAHEAD_LONG is None:
            return self
        return self.model_copy(update={"LOOKAHEAD_LONG": self.OPEN_LOOKAHEAD_LONG})

    def for_obstacles_challenge(self) -> PurePursuitParams:
        """These parameters as the Obstacles Challenge should run them.

        Returns ``self`` unchanged when no Obstacles override is set, so the
        Open path and the un-overridden Obstacles path stay byte-identical.
        """
        if self.OBSTACLES_YAW_GAIN_COMPENSATION is None:
            return self
        return self.model_copy(
            update={"YAW_GAIN_COMPENSATION": self.OBSTACLES_YAW_GAIN_COMPENSATION}
        )


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

    Per-challenge tiers (2026-08-30)
    --------------------------------
    ``SLOW/MEDIUM/FAST/MAX`` are the SHARED base ladder. Either challenge may
    override any of them via ``OPEN_*`` or ``OBSTACLES_*``, resolved by
    :meth:`for_open_challenge` and :meth:`for_obstacles_challenge`. The two
    prefixes are symmetric on purpose: neither challenge is privileged as "the
    default", so a future motor can retune either one without the other's values
    having to move into a prefix first.

    They live here, beside the base ladder, rather than in the
    ``navigation-challenges/`` overlay tree because the spread between tiers is
    a property of THIS DRIVETRAIN'S HEADROOM -- the same reason the ladder lives
    with the motor profile at all. A slower motor with no headroom to spare
    omits both prefixes and the two challenges share one ladder; that fallback
    is the point, not a degenerate case.

    The two challenges want opposite things. Obstacles degrades monotonically
    with speed (measured 2026-08-28: in-time 38/256 at 0.156 m/s, 16 at 0.50,
    9 at 0.60), so it wants the conservative rungs. Open's binding constraint is
    the 180 s round limit rather than sign clearance, so it can spend headroom.

    A tier above its own cap is INERT -- ``core_navigator`` clamps the selected
    zone to ``max_mps()``. Raising ``OPEN_FAST_MPS`` without raising
    ``OPEN_MAX_MPS`` therefore buys nothing and reports nothing, which is
    exactly how the removed ``for_obstacles()`` profile came to ship a
    measured-inert speed half. ``_challenge_tiers_below_challenge_cap`` now
    rejects that combination instead of letting it run.

    Attributes:
        MIN_MPS: Friction floor. A clamp on the others, not a tier itself.
        MAX_MPS: Upper bound on any tier.
        CREEP_MPS: Contact zone, and the heading limiter's floor.
        SLOW_MPS: Near obstacles.
        MEDIUM_MPS: Moderate clearance.
        FAST_MPS: Open track.
        OPEN_MAX_MPS: Open-Challenge cap. ``None`` -> use ``MAX_MPS``.
        OPEN_SLOW_MPS: Open-Challenge slow tier. ``None`` -> use ``SLOW_MPS``.
        OPEN_MEDIUM_MPS: Open-Challenge medium tier. ``None`` -> ``MEDIUM_MPS``.
        OPEN_FAST_MPS: Open-Challenge fast tier. ``None`` -> ``FAST_MPS``.
        OBSTACLES_MAX_MPS: Obstacles cap. ``None`` -> use ``MAX_MPS``.
        OBSTACLES_SLOW_MPS: Obstacles slow tier. ``None`` -> use ``SLOW_MPS``.
        OBSTACLES_MEDIUM_MPS: Obstacles medium tier. ``None`` -> ``MEDIUM_MPS``.
        OBSTACLES_FAST_MPS: Obstacles fast tier. ``None`` -> ``FAST_MPS``.

    ``MIN_MPS`` and ``CREEP_MPS`` deliberately have NO per-challenge form. The
    floor is stiction and the creep tier is a servo-slew budget argued in
    centimetres of travel (see above); neither becomes different because the
    robot is driving a different challenge with the same hardware.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_MPS: float = Field(default=0.0499, gt=0.0, validation_alias=_alias("MIN_MPS"))
    MAX_MPS: float = Field(default=0.156, gt=0.0, validation_alias=_alias("MAX_MPS"))
    CREEP_MPS: float = Field(default=0.1014, gt=0.0, validation_alias=_alias("CREEP_MPS"))
    SLOW_MPS: float = Field(default=0.117, gt=0.0, validation_alias=_alias("SLOW_MPS"))
    MEDIUM_MPS: float = Field(default=0.1326, gt=0.0, validation_alias=_alias("MEDIUM_MPS"))
    FAST_MPS: float = Field(default=0.156, gt=0.0, validation_alias=_alias("FAST_MPS"))

    CORNER_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("CORNER_MPS"))
    """Speed while a corner is PREVIEWED, independent of the clearance ladder.

    Corner speed is otherwise set reactively: forward clearance falls as a wall
    comes into range and the ladder steps down. But ``path_turn_ahead`` knows a
    corner is coming from the PLAN, metres before any wall is in LIDAR range, so
    a preview-driven tier can slow the chassis before the geometry forces it to.

    Evidence that the preview is the better signal: the first-lap corner cap is
    the only thing currently using it, and disabling it costs 15 cases on
    balanced128 (94 -> 79). It is gated to lap 1 purely because it was
    introduced as first-lap caution, not because later laps were measured and
    found not to want it.

    ``None`` falls back to ``slow_mps()``, which is what the first-lap cap has
    always used -- so leaving this unset preserves today's behaviour exactly.
    Setting it allows a corner tier BETWEEN slow and medium, which forcing
    ``slow`` cannot express.
    """

    OPEN_MAX_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OPEN_MAX_MPS"))
    OPEN_SLOW_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OPEN_SLOW_MPS"))
    OPEN_MEDIUM_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OPEN_MEDIUM_MPS"))
    OPEN_FAST_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OPEN_FAST_MPS"))

    OBSTACLES_MAX_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OBSTACLES_MAX_MPS"))
    OBSTACLES_SLOW_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OBSTACLES_SLOW_MPS"))
    OBSTACLES_MEDIUM_MPS: float | None = Field(
        default=None, gt=0.0, validation_alias=_alias("OBSTACLES_MEDIUM_MPS")
    )
    OBSTACLES_FAST_MPS: float | None = Field(default=None, gt=0.0, validation_alias=_alias("OBSTACLES_FAST_MPS"))

    _CHALLENGE_PREFIXES = ("OPEN", "OBSTACLES")
    """Every per-challenge override prefix, so the validator cannot fall behind
    the fields it is meant to police."""

    @model_validator(mode="after")
    def _challenge_tiers_below_challenge_cap(self) -> SpeedControlParams:
        """A per-challenge tier above that challenge's cap is inert -- reject it.

        ``core_navigator`` clamps the selected zone to ``max_mps()``, so a fast
        tier above the cap silently resolves to the cap. That failure mode has
        already shipped twice: the pre-2026-08-09 ladder where MEDIUM and FAST
        both resolved to the ceiling, and the removed ``for_obstacles()``
        profile whose FAST_SPEED 0.30 sat above a 0.156 ceiling. Both looked
        like tuning and were measuring nothing, and neither announced itself.
        """
        for prefix in self._CHALLENGE_PREFIXES:
            override_cap = getattr(self, f"{prefix}_MAX_MPS")
            cap = override_cap if override_cap is not None else self.MAX_MPS
            for tier in ("SLOW", "MEDIUM", "FAST"):
                name = f"{prefix}_{tier}_MPS"
                value = getattr(self, name)
                if value is not None and value > cap:
                    msg = (
                        f"speed.{name} ({value}) exceeds the {prefix.title()}-Challenge cap "
                        f"({cap}); core_navigator clamps every tier to max_mps(), so this "
                        f"tier would be silently inert. Raise {prefix}_MAX_MPS or lower the tier."
                    )
                    raise ValueError(msg)
        return self

    def _for_challenge_prefix(self, prefix: str) -> SpeedControlParams:
        """Apply one challenge's ``<PREFIX>_*`` overrides onto the base ladder.

        Returns ``self`` unchanged when that challenge defines no overrides, so
        a drivetrain with no headroom to spare shares one ladder across both
        challenges without any caller needing to know which case it is in.
        """
        overrides = {
            tier: value
            for tier in ("MAX_MPS", "SLOW_MPS", "MEDIUM_MPS", "FAST_MPS")
            if (value := getattr(self, f"{prefix}_{tier}")) is not None
        }
        return self.model_copy(update=overrides) if overrides else self

    def for_open_challenge(self) -> SpeedControlParams:
        """This ladder as the Open Challenge should run it."""
        return self._for_challenge_prefix("OPEN")

    def for_obstacles_challenge(self) -> SpeedControlParams:
        """This ladder as the Obstacles Challenge should run it.

        Not to be confused with the removed ``NavigationTuning.for_obstacles()``
        classmethod, which baked a whole tuning profile in code. This applies
        only the speed tiers a motor profile declares for itself.
        """
        return self._for_challenge_prefix("OBSTACLES")

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

    def corner_mps(self) -> float:
        """Speed while a corner is previewed. Falls back to the slow tier."""
        if self.CORNER_MPS is None:
            return self.slow_mps()
        return min(self.CORNER_MPS, RobotSpecs.MAX_SPEED_MPS)


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
