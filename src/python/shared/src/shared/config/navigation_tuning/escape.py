"""Collision-recovery (K-turn, slalom, stuck-detection) tuning group."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning._shared import _alias
from shared.domain.steering import angle_rad_to_steering_norm


class EscapeManeuverParams(BaseModel):
    """Escape maneuver parameters for collision recovery.

    When collision risk is detected, the robot executes escape maneuvers
    (K-turn, slalom) to clear obstacles and resume navigation.

    Attributes:
        POSE_TRAIL_MIN_STEP_M: Spacing between recorded breadcrumbs on the
            pose trail a retrace-reverse follows. Thins a stationary or
            creeping robot's trail, which would otherwise fill the buffer with
            one position, without dropping resolution on a moving one. Note
            this is a distance-per-tick threshold and so interacts with speed:
            at 0.156 m/s and 20 Hz the chassis advances ~0.008 m per tick and
            the thinning bites, while at 0.234 m/s it advances ~0.012 m and
            nothing is thinned at all. A module constant until 2026-08-22,
            which meant the interaction was invisible and the value could not
            be moved with the profile that changed the speed
        POSE_TRAIL_LEN: Breadcrumbs kept, oldest evicted first. ~1.3 m of
            travel at the default spacing, comfortably more than any retrace
            distance worth driving
        REV_SPEED: Reverse speed during escapes (m/s, negative)
        REV_STEER_DEG: Road-wheel steering angle held while reversing out.
            Named ``REV_STEERING_SCALE`` until 2026-08-21, which was doubly
            wrong: it is not a scale (every call site uses it as
            ``steering = value * side_sign``, a magnitude) and it was
            normalised, so it meant 44 deg of road wheel on the bench-measured
            55 deg chassis and would have silently become 68 deg on a 270 deg
            servo
        K_TURN_MIN_S: Minimum K-turn duration, seconds (OBSTACLE risk)
        K_TURN_MAX_S: Maximum K-turn duration, seconds (CRITICAL risk)
        SLALOM_REVERSE_S: Time spent reversing during slalom, seconds
        SLALOM_FORWARD_S: Time spent forward turning during slalom, seconds
        STUCK_MOVE_THRESHOLD: Distance threshold to detect stuck (m)
        STUCK_TIMEOUT_S: Time without movement before declaring stuck
        SIDE_CORRECTION_STEER_DEG: Road-wheel steering angle for a side-threat
            correction. Physical degrees for the same reason as REV_STEER_DEG
        SIDE_CORRECTION_SPEED: Forward speed during a side-threat correction
        SIDE_CORRECTION_S: Duration of a side-threat correction, seconds
        ESCALATE_AFTER_ATTEMPTS: Consecutive escapes before escalating (longer
            duration, opposite side) instead of repeating an identical pulse
        ESCAPE_SIDE_COMMIT_ATTEMPTS: Consecutive escape attempts made toward one
            side before switching to the other. Escapes used to flip side on
            every attempt, so successive attempts rotated the chassis opposite
            ways and cancelled out -- measured on real hardware as 40 s of
            rocking in place with zero net translation. Committing to a side for
            more than one attempt is what lets a wedged robot actually walk out
        MAX_ESCAPE_S: Hard cap on any single escalated escape, seconds
        STUCK_CONFIRMATION_CHECKS: Consecutive below-threshold stuck checks
            required before StuckDetector declares the robot stuck
        STUCK_ESCALATION_PER_ATTEMPT_S: Time added to a stuck-reverse
            maneuver's duration per repeated stuck-escape attempt
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    POSE_TRAIL_MIN_STEP_M: float = Field(default=0.01, gt=0.0, validation_alias=_alias("POSE_TRAIL_MIN_STEP_M"))
    POSE_TRAIL_LEN: int = Field(default=128, gt=0, validation_alias=_alias("POSE_TRAIL_LEN"))
    REV_SPEED: float = Field(default=-0.20, validation_alias=_alias("REV_SPEED"))  # Reverse speed
    # 44.0 deg is what the previous normalised 0.8 meant at the bench-measured
    # 55 deg road-wheel limit, so this conversion changed no behaviour.
    ESCAPE_MIRRORS_REVERSE: bool = Field(
        default=False, validation_alias=_alias("ESCAPE_MIRRORS_REVERSE")
    )
    """Steer the OPPOSITE way on an escape's REVERSE leg.

    The same mechanism the bay exit had, outside the bay. Every escape
    manoeuvre is built from ``rev_steer_norm() * _escape_steer_sign_for_attempt()``
    -- the SAME sign for STUCK_FORWARD and STUCK_REVERSE, changing only after
    ESCAPE_SIDE_COMMIT_ATTEMPTS. On an Ackermann chassis a reverse at the same
    lock UNDOES the forward leg's rotation, so the pair is a pendulum rather
    than a k-turn.

    MEASURED 2026-09-11 over nine 2026-09-10 obstacles runs, segmenting legs by
    the sign of the commanded speed: of 243 consecutive forward/reverse pairs
    that steered on both legs, 208 (85.6%) held the SAME sign. In the gaps
    BETWEEN escapes the wheel travels 0.120 m of absolute path for 0.033 m of
    signed path and under 0.10 m of displacement in 34 of 52 intervals, at
    100% "driving" -- not stalled, not commanded to stop, just rocking.

    That is the same signature BAY_EXIT_GUARD_MIRRORS_REVERSE was shipped
    against on hardware (0 flips in 111 and 140 legs, 815-1070 deg of rotation
    for 2-7 net, 0/2 out of the bay; mirrored, 95% efficient and 3/3 out).

    Ships FALSE here and TRUE for Obstacles
    (``OBSTACLES_ESCAPE_MIRRORS_REVERSE``), for the same division of evidence
    that scopes ``K_TURN_FIT_REAR_GAP``: every measurement above comes from
    OBSTACLES bags, where the thing the chassis is rocking against is a pillar.
    Open's escapes fire in corners against WALLS (81% at 45-90 deg), a geometry
    none of this was measured on, and Open is at 638/640 -- it has more to lose
    from an unmeasured change than to gain.

    The simulator cannot screen either side of that: its contact model never
    slides along a wall, which is the situation an escape exists for. So the
    Obstacles default is set from bag evidence plus the bay's hardware result,
    and the first hardware round with it on is the A/B.
    """
    OBSTACLES_ESCAPE_MIRRORS_REVERSE: bool | None = Field(
        default=True, validation_alias=_alias("OBSTACLES_ESCAPE_MIRRORS_REVERSE")
    )
    """Obstacles-Challenge ``ESCAPE_MIRRORS_REVERSE``. ``None`` -> shared field.

    Resolved by :meth:`for_obstacles_challenge`. See ``ESCAPE_MIRRORS_REVERSE``
    for why the scope is Obstacles and not the whole tree.
    """


    SIDE_CORRECTION_BLENDS: bool = Field(
        default=False, validation_alias=_alias("SIDE_CORRECTION_BLENDS")
    )
    """Let a forward side correction BIAS the planned steering instead of replacing it.

    A latched manoeuvre makes the navigator return before the planner runs, so
    for its whole duration the router's deformation is not merely overridden --
    it is never computed. MEASURED 2026-09-11 on the two Obstacles rounds that
    wedged at one point, and again on the two that completed 3/3 laps but ran
    244 s and 303 s against a 180 s limit:

    | | in a manoeuvre | outside one |
    |---|---|---|
    | commanded steering IS the manoeuvre's | 100% | -- |
    | ``steer_target`` published | 7-8% | 98-99% |
    | steering agrees with the router's request | not computable | 98-100% |

    So neither layer is wrong. The planner steers correctly whenever it runs,
    and the correction is a reasonable reflex; they simply never run together,
    and alternate at 2.7-4.4x absolute-over-signed wheel travel while the
    chassis goes nowhere. ``side_correction`` dominates those windows at 184
    and 198 ticks against the k-turn's 22.

    SCOPED TO THE FORWARD CORRECTION ONLY. A k-turn or a stuck reverse is a
    real manoeuvre that needs the chassis to itself, and a side correction that
    has switched to reverse because it is already touching is one too -- the
    planner has no model for backing off a wall it is against. What qualifies
    is the 16.5 deg, 0.20 s nudge at 0.1 m/s, which never needed to own a tick.

    REFUTED, and kept only as the record of why. The scoping above is exactly
    what makes it inert: the forward nudge it qualifies is a rounding error in
    the population it was built for. Ticks running a ``side_correction``, split
    by the branch that produced them:

    | run | forward | already touching (reverse) |
    |---|---|---|
    | run_20260911_131459 | 44 (0.8%) | 550 (10.2%) |
    | run_20260911_132017 | 12 (0.3%) | 572 (13.1%) |
    | run_20260911_110404 | 0 (0.0%) | 143 (21.1%) |
    | run_20260911_110734 | 0 (0.0%) | 88 (16.8%) |

    Practically every side correction on hardware is the ``already_touching``
    reverse, which this deliberately excludes. So the flag changes 0.0-0.8% of
    driving ticks and cannot move the alternation it was written for.

    Widening it to cover the reverse branch is NOT the obvious next step: the
    planner genuinely has no model for backing off something the chassis is
    already against. The measurement it points at instead is WHY the robot is in
    contact 10-21% of the time at all, which is upstream of every manoeuvre.

    Ships FALSE for the original reason too: summing two steering signals can
    saturate the wheel or produce a curvature neither layer asked for.
    """

    ESCAPE_RESEEK_TURN_DEG: float = Field(
        default=90.0, ge=0.0, validation_alias=_alias("ESCAPE_RESEEK_TURN_DEG")
    )
    """Rotation across one manoeuvre past which the waypoint index is re-seeked.

    MEASURED 2026-09-11, and it cost all three evening rounds. A k_turn swung the
    chassis by 180-300 deg (yaw -161 -> +138, -170 -> +126, -11 -> +41), and
    afterwards ``_waypoint_index`` FROZE for 20-45 s. It never stepped BACKWARD,
    which is precisely why every backward-jump guard in this tree read clean; it
    stopped advancing, because advancing requires REACHING a waypoint the robot
    was now driving away from. Drawdown from peak lap progress: 220, 86 and 227
    deg -- 0.24 to 0.63 of a lap the WRONG way, at POSITIVE commanded speed, with
    ``crosstrack_error_m`` at 0.0-0.5 m the whole time so the tracker believed it
    was on-path.

    The localizer is NOT involved: out-of-track beam fraction reads p50 0.006-0.013
    on those runs against 0.005 on a clean 3-lap round, with yaw+180 and pose+0.5 m
    negative controls at 0.34-0.52 in the same query. The pose was honest; the
    robot really did turn around.

    90 deg because the threshold has to separate a manoeuvre that merely nudged
    the nose from one that changed which way the robot faces. Below 90 the target
    ahead is still ahead; past it the previous target is behind the chassis, where
    ``WaypointController`` abandons the curvature formula and saturates to FULL
    LOCK -- a U-turn attempt against a 0.29 m minimum radius inside a 1.0 m
    corridor.

    A side correction is 16.5 deg over 0.20 s and cannot reach this; a k_turn
    against an obstacle is what does. 0 re-seeks after EVERY manoeuvre, which is
    not what this is for -- a manoeuvre that did not turn the robot has nothing to
    re-aim.
    """

    REV_STEER_DEG: float = Field(
        default=44.0, validation_alias=_alias("REV_STEER_DEG")
    )  # Road-wheel angle while reversing
    K_TURN_MIN_S: float = Field(default=0.54, gt=0.0, validation_alias=_alias("K_TURN_MIN_S"))
    K_TURN_MAX_S: float = Field(default=1.08, gt=0.0, validation_alias=_alias("K_TURN_MAX_S"))
    SLALOM_REVERSE_S: float = Field(default=0.40, gt=0.0, validation_alias=_alias("SLALOM_REVERSE_S"))
    SLALOM_FORWARD_S: float = Field(default=0.50, gt=0.0, validation_alias=_alias("SLALOM_FORWARD_S"))
    STUCK_MOVE_THRESHOLD: float = Field(
        default=0.03, validation_alias=_alias("STUCK_MOVE_THRESHOLD")
    )  # 3cm movement threshold
    STUCK_TIMEOUT_S: float = Field(default=2.0, gt=0.0, validation_alias=_alias("STUCK_TIMEOUT_S"))
    # 16.5 deg == the previous normalised 0.3 at the 55 deg road-wheel limit.
    SIDE_CORRECTION_STEER_DEG: float = Field(default=16.5, validation_alias=_alias("SIDE_CORRECTION_STEER_DEG"))
    SIDE_CORRECTION_SPEED: float = Field(default=0.1, validation_alias=_alias("SIDE_CORRECTION_SPEED"))
    SIDE_CORRECTION_S: float = Field(default=0.20, gt=0.0, validation_alias=_alias("SIDE_CORRECTION_S"))
    ESCALATE_AFTER_ATTEMPTS: int = Field(default=3, validation_alias=_alias("ESCALATE_AFTER_ATTEMPTS"))
    ESCAPE_SIDE_COMMIT_ATTEMPTS: int = Field(default=2, ge=1, validation_alias=_alias("ESCAPE_SIDE_COMMIT_ATTEMPTS"))
    MAX_ESCAPE_S: float = Field(default=1.8, gt=0.0, validation_alias=_alias("MAX_ESCAPE_S"))
    """Hard cap on any single escalated escape, seconds. Raised 1.0 -> 1.8 (2026-09-07).

    An escape is terminated by ELAPSED TIME, and 1.0 s could not deliver the
    rotation the manoeuvre exists to produce. At `REV_SPEED` 0.2 m/s that is
    0.20 m of path, which at the chassis's MEASURED 0.29 m minimum turn radius
    (`simulation.MIN_TURN_RADIUS_M`) is **40 deg** -- against the 90 deg+ needed
    to clear a corner. Hardware agreed: over 90 escape episodes on the 09-07
    runs an escape achieved a median **27.4 deg**, 39% under 20 deg, and
    re-triggered up to 50 times because the same corner was still there. Escape
    windows covered 34-65% of every run.

    Swept over 240 runs at the honest turn radius (`diag_escape_duration.py`):

        MAX_ESCAPE_S  in-time  laps>=3  collided  stuck  timed
        1.0 (was)         26       28         8      1      9
        1.8 (now)         29       34        11      0      0
        2.3               31       33        11      0      0
        3.0               22       22        15      0      1

    **Timeouts collapse to ZERO at 1.8 s.** 2.3 s is not better once noise is
    allowed for, and 3.0 s is clearly worse, so the optimum is real rather than
    "longer is better". 1.8 is chosen as the shortest manoeuvre that clears them.

    COST: collisions 8 -> 11, and that is INTRINSIC -- it appears at every
    duration that clears the timeouts. Terminating on achieved YAW was tried to
    recover it and is REFUTED: a 60 deg target is byte-identical to off, because
    the escape never gets that far before the time cap binds. It under-rotates;
    it does not overshoot. The open candidate is terminating on the TRIGGERING
    OBSTACLE BEING CLEAR.

    `K_TURN_MIN_S`/`K_TURN_MAX_S` were scaled with it (0.54/1.08), so the
    manoeuvre that spends this budget can actually reach it.
    """
    STUCK_CONFIRMATION_CHECKS: int = Field(default=3, validation_alias=_alias("STUCK_CONFIRMATION_CHECKS"))
    STUCK_ESCALATION_PER_ATTEMPT_S: float = Field(
        default=0.10, gt=0.0, validation_alias=_alias("STUCK_ESCALATION_PER_ATTEMPT_S")
    )
    K_TURN_FIT_REAR_GAP: bool = Field(default=False, validation_alias=_alias("K_TURN_FIT_REAR_GAP"))
    """Truncate a reversing escape to the rear room the LIDAR actually measures.

    ``K_TURN_MIN_S``/``K_TURN_MAX_S`` pick the reverse DISTANCE from the
    severity of what is in FRONT: 0.54 s at OBSTACLE risk, 1.08 s at CRITICAL,
    which at ``REV_SPEED`` 0.20 m/s are 10.8 cm and 21.6 cm. Neither reads a
    single number about what is BEHIND, and the logic is inverted -- the more
    threatening the thing ahead, the further the chassis commits backwards into
    space it never consulted.

    Measured over 46 escape episodes on the 09-10 bags: the rear gap is p50
    17 cm and p10 7 cm, and the reverse DID NOT FIT in 35% of them. The
    existing rear guard (``_reversing_into_unseen_wall``) does not catch this:
    it compares the gap at the START of the manoeuvre against ``CONTACT_DIST``,
    so a 17 cm gap authorises a 21.6 cm reverse and the chassis is driven into
    the obstacle it was escaping. The number needed to stop that is already
    computed every scan -- ``LidarClearances.back_m`` -- and simply never read.

    When set, the reverse is capped to the measured rear room less
    ``CONTACT_DIST``. It is a CEILING, not a replacement: front severity still
    proposes the duration and a reverse that already fits is untouched. A rear
    sector that measured nothing is left alone rather than capped to zero, so
    this cannot silently delete the manoeuvre on a mount with no rear slot.

    Ships FALSE here and TRUE for Obstacles (``OBSTACLES_K_TURN_FIT_REAR_GAP``).
    The evidence is entirely from Obstacles bags -- the thing behind the chassis
    at 7-17 cm is a pillar -- and Open has a specific reason to be left alone:
    its escapes fire in corners against WALLS, where shortening the reverse
    under-rotates and re-triggers, feeding the corner escape loop that already
    costs ~20% of runs against a 180 s budget. Nothing has been measured that
    says Open wants this.

    NOT sim-screenable: the contact model never slides along a wall, so it
    cannot represent a reverse that does not fit -- which is exactly the 35%.
    Bag measurement plus hardware.
    """

    OBSTACLES_K_TURN_FIT_REAR_GAP: bool | None = Field(
        default=True, validation_alias=_alias("OBSTACLES_K_TURN_FIT_REAR_GAP")
    )
    """Obstacles-Challenge ``K_TURN_FIT_REAR_GAP``. ``None`` -> use the shared field.

    Resolved by :meth:`for_obstacles_challenge`, which ``CoreNavigator`` calls
    once at construction -- the same shape, and the same discriminator
    (``sign_router is not None``), as ``ClearanceZones.OBSTACLES_CONTACT_DIST``
    and the speed ladder's ``OBSTACLES_*`` tiers.

    Asymmetric on purpose: there is no ``OPEN_*`` half, because nothing has
    been measured that wants Open to differ from the shared value, and an unset
    knob that nothing has ever moved reads as tuning that exists.
    """

    MIN_HISTORY_FOR_DISTANCE: int = Field(
        default=2, validation_alias=_alias("MIN_HISTORY_FOR_DISTANCE")
    )  # Poses needed before StuckDetector can measure distance travelled
    STUCK_HISTORY_FLOOR_S: float = Field(
        default=3.0, gt=0.0, validation_alias=_alias("STUCK_HISTORY_FLOOR_S")
    )

    @staticmethod
    def frames(seconds: float, control_hz: float) -> int:
        """Convert a duration into control ticks at ``control_hz``.

        Every escape duration is stored in SECONDS and converted here, rather
        than stored as a frame count. A frame count silently means a different
        thing at a different loop rate: at the shipped 20 Hz a
        ``stuck_timeout`` of 40 frames is 2 s, and at 50 Hz the same 40 frames
        is 0.8 s -- so raising CONTROL_HZ would make the robot declare itself
        stuck two and a half times sooner, truncate every escape, and give up
        on parking early, with nothing raising and no config edited.

        Rounds rather than truncates, and floors at one tick: a duration
        shorter than a single tick is still a maneuver the caller asked for,
        and zero frames would skip it entirely.
        """
        return max(1, round(seconds * control_hz))

    def k_turn_min_frames(self, control_hz: float) -> int:
        """Minimum K-turn duration in ticks (OBSTACLE risk)."""
        return self.frames(self.K_TURN_MIN_S, control_hz)

    def k_turn_max_frames(self, control_hz: float) -> int:
        """Maximum K-turn duration in ticks (CRITICAL risk)."""
        return self.frames(self.K_TURN_MAX_S, control_hz)

    def slalom_reverse_frames(self, control_hz: float) -> int:
        """Ticks spent reversing during a slalom."""
        return self.frames(self.SLALOM_REVERSE_S, control_hz)

    def slalom_forward_frames(self, control_hz: float) -> int:
        """Ticks spent forward-turning during a slalom."""
        return self.frames(self.SLALOM_FORWARD_S, control_hz)

    def stuck_timeout_frames(self, control_hz: float) -> int:
        """Ticks without movement before declaring the robot stuck."""
        return self.frames(self.STUCK_TIMEOUT_S, control_hz)

    def side_correction_frames(self, control_hz: float) -> int:
        """Duration of a side-threat correction, in ticks."""
        return self.frames(self.SIDE_CORRECTION_S, control_hz)

    def max_escape_frames(self, control_hz: float) -> int:
        """Hard cap on any single escalated escape, in ticks."""
        return self.frames(self.MAX_ESCAPE_S, control_hz)

    def stuck_escalation_per_attempt_frames(self, control_hz: float) -> int:
        """Ticks added to a stuck-reverse per repeated attempt."""
        return self.frames(self.STUCK_ESCALATION_PER_ATTEMPT_S, control_hz)

    def stuck_history_floor_frames(self, control_hz: float) -> int:
        """Minimum pose history the StuckDetector keeps, in ticks."""
        return self.frames(self.STUCK_HISTORY_FLOOR_S, control_hz)

    def rev_steer_norm(self) -> float:
        """Reverse-escape steering as a normalised command for the actuator.

        The stored value is a physical road-wheel angle, so this is the one
        place the servo's reach enters: a wider servo produces a SMALLER
        normalised command for the same 44 degrees, rather than the same
        command meaning a wider angle. Mirrors the ``*_mps()`` accessors on
        :class:`~shared.config.navigation_tuning.motion.SpeedControlParams`,
        for the same reason -- the conversion belongs next to the value, not
        repeated at four call sites that could each drift.
        """
        return angle_rad_to_steering_norm(math.radians(self.REV_STEER_DEG), RobotSpecs.MAX_STEERING_ANGLE)

    def for_obstacles_challenge(self) -> EscapeManeuverParams:
        """These parameters as the Obstacles Challenge should run them.

        Returns ``self`` unchanged when no Obstacles override is set, so the
        Open path and the un-overridden Obstacles path stay byte-identical.
        """
        overrides = {
            "K_TURN_FIT_REAR_GAP": self.OBSTACLES_K_TURN_FIT_REAR_GAP,
            "ESCAPE_MIRRORS_REVERSE": self.OBSTACLES_ESCAPE_MIRRORS_REVERSE,
        }
        # Dropping the Nones rather than writing them back is what keeps the
        # promise above: with every override unset this returns `self`, so the
        # Open path and an un-overridden Obstacles path stay byte-identical.
        set_overrides = {name: value for name, value in overrides.items() if value is not None}
        if not set_overrides:
            return self
        return self.model_copy(update=set_overrides)

    def side_correction_steer_norm(self) -> float:
        """Side-threat correction steering as a normalised actuator command."""
        return angle_rad_to_steering_norm(math.radians(self.SIDE_CORRECTION_STEER_DEG), RobotSpecs.MAX_STEERING_ANGLE)
