"""Blind-navigation tuning groups.

Covers corridor width estimation, corridor following, direction inference,
LIDAR pose search, and odometry/IMU fusion.
"""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning._shared import _alias


class CorridorEstimatorParams(BaseModel):
    """Blind corridor-width estimation parameters.

    Attributes:
        MIN_SAMPLES: Width readings a corridor must accumulate before its
            estimate is trusted. Readings are attributed to a corridor by
            heading, so a handful taken while the chassis is still swinging
            through a corner can land in the wrong one.
        PLAUSIBLE_WIDTH_MARGIN_M: How far outside the legal [NARROW, WIDE]
            band a measured width may still fall and be treated as a
            plausibility check failure rather than accepted. Beyond
            NARROW - this margin or WIDE + this margin, the inward ray has
            missed the inner block entirely (a corner), not just measured a
            noisy corridor.
        MAX_START_SAMPLES: Length of the rolling window of stationary width
            readings kept before the start button is pressed. Enough to
            outvote the narrow prior comfortably (the estimator needs
            MIN_SAMPLES agreeing readings), and at the 20 Hz control rate it
            spans the last second before the button is pressed. A window
            rather than a total, because the robot is often powered on well
            away from the track and only what it sees once placed should
            count.
        DECISION_BOUNDARY_M: Width (m) at which a raw measurement snaps to
            WIDE rather than NARROW. Defaults to the midpoint of the two
            legal widths (0.8 m against a 0.03 m LIDAR sigma, better than
            4-sigma either way), but is its own tunable rather than always
            being that midpoint -- shifting it off-centre would trade a
            false NARROW for a false WIDE without moving either corridor
            width.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_SAMPLES: int = Field(default=12, validation_alias=_alias("MIN_SAMPLES"))
    PLAUSIBLE_WIDTH_MARGIN_M: float = Field(default=0.25, validation_alias=_alias("PLAUSIBLE_WIDTH_MARGIN_M"))
    MAX_START_SAMPLES: int = Field(default=20, validation_alias=_alias("MAX_START_SAMPLES"))
    DECISION_BOUNDARY_M: float = Field(
        default=(CorridorDimensions.NARROW + CorridorDimensions.WIDE) / 2.0,
        validation_alias=_alias("DECISION_BOUNDARY_M"),
    )


class CorridorFollowerParams(BaseModel):
    """Blind corridor-following and corner-turn parameters.

    Attributes:
        TURN_CLEARANCE_M: Forward clearance (m) at which the corner turn
            begins. Must stay strictly below
            DirectionEstimatorParams.CORNER_CLEARANCE_M -- see the
            cross-group check on NavigationTuning.
        NARROW_TURN_CLEARANCE_M: Forward clearance (m) at which the corner
            turn begins instead of TURN_CLEARANCE_M, once the corridor
            currently being creep-followed reads as NARROW
            (CorridorWidthEstimator.classify_width). Must stay strictly below
            TURN_CLEARANCE_M -- see the cross-group check on NavigationTuning.

            The corner-opening signal direction inference hunts for becomes
            visible only once the chassis is close enough that the inner
            block's near edge -- set by the *cross* corridor's own width --
            is behind it. In a 1.0 m corridor that happens with ~0.4 m of
            forward clearance still in hand, well before TURN_CLEARANCE_M
            (0.60 m) commits the turn, so the direction gate gets a fair
            window while the chassis is still square. In a uniform 0.6 m
            corridor the near edge sits exactly at TURN_CLEARANCE_M's own
            value (both equal CorridorDimensions.NARROW), so the window is
            zero: the turn always commits at the same instant the opening
            would become visible, direction never settles, and the round
            spends its entire budget in blind creep. Confirmed by sweeping
            TURN_CLEARANCE_M down against the live sim/navigator: 0.60 never
            settles, 0.50 settles but only completes 1 of 3 laps, 0.40
            settles and completes 2 of 3 laps -- see
            open_challenge_narrow_corridor_root_cause_2026_08_15 for the full
            trace. 0.40 keeps a 0.10 m margin above _MIN_FORWARD_CLEARANCE_M
            (RobotSpecs.LENGTH, 0.30 m) so the corner-turn branch still fires
            cleanly instead of folding into the emergency back-off branch.
        CENTERING_GAIN_DEG_PER_M: Degrees of road-wheel steering per metre of
            lateral offset from the corridor centreline. Zero since 2026-08-22
            -- the blind creep holds heading and does not chase the centreline,
            because the lateral correction is what swings the chassis past
            ALIGNMENT_TOLERANCE_RAD and starves the direction gate. Left as a
            zeroed gain rather than deleted code so the branch survives for a
            chassis that needs it.
        HEADING_GAIN: Road-wheel steering angle per unit of heading error
            against the corridor axis -- dimensionless, since both sides are
            angles. Centring on offset alone is undamped: in a steered
            vehicle heading is the integral of steering and position the
            integral of heading, so the two are 90 degrees out of phase and
            proportional-on-position is an oscillator. Measured on real
            hardware 2026-08-07: 112 steering sign flips in 177 s, a 3.2 s
            limit cycle, 45% of ticks pinned at MAX_CENTERING_STEER_DEG,
            heading 30 deg off axis at the median. That is what starves the
            direction gate, which needs the chassis square to a corridor at
            the moment one side opens. Raising CENTERING_GAIN_DEG_PER_M
            cannot fix it and makes it worse (see MAX_CENTERING_STEER_DEG);
            the missing term is this one.
        MAX_CENTERING_STEER_DEG: Hard cap, in degrees of road-wheel angle, on
            the steering that gain may produce. Both are deliberately timid:
            the counter-phase four-wheel chassis responds violently, and
            oscillation swings the heading past the direction estimator's
            alignment gate, which then refuses every reading.
        MAX_CORNER_STEER_DEG: The angle the corner-turn and back-off branches
            steer AT -- not a cap they occasionally reach, but the value they
            command outright. Split from MAX_CENTERING_STEER_DEG on 2026-08-29
            because the two are sized by unrelated things and, after the
            simulator was calibrated, wanted opposite values.

            This one is fixed by GEOMETRY. The corner branch commits its turn
            at TURN_CLEARANCE_M of forward clearance, so the arc it drives has
            to fit inside that: radius = wheelbase / ((1 + REAR_STEER_RATIO) *
            YAW_GAIN * tan(angle)). At the shipped 13.75 deg and the
            bag-calibrated YAW_GAIN of 0.55 that radius is 0.706 m against a
            0.60 m commit clearance -- the turn is geometrically impossible, so
            the robot runs out of room mid-corner, noses into the outer wall
            and drops into the back-off branch, where it is never square to a
            corridor and the direction estimator can never settle. Measured
            over the 128-scenario Open corpus: every one of the 31 collisions
            was a run that never settled, and no run that settled collided.

            The sibling above stays timid because it is sized by STABILITY --
            the 2026-08-07 limit cycle -- which the geometry has no say in.
            Raising a single shared constant to satisfy this one necessarily
            loosened that one; the sweep showed the cost as `stuck` runs
            climbing monotonically past 24 deg.

    All three steering values are PHYSICAL road-wheel angles, not normalised
    commands. They were normalised ([-1, 1], i.e. fractions of full lock)
    until 2026-08-21, which meant every one of them silently re-scaled
    whenever the steering geometry was recalibrated: swapping to a 270 deg
    servo (55 deg -> 85 deg at the road wheel) multiplied the blind
    corridor-follower's authority by 1.55x without anyone editing a tuning
    file. That is the opposite of what the numbers above argue for -- the
    2026-08-07 measurement says this loop is already prone to oscillation,
    and the hardware change quietly pushed it further that way.

    Expressed in degrees the values mean the same thing on any chassis, and
    ``max_wheel_angle_deg`` goes back to being what it should be: a statement
    about the servo's reach, which changes where saturation happens and
    nothing else.
        STEER_CAP_FROM_COMMIT_DISTANCE: Scale the steering cap by the distance
            the branch actually commits at, instead of applying
            MAX_CORNER_STEER_DEG to all three.

            The geometry argument above fixes the cap against
            TURN_CLEARANCE_M = 0.60. Two other branches commit at shorter
            distances and inherited the same angle, so the turn they drive
            cannot fit for exactly the reason the 13.75 deg value could not fit
            0.60:

            ===================================  ========  =======================
            branch                               commit    radius at 21.25 deg
            ===================================  ========  =======================
            corner, wide corridor (anchor)       0.60 m    0.444 m  fits
            corner, NARROW_TURN_CLEARANCE_M      0.40 m    0.444 m  DOES NOT FIT
            back-off, MIN_FORWARD_CLEARANCE_M    0.30 m    0.444 m  1.5x too wide
            ===================================  ========  =======================

            Holding radius proportional to the commit distance gives
            ``tan(cap) = tan(MAX_CORNER_STEER_DEG) * TURN_CLEARANCE_M / d``,
            which reproduces 21.25 deg exactly at 0.60 m -- so the wide-corner
            case, the only one that has been measured, does not move -- and
            yields 30.2 deg at 0.40 m and 37.9 deg at 0.30 m. Note the ratio
            form cancels ``(1 + REAR_STEER_RATIO) * YAW_GAIN``, so it does not
            depend on those being right, only on the anchor being right.

            Measured 2026-08-31 over the 640-case Open space: all 18 collisions
            hit the OUTER wall and none hit the inner block, so the error is
            always "turned too late", never "too tight" -- there is clearance
            to spare on the inside and a tighter cap is the low-risk direction.
            Those runs spend 3943 of 6657 blind creep ticks (59%) in the
            back-off branch against 676 in the corner branch, which is why the
            back-off distance is the one that matters most here.

            Measured 2026-08-31 over the full 640-case space: 596 -> 612 ok,
            collisions 18 -> 3, zero new collisions, inner-block 0 -> 0, sim
            time mean -0.01 s. The 3 survivors are exactly the 3 runs that
            logged no corner-branch and no back-off ticks.

            37.9 deg is commandable: ``max_wheel_angle_deg`` = 85.0 on the
            current servo IS a true road-wheel angle (confirmed 2026-08-31; the
            previous servo's was 180). The remaining unknown is understeer --
            the chassis turns wider than the bicycle model at a given angle --
            but that error is in the safe direction, since it makes this cap
            under-correct rather than over-correct. NOT track-validated.
        CORNER_SPEED_SCALE: Fraction of creep speed while turning a corner
            blind, which is committed on one comparison rather than a plan.
        REVERSE_SPEED_SCALE: Fraction of creep speed while backing off.
        TURN_ARC_HALF_FOV_DEG: Half-width (deg) of the arc searched for a way
            through before committing to a corner turn. Wider than
            LidarSectorParams.DIRECTION_ARC_HALF_FOV_DEG on purpose -- that
            8 deg cone cannot tell a corridor that has ended from a chassis
            pointed obliquely at the wall beside it.
        TURN_OPEN_RANGE_M: If any bearing within that arc has at least this
            much room, the corridor has not ended and the turn is refused.
        CORNER_LEAK_MARGIN_M: Added to CorridorDimensions.WIDE to get the
            side-range limit past which a side has "opened" (leaked past the
            end of the inner block) and is no longer treated as a corridor
            wall to centre against.
        MIN_FORWARD_CLEARANCE_M: Back off when the wall ahead is this close.
            Defaults to the chassis length, but is its own tunable rather
            than that constant: the direction should have settled long
            before this -- measured, it resolves after about 0.8 m of travel
            with roughly 0.5 m to spare. Reaching here means it did not, so
            driving on into the corner with no plan is not an option.
            Stopping is not either, and used to be what happened. With no
            direction there is no plan to hand over to and nothing else is
            steering, so a stopped robot stays stopped: go_open_0020 sat at
            zero speed for 400 ticks with the wall 0.13 m away and the round
            expired around it. The corner branch (TURN_CLEARANCE_M) carries a
            comment warning of exactly that deadlock; this branch
            reintroduced it.
        MIN_REVERSE_CLEARANCE_M: Room needed behind before backing off is
            allowed. Defaults to the chassis length, but is its own tunable:
            backing blindly into whatever is behind trades one wall for
            another, and with less than this the robot is boxed at both ends
            and holding still is genuinely all that is left. The robot must
            never cover ground backwards: the round is driven in the
            direction drawn on the day, and a robot reversing down a
            corridor is going the wrong way regardless of which way it is
            pointing. Clearance recovers within a few ticks, at which point
            the forward branches take over again.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    TURN_CLEARANCE_M: float = Field(default=0.60, validation_alias=_alias("TURN_CLEARANCE_M"))
    NARROW_TURN_CLEARANCE_M: float = Field(default=0.40, validation_alias=_alias("NARROW_TURN_CLEARANCE_M"))
    # 44.0 deg/m, 0.767945 and 13.75 deg are exactly what the previous
    # normalised 0.8 / 0.8 / 0.25 resolved to against the 55 deg road-wheel
    # limit they were tuned at, so the conversion changed no behaviour on the
    # base chassis. HEADING_GAIN keeps its name because it keeps its meaning;
    # only its units stopped depending on the servo.
    # Zero since 2026-08-22: the creep does not centre at all, because centring
    # is what swings the heading past the direction estimator's alignment gate
    # and stops it settling. See the corridor_follower.toml comment for the
    # trace and the measurements. Was 44.0.
    CENTERING_GAIN_DEG_PER_M: float = Field(default=0.0, validation_alias=_alias("CENTERING_GAIN_DEG_PER_M"))
    HEADING_GAIN: float = Field(default=0.767945, validation_alias=_alias("HEADING_GAIN"))
    MAX_CENTERING_STEER_DEG: float = Field(default=13.75, validation_alias=_alias("MAX_CENTERING_STEER_DEG"))
    MAX_CORNER_STEER_DEG: float = Field(default=21.25, validation_alias=_alias("MAX_CORNER_STEER_DEG"))
    STEER_CAP_FROM_COMMIT_DISTANCE: bool = Field(
        default=True, validation_alias=_alias("STEER_CAP_FROM_COMMIT_DISTANCE")
    )
    CORNER_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("CORNER_SPEED_SCALE"))
    REVERSE_SPEED_SCALE: float = Field(default=0.6, validation_alias=_alias("REVERSE_SPEED_SCALE"))
    TURN_ARC_HALF_FOV_DEG: float = Field(default=15.0, validation_alias=_alias("TURN_ARC_HALF_FOV_DEG"))
    TURN_OPEN_RANGE_M: float = Field(default=1.00, validation_alias=_alias("TURN_OPEN_RANGE_M"))
    CORNER_LEAK_MARGIN_M: float = Field(default=0.35, validation_alias=_alias("CORNER_LEAK_MARGIN_M"))
    BAY_EXIT_REVERSE_M: float = Field(default=0.05, validation_alias=_alias("BAY_EXIT_REVERSE_M"))
    """How far to back straight out of the parking pocket before turning.

    The lot is 0.45 m along the wall against a 0.30 m chassis, so a centred
    placement has ~7.5 cm of slack at each end. This stays inside that with
    margin for an off-centre placement.

    Bounded by GEOMETRY at the top end, because there is no rear sensing on this
    mount -- backing until something appears is not an option here. Do not raise
    it toward 0.075 to buy a wider turn: the slack is only that large when the
    robot is exactly centred, and it is placed by hand.

    The exact value IS measured, and it is sharp. Swept on 16 scenarios after
    the 2026-08-29 kinematics calibration (``diag_bay_start.py``): 0.06 collides
    16/16 at 0.14 m, 0.05 travels a 1.10 m median with zero collisions, 0.04
    gives 1.02 m. 0.06 was the shipped default and was fitted against the
    pre-calibration simulator, which cornered 1.83x harder than the car -- one
    centimetre either side of this value is the difference between hitting a fin
    and not. Re-sweep after any change to ``yaw_gain`` or the wheel angle.

    This does NOT make the in-bay start work: every point in the sweep still
    ends stuck with 0 laps. It only stops the exit from being a collision.
    """

    BAY_EXIT_STEER_NORM: float = Field(default=1.0, validation_alias=_alias("BAY_EXIT_STEER_NORM"))
    """Steering magnitude for the swing out of the pocket, 0..1 of full lock.

    Was hardcoded at full lock. That is the worst available command here, and
    the geometry says so: at ``max_wheel_angle_deg`` the counter-phase 4WS
    turning radius is ``(wheelbase/2) / tan(angle)`` -- 8 mm at the shipped
    85 deg -- so the chassis spins about its own centre. The pocket is 0.20 m
    deep and the chassis diagonal is 0.357 m, so rotating in place is not
    something the pocket has room for. What is needed is TRANSLATION out of the
    opening, which is a wider arc, not a tighter one.

    Kept as a tuning field rather than a constant because the radius it implies
    depends on ``max_wheel_angle_deg``, which is NOT BENCH-VERIFIED (the servo
    profile's own comment records the previous estimate being wrong by 1.28x).
    A value tuned in sim against 85 deg does not transfer if the real lock is
    nearer 30 deg, where the same command gives a 0.165 m radius instead of
    8 mm. Re-sweep this once the wheel angle is measured.

    MEASURED INERT 2026-08-29, and left at full lock for that reason. Swept over
    0.2/0.4/0.6/0.7/1.0 against every reverse distance after the kinematics
    calibration: 0.4, 0.7 and 1.0 produce byte-identical distances, collisions
    and stuck counts. The reasoning above about turning radius is sound and the
    outcome still does not depend on it, so something downstream saturates
    before this reaches the wheels -- do not tune it expecting an effect, and do
    not trust it as an explanation for a change in behaviour, until that is
    found. ``BAY_EXIT_REVERSE_M`` is the only knob of the three that moves the
    result at all.
    """

    BAY_EXIT_REVERSE_STEER_NORM: float = Field(default=0.0, validation_alias=_alias("BAY_EXIT_REVERSE_STEER_NORM"))
    """Steering magnitude DURING the reverse leg out of the pocket, 0..1.

    0.0 backs straight, and 0.0 is what wins. The reasoning below argued the
    opposite and was refuted on 2026-08-29 once the simulator stopped cornering
    1.83x harder than the car: every non-zero value collapses to 0.02 m and
    stuck -- worse than the collision it was meant to avoid, and worse at 0.5
    than the straight reverse at every reverse distance swept. It is kept as a
    field, at 0.0, so the refutation stays recorded rather than being silently
    deleted along with the knob.

    The argument it was introduced on: the pocket opens SIDEWAYS, so a straight
    reverse buys room ahead of the nose and nothing on the axis that matters,
    and the following forward swing then drives into the front fin before the
    wheels have finished slewing to lock.

    Non-zero backs on a curve, applied with the sign INVERTED the way
    ``follow_corridor``'s reverse branch already does -- reversing swings the
    nose away from the steer direction, so steering toward the wall walks the
    nose out toward the open corridor. That gains lateral offset without any
    forward travel, which is the whole difficulty here. Plausible, and measurably
    not what happens.
    """

    BAY_EXIT_HOLD_STEER: bool = Field(default=True, validation_alias=_alias("BAY_EXIT_HOLD_STEER"))
    """Hold the forward leg's steering through the reverse leg instead of centring.

    The servo slews at ``MAX_STEERING_RATE`` (1.2 rad/s), so full lock takes
    1.24 s = 25 ticks, while a stroke bounded by ``BAY_EXIT_REVERSE_M`` lasts
    about 9. Commanding centre on the reverse throws the slew away every cycle:
    measured on an in-bay start, the wheels reach **30.9 deg of the 85 deg
    commanded** -- 36% of full lock -- ramping up for 9 ticks and back down for
    11, never arriving.

    That also explains why ``BAY_EXIT_STEER_NORM`` swept byte-identical at
    0.4/0.6/0.8/1.0: every command at or above 0.364 is clipped by the slew to
    the same achievable angle, and only 0.2 (17 deg, reachable within a stroke)
    behaved differently. The knob was never inert -- it was unreachable.

    Lengthening the stroke instead is not available: the servo needs ~0.14 m of
    travel and the pocket has 7.5 cm of slack. Holding costs no travel.

    Distinct from ``BAY_EXIT_REVERSE_STEER_NORM``, which applies the INVERTED
    sign and slews further still, to opposite lock (refuted 2026-08-29).
    """

    BAY_EXIT_FALLBACK_FRAMES: int = Field(default=0, ge=0, validation_alias=_alias("BAY_EXIT_FALLBACK_FRAMES"))
    """Ticks to give the configured exit before switching to the OTHER one. 0 = never.

    The two manoeuvres are complementary, each scoring 254/256 under the contact
    model where the other scores 0:

    | 256 corpus         | no-slide | sliding |
    |--------------------|----------|---------|
    | reverse-then-swing |  254/256 |   0/256 |
    | cycle              |    0/256 | 254/256 |

    The reverse-then-swing exit needs the rotation-clipping ratchet that contact
    provides; the cycle exit needs to be able to slide along a fin. Which
    applies to the real robot is unknown -- the simulator has no sliding, scales
    yaw with speed so a stationary chassis cannot move at all, and ignores a
    steering servo with 35-70x the torque needed to scrub a wheel in place.

    Rather than bet on one, run one and switch. The failing manoeuvre costs only
    its budget: neither damages anything (2 collisions in 256 either way, the
    rest simply stuck), so the trade is budget against coverage.

    Distinct from ``BAY_EXIT_MAX_FRAMES``, which hands control to
    ``CoreNavigator`` -- refuted, 0/64 at every budget, because the escape
    ladder cannot leave a pocket at all.
    """

    BAY_EXIT_CYCLE: bool = Field(default=True, validation_alias=_alias("BAY_EXIT_CYCLE"))
    """Use the alternating arc/straight-reverse exit instead of reverse-then-swing.

    A shuffle at CONSTANT steering magnitude cannot accumulate:
    ``dy/dtheta = sin(theta) / (k tan(delta))`` is independent of speed and its
    sign, so ``y`` is a state function of ``theta`` and any cycle returning
    theta returns y. The reverse-then-swing manoeuvre therefore only escapes by
    leaning on wall contact, whose turn-clipping breaks that conservation --
    and its 254/256 does NOT survive giving the contact model the ability to
    slide along a surface (0/256 with sliding).

    Asymmetric legs break the conservation without contact: steer the forward
    arc, back STRAIGHT. The reverse returns no rotation, so each cycle keeps
    the theta the arc won and buys back the room it spent.

    Forward leg ends on FORWARD CLEARANCE, not a fixed distance -- how far the
    arc runs before the fin is in the way depends on how far round the chassis
    already is.
    """

    BAY_EXIT_CYCLE_REVERSE_M: float = Field(default=0.09, gt=0.0, validation_alias=_alias("BAY_EXIT_CYCLE_REVERSE_M"))
    """How far the cycle manoeuvre's straight reverse runs before arcing again.

    Its OWN constant rather than sharing ``BAY_EXIT_REVERSE_M``, which the
    reverse-then-swing exit needs pinned at exactly 0.05 (0.055 scores 0/64
    there). The two manoeuvres want different values for the same-named
    quantity, and one field serving both is a landmine: tuning the cycle would
    silently break the fallback.

    0.09 exceeds the pocket's 7.5 cm of slack on purpose -- the stall backstop
    ends the leg on geometry, so the reverse uses ALL the room available at the
    current heading rather than a fixed stroke. Measured against 0.05: exit
    267 -> 229 ticks and laps>=1 29 -> 33, which is parity with the
    parallel-start control.

    Do not sweep it DOWN again: shortening this leg costs cycles far faster
    than it saves ticks within one. Measured 2026-09-03 at the shipped forward
    distance -- 0.06 -> 213 ticks, 0.04 -> 317, 0.03 -> 391, 0.02 -> 417. The
    leg does not even reach 0.09 (``rev_m`` 0.041, ended by the stall
    backstop), so the reachable stroke, not this bound, is what sizes it.
    """

    BAY_EXIT_GUARD_MIRRORS_REVERSE: bool = Field(
        default=True, validation_alias=_alias("BAY_EXIT_GUARD_MIRRORS_REVERSE")
    )
    """Mirror the lock on the guarded ratchet's reverse leg instead of holding it.

    **SHIPPED TRUE 2026-09-10 ON HARDWARE. Without it the manoeuvre is a
    PENDULUM, not a ratchet.** Measured with `diag_bag_bay_ratchet.py`, which
    segments the phase into legs by the sign of the commanded speed:

    | | held OFF | held ON |
    |---|---|---|
    | steer sign across a reversal | **HELD 111 / FLIPPED 0** | HELD 0 / FLIPPED 4 |
    | consecutive legs cancelling | **87%** (and 72% on a second run) | **0%** |
    | rotation spent / kept | 815-1070 deg / 2-7 | **75 deg / 71** |
    | reversals | 111-324 | **0, 2, 4** |
    | net travel, out of the bay | 0.000 m, **0/2** | 11-20 m, **3/3** |
    | bay duration | 25-126 s | **2.8 / 13.4 / 21.1 s** |

    Held OFF, forward legs turned +1.94 deg each and the reverse legs that
    followed turned -1.74, on the SAME lock -- the reverse retraces the forward
    arc and gives the rotation back. Mirrored, both directions turn the same
    way and the yaw ADDS (4 adds, 0 cancels).

    **This is why four other bay levers failed the same night.** Raising the
    speed out of the motor deadband (stall 56.1% -> 0.4%), budgeting the
    guard's coast from measured rather than commanded speed, tolerating 15 mm
    of predicted fin overlap, and lengthening the leg cap each moved their own
    metric and left the outcome untouched, because they change how far each
    swing goes and not that the swings cancel.

    CONFOUND, stated because the runs cannot separate it:
    ``BAY_EXIT_GUARD_MEASURED_COAST`` and ``BAY_EXIT_CLEARANCE_TOLERANCE_M``
    were also on for the 3/3. They stay INERT on master: the attribution here
    is MECHANISTIC -- the probe shows the steering sign flipping and the
    cancellation going to zero, which only this flag can do -- not merely an
    outcome that happened alongside them.

    The sim verdict that kept this off was already VOID before tonight (95% of
    the mirrored arm's rotation there was the contact resolver, and that model
    never slides along a wall), so nothing measured is being overturned -- only
    a refutation that had already been withdrawn.

    The wall ratchet holds ONE steering angle across both legs, which only
    rotates the chassis if the forward arc and the reverse that follows it do
    not retrace one another. That held while the model turned inside 0.015 m.
    Against the MEASURED 0.29 m floor
    (``chassis_has_a_minimum_turn_radius``, shipped in ``72e7172b``) it is
    false: same lock, same radius, opposite direction is the same arc walked
    backwards, and the net rotation is zero.

    **That is not a theory, it is the current state of the robot.**
    ``diag_bay_start.py`` over every committed Obstacles scenario: **16/16 out
    of the bay at ``72e7172b~1``, 0/16 at ``72e7172b``**, and 0/16 today with
    the manoeuvre burning 267 forward and 272 reverse legs in 600 ticks for
    1 cm of net progress. The parallel-start control arm is 16/16 in every one
    of those runs, so it is this manoeuvre and not general handling. Obstacles
    DEFAULTS to the in-bay start, so on the shipped configuration the robot
    cannot begin that round.

    Mirroring is the parallel-parking exit: the two arcs curve opposite ways, so
    rotation accumulates at ANY turn radius rather than depending on one small
    enough to pivot in the pocket. It is what ``_cycle_command`` already does,
    and what ``BAY_EXIT_CYCLE_REVERSE_STEER_NORM`` exists for.

    It is not free. Holding made the servo pause free because both legs asked
    for the same angle; mirroring swings the full arc at every reversal, and
    ``_begin_leg`` reinstates that standstill automatically because the budget
    is computed from the two angles rather than tuned. At full lock that is the
    whole 2 x MAX_WHEEL_ANGLE_DEG swing per leg.

    Do NOT reach for the turn radius instead. 0.29 m is measured and verified on
    hardware; the ratchet was tuned against a model that over-rotated 12.7x, so
    the ratchet is what is wrong.

    **THIS FIELD CANNOT BE SETTLED IN THIS SIMULATOR. It ships FALSE for want
    of evidence, not on a verdict.** Two measurements exist and NEITHER stands:

    * The "MEASURED AND WORSE" reading that used to sit here -- 0.05 m against
      0.97 m held -- predates ``966b2b36``, so both arms were scored on a dead
      reckoning that over-read outward travel 31x. VOID.
    * Re-measured on the corrected tree (2026-09-10) it looked like a clear win:
      0.0296 m of TRUE outward travel over 64 legs against 0.0027 m over 364
      held, and 13.10 degrees of rotation against 1.06. Then run again with
      ``--no-slide``: the rotation collapses to **1.48 degrees** and the outward
      travel to **0.0015 m**, BELOW the held lock's 0.0025. Roughly 95% of that
      apparent win was the simulator's slide-on-contact resolver, not the
      ratchet. VOID TOO.

    Which makes the contact model the blocker rather than the lock convention.
    The ratchet's entire mechanism is leaning on a wall, and
    ``sim_contact_model_does_not_slide`` measured this simulator's wall
    behaviour at 56x less progress than the real thing at 20 degrees, while
    ``bay_ratchet_does_not_rotate_on_hardware`` has the robot managing 17 deg in
    20 s against ~6 s here. An arm whose result is 95% supplied by that model is
    a statement about the model. MEASURE THE SLIP FIRST.

    Independent of the lock convention, the guard steering it is blind: on the
    sliding run it reported +50 mm of slack while the chassis touched a fin 17
    times, its dead-reckoned pose having drifted 28.7 mm against a
    ``BAY_EXIT_CLEARANCE_MARGIN_M`` of 1 mm. See
    ``BAY_EXIT_DR_USES_MEASURED_YAW``.
    """

    BAY_EXIT_DR_USES_MEASURED_YAW: bool = Field(
        default=False, validation_alias=_alias("BAY_EXIT_DR_USES_MEASURED_YAW")
    )
    """Seed the guard's dead-reckoned yaw from the MEASURED yaw each tick.

    ``_dead_reckon`` integrates its own ``_dr_yaw`` from wheel travel and then
    clamps it to ``_wall_feasible_yaw_rad(self._dr_out)``. That clamp is a
    SELF-FULFILLING PROPHECY. At the judges' placement the limit is 1.15
    degrees, and ``_dr_out`` only grows by ``step * sin(_dr_yaw)`` -- 2% of
    travel at that angle -- so the model cannot believe in rotation until it
    believes in outward travel, and cannot earn outward travel without
    rotation. Measured 2026-09-10 with the mirrored reverse: the chassis really
    turned **13.1 degrees** while the dead reckoning read about 2, an error of
    **-11.7 degrees**. Over a 0.15 m half-length that is ~30 mm of corner
    displacement, which is the whole 28.7 mm the guard's pose had drifted.

    The yaw does not need to be modelled at all. ``command`` already TAKES
    ``yaw_rad`` and ``_track_rotation`` already uses it -- ``rotation_complete``,
    the manoeuvre's own release test, has been reading measured yaw since it was
    written. So the manoeuvre knows it rotated 13.1 degrees and only the guard
    does not. This field closes that gap rather than adding a sensor.

    Deliberately NOT extended to ``_dr_out`` and ``_dr_along``: those are not
    observable from inside the pocket (``_fin_rects`` explains why, and at
    ``WALL_OFFSET`` 0.1 m against a 0.194 m chassis the wall-facing flank sits
    3 mm away, under the sensor floor). Yaw is the one term of the three that is
    measured, and per-axis drift says it is also the one that walks: over one
    mirrored run the along error stayed at +0.8 mm and the out error is the yaw
    error projected.

    **MEASURED, AND INERT ON THE OUTCOME. Ships FALSE.** It does what it says --
    the yaw error over a run falls from -11.7 degrees to a median of -0.0000 --
    and changes nothing else: 256 legs, the same veto pattern, the same
    -0.1970 m best exit, 0/16 either way. Kept and documented rather than
    dropped because it names where the remaining error is.

    Which is that ``_dr_out`` and ``_dr_along`` CANCEL between legs. Both
    integrate ``step * f(_dr_yaw)`` with ``step`` SIGNED, so a reverse leg
    subtracts what the forward leg added: measured per leg with the yaw exact,
    ``out`` reads 0.0002, 0.0005, 0.0002, 0.0006, 0.0002, 0.0007 -- oscillating,
    not ratcheting -- while the physics reached 0.0296. That is the free-space
    conservation ``_guarded_command``'s docstring says the WALL breaks, and the
    code clips the yaw against the wall without ever collecting the outward gain
    the clip produces. Do NOT patch that from the formula: the same run says 95%
    of the physics' outward travel came from the slide-on-contact resolver, so
    what the wall really pays out is not established here at all.
    """

    BAY_EXIT_ARC_STEER_NORM: float = Field(
        default=1.0, ge=0.0, le=1.0, validation_alias=_alias("BAY_EXIT_ARC_STEER_NORM")
    )
    """Steering magnitude both bay-exit legs hold, 0..1 of full lock.

    **FULL LOCK, and the reasoning that said otherwise was answering a different
    question.** Turn radius at 85 deg is 17 mm, so in FREE space the chassis
    pivots about its own centre and translates nothing -- which is why this
    shipped at 0.3 while the exit relied on arcing its way out. Under
    ``BAY_EXIT_CLEARANCE_GUARD`` the rotation is not free: the wall behind the
    pocket clips the yaw at 1.15 deg, and the manoeuvre's whole output is the
    outward creep the chassis makes while PINNED against that clip. What the
    angle buys is therefore how fast the yaw crosses from one side of the clip
    to the other at a leg change, and that crossing is dead distance -- the
    outward gain over it cancels by symmetry.

    The crossing costs ``2 theta_max L / (tan(delta) YAW_GAIN)``: 14.5 mm at
    0.3, 4.1 mm at 0.7, **0.6 mm at full lock**, against a leg the fin clearance
    bounds to roughly 12-20 mm. At 0.3 the crossing IS the leg, so nothing is
    ever pinned and the ratchet runs at a fifth of its rate. Measured 2026-09-04,
    solid walls, guard on, speed scale 0.35, over the committed set: 0.3 -> 0/4
    out of the bay after 8.09 m of shuffling and 0.03 m of outward travel, 0.6 ->
    0/4 after 7.38 m, **1.0 -> 3/4 out in 127 ticks**. The earlier standstill
    sweep (0.2 -> 203 ticks, 0.3 -> 195, 0.45 -> 199) measured the OPPOSITE-lock
    cycle, where this angle also sized the servo swing between legs; holding one
    angle makes that swing zero and the sweep with it.
    """

    BAY_EXIT_FORWARD_M: float = Field(default=0.08, gt=0.0, validation_alias=_alias("BAY_EXIT_FORWARD_M"))
    """How far the cycle manoeuvre's forward arc runs before backing up again.

    Bounded by GEOMETRY, like ``BAY_EXIT_REVERSE_M``, and for the same reason:
    the pocket cannot be sensed from inside it. A clean raycast at the bay pose
    puts the fin ahead at 0.215 m, but the live pipeline reads 0.05-0.13 m and
    fluctuates every tick -- the chassis sits 0.1 m from one wall and 3 mm from
    the other, so a forward-cone minimum measures its surroundings and its own
    noise. An earlier version gated this leg on that reading and the arc got one
    tick per cycle: 53 cycles, 0.1 deg of rotation, 0.192 m travelled.

    Deliberately LONGER than the 7.5 cm of slack at each end of the lot, which
    is what it was originally sized against. Overshooting hands the leg's end
    to the stall backstop (``BAY_EXIT_LEG_STALL_TICKS``) in the cycles whose
    nose meets a fin, and lets the ones that do not run further -- trading a
    guarantee from the lot's dimensions for a sensed one, which is the second
    reason that backstop stays at 6.

    Fewer, longer cycles is the ONLY lever that reaches the standstill: the
    servo pause is ``ceil(swing / (MAX_STEERING_RATE / CONTROL_HZ))`` = 8 ticks
    per leg CHANGE, so it is paid per cycle no matter how the legs are shaped.
    Measured 2026-09-03 over the 256 corpus with sliding contact, against 0.05:
    exit 229 -> 195 ticks (-15%), of which standstill 65 -> 57, one whole cycle
    removed. Out-of-bay held at 254/256 -- the same two scenarios, and in both
    the manoeuvre never RAN (``bay_exit_ticks`` 0, so
    ``direction_from_parking_bay`` did not recognise the pocket); the exit
    itself is 254/254. Laps within noise (clean laps>=1 127 -> 125, >=3 72 ->
    71, against a parallel-start control's 123/74) and collisions 8 -> 4.

    0.08 is a bracketed optimum, not a direction to push: 0.06 gives 213 ticks,
    0.09 gives 229, and 0.10 is byte-identical to 0.09 because above ~0.09 the
    bound goes inert and every leg ends on the stall backstop instead. Only
    meaningful with ``BAY_EXIT_CYCLE``.

    **READ THIS BEFORE TRUSTING THE NUMBERS ABOVE.** They are all measured with
    ``--solid-walls``, which lets the chassis grind along a fin and keep going.
    WRO does not: "the parking lot limitations cannot be touched by the robot.
    When they are touched, the robot is stopped and no points for the parking can
    be scored" (ruled 2026-09-03). Every leg here that ends on the stall backstop
    ends because a fin stopped it, so those runs model a rule violation as a
    success.

    In the simulator's DEFAULT model, where contact ends the run, this constant is
    **INERT**: 0.05 and 0.08 are byte-identical over all 256 scenarios (out of bay
    256/256, laps>=1 119, collided 95), because the manoeuvre is released by
    ``is_clear`` after ~22 ticks on a single forward arc and the reverse leg never
    runs. It is kept at 0.08 because it is free there and better under the other
    model, NOT because 229 -> 195 is a result that survives the rules.
    """

    BAY_EXIT_CYCLE_REVERSE_STEER_NORM: float = Field(
        default=0.0, ge=0.0, le=1.0, validation_alias=_alias("BAY_EXIT_CYCLE_REVERSE_STEER_NORM")
    )
    """Steering on the cycle manoeuvre's reverse leg, applied OPPOSITE to the arc.

    0 backs straight: the reverse returns no rotation, so the heading the arc
    won is kept but not added to. Non-zero is the classic three-point turn --
    backing with the wheels reversed swings the tail the other way, so the nose
    turns the SAME sense on both legs and heading accumulates twice as fast.

    The cost is servo travel. Alternating between +arc and -arc asks for twice
    the arc angle between legs, ~25 ticks at ``MAX_STEERING_RATE`` against a leg
    of ~11 at creep, so the commanded angle may never be reached. A flat sweep
    of this field means slew clipping, not indifference -- the same effect that
    made ``BAY_EXIT_STEER_NORM`` read as inert before ``BAY_EXIT_HOLD_STEER``.
    Lengthening the legs is bounded by the pocket's 7.5 cm of slack.

    Only meaningful with ``BAY_EXIT_CYCLE``. Distinct from
    ``BAY_EXIT_REVERSE_STEER_NORM``, which belongs to the reverse-then-swing
    manoeuvre.
    """

    BAY_EXIT_SPEED_SCALE: float = Field(default=0.35, gt=0.0, le=1.0, validation_alias=_alias("BAY_EXIT_SPEED_SCALE"))
    """Extra speed scale applied to BOTH cycle legs, on top of the corner/reverse scales.

    The pocket is short of stopping distance, not of speed. A leg ends by
    COMMANDING zero, but the drivetrain is a first-order lag with
    ``speed_response_tau_s`` = 0.35 s, so at the shipped creep the chassis coasts
    ``v * tau`` ~ 40 mm after the command. Measured 2026-09-03: the forward leg's
    bound fires at 26 mm of along-wall travel and the chassis carries on to 63 mm
    during the servo settle, which is what puts it into a fin -- 20 mm of leg
    against 40 mm of coast.

    No value of ``BAY_EXIT_FORWARD_M`` can fix that, because the bound governs
    where the robot stops COMMANDING motion, not where it stops. Stopping
    distance is linear in speed, so this is the constant that reaches it.

    Its OWN scale rather than lowering ``CORNER_SPEED_SCALE``, which the corner
    turn and the back-off branch also read -- slowing the bay exit must not slow
    ordinary cornering.

    The 2026-09-03 refutation is VOID: it was measured against the
    contact-bounded cycle, where the servo slewed while the leg ran, so halving
    the speed doubled the wheel angle reached per metre and the extra yaw grew
    the swept extent faster than the shorter coast shrank the travel. The
    guarded exit holds ONE angle, so that coupling is gone and only the coast
    is left.

    **Now the manoeuvre's sharpest cliff.** Measured 2026-09-04, guard on, full
    lock, solid walls, committed set: **1.0 -> 0/8 and the chassis never moves
    at all** (the coast alone exceeds the along-wall slack, so no leg is
    admissible and the guard correctly refuses), 0.5 -> 0/8 with 7 collisions,
    0.35 -> 7/8 out of the bay with 9.0 mm of fin clearance to spare,
    0.2 -> 7/8 with only 3.5 mm. 0.35 rather than 0.2 because they escape
    equally often and 0.35 keeps twice the margin against the dead-reckoning
    error the guard cannot see.
    """

    BAY_EXIT_SPEED_MPS: float = Field(default=0.1, ge=0.0, validation_alias=_alias("BAY_EXIT_SPEED_MPS"))
    """ABSOLUTE speed for the bay-exit legs. 0 keeps the inherited scaling.

    SHIPS AT 0.1, matching ``corridor_follower.toml``. It was 0.0 here until
    2026-09-09, so anything constructing the tuning bare fell back to the
    inherited 0.067 m/s -- the very speed the measurement below rejects.

    The manoeuvre otherwise takes the driving ladder's creep speed and scales it
    down twice, landing at 0.067 m/s -- and the drivetrain does not deliver that.
    Measured on run_20260906_181613 and _181839: the navigator commanded
    0.067 m/s on 876 of 882 ticks, never pausing longer than 0.1 s, while
    /motor/drive_speed read 0 deg/s on 92-97% of them, against the ~110 deg/s
    that speed implies on a 7 cm wheel. The chassis was not waiting between legs
    -- it was being asked for a speed below the motor's usable range and moved
    only when a leg broke static friction. Rotation over 27-44 s was -8.8, +1.6
    and -8.1 degrees.

    Absolute rather than another scale because what this manoeuvre needs is set
    by TORQUE against static friction at full lock, not by any relationship to
    cruising speed. Scaling a number that is already too small cannot fix it.

    The tension is real: the leg must STOP inside the pocket, the drivetrain
    coasts v * SPEED_RESPONSE_TAU_S, and the fin guard refuses any leg it cannot
    stop in time. The simulator has no deadband, so it moves at any commanded
    speed and reports 0.086 m/s colliding in 32/32 scenarios -- it cannot see
    the floor this constant exists to clear, and cannot choose the value. Only
    the robot can.
    """

    ASSUME_BAY_START: bool = Field(default=True, validation_alias=_alias("ASSUME_BAY_START"))
    """Begin an OBSTACLES round believing the robot was placed inside the bay.

    The rules allow two starts and the in-bay one is worth 7 points (1 lap
    required), so it is the one we intend to use -- see
    ``wro_2026_scoring_parking_and_bay_start``. This constant makes that
    intention the DEFAULT rather than something the robot has to recognise.

    Until now the pocket was entered only when ``direction_from_parking_bay``
    recognised it, and that function asks two questions: is forward blocked,
    and do the +/-90 deg rays read wall-on-one-side/open-on-the-other. The
    first is the real one. The second exists to name the travel DIRECTION --
    inner block on the left is counterclockwise -- and as a gate it is the
    fragile half: a single dropped side ray reads as open corridor, both sides
    then read open, and the function returns ``None``. The cost of that miss is
    not a slower start but a DEADLOCK, which is the whole reason
    ``direction_from_parking_bay`` exists: forward is 0.05-0.19 m against a
    fin, below the gate that authorises the creep, there is no rear sensing to
    reverse on, and nothing moves for the rest of the round.

    With this on, forward-blocked alone starts the exit. The direction is no
    longer needed at that moment -- ``BayExit`` ratchets against the outer wall
    without one, and the estimator settles normally once the chassis is clear.

    **Assuming wrongly is self-correcting, which is why the default is safe.**
    A parallel start is a start with forward clearance, and ``BayExit.is_clear``
    is exactly that same threshold, tested on the tick after this latches: the
    belief is dropped before a single command is issued. The failure this
    removes is silent and total; the failure it can introduce is one tick long.

    Off restores recognition-only entry, which is the arm every pre-2026-09-05
    Obstacles measurement was taken on.

    NOT yet measured on the corpus: in simulation the side-ray test passes in
    the scenarios that matter, so this flag is expected to be inert there and
    the numbers in ``BAY_EXIT_CLEARANCE_GUARD`` below still describe the exit.
    It is a HARDWARE robustness change -- the dropout it defends against is a
    property of the real LIDAR, not of the simulator.
    """

    BAY_EXIT_CLEARANCE_GUARD: bool = Field(default=True, validation_alias=_alias("BAY_EXIT_CLEARANCE_GUARD"))
    """Bound the cycle legs by PREDICTED FIN CLEARANCE instead of by contact.

    The shipped manoeuvre ends each leg on the stall backstop, which fires
    because a fin stopped the chassis. Rule 9.24.7 ends the round on that touch,
    so the leg-end signal is itself the violation -- measured 2026-09-03, the
    chassis penetrates a fin by 8.9 cm in 254/254 corpus scenarios.

    The distance bounds cannot fix it. ``BAY_EXIT_FORWARD_M`` and
    ``BAY_EXIT_CYCLE_REVERSE_M`` sweep BYTE-IDENTICAL at 0.02/0.04 because
    contact happens on the FIRST arc, before any bound applies: the straight
    reverse returns no rotation, so yaw accumulates monotonically, and a pocket
    that admits 1.15 degrees of yaw is exhausted long before the leg is.

    With this on, the manoeuvre dead-reckons its own pose in the bay frame from
    wheel odometry and the steering it commanded (slew included), models the two
    fins from ``ParkingLotSpecs``, and reverses the leg when the NEXT pose would
    come within ``BAY_EXIT_CLEARANCE_MARGIN_M`` of one. No LIDAR -- the pocket
    cannot be sensed from inside it -- and no fin contact.

    **Default since 2026-09-04, and it now supersedes both older exits rather
    than merely bounding one.** It is answered before either of them, so
    ``BAY_EXIT_CYCLE`` and the reverse-then-swing exit are only reachable with
    this off. What changed is that the guarded manoeuvre stopped trying to
    escape in free space, which is impossible -- see ``_guarded_command`` -- and
    started ratcheting against the OUTER WALL, which 9.18 permits. Measured on
    the committed set with solid walls, full lock and speed scale 0.35: **7/8
    out of the bay, 5/8 driving a lap, fins TOUCHED 0/8**, against 0/8 and a
    motionless chassis before. The two constants matter as much as the flag:
    at the previous arc it is 0/4 and at the previous speed scale it does not
    move.
    """

    BAY_EXIT_CLEARANCE_MARGIN_M: float = Field(
        default=0.001, ge=0.0, validation_alias=_alias("BAY_EXIT_CLEARANCE_MARGIN_M")
    )
    """Fin clearance the guard refuses to go below, in metres.

    Nominally absorbs the dead-reckoning error the guard cannot see: wheel slip,
    the servo's true angle versus the modelled slew, and the placement tolerance
    of the start pose. Only meaningful with ``BAY_EXIT_CLEARANCE_GUARD``.

    **Lowered 0.005 -> 0.001 on 2026-09-06, because the margin was OPENING A
    DEADLOCK.** Every leg is bounded by ``reach``, and there is a band of
    outward positions where a step of that size lands just under the margin in
    BOTH directions -- so both legs are refused while the modelled pose is still
    perfectly clear, nothing moves, the dead-reckoned pose never changes, and
    the refusal is permanent. Enumerated over the guard's state space, the band
    opens at ``_dr_out`` = 0.0365 m and is 6.0 mm wide at 0.05, and the ratchet
    drives ``out`` straight through it by design.

    Reproduced in the unit fixture: at 0.005 the ratchet freezes at
    ``out`` = 36.9 mm and the guard then refuses both legs for **795 consecutive
    ticks out of 900**. At 0.003 it still stalls (786). At 0.001 the longest
    unbroken block is **1** and the chassis leaves the pocket.

    The size is a real cost and a small one: on the committed set with solid
    walls the exit is 16/16 out of the bay at both values with **fins TOUCHED
    0/16**, and the true minimum fin clearance falls only 9.0 mm -> 5.6 mm.

    The DR error this was meant to absorb is not reachable by any value here
    anyway -- the error measured on run_20260906_192358 was **44 mm**, an order
    above the whole range, and it is bounded by ``_along_slack_m`` instead.

    0.0 also closes the band; 0.001 keeps a nonzero refusal for the case where
    the model is exactly on a fin face.
    """

    BAY_EXIT_LEG_STALL_TICKS: int = Field(default=6, ge=1, validation_alias=_alias("BAY_EXIT_LEG_STALL_TICKS"))
    """Ticks of no wheel travel that end a cycle-manoeuvre leg and start the other.

    The primary leg-end signal, ahead of distance or clearance. Wheel odometry
    stops accumulating exactly when the chassis is blocked, so a leg bounded
    only by distance runs forever once it jams: measured, the reverse leg backed
    6.5 cm onto the rear fin and then held for 174 ticks waiting for 0.05 m that
    could no longer arrive. Six ticks is 0.3 s at 20 Hz -- long enough not to
    trip on a momentary scrape, short enough that a jammed leg costs almost
    nothing. Only meaningful with ``BAY_EXIT_CYCLE``.

    **1 is a cliff**: a single motionless tick ends a leg, so no leg ever runs
    and the manoeuvre burns its whole budget without leaving the pocket -- 600
    ticks, 0/4 out of the bay, measured 2026-09-03. 2 and 3 work and are worth
    2 ticks of the ~195-tick exit, which is not a trade worth taking one step
    from that cliff on a constant that is also the jam backstop on hardware,
    where a real chassis has friction and noise this simulator does not.
    """

    BAY_EXIT_LEG_MAX_S: float = Field(default=0.5, gt=0.0, validation_alias=_alias("BAY_EXIT_LEG_MAX_S"))
    """Wall-clock seconds a GUARDED leg may run before the ratchet reverses anyway.

    ``BAY_EXIT_LEG_STALL_TICKS`` is the equivalent backstop for
    ``_cycle_command``, and ``BAY_EXIT_CLEARANCE_GUARD`` makes that path
    unreachable -- so on the shipped configuration the guarded leg had NO stall
    bound, NO distance bound and no time bound at all. It ended only when the
    predicted fin gap closed, and that prediction is dead-reckoned from wheel
    travel, so a leg whose wheel is not turning cannot generate the evidence
    that would end it. Measured 2026-09-08 over the session's 11 bay runs: legs
    reached 15.25 s and 68% of all bay-exit ticks sat in legs older than 2 s.

    A TIME bound rather than a stall bound because the failure is not that the
    wheel reads zero -- it is that nothing ends the leg when it does, and time
    is the one quantity that still advances when travel does not.

    0.5 s from the breakaway profile measured on the same session: stall rate
    by time since the last reversal runs 3.9% (0-0.2 s), 6.0% (0.2-0.5 s),
    13.5% (0.5-1.0 s), 57.7% (1-2 s), 99.7% (4-8 s). The chassis moves in the
    half second after a reversal and then stops, so a leg held past ~0.5 s is
    spending ticks it cannot convert into travel. The two runs that got out of
    the pocket are the two with the shortest legs (0.33 s, 0.36 s) and the most
    reversals (16, 19).

    Not free: every leg change costs the computed servo standstill
    (``_begin_leg``), so a shorter bound buys breakaway torque with settle
    ticks. Lower this only against a measured exit rate -- the bound that
    maximises reversals is not the one that maximises escapes.
    """

    BAY_EXIT_CLEARANCE_TOLERANCE_M: float = Field(
        default=0.0, ge=0.0, validation_alias=_alias("BAY_EXIT_CLEARANCE_TOLERANCE_M")
    )
    """Predicted fin OVERLAP the guard tolerates, in metres. Ships 0.0 = inert.

    Subtracted from ``BAY_EXIT_CLEARANCE_MARGIN_M``, so the effective threshold
    can go negative without loosening that field's ``ge=0.0``, which exists to
    stop a margin being set backwards by accident.

    **Why this knob rather than another speed.** Measured on hardware
    2026-09-10 (`run_20260910_212129`, at `bay_exit_speed_mps` 0.15 with
    `BAY_EXIT_GUARD_MEASURED_COAST` on): the guard refuses on a PREDICTED gap of
    **4 mm** against a **1 mm** margin, while the pose it predicts from is dead
    reckoned and **~29 mm wrong**
    ([[bay_guard_arbitrates_1mm_with_a_29mm_pose_error]]). It is arbitrating an
    order of magnitude below its own model's error, so a refusal carries no
    information -- and each one FLIPS the leg, paying a full servo swing:
    **324 legs averaging 0.06 s across 39.3 s, 814 deg of rotation for 5.3 net,
    zero net travel, out of the bay 0/1.**

    That also rules out the two levers tried before it on the same night.
    Commanding 0.10 stalls the wheel 56.1% of ticks; commanding 0.15 fixes that
    outright (encoder-zero 0.4%) but the guard then vetoes 38-71%; and
    budgeting the coast from measured rather than commanded speed moved the veto
    rate 39% -> 38%, i.e. not at all, because the coast term was never the
    binding one.

    **The risk is real and physical.** Tolerating predicted overlap means the
    chassis may touch a fin, and a fin here is the parking structure -- which a
    trial on this same night already knocked over. Trial small (0.010-0.020)
    with a hand on the robot, and read `bay_guard_gap_m` afterwards rather than
    trusting that nothing touched.
    """

    BAY_EXIT_GUARD_MEASURED_COAST: bool = Field(
        default=True, validation_alias=_alias("BAY_EXIT_GUARD_MEASURED_COAST")
    )
    """Budget the guard's stopping distance from the MEASURED wheel speed, not
    the commanded one. INERT pending a hardware trial.

    ``_guarded_command`` ends a leg when the reachable pose -- the step plus
    ``speed * SPEED_RESPONSE_TAU_S`` of coast -- would come inside
    ``BAY_EXIT_CLEARANCE_MARGIN_M`` of a fin. That ``speed`` is what was
    COMMANDED, and in this manoeuvre the two diverge badly.

    **Measured on hardware 2026-09-10** (`run_20260910_210503`, `_210538`, with
    `bay_exit_speed_mps` trialled at 0.15):

    | commanded | stall % | delivered p50 | coast budgeted | coast real |
    |---|---|---|---|---|
    | 0.10 (ships) | **56.1%** | ~0 | 35.0 mm | ~0 |
    | 0.15 (trial) | **0.0%** | **0.027** | **52.5 mm** | **9.5 mm** |

    Raising the command FIXES the deadband -- the wheel stops stalling
    entirely -- but delivery is only 18% of it, because the leg runs at
    ``|steer|`` 1.00 where the load is largest. So the guard budgets 52.5 mm of
    coast against an along-wall budget of 31-57 mm for a chassis that would
    actually coast 9.5, and vetoed **39% and 71%** of ticks: 306 and 195
    reversals, **zero net travel, out of the bay 0/2**.

    The manoeuvre is therefore squeezed from both sides -- a command too low to
    turn the wheel at all, and a command high enough that the guard forbids
    using it. Nothing in between exists to tune, which is why this is a code
    change and not another value.

    Takes ``min`` with the command so the estimate may only ever be SLOWER than
    commanded; it cannot approve a step on speed the chassis does not have.
    ``step`` stays command-based and so stays conservative. Note the estimate is
    a max over ``_GUARD_SPEED_WINDOW_TICKS``, not an instant reading, so the
    zero-travel standstill that opens every leg does not read as a stopped
    chassis.

    UNVERIFIED. `SPEED_RESPONSE_TAU_S` 0.35 is itself a SIM CALIBRATION
    (fitted so the simulator matched an mcap recording), never a measured
    drivetrain decay, and the bags still cannot check it -- 2 clean coast
    episodes even across 500 reversals, because the command never stays at zero
    long enough. Bench it.
    """

    BAY_EXIT_GUARD_OVERLAP_RECOVERY: bool = Field(
        default=True, validation_alias=_alias("BAY_EXIT_GUARD_OVERLAP_RECOVERY")
    )
    """Let a leg that IMPROVES a already-violated fin gap run, instead of refusing it.

    ``_predicted_gap`` takes the ``min`` over BOTH fins, so once the modelled
    body overlaps one, the gap is negative at every reach and the fin the
    manoeuvre is moving AWAY from vetoes the leg exactly as hard as the one
    ahead. Both legs refused means no travel, no travel means the dead-reckoned
    pose never changes, and the refusal is then permanent by construction.

    Measured on hardware bag ``run_20260906_192358``: 285 consecutive
    zero-speed ticks, **14.2 s of a 16.6 s exit**, frozen at a 44 mm overlap,
    net rotation 7.5 deg where the two escaping runs of the same session turned
    60-70. The manoeuvre had no way out -- ``_guarded_command`` is answered
    ABOVE the ``BAY_EXIT_FALLBACK_FRAMES`` switch, and that constant ships at 0.

    Off restores the plain margin test, which is the arm every pre-2026-09-06
    bay measurement was taken on.

    Expected INERT in simulation for the same reason ``ASSUME_BAY_START`` is:
    the overlap is fed by wheel slip against a chassis the wall is holding, and
    the contact model does not slip. It is a HARDWARE fix. It is also strictly
    narrower than it sounds -- it applies only where the margin test has already
    stopped describing the situation, and it still never approves a step that
    reduces the gap.
    """

    BAY_EXIT_GUARD_BLOCK_TICKS: int = Field(default=0, ge=0, validation_alias=_alias("BAY_EXIT_GUARD_BLOCK_TICKS"))
    """Unbroken ticks of the guard refusing BOTH legs before it hands over.

    The escape hatch the guarded exit did not have. ``_guarded_command`` was
    answered ABOVE the ``BAY_EXIT_FALLBACK_FRAMES`` switch, and that constant
    ships at 0, so a guard that trapped itself could not be timed out by
    anything -- ``BAY_EXIT_MAX_FRAMES`` (900, i.e. 45 s) fired in none of the
    2026-09-06 hardware runs, one of which stood still for 14.2 s of a 16.6 s
    exit and never left the pocket.

    A guard that is BOUNDING legs alternates block and motion; only one that has
    trapped itself blocks without interruption. 40 ticks is 2 s at 20 Hz, and
    the servo settle is budgeted separately and does not count here.

    Enumerated over the guard's own state space (86400 poses on a 5 mm / 1 deg
    grid): ``BAY_EXIT_GUARD_OVERLAP_RECOVERY`` clears **1200 of the 2440**
    both-legs-blocked poses, and the residue is a band where the modelled pose
    is CLEAR but every step lands just under the margin. No speed change reaches
    that band -- halving the leg speed takes the blocked set from 2440 to
    **12621**, because a shorter step cannot cross it -- so a handover is what
    is left.

    **Ships at 0, i.e. OFF, and that is a deliberate trade rather than caution.**
    Turning it on breaks the guard's defining invariant: with it at 40,
    ``test_guarded_exit_never_steers_the_modelled_pose_into_a_fin`` fails, and so
    does ``test_full_speed_guarded_exit_refuses_to_move_rather_than_touch``,
    whose whole point is that refusing to move IS the correct outcome when the
    coast exceeds the pocket. The handover buys motion by spending fin contact,
    and 9.24.7 ends the round on that touch.

    It is here, reachable and measured, because the hardware failure it answers
    is real and the alternative was a manoeuvre with NO timeout at all. The
    deadlock actually observed (run_20260906_192358) is the overlap class, which
    ``BAY_EXIT_GUARD_OVERLAP_RECOVERY`` clears without touching anything, so
    nothing needs this today. Turn it on only with a corpus measurement of
    TOUCHED alongside the escape count, and expect to trade one against the
    other.

    40 (2 s at 20 Hz) is the value to try first.
    """

    BAY_EXIT_OPEN_SIDE_SECTOR_DEG: float = Field(
        default=15.0, gt=0.0, le=90.0, validation_alias=_alias("BAY_EXIT_OPEN_SIDE_SECTOR_DEG")
    )
    """Half-width of the sector each side of +/-90 deg that scores the open side.

    15 deg. Measured over the 383 bay-exit scans of the three 2026-09-06
    hardware runs, ``_open_side_score`` is correct on 100% of ticks at this
    width with a worst-tick margin of 1.15x and a median near 10x.

    **Do not widen it far.** At +/-30 and +/-45 the sector reaches the pocket's
    END walls rather than the corridor, and a sector MINIMUM collapses outright
    there -- 23.5%/54.2%/50.8% at +/-30, and 2.4%/0.8% on two runs at +/-45.
    The valid-fraction-times-median score survives wider sectors (98.8-100%),
    but its worst-tick margin falls to 0.95x at +/-30, i.e. it inverts on some
    tick. 15 deg is the width at which both halves of the score are still
    reading wall against corridor.
    """

    BAY_EXIT_OPEN_SIDE_VOTES: int = Field(default=5, ge=1, validation_alias=_alias("BAY_EXIT_OPEN_SIDE_VOTES"))
    """Ticks to poll before latching which side of the pocket is open.

    The latch is correct and worth keeping -- the rays stop meaning wall against
    corridor as soon as the chassis rotates -- but taking it on tick 1 rests the
    whole round on the FIRST scan the node ever receives. That is also the one
    frame no diagnostic can see: on ``run_20260906_192315`` and ``_192424``
    recording began 2.6 s and 1.9 s after the exit did, so the deciding tick is
    absent from both bags.

    5 ticks is ~0.5 s at the ~10 Hz scan rate. The measured rule needs no votes
    at all (N=1 suffices on all three runs), so this buys robustness against a
    single corrupt frame at a cost the 180 s round does not notice. Set to 1 to
    restore the tick-1 latch.
    """

    BAY_EXIT_LATCH_DIRECTION: bool = Field(default=True, validation_alias=_alias("BAY_EXIT_LATCH_DIRECTION"))
    """Decide which side is open once, on the first tick, instead of every tick.

    ``open_is_left`` compares single rays at +/-90 deg. That reads wall against
    corridor only while the chassis is parallel to the outer wall; once it
    rotates, both rays point along the pocket at a fin apiece and the
    comparison is noise. A flip inverts the steering sign, so the escape
    becomes a re-entry.

    The side is a fact about the layout, not about the current pose -- the lot
    stands against the OUTER wall, so the opening faces the inner block, which
    is the LEFT of a counterclockwise lap and the RIGHT of a clockwise one
    (verified 64/64 against geometry, both directions). Nothing about it can
    change while the manoeuvre runs, so re-deriving it every tick can only
    introduce error.

    Measured before latching, over 64 in-bay starts: escapes flipped a median
    of 4 times and failures 6, and **all 9 runs that never flipped escaped**
    while 0 of 18 failures managed zero. Suggestive, not conclusive -- the
    distributions overlap, and a wedged chassis sits at exactly the angle that
    makes the rays ambiguous, so flips may be a symptom rather than a cause.
    """

    BAY_EXIT_LATCH_REVERSE: bool = Field(default=False, validation_alias=_alias("BAY_EXIT_LATCH_REVERSE"))
    """Commit to the forward turn once the reverse leg has finished, instead of re-testing it.

    The reverse gate compares ``reverse_start - travelled`` against
    ``BAY_EXIT_REVERSE_M``, and the FORWARD leg moves that quantity back below
    the threshold -- so the gate flips the manoeuvre straight into reverse
    again. Measured on 64 in-bay starts: the reverse leg, which covers 0.05 m
    at creep in about 11 ticks, instead consumed **265-353 ticks against
    235-294 forward**, with the progress figure pinned at 0.045-0.054 m in
    every run. The manoeuvre spends its whole 30 s alternating two opposed
    commands about a knife-edge, which is why the chassis barely rotates: it is
    not driving an arc, it is chattering.

    Latching makes the reverse a one-shot, so the turn is actually held long
    enough to describe an arc. Sequencing state that a sensor cannot re-derive
    is already why ``_reverse_start_m`` exists; this is the other half of it.
    """

    BAY_EXIT_TARGET_YAW_DEG: float = Field(default=70.0, gt=0.0, validation_alias=_alias("BAY_EXIT_TARGET_YAW_DEG"))
    """Rotation from the placement heading at which the exit has turned ENOUGH.

    The chassis is placed along the pocket; leaving it means rotating out of
    that and onto the corridor. Past roughly this much rotation the vehicle is
    aligned with the parking walls rather than across them, and further turning
    carries the nose back around toward the outer wall -- which is what
    run_20260906_112613 did after it had already turned out.

    A yaw threshold rather than a clearance one because IN THE POCKET YAW IS THE
    ONLY SIGNAL THAT WORKS. Forward clearance is the quantity the manoeuvre
    cannot measure there: the wall sits inside MIN_VALID_RANGE_M, so the arc
    reports nothing at all (see ``_nose_in_contact``), while the IMU is
    unaffected by how close the surface is.

    70 deg, not 90: the exit does not need to be square to the corridor before
    driving out, only clear of the pocket and pointing out of it, and the last
    20 deg are the ones taken closest to the far fin.
    """

    BAY_EXIT_CONTACT_DIST_M: float = Field(default=0.08, gt=0.0, validation_alias=_alias("BAY_EXIT_CONTACT_DIST_M"))
    """Forward clearance at or below which the bay exit treats the nose as touching.

    0.08 m sits above the readings a chassis in contact actually produces and
    below the 0.10-0.14 m the manoeuvre holds while merely close to the wall
    (measured across run_20260906_094342 and _112613). NO valid returns counts
    as contact regardless of this value -- see ``_nose_in_contact``.

    Deliberately NOT ``MIN_FORWARD_CLEARANCE_M`` (0.30), which asks a different
    question: that one is "is the pocket behind me", this one is "am I touching".
    A pocket the chassis is still inside satisfies neither.
    """

    BAY_EXIT_CONTACT_RECOVERY_TICKS: int = Field(
        default=0, ge=0, validation_alias=_alias("BAY_EXIT_CONTACT_RECOVERY_TICKS")
    )
    """Ticks of straight reverse commanded when the nose reads as touching.

    12 ticks is ~0.6 s at the node's 20 Hz, about 40 mm at the exit's commanded
    0.067 m/s -- roughly half the pocket's 7.5 cm of end slack, so the leg buys
    room to rotate without crossing the pocket it is trying to leave. Held for a
    fixed count rather than until the arc clears: the arc is SILENT in contact,
    so "reverse until it reads clear" would be waiting on the sensor that just
    went blind.
    """

    BAY_EXIT_MAX_FRAMES: int = Field(default=900, ge=0, validation_alias=_alias("BAY_EXIT_MAX_FRAMES"))
    """Ticks the bay-exit maneuver may hold control before handing over. 0 = forever.

    ``BayExit`` is the only maneuver in the stack with no give-up path.
    ``ParkController`` has ``max_frames``; escape recovery has
    ``MAX_ESCAPE_S`` and ``ESCALATE_AFTER_ATTEMPTS``. This one releases
    only when ``BayExit.is_clear`` reports forward clearance above
    ``MIN_FORWARD_CLEARANCE_M`` -- which cannot happen while the chassis is
    boxed in by a fin.

    Measured 2026-09-01 on an in-bay start: ``_exiting_bay`` was True for
    **600 of 600 ticks** and ``CoreNavigator`` reported ``not_yet_stepped`` for
    every one of them. The whole recovery repertoire -- retrace-reverse, K-turn
    escalation, pivot-out-of-wedge, side-correction -- sits behind a gate that
    never opens, so none of it has ever been tried from the pocket.

    Expiry takes the same path as a clean exit (see the ``is_clear`` branch in
    the simulator and ``track_navigator_node``): it must fall THROUGH to the
    direction-settle block, which rebuilds the path and calls ``replace_path``.
    Releasing without that hands the planner a stale plan still pointing at
    waypoint 0.

    Shipped 0 (unbounded) until 2026-09-06, and only the simulator read it --
    ``track_navigator_node`` never counted the ticks at all, so on hardware
    ``is_clear`` really was the sole release. That was survivable only while a
    no-return forward arc read as CLEAR, which released the manoeuvre by
    accident. Now that ``BayExit.is_clear`` correctly calls a blind arc BLOCKED,
    an unbounded exit can hold the chassis in the pocket for the entire round,
    so this ships non-zero and the node counts against it.

    900 ticks is ~45 s at the node's 20 Hz. The one measured hardware exit
    (run_20260906_094342) took 650 ticks including 24 s of net-zero shuffling,
    and a simulator exit takes ~127 -- so the budget is well clear of a healthy
    manoeuvre and bounds an unhealthy one inside the 180 s round. Expiry is
    logged at WARNING: it means the robot gave up rather than got out.
    """

    BAY_WALL_CLEARANCE_M: float = Field(default=0.20, validation_alias=_alias("BAY_WALL_CLEARANCE_M"))
    """How close a side ray has to be to count as "hard against the outer wall".

    Only used to recognise a start INSIDE the parking bay, where the direction
    is readable off the geometry without moving -- see
    ``_direction_from_parking_bay``. The lot is 0.20 m deep and the chassis
    0.194 m wide, so centred in the pocket the wall ray is ~0.10 m; 0.20 leaves
    room for an off-centre hand placement without reaching the ~0.5 m a
    corridor wall sits at when the robot is merely close to one.

    Paired with ``TURN_CLEARANCE_M`` on the opposite side, so the test is
    "pinned one side, open the other", not "near a wall".
    """
    MIN_FORWARD_CLEARANCE_M: float = Field(
        default=RobotSpecs.LENGTH, validation_alias=_alias("MIN_FORWARD_CLEARANCE_M")
    )
    MIN_REVERSE_CLEARANCE_M: float = Field(
        default=RobotSpecs.LENGTH, validation_alias=_alias("MIN_REVERSE_CLEARANCE_M")
    )


class DirectionEstimatorParams(BaseModel):
    """Blind travel-direction inference parameters.

    Attributes:
        ALIGNMENT_TOLERANCE_RAD: Maximum heading error against the nearest
            track axis (radians) for a side-ray reading to be trusted. Off
            axis the side rays cut a diagonal and read long for no good
            reason. Shared with
            :func:`~src.navigation.corridor_estimator.measure_corridor_width`,
            which gates the same side rays on the same geometry -- the two
            must agree or a scan can be trusted for width and rejected for
            direction. Deliberately not one of the ``heading`` zones: those
            modulate speed, and retuning speed must not move this gate.
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
        PLAUSIBLE_SPAN_THRESHOLD_M: Maximum sum of left + right ranges (m)
            that still represents one corridor. Exceeding this sum means one
            ray ran off-track into an adjacent corridor rather than both
            reading walls of the current one. The margin absorbs scanning
            slightly off-axis.
        MIN_VOTES: Agreeing observations DirectionEstimator requires before
            settling on a direction. Voting rather than trusting a single
            scan, for the same reason CorridorWidthEstimator does: a ray
            slipping past a block corner produces brief, clustered
            misreadings, and one of those arriving first should not decide
            the round.
        GATE_LOG_PERIOD_TICKS: Log one direction-gate verdict every this many
            unresolved blind-creep ticks, in track_navigator_node's
            direction-not-yet-settled logging. Diagnostic only: at the
            ~20 Hz control rate, logging every tick during a prolonged
            corridor-follower hold would flood the log; this keeps enough
            resolution to see which gate is refusing readings without
            drowning it out.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    ALIGNMENT_TOLERANCE_RAD: float = Field(
        default=math.radians(25.0), validation_alias=_alias("ALIGNMENT_TOLERANCE_RAD")
    )
    CORNER_CLEARANCE_M: float = Field(default=1.00, validation_alias=_alias("CORNER_CLEARANCE_M"))
    MAX_IN_TRACK_RANGE_M: float = Field(default=4.5, validation_alias=_alias("MAX_IN_TRACK_RANGE_M"))
    MIN_ASYMMETRY_M: float = Field(default=0.20, validation_alias=_alias("MIN_ASYMMETRY_M"))
    PLAUSIBLE_SPAN_THRESHOLD_M: float = Field(default=1.25, validation_alias=_alias("PLAUSIBLE_SPAN_THRESHOLD_M"))
    MIN_VOTES: int = Field(default=5, validation_alias=_alias("MIN_VOTES"))
    GATE_LOG_PERIOD_TICKS: int = Field(default=5, validation_alias=_alias("GATE_LOG_PERIOD_TICKS"))


class LocalizationParams(BaseModel):
    """LIDAR-based pose search (LidarLocalizer) parameters.

    Attributes:
        SEARCH_RADIUS_M: Half-width (m) of the initial pose search window.
        PASSES: Number of coarse-to-fine grid-search passes.
        GRID_POINTS: Candidates per axis per search pass.
        RESIDUAL_CLIP_M: Per-ray residual clipping distance (m) for the cost
            function (outlier rejection).
        MAX_SPEED_MPS: Upper bound on real motion between ticks, used to
            reject a candidate that implies impossible speed. Deliberately
            above the measured real top speed to leave headroom for a faster
            drivetrain later without this guard needing to move with it.

            That headroom was NOT enough. 0.25 was sized against the retired
            motor's 0.156 m/s; the current drivetrain measured ~0.58 m/s at
            max_duty=0.5 and ~0.9 m/s open-loop (2026-08-29 bench), so the
            guard sat BELOW the robot's real operating speed and silently
            froze pose whenever it drove quickly -- returning the prior
            position instead of the LIDAR match.

            The cost was not only bad localization. Pose was being used as the
            INDEPENDENT check on encoder distance, and on run_20260829_003233
            the two agreed to 0.5% (28.46 m encoder vs 28.62 m pose). Both
            were under-reporting -- the encoder because counts_per_rev was too
            high, pose because fast updates were discarded -- and the apparent
            agreement was taken as confirmation that the encoder was correct.
            Two measurements suppressed in the same direction are not
            corroboration. Raise this whenever the drivetrain gets faster.
        JUMP_CONFIRM_TOLERANCE_M: How close two consecutive ticks' rejected
            candidates must be to count as the same correction confirming
            itself (see LidarLocalizer._reject_implausible_speed).
        RELOCALIZE_COST_THRESHOLD: Mean clipped squared residual (m^2), over
            real returns only, above which the winning candidate is judged not
            to explain the scan at all -- i.e. the local search is in the wrong
            basin, not merely imprecise. See LidarLocalizer._fit_cost for why
            no-return rays are excluded. Measured by replaying three hardware
            runs from 2026-09-07 through the localizer: the two that completed
            three laps sat at a median 0.0097/0.0101 and a 99th percentile of
            0.0244/0.0134, and across 1945 scans NEITHER ever crossed 0.03.
            run_20260907_205830, whose estimate latched 1.5-3 m off at t=8.0 s
            and stayed there for 48 s, sat at a median 0.0432.
        RELOCALIZE_AFTER_SCANS: Consecutive scans over the threshold before a
            global relocalization fires. The two healthy runs never produced a
            streak of even 1, so this is headroom on top of headroom; it is
            what keeps a burst of dropouts or a transient ambiguity from
            triggering a jump the robot does not need.
        RELOCALIZE_GRID_STEP_M: Candidate spacing (m) of the global search's
            free-space grid. Replaying run_20260907_205830 seeded at its own
            corrupted pose, 0.03 recovered it in a single relocalization: the
            fit cost went 0.0432 -> 0.0101, matching the healthy runs, and the
            fraction of beams projecting off-track went 72.4% -> 0.0%.
        RELOCALIZE_ACCEPT_RATIO: How much better the global winner must fit
            before it is allowed to replace the local estimate, as a fraction
            of the local cost. Guards the case where the high cost means the
            WALL MODEL is wrong rather than the pose -- routine during blind
            operation, while corridor widths are still being estimated. Without
            it the balanced-128 Open sweep dropped 128/128 -> 127/128; the
            hardware rescue clears it with room to spare (0.0101 against a
            0.0432 local cost, a ratio of 0.23).
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SEARCH_RADIUS_M: float = Field(default=0.15, validation_alias=_alias("SEARCH_RADIUS_M"))
    PASSES: int = Field(default=4, validation_alias=_alias("PASSES"))
    GRID_POINTS: int = Field(default=5, validation_alias=_alias("GRID_POINTS"))
    RESIDUAL_CLIP_M: float = Field(default=0.25, validation_alias=_alias("RESIDUAL_CLIP_M"))
    MAX_SPEED_MPS: float = Field(default=0.60, validation_alias=_alias("MAX_SPEED_MPS"))
    JUMP_CONFIRM_TOLERANCE_M: float = Field(default=0.05, validation_alias=_alias("JUMP_CONFIRM_TOLERANCE_M"))
    RELOCALIZE_COST_THRESHOLD: float = Field(default=0.03, validation_alias=_alias("RELOCALIZE_COST_THRESHOLD"))
    RELOCALIZE_AFTER_SCANS: int = Field(default=15, validation_alias=_alias("RELOCALIZE_AFTER_SCANS"))
    RELOCALIZE_GRID_STEP_M: float = Field(default=0.03, validation_alias=_alias("RELOCALIZE_GRID_STEP_M"))
    RELOCALIZE_ACCEPT_RATIO: float = Field(default=0.5, validation_alias=_alias("RELOCALIZE_ACCEPT_RATIO"))


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
