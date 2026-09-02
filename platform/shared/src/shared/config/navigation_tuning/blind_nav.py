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
    """

    BAY_EXIT_ARC_STEER_NORM: float = Field(
        default=0.3, ge=0.0, le=1.0, validation_alias=_alias("BAY_EXIT_ARC_STEER_NORM")
    )
    """Steering magnitude for the cycle manoeuvre's forward arc, 0..1 of full lock.

    Moderate on purpose. Turn radius is ``wheelbase / tan(delta)``: at full lock
    (85 deg) that is **17 mm**, so the chassis pivots about its own centre and
    translates nothing, which is the opposite of what a 0.20 m deep pocket
    needs. 0.5 is ~42 deg and ~0.21 m of radius, which actually moves the body
    sideways. Only meaningful with ``BAY_EXIT_CYCLE``.
    """

    BAY_EXIT_FORWARD_M: float = Field(default=0.05, gt=0.0, validation_alias=_alias("BAY_EXIT_FORWARD_M"))
    """How far the cycle manoeuvre's forward arc runs before backing up again.

    Bounded by GEOMETRY, like ``BAY_EXIT_REVERSE_M``, and for the same reason:
    the pocket cannot be sensed from inside it. A clean raycast at the bay pose
    puts the fin ahead at 0.215 m, but the live pipeline reads 0.05-0.13 m and
    fluctuates every tick -- the chassis sits 0.1 m from one wall and 3 mm from
    the other, so a forward-cone minimum measures its surroundings and its own
    noise. An earlier version gated this leg on that reading and the arc got one
    tick per cycle: 53 cycles, 0.1 deg of rotation, 0.192 m travelled.

    Sized against the 7.5 cm of slack at each end of the lot. A stall backstop
    (``BAY_EXIT_LEG_STALL_TICKS``) ends the leg early when it meets a fin
    first. Only meaningful with ``BAY_EXIT_CYCLE``.
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

    BAY_EXIT_LEG_STALL_TICKS: int = Field(default=6, ge=1, validation_alias=_alias("BAY_EXIT_LEG_STALL_TICKS"))
    """Ticks of no wheel travel that end a cycle-manoeuvre leg and start the other.

    The primary leg-end signal, ahead of distance or clearance. Wheel odometry
    stops accumulating exactly when the chassis is blocked, so a leg bounded
    only by distance runs forever once it jams: measured, the reverse leg backed
    6.5 cm onto the rear fin and then held for 174 ticks waiting for 0.05 m that
    could no longer arrive. Six ticks is 0.3 s at 20 Hz -- long enough not to
    trip on a momentary scrape, short enough that a jammed leg costs almost
    nothing. Only meaningful with ``BAY_EXIT_CYCLE``.
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

    BAY_EXIT_MAX_FRAMES: int = Field(default=0, ge=0, validation_alias=_alias("BAY_EXIT_MAX_FRAMES"))
    """Ticks the bay-exit maneuver may hold control before handing over. 0 = forever.

    ``BayExit`` is the only maneuver in the stack with no give-up path.
    ``ParkController`` has ``max_frames``; escape recovery has
    ``MAX_ESCAPE_FRAMES`` and ``ESCALATE_AFTER_ATTEMPTS``. This one releases
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

    Ships 0 (unchanged behaviour). Raising it is a real behaviour change on the
    in-bay start ONLY, which currently scores 0 laps in 64/64 scenarios.
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
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SEARCH_RADIUS_M: float = Field(default=0.15, validation_alias=_alias("SEARCH_RADIUS_M"))
    PASSES: int = Field(default=4, validation_alias=_alias("PASSES"))
    GRID_POINTS: int = Field(default=5, validation_alias=_alias("GRID_POINTS"))
    RESIDUAL_CLIP_M: float = Field(default=0.25, validation_alias=_alias("RESIDUAL_CLIP_M"))
    MAX_SPEED_MPS: float = Field(default=0.60, validation_alias=_alias("MAX_SPEED_MPS"))
    JUMP_CONFIRM_TOLERANCE_M: float = Field(default=0.05, validation_alias=_alias("JUMP_CONFIRM_TOLERANCE_M"))


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
