"""Waypoint generation geometry tuning group."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias
from shared.domain.enums import CorridorSide


class WaypointParams(BaseModel):
    """Waypoint generation geometry parameters.

    Attributes:
        ARC_RADIUS: Corner arc radius (m). Must exceed the Ackermann minimum
            turning radius (~0.329 m, from the measured WHEELBASE=0.19/
            MAX_STEERING_ANGLE=0.5236) — enforced by a fail-fast width
            assertion in ``calculate_waypoints``.
        DEDUPE_DISTANCE_M: Distance below which consecutive generated
            waypoints are treated as duplicates and merged.
        WIDE_CENTER_BIAS_M: How far (m) to shift corridor centreline waypoints
            off centre, for corridors ABOVE NARROW_WIDTH_THRESHOLD_M.
            Magnitude only -- which side it shifts toward is
            WIDE_CENTER_BIAS_SIDE, so the two can be tuned independently and a
            side can be A/B'd without touching the distance.
            Named WIDE_ rather than left bare since the 2026-08-29 narrow/wide
            split: a bare CENTER_BIAS_M read as "the" bias at every call site
            and would silently keep meaning that after the split, which is
            exactly the kind of name that hides a behaviour change.
        OBSTACLES_CENTER_BIAS_M: The same shift, for the Obstacles Challenge.
            Defaults 0.15 -- MORE inner bias than Open's 0.10, which is the
            opposite of what the geometry argues for. Read the measurement
            before changing it, because the geometric case is seductive and
            wrong.
            The geometric case: every Obstacles corridor is 1.0 m by rule and
            signs sit only 0.10 m either side of its centre, so a centred path
            leaves symmetric room and an inner bias spends clearance against
            the inner block -- the chassis was measured passing within
            0.005-0.116 m of that block while cornering, which correctly fires
            the reactive escape mid-corner, and that reverse-and-swing recovery
            is what reads as a U-turn on screen.
            The measurement says otherwise. Swept on subset64 (sighted, lane
            on) at 0.00/0.05/0.10/0.15/0.20/0.25/0.30: collisions
            25/18/18/**16**/17/17/33, laps>=3 39/46/46/**48**/47/47/31,
            in-time 18/31/30/**35**/33/21/12. Centred is the WORST arm tried;
            0.30 collapses on inner-block strikes, giving a clean peak at 0.15.
            Why: the chassis drifts OUTWARD while tracking (see
            centre_bias_is_tracking_not_sign_2026_08_09 -- a tracking defect,
            not a routing preference), so the DRIVEN line only lands near
            centre when the PLANNED line is pulled inward. Centring the plan
            puts the driven line ~0.10 m outward, i.e. onto the outer sign row
            at 0.40, and sign collisions rose 16 -> 25 accordingly.
            So this value is compensation, and it should be re-derived (likely
            downward, toward the geometric answer) if the outward drift is ever
            fixed. It is not evidence the drift is acceptable.
            Kept as its own field rather than a challenge branch on
            ``WIDE_CENTER_BIAS_M`` so Open's value can still be tuned without
            touching Obstacles, matching how ``CorridorDimensions``
            already carries a separate ``OBSTACLES_WIDTH``.
        NARROW_CENTER_BIAS_M: The same shift, for corridors at or below
            NARROW_WIDTH_THRESHOLD_M. Defaults 0.0 -- narrow corridors are
            planned down the true centreline while wide ones keep
            WIDE_CENTER_BIAS_M's inner racing line.
            Why split: the bias budget is ``width/2 - RobotSpecs.WIDTH/2``, so
            the same absolute shift costs a far larger FRACTION of a 0.6 m
            corridor than of a 1.0 m one. A single value has to be safe in the
            narrow case and therefore leaves lap time on the table in the wide
            one. Measured on run_20260829_020308 (3 laps, hardware): in the
            narrowest quartile the chassis ran a median 0.113 m and a p05 of
            0.069 m from the INNER wall against 0.435 m from the outer -- i.e.
            ~0.17 m off centre where 0.10 m was planned, because tracking error
            adds to the commanded bias rather than averaging out. The shipped
            WIDE_CENTER_BIAS_M comment already named this the condition for
            revisiting it ("the only reason to go further would be a
            measurement of real crosstrack against this geometry"); this is
            that measurement.
            NOT free: ``corner_arc_radius`` is ``max(W_entry, W_exit)/2 -
            center_bias``, so removing the bias WIDENS a narrow-to-narrow
            corner's arc (0.20 -> 0.30 m here) and makes it demand LESS
            steering, not more. Separation and corner sharpness are coupled
            through this one number and cannot both be raised by tuning it.
        NARROW_WIDTH_THRESHOLD_M: Corridor width (m) at or below which
            NARROW_CENTER_BIAS_M applies instead of WIDE_CENTER_BIAS_M. Defaults
            0.8, midway between the rule widths in
            ``CorridorDimensions.NARROW`` (0.6) and ``.WIDE`` (1.0), so it
            classifies both cleanly with the most room for measurement error
            on either side. A threshold rather than a smooth interpolation
            because the rules only ever present those two widths -- an
            interpolation would invent behaviour for widths the track cannot
            have.
        WIDE_CENTER_BIAS_SIDE: Which boundary WIDE_CENTER_BIAS_M shifts the
            path toward.
            Was fixed at OUTER and spelled into the constant's own name
            (OUTER_WALL_BIAS), which made the preference an assumption of the
            code rather than a setting. Clearance is symmetric either way --
            0.05 off centre leaves 0.353 m to the near boundary in a 1.0 m
            corridor and 0.153 m in a 0.6 m one, whichever side it is -- so the
            outward choice bought nothing and lengthened every lap, since a
            path further from the inner block is a longer way round.
        NARROW_CENTER_BIAS_SIDE: The same, for NARROW_CENTER_BIAS_M.
            Split from the wide side 2026-08-29, alongside the magnitudes: a
            shared side can only express "narrow hugs the same boundary, less
            far", and cannot reach the case the measurements point at.
            Narrow corridors are where the inner block is the binding
            constraint -- measured on run_20260829_020308, the chassis ran a
            median 0.113 m from the inner wall against 0.435 m outer -- so the
            useful direction to tune narrow is AWAY from it, i.e. OUTER, which
            a shared INNER side can only approach by passing through zero.
            Currently INNER with a 0.0 magnitude, so it is inert and the plan
            is centred; it exists so that trade can be made without a code
            change. Note the wide reasoning does NOT transfer: an outer bias
            costs lap time, and in a narrow corridor it buys margin where the
            margin actually is.
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
    # 0.10 to match waypoint/waypoints.toml. These field defaults are a second,
    # independent copy of the shipped values and had drifted to half of it: a
    # bare NavigationTuning() planned a line 5 cm off where the checked-in
    # config puts it, which is 5% of an Obstacles corridor. Nothing failed --
    # every diagnostic that builds tuning bare (diag_sign_sweep.tuning() among
    # them) just quietly measured a different car. Keep in step with the TOML;
    # test_navigation_tuning.py::test_field_defaults_match_shipped_toml enforces it.
    WIDE_CENTER_BIAS_M: float = Field(default=0.10, validation_alias=_alias("WIDE_CENTER_BIAS_M"))
    NARROW_CENTER_BIAS_M: float = Field(default=0.0, validation_alias=_alias("NARROW_CENTER_BIAS_M"))
    UNCONFIRMED_WIDTH_INNER_BIAS_M: float = Field(
        default=0.05, ge=0.0, validation_alias=_alias("UNCONFIRMED_WIDTH_INNER_BIAS_M")
    )
    """Inner bias for a narrow corridor still on the PRIOR rather than a measurement.

    **Measured over the FULL 640-case Open space 2026-08-31: 618 -> 631 ok,
    collisions 1 -> 0, incompletes 21 -> 9, sim time -12.80 s mean** (555 of the
    610 cases that finished in both arms were faster; 21 fixed against 8
    regressed). Every gain AND every residual failure sits in a mixed-width
    layout -- the uniform ones (nnnn, wwww) do not move at all, which is the
    fingerprint of a width-belief step rather than a general handling change.
    0.0 restores the previous behaviour exactly.

    NOT track-validated, and the simulator weaves 1.6-3.3x more than hardware,
    so the magnitude is expected to shrink there even though the mechanism
    below was measured on real runs.

    A blind round begins believing every corridor NARROW, and both hypotheses
    share the fixed OUTER wall, so the whole width error lands as a lateral
    displacement of the planned line:

    ```
    believed narrow (0.6), NARROW_CENTER_BIAS_M 0.0   ->  cy = MAX - 0.30
    confirmed wide  (1.0), WIDE_CENTER_BIAS_M   0.10  ->  cy = MAX - 0.60
                                                  step  =  0.30 m
    ```

    That step is measured, not derived: ``REPLAN_BLEND_TICKS``' own docstring
    records every first-lap width update stepping crosstrack by ~0.30 m in a
    single 50 ms tick across six hardware runs -- ten times the ~0.03 m the
    chassis can physically travel in that time -- which threw heading error
    past ``heading.CRAWL`` and pinned the limiter for 82-100% of the following
    ticks. Reported independently from the driver's seat 2026-08-31 as a
    decaying zigzag on WIDE corridors during lap 1 only, which is exactly the
    signature: wide corridors are where the belief flips and where there is
    room to swing, and it is gone by lap 2 because the widths have settled.

    Pre-positioning the unconfirmed line inward shrinks the step to
    ``0.30 - value``. It cannot be cancelled: the narrow bias budget is
    ``width/2 - RobotSpecs.WIDTH/2`` = 0.203 m, so ~0.10 m of step is
    irreducible while the plan must also fit a corridor that may really be
    narrow. Assuming WIDE outright -- the trick ``CORNER_ARC_ASSUME_WIDE``
    plays on the corner arcs -- is NOT available here: it puts the line at
    MAX - 0.60, which is *on* the inner wall of a true narrow corridor. An arc
    sized from a wrong belief is a timing error; a centreline from one is a
    position error straight into a wall.

    A SEPARATE field rather than raising ``NARROW_CENTER_BIAS_M``, because
    that value is 0.0 on evidence and raising it would pay this cost in
    corridors that are genuinely narrow. Measured on run_20260829_020308, the
    chassis ran a median 0.113 m and a p05 of 0.069 m from the INNER wall in
    the narrowest quartile -- tracking error ADDS to the commanded bias rather
    than averaging out, so there is little room there to spend. This field
    spends it only while the corridor might not be narrow at all, over the
    ``corridor_estimator.MIN_SAMPLES`` = 12 readings (~0.5 m of travel) before
    the width resolves, and hands it straight back on a confirmed-narrow
    reading.

    Both branches then move by ``0.30 - value`` instead of 0.30:

    - confirms WIDE:   inward, onto the wide line
    - confirms NARROW: outward, into the side measured to have 0.435 m spare

    **The ceiling is CLEARANCE, and 0.05 is not known to be the optimum.**
    Measured at 0.15 on balanced128: **-16 cases** (18 worse, 2 better), almost
    all ``ok -> incomplete``. 0.15 leaves a corridor that really is narrow only
    ``0.6/2 - 0.15 - RobotSpecs.WIDTH/2`` = 0.053 m of inner margin, against
    the ~0.07 m of inward tracking drift measured on run_20260829_020308 --
    i.e. negative margin. The chassis is pressed toward the inner block, which
    reads as dithering and a lost round rather than a clean collision.

    This bias does NOT touch the corner arc, despite the coupling
    ``NARROW_CENTER_BIAS_M`` documents. ``calculate_waypoints`` passes
    ``corner_arc_radius`` a bias computed WITHOUT ``confirmed=``, so the arc
    always sees the confirmed value and is invariant to this field at every
    magnitude (pinned by ``test_the_arc_is_invariant_to_this_field``). An
    earlier version of this docstring claimed 0.10 and 0.15 tightened the arc
    to 0.400/0.350 m and that raising the field required decoupling it first.
    That was wrong: the arc was already decoupled, and the 0.15 result is the
    clearance cost alone.

    So the practical ceiling is ``width/2 - RobotSpecs.WIDTH/2`` = 0.203 m less
    the tracking drift, i.e. around 0.13 -- and **0.10 is simply untested**. It
    was skipped on the mistaken coupling argument above, not on evidence. The
    dose-response measured either side of 0.05 (0.025 is -9 cases against it,
    0.0 is -13) is still climbing at 0.05, so a value between it and the
    clearance ceiling may well be better. Measure before assuming otherwise.

    Complementary to ``REPLAN_BLEND_TICKS``, which changes how the step is
    APPLIED rather than how big it is; that one is also due a re-test, having
    been judged while replanning was independently corrupting the waypoint
    index -- and now also while this field is shrinking the step it fades.
    """
    # 0.8 = midway between CorridorDimensions.NARROW (0.6) and .WIDE (1.0).
    # Spelled as a literal rather than computed from them for the same reason
    # every other default here is: this class is a second, independent copy of
    # the shipped TOML, and test_field_defaults_match_shipped_toml compares the
    # two directly.
    NARROW_WIDTH_THRESHOLD_M: float = Field(default=0.8, validation_alias=_alias("NARROW_WIDTH_THRESHOLD_M"))
    OBSTACLES_CENTER_BIAS_M: float = Field(default=0.15, validation_alias=_alias("OBSTACLES_CENTER_BIAS_M"))
    WIDE_CENTER_BIAS_SIDE: CorridorSide = Field(
        default=CorridorSide.INNER, validation_alias=_alias("WIDE_CENTER_BIAS_SIDE")
    )
    NARROW_CENTER_BIAS_SIDE: CorridorSide = Field(
        default=CorridorSide.INNER, validation_alias=_alias("NARROW_CENTER_BIAS_SIDE")
    )
    NUM_INTERMEDIATE_ARC_POINTS: int = Field(default=3, validation_alias=_alias("NUM_INTERMEDIATE_ARC_POINTS"))
    STRAIGHT_WAYPOINT_COUNT: int = Field(default=8, validation_alias=_alias("STRAIGHT_WAYPOINT_COUNT"))
    MAIN_LOOP_REACHED_DISTANCE_M: float = Field(default=0.20, validation_alias=_alias("MAIN_LOOP_REACHED_DISTANCE_M"))
    CONTROLLER_REACHED_DISTANCE_M: float = Field(default=0.01, validation_alias=_alias("CONTROLLER_REACHED_DISTANCE_M"))
    REPLAN_HEADING_TIE_MARGIN_M: float = Field(default=0.15, validation_alias=_alias("REPLAN_HEADING_TIE_MARGIN_M"))

    CORNER_CAUTION_ALL_LAPS: bool = Field(default=False, validation_alias=_alias("CORNER_CAUTION_ALL_LAPS"))
    """Apply the previewed-corner speed on EVERY lap, not only the first.

    ``FIRST_LAP_CORNER_CAUTION`` is gated to lap 1 because it was introduced as
    first-lap caution -- the lap that has never been driven. Later laps were
    never measured and found not to want it; they were simply out of scope.

    That gating is worth revisiting because corner behaviour, not straight-line
    speed, is what sets lap time on this track: measured 2026-08-30, the largest
    single time gain of the session came from fixing corner GEOMETRY
    (``CORNER_ARC_ASSUME_WIDE``, -7.9 s at unchanged speed), while every attempt
    to raise the speed ladder either lost cases or had to buy them back with
    earlier slowdowns that cost more time than the higher ceiling gained.

    Pairs with ``speed.CORNER_MPS`` so the tier can sit between slow and medium
    rather than being forced to ``slow``.
    """

    CORNER_ARC_ASSUME_WIDE: bool = Field(default=True, validation_alias=_alias("CORNER_ARC_ASSUME_WIDE"))
    """Size every corner arc as if both corridors were WIDE, ignoring the belief.

    **OPEN-CHALLENGE-ONLY IN EFFECT, without needing a gate.** The flag
    substitutes WIDE for the measured widths, and every Obstacles corridor is
    1.0 m by rule -- so on that challenge the substitution is the IDENTITY and
    the radius is unchanged (verified 2026-08-30 at both
    OBSTACLES_CENTER_BIAS_M and WIDE_CENTER_BIAS_M). An Obstacles A/B on this
    would return byte-identical arms by construction, which is the same dead-end
    that cost two 256-scenario runs on MAX_CORNER_STEER_DEG the same day.

    That equivalence holds only while Obstacles corridors are uniformly wide. If
    a future round presents a narrow one, this stops being a no-op there and
    wants a real gate.

    Measured on balanced128: 94 -> 105 alone (+11), improving EVERY width bucket
    and cutting collisions in all five (1->0, 7->4, 10->9, 6->2, 1->0). Paired
    with the L1 ladder it gives 121/128 against 120 for the pre-2026-08-30
    configuration, at ~12.5 s faster per run.

    The arc is tangent to both centrelines, so for a 90 deg corner its radius IS
    the turn-entry distance: the turn starts ``r`` metres before the corner.
    With the shipped bias that makes entry 0.300 m for narrow->narrow and
    0.450 m for anything touching a wide corridor.

    A blind round starts believing every corridor NARROW. So on a narrow->wide
    corner it plans a 0.300 m entry where the true geometry wants 0.450 m and
    commits **0.15 m late** -- 0.38 s at the 0.40 m/s medium tier. Confirming
    wide takes ``corridor_estimator.MIN_SAMPLES`` = 12 readings, about 0.5 m of
    travel, so the correction normally lands AFTER the correct entry point has
    gone past. Late is the default on every corner touching a wide corridor,
    which is why the symptom is specific to wide ones and to first laps.

    Setting this trades that for the opposite error: a narrow->narrow corner
    turns 0.15 m EARLY. That is the safe direction -- turning early into a
    corridor wider than planned costs a little line, turning late is what puts
    the nose into the outer wall. It does erode narrow->narrow clearance from
    +0.153 m to +0.070 m (see ``corner_arc_radius``), so it is a real trade and
    not free.
    """

    FIRST_LAP_CORNER_CAUTION: bool = Field(default=True, validation_alias=_alias("FIRST_LAP_CORNER_CAUTION"))
    """Cap the FIRST lap's corners at ``slow_mps``, on the lap never yet driven.

    Added because real hardware wedged at a mixed-width corner whose PLANNED arc
    is safe by construction (2026-08-28), pointing at control tracking error
    eating the plan's margin rather than the plan itself. Slowing the one lap
    that has never been driven was meant to buy the tracking loop margin.

    **The 2026-08-30 hardware evidence says it does the opposite.** Over four
    track runs it pins lap 1 to ``slow_mps`` -- median commanded 0.220 m/s
    against 0.400 on later laps -- and lap 1 carries double the heading error
    (``|angle_error|`` p90 1.38 rad vs 0.68). Lookahead occupancy and turn-preview
    activity are near-identical across laps, so speed is the only variable that
    moves between them. That is the same shape as the 2026-08-09 finding which
    retired the four-rung heading ladder: the middle rungs taxed every corner
    and cost 33% of lap time without catching a dangerous case.

    A corner is plausibly a STEERING problem rather than a braking one -- the
    servo slews at a fixed rate, so creeping through the arc spends more ticks
    at high heading error rather than fewer. This flag exists so that can be
    measured instead of argued.
    """

    FIRST_LAP_CORNER_CAUTION_NARROW_ONLY: bool = Field(
        default=False, validation_alias=_alias("FIRST_LAP_CORNER_CAUTION_NARROW_ONLY")
    )
    """Restrict ``FIRST_LAP_CORNER_CAUTION`` to corridors planned as NARROW.

    The cap is not uniformly good or bad -- it crosses over with corridor width.
    Measured on balanced128, 2026-08-30, cap ON vs OFF by how many of the four
    corridors are wide:

    | wide | cap ON | cap OFF |
    |------|--------|---------|
    |    0 |    88% |     75% |
    |    1 |    75% |     47% |
    |    2 |    73% |     56% |
    |    3 |    66% |     72% |
    |    4 |    88% |    100% |

    Protective where corridors are tight (75% vs 47% at one wide corridor) and
    harmful where they are not (100% vs 88% at all-wide). Slowing to ``slow_mps``
    buys margin the robot needs in a narrow corridor and spends margin it does
    not need in a wide one. This flag applies the cap only where the measurement
    says it earns its keep.

    Width is read back from the planned path rather than from the estimator --
    see ``CoreNavigator._cache_corridor_widths`` -- so it reflects the geometry
    actually being driven, including after a replan.
    """

    DEFER_CURRENT_CORRIDOR_REPLAN: bool = Field(
        default=True, validation_alias=_alias("DEFER_CURRENT_CORRIDOR_REPLAN")
    )
    """Hold a width change back until the robot has left the corridor it describes.

    **OPEN-CHALLENGE-ONLY, by construction rather than by this flag.** Both
    callers build the gate only for Open (``None`` otherwise, which keeps the
    pre-gate control flow byte-for-byte). On Obstacles the estimator is
    ``fixed=True`` and the bias comes from ``OBSTACLES_CENTER_BIAS_M``, so the
    gate's confirmed-ness trigger would rebuild an identical path and re-seek
    the waypoint index for nothing.

    **Measured over the full 640-case Open space 2026-08-31: 634 -> 638 ok,
    failures 6 -> 2, no collisions in either arm, sim time -4.74 s mean** (497
    of 633 cases faster). 5 fixed against 1 regressed, and no width bucket got
    worse. Screened on balanced128 first, where it was 128/128 in both arms --
    that corpus has no verdict headroom left, so the -5.17 s mean was the only
    signal and the full space was needed to see the +4.

    It also closes the case cluster {38, 64, 71, 207, 264, 368, 407, 426} that
    flipped as a unit under every previous attempt on this step: shrinking the
    step (``UNCONFIRMED_WIDTH_INNER_BIAS_M``) or fading it
    (``REPLAN_BLEND_TICKS``) traded those cases against each other, while
    removing it passes all eight.

    The replan step is not inherent to replanning, only to replanning
    *underneath* the chassis: a width update for the corridor the robot is
    standing in moves the line it is actively tracking, while the same update
    for any other corridor costs nothing because the robot arrives on the new
    line instead of being displaced onto it.

    Third attack on the same ~0.30 m first-lap step, and the only one that
    removes it rather than reshaping it.
    ``UNCONFIRMED_WIDTH_INNER_BIAS_M`` shrinks it (0.30 -> 0.25, worth +13 cases
    over the 640-case space). ``REPLAN_BLEND_TICKS`` spreads it over time and is
    refuted twice -- most recently at -16 cases AND a new collision on the full
    640 with both of its original confounds removed.

    Costs one traverse of slightly-off centring in the corridor whose width just
    changed, corrected from the next lap. Cannot disturb corner geometry:
    ``CORNER_ARC_ASSUME_WIDE`` already sizes every arc independently of the
    belief, so nothing deferred here reaches an arc.

    A separate, unintended fix rode in with the same wiring and is worth
    knowing about because it is NOT what this flag controls. The gate replans
    when confirmed-ness moves, and the code it replaced returned early on
    ``if not estimator.observe(...)`` -- so a corridor that confirmed at the
    value the prior already held never replanned, and
    ``UNCONFIRMED_WIDTH_INNER_BIAS_M`` went on being applied to a corridor that
    was no longer unconfirmed. Releasing it correctly is worth about +3 on the
    640-case space (631 -> 634) with this flag still OFF, which is why the
    A/B's baseline reads 634 rather than the 631 that ``2a0e9e28`` measured.
    That +3 is a cross-run comparison, not a paired arm, so it is the weaker of
    the two numbers here.

    See :class:`~src.navigation.deferred_width_belief.DeferredWidthBelief`.
    SIM ONLY -- not track-validated, and the simulator weaves 1.6-3.3x more
    than hardware.
    """

    ADVANCE_PAST_PASSED_WAYPOINT: bool = Field(
        default=False, validation_alias=_alias("ADVANCE_PAST_PASSED_WAYPOINT")
    )
    """Retire a waypoint the chassis has SWEPT PAST, not only one it drove near.

    ``CoreNavigator.step`` advances the index when the next waypoint tests
    strictly closer than the current one. Overshoot a waypoint and that test
    can never close -- both it and its successor recede every tick -- leaving
    entry into a 0.20 m circle the chassis has already left as the only exit.
    The index freezes, and pure pursuit then steers CORRECTLY for a target that
    is now behind: it turns around to go back for it.

    Measured on hardware 2026-08-31, run_20260831_224600, first corner: index
    frozen on waypoint 8 at (0.35, 1.00) for four seconds while heading ran
    172 -> 151 -> 130 -> 107 -> 66 -> 14 deg. About 160 deg of rotation where
    the corner needed 90, ending perpendicular to the corridor with forward
    clearance collapsing 0.21 -> 0.11 m into the wall. The index then jumped
    8 -> 11 -- the re-seek finding a later point after the overshoot.

    The local-frame ahead/behind test already knows the chassis has passed it;
    the machinery exists and has been in the Obstacles path for a while (see
    ``sign_router.STALE_TARGET_RESCUE``). It was never available to Open, whose
    ``rescue_behind`` is gated on a sign router being present -- deliberately,
    to leave Open's advance pipeline byte-identical. This is that gate lifted,
    on Open's own evidence.

    A separate flag rather than reusing STALE_TARGET_RESCUE, which is namespaced
    under sign_router, was measured on Obstacles (2-5%, and refuted there), and
    carries the presence check that makes it structurally unreachable from Open.
    Same geometry, different challenge, different evidence.

    Defaults False pending a corpus number. Unlike the fail-safe clearance work,
    this IS simulator-testable -- the overshoot is ordinary geometry, not a
    wedged-against-a-wall state the absorbing contact model cannot reach.
    """

    REPLAN_BLEND_TICKS: int = Field(default=0, ge=0, validation_alias=_alias("REPLAN_BLEND_TICKS"))
    """Ticks over which a replanned path is faded in, instead of swapped at once.

    **DEFAULTED OFF 2026-08-30, after measuring it.** Fading at 20 ticks removes
    the discontinuity exactly as designed -- path jumps 2 -> 0 and the heading
    limiter halves on a single scenario -- and still costs **11 cases** on the
    balanced128 corpus (94 -> 83). The step is real but is not what makes first
    laps bad: runs with NO replan at all show the same first-lap corner problem,
    slightly worse (limiter 38.2% vs 33.3%). A path that slides under the robot
    for a second is evidently worse than one that jumps once and settles.

    Kept, with its sweep mode, because the discontinuity it targets is measured
    and real -- it just needs a better remedy than a linear fade. Do not re-enable
    without a corpus number.

    ``replace_path`` used to install the new centreline in a single tick. The
    path is what crosstrack and the steering target are measured against, so
    that made the TARGET teleport: measured on hardware 2026-08-30 across six
    runs, every first-lap corridor-width belief update stepped crosstrack by
    ~0.30 m in one 50 ms tick, which is ten times the ~0.03 m the chassis can
    physically travel in that time.

    The consequence was not a small transient. The step threw heading error
    past ``heading.CRAWL`` (1.0 rad), so the heading limiter dropped the robot
    to ``creep_mps`` and it crawled back onto a line that had moved under it:
    in 4 of 5 recorded jumps the limiter went from 0% of ticks to 82-100%
    immediately after. First laps spent 33% of their ticks there against 6% on
    later laps, and the single run that completed cleanly was the one with no
    replan at all.

    Fading the geometry in over ~1 s keeps the same final path -- this changes
    only how fast the target gets there, never where it ends up. Zero restores
    the original single-tick swap.

    Blending is index-wise, which is safe because the planned path has the same
    waypoint count for every corridor-width combination (44 across all 16 on
    the Open geometry, verified 2026-08-30). ``CoreNavigator`` checks the counts
    match anyway and falls back to the instant swap if they ever do not, so a
    future geometry that breaks that assumption degrades to today's behaviour
    rather than interpolating between mismatched indices.
    """
