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
