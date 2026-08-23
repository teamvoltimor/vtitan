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
        CENTER_BIAS_M: How far (m) to shift corridor centreline waypoints off
            centre. Magnitude only -- which side it shifts toward is
            CENTER_BIAS_SIDE, so the two can be tuned independently and a
            side can be A/B'd without touching the distance.
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
            ``CENTER_BIAS_M`` so Open's value can still be tuned without
            touching Obstacles, matching how ``CorridorDimensions``
            already carries a separate ``OBSTACLES_WIDTH``.
        CENTER_BIAS_SIDE: Which boundary CENTER_BIAS_M shifts the path toward.
            Was fixed at OUTER and spelled into the constant's own name
            (OUTER_WALL_BIAS), which made the preference an assumption of the
            code rather than a setting. Clearance is symmetric either way --
            0.05 off centre leaves 0.353 m to the near boundary in a 1.0 m
            corridor and 0.153 m in a 0.6 m one, whichever side it is -- so the
            outward choice bought nothing and lengthened every lap, since a
            path further from the inner block is a longer way round.
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
    CENTER_BIAS_M: float = Field(default=0.10, validation_alias=_alias("CENTER_BIAS_M"))
    OBSTACLES_CENTER_BIAS_M: float = Field(default=0.15, validation_alias=_alias("OBSTACLES_CENTER_BIAS_M"))
    CENTER_BIAS_SIDE: CorridorSide = Field(default=CorridorSide.INNER, validation_alias=_alias("CENTER_BIAS_SIDE"))
    NUM_INTERMEDIATE_ARC_POINTS: int = Field(default=3, validation_alias=_alias("NUM_INTERMEDIATE_ARC_POINTS"))
    STRAIGHT_WAYPOINT_COUNT: int = Field(default=8, validation_alias=_alias("STRAIGHT_WAYPOINT_COUNT"))
    MAIN_LOOP_REACHED_DISTANCE_M: float = Field(default=0.20, validation_alias=_alias("MAIN_LOOP_REACHED_DISTANCE_M"))
    CONTROLLER_REACHED_DISTANCE_M: float = Field(default=0.01, validation_alias=_alias("CONTROLLER_REACHED_DISTANCE_M"))
    REPLAN_HEADING_TIE_MARGIN_M: float = Field(default=0.15, validation_alias=_alias("REPLAN_HEADING_TIE_MARGIN_M"))
