"""Traffic-sign routing and blind sign-discovery tuning groups."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from shared.config.navigation_tuning._shared import _alias


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
        PIN_CORNER_GUARD: Require the ROBOT's own position to still read as
            squarely in the corridor before the depth pin may fire, not just
            the (0.2-0.4 m ahead) lookahead waypoint. ``False`` restores the
            pin as first measured, which cost 11 corner-adjacent wall
            collisions. Only meaningful with ``DEPTH_PIN``.
        PIN_HEADING_GUARD: Release the depth pin once the robot's heading has
            rotated more than ``PIN_HEADING_GUARD_DEG`` since the pin first
            engaged on the current sign, even if ``PIN_CORNER_GUARD``'s
            position check still reads squarely-in-corridor. Traced on a
            sighted wall collision (go_obstacles_0049, subset64): the pin held
            a commanded point frozen for 46 ticks (~2.3 s) while the robot's
            yaw rotated 67 deg mid-corner, because the position-only guard
            never tripped -- the raw waypoint stayed squarely in its corridor
            the whole time even though the chassis had already committed to
            the turn. Steering saturated chasing the frozen target and the
            chassis crashed. Defaults ``True`` at 35 deg: measured over the
            full 256-scenario corpus (sighted), wall collisions 49 -> 3,
            laps>=3 14 -> 22, in-time 13 -> 19 -- the only threshold tried
            (25/30/35/40/45) that improves laps>=3 rather than just trading
            wall strikes for sign strikes 1:1. Only meaningful with
            ``DEPTH_PIN``.
        STALE_TARGET_RESCUE: Advance the waypoint index past any waypoint
            that reads as behind the chassis in its own local frame, not just
            one a closer-next-waypoint check catches. A robot cutting a
            corner sharply enough (more steering authority than the waypoint
            polyline's spacing assumed) can leave both the current and next
            waypoint reading as farther away simultaneously, freezing the
            index; the ordinary lookahead search then returns a distant
            point the chassis's actual trajectory never converges toward.
            Root-caused under the ``wideonly`` hardware profile
            (go_obstacles_0042, subset64) but gated here on sign_router
            presence -- this only ever applies to Obstacles Challenge runs,
            never Open, regardless of this flag. Defaults ``False``:
            unmeasured over the corpus, ships off until it is.
        SIGN_AWARE_SPEED: Cap speed at the ``slow`` tier whenever the router
            actually deformed the target waypoint by more than
            ``SIGN_DEFORM_SPEED_THRESHOLD_M`` this tick. Neither the
            clearance nor heading-error speed ladders react to a sign
            deformation -- it biases the steering target sideways without
            necessarily shrinking forward LIDAR clearance or growing heading
            error -- so the chassis can stay at full speed while still
            asymptotically closing the same ~6.5cm shortfall
            ``SIGN_AWARE_LOOKAHEAD`` targets. Unlike that knob (measured
            worse, see its own docstring), this reacts to the deformation the
            router already applied rather than proximity to a sign, so it
            cannot mis-trigger on a sign that isn't currently biasing
            anything. Defaults ``False``: unmeasured over the corpus, ships
            off until it is.
        SIGN_DEFORM_SPEED_THRESHOLD_M: Deformation magnitude (m) above which
            ``SIGN_AWARE_SPEED`` caps speed. Only meaningful with
            ``SIGN_AWARE_SPEED``.
        SIGN_AWARE_LOOKAHEAD: Arm the short pursuit lookahead whenever a
            routed (not-yet-passed) sign sits within ``ACTIVATION_DIST_M`` of
            the chassis, the same way an upcoming corner already does. The
            sign-avoidance offset is applied to whichever point the lookahead
            search picks, but the search's OWN lookahead choice is driven by
            crosstrack error measured against the raw, undeformed path -- by
            design the robot stays close to that path during a sign pass, so
            crosstrack never rises enough to shorten the lookahead, and the
            long lookahead hands the router a distant point to bias, which
            curvature's quadratic relationship to lookahead turns into a
            weak, undershooting correction. Traced as a consistent ~6.5cm
            shortfall between the commanded line and the chassis at the
            moment it draws level with a sign (subset64,
            go_obstacles_0009/0011/0020/0046). Defaults ``False``: unmeasured
            over the corpus, ships off until it is.
        SIGN_LANE_PLANNER: Shift the PLANNED PATH onto a pass-side lane
            through each signed corridor, instead of only overriding the
            pursuit target near the sign. Every other lever tried against the
            ~6.5cm shortfall changes when or how hard the existing carrot-chase
            fires; this changes the maneuver. Because the polyline itself
            moves, ``cross_track_error`` (which ``select_lookahead`` gates on,
            and which by construction never rises during a carrot-only
            deformation) finally registers the offset, and the lateral travel
            is spread over the corridor's whole straight rather than demanded
            in the last ``ACTIVATION_DIST_M``. Obstacles-only by construction:
            the transform is driven by the routed sign list, and Open
            Challenge has no ``SignRouter``, so its path is returned
            unmodified. See ``navigation.planning.sign_lane``. Defaults
            ``False``: unmeasured over the corpus, ships off until it is.
        SIGN_LANE_RAMP_M: Along-corridor distance (m) over which the lane
            transitions on and off the corridor centreline. Ramp endpoints are
            clamped into the corridor's straight span (see
            ``sign_lane._control_points``), so past roughly 0.90 this
            saturates: 0.90 and 1.20 measure byte-identical, which is the
            expected shape rather than a disconnected knob. Swept on subset64
            sighted at 0.30/0.50/0.70/0.90/1.20 -- collisions
            57/53/55/53/53, laps>=3 7/11/9/11/11, in-time 3/6/4/7/7. Defaults
            0.90: the shortest value that reaches the saturated optimum.
            Only meaningful with ``SIGN_LANE_PLANNER``.
        SIGN_LANE_HOLD_M: Along-corridor half-width (m) of the full-offset
            plateau held either side of a sign's own depth. Swept on subset64
            sighted at 0.10/0.25/0.40/0.55 -- collisions 56/53/53/62,
            laps>=3 8/11/11/2. Defaults 0.25, a genuine peak rather than a
            flat knob. Only meaningful with ``SIGN_LANE_PLANNER``.
        SIGN_LANE_CORNER_ENTRY_M: How far past a corridor's straight the lane
            may extend into the corner arcs either side (m), used as
            transition runway. ``0.0`` confines it to the straight. Measured
            over the 256-scenario corpus, 1211 of 1282 signs sit at a section
            BOUNDARY (along-corridor depth 1.00 or 2.00), where the straight
            offers no near-side runway at all -- the lane reaches full offset
            on its first waypoint, hard against a corner arc still exactly on
            the centreline, putting an abrupt lateral step at the corner exit
            beside the inner square. That is where the lane's wall collisions
            were traced. Spending corner arc is safe here specifically
            because no sign ever occupies a corner (0 of 1282), so the arc is
            free space; the profile is applied there as a SHIFT rather than
            an absolute lateral, translating the turn instead of flattening
            it. Swept on subset64 sighted at
            0.00/0.20/0.35/0.50/0.65/0.80/0.95 -- collisions
            53/42/32/**17**/22/20/20, wall 6/2/2/**0**/4/2/2, laps>=3
            11/22/32/**47**/42/44/44. Defaults 0.50, a sharp peak that also
            takes wall collisions to zero: past it the borrow starts
            distorting the turn it is riding through. This one parameter
            dominates everything else in the lane planner. Only meaningful
            with ``SIGN_LANE_PLANNER``.
        SIGN_LANE_OFFSET_FRAC: Fraction of the full avoidance offset the LANE
            carries; the carrot override still commands the full value at the
            pass. Exists to buy back the wall collisions the lane costs
            (3 -> 23 over the full corpus at 1.0), and the geometry says why
            they appear: the chassis half-diagonal (0.179 m) plus
            ``WALL_CLEARANCE_MARGIN_M`` is 0.219 m, so a sign near the centre
            of a 1.0 m corridor puts a full-offset lane at 0.22 -- exactly on
            ``clamp_lateral``'s floor -- and holds it there for the whole
            straight, where the carrot-only deformation merely grazed it.
            Below 1.0 the lane runs further from the wall while still doing
            the job it was added for: removing most of the lateral travel
            from the last 1.4 m. Only meaningful with ``SIGN_LANE_PLANNER``.
        SIGN_LANE_SUPPRESS_DEFORM: Stop applying the carrot-level
            ``deform_waypoint`` override once the lane planner is placing the
            path. The two never stacked -- ``_apply_deformation`` REPLACES the
            target's lateral coordinate with an absolute value derived from
            the sign, so with a lane in place it re-commands the same line
            rather than adding a second offset -- so this is a question of
            whether the override still EARNS its cost, not of double-counting.
            Defaults ``True``, and that answer reverses once
            ``SIGN_LANE_CORNER_ENTRY_M`` gives the lane real runway:
            - Without runway the override was essential (subset64: lane alone
              61/64 collisions, lane + override 55/64, baseline 56/64) --
              the lane could not reach its own line unaided, so the override
              finished the job.
            - With runway the lane arrives on the line by itself and the
              override is mostly a time tax. Full 256 corpus, sighted:
              suppressed 70 collisions / laps>=3 186 / **in-time 128**;
              not suppressed 64 / 192 / **in-time 82**. Six more three-lap
              finishes for 46 fewer inside the round limit -- and ``in-time``
              is the competition result. The override's depth pin holds the
              commanded point abeam a sign instead of letting it advance,
              which is exactly the behaviour that costs seconds once the
              chassis no longer needs the help.
            Only meaningful with ``SIGN_LANE_PLANNER``.
        PIN_HEADING_GUARD_DEG: Heading drift (degrees) since pin engagement
            that releases the pin when ``PIN_HEADING_GUARD`` is set. Only
            meaningful with ``PIN_HEADING_GUARD``.
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
        CORRIDOR_FLIP_TICKS: Consecutive ticks a refined sign estimate must
            agree on a NEW corridor before its label is moved there. A sign
            sitting on a corner boundary otherwise flips corridor — and with
            it the deformation's lateral axis — on millimetre-scale estimate
            jitter. Defaults to 1 (immediate reassignment, mechanism inert):
            the oscillation is real and confirmed, but suppressing it measured
            flat over the corpus. See sign_router.toml.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SIGN_CLEARANCE_MARGIN_M: float = Field(default=0.075, validation_alias=_alias("SIGN_CLEARANCE_MARGIN_M"))
    DEFORM_DEPTH_BUFFER_M: float = Field(default=0.5, validation_alias=_alias("DEFORM_DEPTH_BUFFER_M"))
    WALL_CLEARANCE_MARGIN_M: float = Field(default=0.04, validation_alias=_alias("WALL_CLEARANCE_MARGIN_M"))
    ACTIVATION_DIST_M: float = Field(default=1.40, validation_alias=_alias("ACTIVATION_DIST_M"))
    PASSED_DIST_M: float = Field(default=1.60, validation_alias=_alias("PASSED_DIST_M"))
    DEPTH_PIN: bool = Field(default=True, validation_alias=_alias("DEPTH_PIN"))
    PIN_CORNER_GUARD: bool = Field(default=True, validation_alias=_alias("PIN_CORNER_GUARD"))
    PIN_HEADING_GUARD: bool = Field(default=True, validation_alias=_alias("PIN_HEADING_GUARD"))
    SIGN_AWARE_LOOKAHEAD: bool = Field(default=False, validation_alias=_alias("SIGN_AWARE_LOOKAHEAD"))
    SIGN_AWARE_SPEED: bool = Field(default=False, validation_alias=_alias("SIGN_AWARE_SPEED"))
    STALE_TARGET_RESCUE: bool = Field(default=False, validation_alias=_alias("STALE_TARGET_RESCUE"))
    SIGN_LANE_PLANNER: bool = Field(default=False, validation_alias=_alias("SIGN_LANE_PLANNER"))
    SIGN_LANE_SUPPRESS_DEFORM: bool = Field(default=True, validation_alias=_alias("SIGN_LANE_SUPPRESS_DEFORM"))
    SIGN_LANE_RAMP_M: float = Field(default=0.90, validation_alias=_alias("SIGN_LANE_RAMP_M"))
    SIGN_LANE_HOLD_M: float = Field(default=0.25, validation_alias=_alias("SIGN_LANE_HOLD_M"))
    SIGN_LANE_OFFSET_FRAC: float = Field(default=1.0, gt=0.0, le=1.0, validation_alias=_alias("SIGN_LANE_OFFSET_FRAC"))
    SIGN_LANE_CORNER_ENTRY_M: float = Field(default=0.50, ge=0.0, validation_alias=_alias("SIGN_LANE_CORNER_ENTRY_M"))
    SIGN_DEFORM_SPEED_THRESHOLD_M: float = Field(default=0.02, validation_alias=_alias("SIGN_DEFORM_SPEED_THRESHOLD_M"))
    PIN_HEADING_GUARD_DEG: float = Field(default=35.0, validation_alias=_alias("PIN_HEADING_GUARD_DEG"))
    DETECTION_MATCH_DIST_M: float = Field(default=0.30, validation_alias=_alias("DETECTION_MATCH_DIST_M"))
    MIN_CONFIDENCE: float = Field(default=0.25, validation_alias=_alias("MIN_CONFIDENCE"))
    SETTLE_TICKS: int = Field(default=150, validation_alias=_alias("SETTLE_TICKS"))
    ESCAPE_MASK_RADIUS_M: float = Field(default=0.12, validation_alias=_alias("ESCAPE_MASK_RADIUS_M"))
    COMMIT_HYSTERESIS: bool = Field(default=False, validation_alias=_alias("COMMIT_HYSTERESIS"))
    CORRIDOR_FLIP_TICKS: int = Field(default=1, ge=1, validation_alias=_alias("CORRIDOR_FLIP_TICKS"))


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
