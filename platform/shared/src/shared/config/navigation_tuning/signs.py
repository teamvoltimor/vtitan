"""Traffic-sign routing and blind sign-discovery tuning groups."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning._shared import _alias
from shared.domain.steering import angle_rad_to_steering_norm


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

            Together with ``SIGN_CLEARANCE_MARGIN_M`` this decides how a
            squeeze is split between wall and pillar, and it binds on 646 of
            the corpus's 1282 signs. Spending it the other way has been
            measured and refuted -- a squeezed lane moved toward the midpoint
            of its free gap trades 31 sign collisions for 58 wall collisions
            (229/256 against a 202/256 baseline), and is worse at every
            intermediate fraction too. See ``sign_router.clamp_lateral`` and
            ``sign_lane``'s module docstring; the short version is that the
            adjustable range is 3.1 cm against a 6.3-6.6 cm crosstrack
            shortfall, so tracking error dominates the geometry.
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
            anything. Ships ``True``, in both this default and
            ``shared/config/navigation/signs/sign_router.toml`` -- they are
            held in step by ``test_field_defaults_match_shipped_toml``, so a
            bare ``SignRouterParams()`` gets the value that actually races.
            Measured 2026-09-04 over the 256-scenario corpus, blind: rev-run
            (rule 9.21) 14 -> 9 and the same 5 runs leave ``unscored``, giving
            +2 clean / +2 in-time / +2 laps>=3. Total collisions do NOT move
            (15 -> 15; 2 shift sign -> wall) and timeouts cost +1, so neither
            is the reason it ships.
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
        RETRACE_ESCAPE: Make a reversing escape follow the ground the chassis
            just occupied, instead of backing along an arc into space it has
            never been. Obstacles-only by construction (gated on
            ``sign_router`` presence, ``None`` for Open), like
            ``STALE_TARGET_RESCUE``.
            Two independent reasons, one measured and one structural:
            - The arc is what produces wall strikes. Blind with the escape
              mask off, sign collisions fall 57 -> 41 but wall collisions rise
              0 -> 13, in a corridor 1.0 m wide. Letting the escape fire on a
              sign is the only change that has moved blind sign collisions;
              the arc is the part that costs.
            - It removes the dependence on rear sensing. The rear sector is
              already mostly masked by mount occlusion (-160..-115 and
              115..175 deg), leaving a ~25 deg slot straight back. If that
              slot is absent on a future chassis, the sector has no valid rays
              and ``compute_rear_clearance`` reports the same 10 m it reports
              for open road. The reverse guard now refuses on that case
              (``SectorRanges.measured``) rather than reading it as clear,
              but refusing is still a stop, not a way through.
              Retraced ground is known free because the chassis was standing
              on it, so no rear sensor is consulted at all.
            **MEASURED AND REJECTED as a replacement for the arc. Ships
            False.** Blind, subset64, lane on, mask off:
            arc reverse 54 collisions (wall 13, sign 41, laps>=1 24);
            retrace 60 (wall **7**, sign 53, laps>=1 10). Retracing does
            halve the wall strikes -- the safety half of the idea works -- but
            it gives back most of the sign gain, and the reason is structural:
            backing straight out does not REPOSITION the chassis, so the
            forward re-approach repeats the line that just failed. The arc's
            lateral displacement was doing real work, not just causing damage.
            Not a tuning gap: retrace distance 0.12/0.25/0.40/0.60 measures
            61/60/59/60.
            What this does establish is that the two halves are separable --
            the repositioning has to happen, but it does not have to happen
            while reversing blind. The next shape to try is a reverse ARC
            validated against the KNOWN track geometry rather than against
            rear LIDAR: the track is 3x3 m with 1.0 m corridors and a fixed
            inner block, all of it known before the robot is placed, so a
            swept-path check needs no rear sensing either.
        RETRACE_DIST_M: How far back along the trail to aim while retracing (m).
            Only meaningful with ``RETRACE_ESCAPE``.
        RETRACE_STEER_GAIN_DEG: Proportional gain on the retrace's lateral
            error, as the road-wheel angle commanded when the target sits at
            45 degrees off the chassis (``lateral / distance == 1``). Reverse
            pure pursuit, so the sign is inverted relative to the forward
            case. Only meaningful with ``RETRACE_ESCAPE``.
        SIGN_CONTACT_EVADE: React to an imminent SIGN contact by steering away
            and creeping, instead of either ignoring it or reversing.
            Fills a gap between the two responses that exist today. A LIDAR
            return close enough to read CRITICAL is either masked as belonging
            to a routed sign -- in which case nothing happens at all, on the
            reasoning that the planner has it handled -- or it is unmasked and
            triggers the full escape maneuver, which reverses and swings.
            Sighted, the first is correct: the lane is planned a corridor
            ahead and does have it handled. Blind lap 1 satisfies neither
            assumption, because a sign discovered 1.5 m away cannot be planned
            around at all. Measured there (subset64, lane on): disabling the
            mask outright cuts sign collisions 57 -> 41 and lifts laps>=1
            14 -> 24, but buys 13 new WALL collisions, because reverse-and-
            swing is a poor move in a 1.0 m corridor.
            So this is the middle rung: keep the mask (no reversing at a sign)
            but stop treating a masked threat as no threat. Fires only when the
            RAW scan reads CRITICAL while the MASKED scan does not -- i.e. the
            imminent contact is specifically a sign the router owns.
            **BOTH TRIGGERS MEASURED AND REJECTED. Ships False. Do not retry
            either shape without new information.**
            - Gated on the LIDAR risk tier (raw CRITICAL, masked not): flat --
              57/58/57/58 collisions across steer 0.0/0.25/0.45/0.65. A return
              only reads CRITICAL at contact range, by which point no steering
              command can help. That is precisely why REVERSING works there
              and steering does not.
            - Predicted geometrically from the router's own sign positions
              (along-track distance and lateral clearance, the code below):
              far WORSE -- 64/64 collisions and laps>=3 to zero at every
              steer value. Signs sit only 0.10 m off the corridor centreline,
              inside the ~0.122 m half-width clearance this tests against, so
              the trigger fires almost continuously and a sustained steering
              bias integrates into a large heading error that destroys
              tracking outright.
            The gap between "too late to act" and "fires constantly" is the
            real difficulty here, and a steering nudge does not fit in it. The
            one thing that HAS moved blind sign collisions is letting the
            escape maneuver fire (mask 0.0), so a wall-aware escape looks more
            promising than any further nudge tuning.
        SIGN_CONTACT_DIST_M: Along-track distance (m) within which a routed
            sign predicted to pass closer than the chassis and sign half-widths
            allow triggers ``SIGN_CONTACT_EVADE``. Only meaningful with it.
        SIGN_CONTACT_STEER_DEG: Road-wheel angle steered away from the
            offending sign when ``SIGN_CONTACT_EVADE`` fires. Only meaningful
            with it.
        SIGN_LANE_COMMIT_AHEAD_M: Distance ahead of the chassis within which a
            lane rebuild may NOT move the path (m). ``0.0`` disables it, which
            is the behaviour that shipped first.
            Only matters when the sign layout changes mid-run, i.e. blind
            discovery. There the layout changes constantly -- measured ~260
            rebuilds per run -- and the lane's ramp is designed to start
            BEFORE the corridor is entered, which is impossible once the sign
            that triggered it was only discovered 1.5 m into that corridor.
            The rebuilt path then has its ramp behind the chassis: measured,
            10% of rebuilds (614 of 6234) moved the path AWAY from the robot,
            by up to 0.301 m -- about a full lane offset -- stranding it
            off-path with no runway to rejoin.
            Holding the near field fixed keeps a rebuild from rewriting what
            the chassis is already committed to, so a newly discovered sign
            bends the path ahead of the robot instead of underneath it.
        EXPLORE_LAP_SPEED_FRAC: Speed ceiling for the FIRST lap of a
            discovering (blind) Obstacles run, as a fraction of the normal
            ceiling. ``1.0`` disables it. Applies only while
            ``SignRouter.is_discovering`` and no lap has been completed, so a
            sighted run and every later lap are untouched.
            Measured (subset64, blind, lane on, parking off): **78% of blind
            failures happen during lap 1**, 9% in lap 2, 0% in lap 3, and of
            the runs that survive lap 1 more than half finish all three. Blind
            is a reconnaissance problem, not a uniformly broken one: signs sit
            inside corridors and the next corridor is outside a 102 deg FOV
            until the corner is turned, so observations top out at ~2.3 m no
            matter how discovery is tuned -- but discovered signs PERSIST
            across laps, so laps 2-3 run against a full map.
            Note this is not the ``SIGN_AWARE_SPEED`` lever wearing a
            different hat. That one failed in SIGHTED mode, where the shortfall
            is curvature-limited and extra time cannot buy turning radius. A
            blind first lap is INFORMATION-limited: a sign that only exists
            once it is 1.5 m away gives twice the runway at half the speed.
            Different constraint, so the same knob can legitimately behave
            differently -- and it did: measured BLIND on 2026-09-04 that lever
            earned its way on and now ships TRUE, so "refuted" applies only to
            the sighted result, never unqualified.
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
            unmodified. See ``navigation.planning.sign_lane``.
            **Defaults ``True``, validated on the full 256 corpus
            2026-08-17**, both modes measured against an OFF arm in the same
            run rather than a remembered baseline:
            sighted 242 -> 62 collisions (sign 231 -> 55), laps>=3 14 -> 194,
            in-time 13 -> 139;
            blind 248 -> 231 collisions (wall 6 -> 1), laps>=3 8 -> 27,
            in-time 4 -> 13.
            Blind gains far less because the lane's whole advantage is approach
            runway and lap 1 has none -- 78% of blind failures are lap 1, where
            a corridor's signs stay invisible until the robot is inside it
            (detection is FOV-capped at ~2.3 m by geometry). Its timeouts rise
            4 -> 10, which is survival, not a new failure: laps>=1 rises
            22 -> 39 in the same arm, so runs that used to crash out early now
            last long enough to run out of clock.
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
        SIGN_LANE_SPLIT_OVERLAP: Give each sign a flat hold over the stretch
            where it is actually passed, by splitting overlapping plateaux at
            their midpoint instead of letting one dip through another. A plateau
            nested inside another puts a HOLE in the enclosing sign's hold
            window, and the plan passes that sign on the dip. Measured blind over
            the 256 corpus: a plan governed by ANOTHER spec's plateau is
            wrong-side 58% of the time against a 13% base rate (4.3x), and
            single-spec corridors NEVER fail (0/60). Legal WRO geometry cannot
            overlap -- a section holds at most two signs, 1.00 m apart, against a
            0.50 m plateau -- so every overlap is a discovery artifact of the
            measured 2.50x spec duplication.

            REFUTED and defaults ``False``. Blind, 256 corpus, on top of the
            shipped relabel: pass-side does fall 121 -> 110, but every other
            column loses -- collisions 94 -> 106, laps>=1 74 -> 68, laps>=3
            42 -> 40, in-time 29 -> 27 -- and laps driven FALL 229 -> 210, so
            there is no survivorship excuse: sign collisions per lap rise
            0.336 -> 0.410 (+22%) and total per lap 0.410 -> 0.505 (+23%). The
            4.3x lift the conditioner showed was CORRELATION: it marks crowded
            corridors, which fail for reasons the profile shape does not
            capture, and splitting the plateaux makes the lane switch targets
            sooner and drives the chassis into more signs. Only meaningful with
            ``SIGN_LANE_PLANNER``.
        SIGN_LANE_RELABEL_UNSATISFIABLE: When a sign's clamped lane target lands
            on the forbidden side of it, move the sign to the OTHER face of its
            corner rather than keeping a corridor whose instruction cannot be
            satisfied. Only the two faces the corner tie-break is choosing
            between are considered -- any section that makes the arithmetic
            positive would pass a naive check, including one across the track.
            Measured blind over the 256 corpus: 29% of specs at a pass are
            inverted under their settled corridor, and 45/45 of them are
            satisfiable under the other face with ~21.7 cm available.

            Defaults ``True`` (2026-08-25), the first sign lever to clear a bar
            set BEFORE the run -- pass-side 140 -> 121, laps>=3 33 -> 42, in-time
            24 -> 29, lane delivery +0.15x -> +0.97x. The raw sign-collision
            column rises 60 -> 77, which is mostly survivorship: laps driven rise
            196 -> 229, so per lap it is 0.306 -> 0.336 (+10%) while wall falls
            0.138 -> 0.074 (-46%) and TOTAL collisions per lap fall 0.444 ->
            0.410. Read the raw sign column against laps-driven, never alone.

            UNVALIDATED ON HARDWARE. The tracking half of these numbers sits on
            top of a ~1.42x understeer the sim does not model.

            Preferred over ``SIGN_LANE_SKIP_UNSATISFIABLE``, which fixes the same
            defect by deleting the lane and costs pass-side 140 -> 155.

            NOW INERT (2026-08-26), subsumed by
            ``SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR``. A target was unsatisfiable
            because the sign had been filed on the perpendicular face; fixing the
            face upstream leaves nothing for this to repair. On/off over the 256
            corpus is bit-identical -- every column, laps-driven 301.0 and
            escapes 4210 included -- so it never fires under the shipped config.

            Kept ``True`` deliberately rather than deleted: "inert on this corpus"
            is not "inert", and the numbers above are simulation only. It stays a
            cheap backstop for geometry the corpus does not contain. Do not read
            its 140 -> 121 gain as live -- that was earned in the regime the depth
            rule removed.

        SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR: Resolve a corner sign by which face
            its DEPTH lies along, rather than which face is nearest. See
            ``depth_consistent_corridor``. Upstream of
            ``SIGN_LANE_RELABEL_UNSATISFIABLE``: that one repairs a lane target
            already computed on the wrong axis, this one stops the axis being
            chosen wrongly, and the relabel still runs on top.

            Measured 2026-08-26 blind over the 256 corpus with the belief offset
            removed, so it is the tie-break and not localization: 42.1% of
            published specs are filed against the perpendicular face, giving a
            "distance past the corner" of exactly 0.40 m (x568) or 0.60 m (x483)
            -- the lateral offsets a sign may take. Confined to boundary signs
            (44.9% at depth 1.00, 44.8% at 2.00, 0 of 153 mid-straight), which is
            an unresolved exact tie rather than noise. That 42% is measured AFTER
            the relabel, which writes the same ``_sign_corridors``, so the
            shipped knob is not already collecting it.

            Defaults ``True`` (2026-08-26) on a bar set before the run. Corpus
            A/B, off arm = then-shipped: pass-side 121 -> 73 (-40%), laps>=3
            42 -> 56, in-time 29 -> 40, timeouts flat at 10. Raw collisions
            94 -> 127 and raw sign 77 -> 103 are SURVIVORSHIP -- laps driven rise
            229 -> 301, so sign per lap is 0.336 -> 0.342 and total per lap
            0.410 -> 0.422, both flat. Read the raw column against laps driven or
            this reads as a 35% regression.

            Costs, not hidden: total collisions per lap +2.9%, 12 scenarios that
            reached 3 laps no longer do (26 others start to, for the +14), and
            pass-side 73/256 is still 28% of runs -- improved, not solved.

            The stuck column moves 5 -> 10 and that one is NOT a cost: all 8
            newly-stuck runs were already failures, six of them at 0 laps, each
            previously ending in a sign collision or a terminal wrong-side pass.
            They now survive past that point and wedge later instead, so the
            failure MODE changed and the outcome did not. Three stopped being
            stuck, for net +5.

            Mechanism confirmed on an INDEPENDENT metric, not on the objective
            the fix minimizes. Depth-violation collapsing to ~0 proves nothing
            (that is what is being optimized); instead, with the belief offset
            removed, the share of specs whose assigned axis disagrees with the
            layout invariant goes 42.1% -> 0.0% (0/2777, every depth bucket).

            UNVALIDATED ON HARDWARE, like the relabel: the ~1.42x understeer is
            invisible in sim, and the laps-driven gain is where that would bite.
        SIGN_LANE_SKIP_UNSATISFIABLE: Drop a sign from the lane profile when its
            own clamped target lands on the FORBIDDEN side of it, instead of
            planning a line that violates the pass-side rule by construction.
            A sign discovered near the corner diagonal is ambiguous between two
            corridors; under either label it reads as past that corridor's
            straight and hard against the inner square, so ``clamp_lateral``
            caps the target at the corridor bound -- which for such a sign is
            the wrong side of the sign itself. Traced on a green WEST spec at
            x=0.993: the lane wants 1.272, the clamp gives 0.781, i.e. 0.212 m
            the wrong side. Measured 2026-08-25 over the 256 corpus, blind:
            specs INSIDE their corridor's straight plan wrong-side 5% of the
            time (n=722), specs PAST the corner 29% (n=635), and 635/1357
            passes involve one. Defaults ``False``: planning no lane also
            removes avoidance geometry, so read the SIGN column against the
            WALL column before shipping it. Only meaningful with
            ``SIGN_LANE_PLANNER``.
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
            path. The two never stacked -- ``apply_deformation`` REPLACES the
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
            Defaults ON since 2026-09-07. The corpus reads FLAT both times it
            has been measured (in-time 59 = 59), and that is the point: what
            this holds still is the aim point when two tracks of the same
            pillar compete, and duplicates sit 0.012 m apart in the sim against
            0.21 m on hardware. The corpus was pricing a defect 17x smaller
            than the real one. On recorded detections it cuts aim-point jumps
            38 -> 21 with committed ticks unchanged. See sign_router.toml.
        ESCAPE_MASK_RADIUS_M: How close a LIDAR return must land to a routed
            sign to be attributed to it and withheld from the reactive escape
            trigger. Zero disables the mapped/unmapped split entirely, which
            restores the pre-fix behaviour where the escape maneuver fires on
            every sign pass.

            A persistence-gated override was tried (a MASK_OVERRIDE_TICKS
            field, since removed): once the raw scan read CRITICAL while the
            masked scan did not for N consecutive ticks, trust the raw scan
            instead of the mask, on the theory that a genuine collision
            course stays critical while a normal successful pass's masked-
            critical dip recovers within a tick or two (traced on
            go_obstacles_0009: discovery correctly identifies and routes the
            struck sign, in the same rigid-transform-related frame as its
            believed pose, and the router's plan still produces near-zero
            real clearance -- the mask has no way to distinguish that from a
            plan that IS working). REFUTED on the full 256-scenario blind
            corpus at every threshold tried (1/2/3/5/8/12 ticks): 206-216/256
            collisions vs the 202/256 baseline, every single value worse, with
            wall collisions and escape rate both far above baseline at low
            tick counts (e.g. 1 tick: wall 3->45, escapes/lap 6.3->45.4) and
            only asymptoting back TOWARD baseline (never past it) as the
            threshold grew large enough to rarely trigger at all. Read as:
            persistence does not actually distinguish the two cases well
            enough in practice -- a normal pass apparently sustains a masked-
            critical dip for long enough, often enough, that any threshold
            short enough to still react before contact also fires on normal
            passes and destabilises otherwise-fine runs into wall strikes.
            Do not re-try a provenance-mask override of this shape without a
            genuinely different distinguishing signal (not raw persistence).
        CORRIDOR_FLIP_TICKS: Consecutive ticks a refined sign estimate must
            agree on a NEW corridor before its label is moved there. A sign
            sitting on a corner boundary otherwise flips corridor — and with
            it the deformation's lateral axis — on millimetre-scale estimate
            jitter. Defaults to 1 (immediate reassignment, mechanism inert):
            the oscillation is real and confirmed, but suppressing it measured
            flat over the corpus. See sign_router.toml.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    SIGN_CLEARANCE_MARGIN_M: float = Field(default=0.10, validation_alias=_alias("SIGN_CLEARANCE_MARGIN_M"))
    DEFORM_DEPTH_BUFFER_M: float = Field(default=0.5, validation_alias=_alias("DEFORM_DEPTH_BUFFER_M"))
    WALL_CLEARANCE_MARGIN_M: float = Field(default=0.04, validation_alias=_alias("WALL_CLEARANCE_MARGIN_M"))
    ACTIVATION_DIST_M: float = Field(default=1.40, validation_alias=_alias("ACTIVATION_DIST_M"))
    PASSED_DIST_M: float = Field(default=1.60, validation_alias=_alias("PASSED_DIST_M"))
    DEPTH_PIN: bool = Field(default=True, validation_alias=_alias("DEPTH_PIN"))
    PIN_CORNER_GUARD: bool = Field(default=True, validation_alias=_alias("PIN_CORNER_GUARD"))
    PIN_HEADING_GUARD: bool = Field(default=True, validation_alias=_alias("PIN_HEADING_GUARD"))
    SIGN_AWARE_LOOKAHEAD: bool = Field(default=True, validation_alias=_alias("SIGN_AWARE_LOOKAHEAD"))

    SIGN_LIDAR_PROPOSE: bool = Field(default=False, validation_alias=_alias("SIGN_LIDAR_PROPOSE"))
    """Let the LIDAR propose a sign's POSITION for the camera to colour.

    The camera stops resolving signs past ~1.1 m while pillar-shaped LIDAR
    clusters appear at a median 1.31 m, so this folds the earlier evidence in as
    geometry only: a proposal casts no colour vote, is never published on its own
    (``ObservedSignMap.newly_confirmed`` requires a vote), and cannot reach a
    steering command, because the pass-side rule is colour-keyed. What it buys is
    that when the colour does arrive, the position is already settled instead of
    being established from scratch inside the last 0.3 m -- and that the position
    is a LIDAR range rather than a pinhole estimate, which is what put believed
    pillars ON THE WALLS on hardware.

    Scored against the simulator's known layout, the detector finds 100% of signs
    (84/84) at 46% precision, which the placement lattice lifts to 84%. It is
    OFF by default until the corpus says what it does to laps and collisions.

    DISTINCT from ``SIGN_LIDAR_ALIGN``, which STEERS toward the nearest narrow
    cluster. That acts on an unconfirmed candidate; this one only records where
    it was.
    """

    SIGN_LIDAR_ALIGN: bool = Field(default=False, validation_alias=_alias("SIGN_LIDAR_ALIGN"))
    """Steer toward a narrow LIDAR object ahead that the camera has not classified.

    The LIDAR resolves the pillars: measured on run_20260906_163641 and _163854,
    a return exists at the camera's own bearing on 98-100% of red/green
    detections, and the object there is 3.8-6.6 cm across at the median -- the
    5 cm sign itself, not the wall behind it. So the LIDAR can say "pillar-sized
    thing ahead" before the classifier can say what colour it is, and bringing
    it toward the centre of frame helps because the classifier is worst at the
    edge (see SignDiscoveryParams.FRAME_EDGE_TOLERANCE_PX).

    Suppressed once the router has COMMITTED to a sign, because then the
    pass-side lane owns the lateral decision and turning toward a pillar to look
    at it would steer into the obstacle the lane is avoiding. Written for the
    opposite case: run_20260906_163854, where a red was never classified at all
    and was passed on the wrong side.
    """

    SIGN_LIDAR_ALIGN_MIN_M: float = Field(default=0.40, gt=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_MIN_M"))
    """Closest range that still leaves room to act on the alignment."""

    SIGN_LIDAR_ALIGN_MAX_M: float = Field(default=1.50, gt=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_MAX_M"))
    """Furthest range considered. Beyond this the bearing error is small anyway."""

    SIGN_LIDAR_ALIGN_FOV_DEG: float = Field(default=45.0, gt=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_FOV_DEG"))
    """Half-angle searched ahead. Wider than the deadband so an off-centre pillar is seen."""

    SIGN_LIDAR_ALIGN_DEPTH_M: float = Field(default=0.08, gt=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_DEPTH_M"))
    """How far behind the closest return a ray may be and still count as the same surface."""

    SIGN_LIDAR_ALIGN_MAX_WIDTH_M: float = Field(
        default=0.15, gt=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_MAX_WIDTH_M")
    )
    """Arc width above which the object is a wall, not a pillar.

    15 cm against a measured p50 of 3.8-6.6 cm: generous enough for a partly
    occluded sign, far below a wall run. 79-81% of the objects at a camera
    detection's bearing fall under it."""

    SIGN_LIDAR_ALIGN_DEADBAND_DEG: float = Field(
        default=8.0, ge=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_DEADBAND_DEG")
    )
    """Bearing error below which nothing is done -- a centred pillar needs no help."""

    SIGN_LIDAR_ALIGN_GAIN: float = Field(default=0.35, ge=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_GAIN"))
    """Steering per radian of bearing error."""

    SIGN_LIDAR_ALIGN_MAX_STEER: float = Field(
        default=0.15, ge=0.0, validation_alias=_alias("SIGN_LIDAR_ALIGN_MAX_STEER")
    )
    """Hard cap on the nudge. Small on purpose: this is a look-at-it bias, not a manoeuvre."""

    SIGN_AWARE_SPEED: bool = Field(default=True, validation_alias=_alias("SIGN_AWARE_SPEED"))
    STALE_TARGET_RESCUE: bool = Field(default=False, validation_alias=_alias("STALE_TARGET_RESCUE"))
    SIGN_LANE_PLANNER: bool = Field(default=True, validation_alias=_alias("SIGN_LANE_PLANNER"))
    RETRACE_ESCAPE: bool = Field(default=False, validation_alias=_alias("RETRACE_ESCAPE"))
    RETRACE_DIST_M: float = Field(default=0.25, gt=0.0, validation_alias=_alias("RETRACE_DIST_M"))
    # 55.0 deg is what the previous normalised 1.0 meant at the bench-measured
    # 55 deg road-wheel limit, so this conversion changed no behaviour.
    RETRACE_STEER_GAIN_DEG: float = Field(default=55.0, ge=0.0, validation_alias=_alias("RETRACE_STEER_GAIN_DEG"))
    SIGN_CONTACT_EVADE: bool = Field(default=False, validation_alias=_alias("SIGN_CONTACT_EVADE"))
    SIGN_CONTACT_DIST_M: float = Field(default=0.60, gt=0.0, validation_alias=_alias("SIGN_CONTACT_DIST_M"))
    # 19.25 deg == the previous normalised 0.35 at the 55 deg road-wheel limit.
    SIGN_CONTACT_STEER_DEG: float = Field(default=19.25, ge=0.0, validation_alias=_alias("SIGN_CONTACT_STEER_DEG"))
    SIGN_LANE_COMMIT_AHEAD_M: float = Field(default=0.0, ge=0.0, validation_alias=_alias("SIGN_LANE_COMMIT_AHEAD_M"))
    EXPLORE_LAP_SPEED_FRAC: float = Field(
        default=1.0, gt=0.0, le=1.0, validation_alias=_alias("EXPLORE_LAP_SPEED_FRAC")
    )
    SIGN_LANE_SUPPRESS_DEFORM: bool = Field(default=True, validation_alias=_alias("SIGN_LANE_SUPPRESS_DEFORM"))
    SIGN_LANE_RAMP_M: float = Field(default=0.90, validation_alias=_alias("SIGN_LANE_RAMP_M"))
    SIGN_LANE_HOLD_M: float = Field(default=0.25, validation_alias=_alias("SIGN_LANE_HOLD_M"))
    SIGN_LANE_SPLIT_OVERLAP: bool = Field(default=False, validation_alias=_alias("SIGN_LANE_SPLIT_OVERLAP"))
    SIGN_LANE_RELABEL_UNSATISFIABLE: bool = Field(
        default=True, validation_alias=_alias("SIGN_LANE_RELABEL_UNSATISFIABLE")
    )
    SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR: bool = Field(
        default=True, validation_alias=_alias("SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR")
    )
    SIGN_LANE_SKIP_UNSATISFIABLE: bool = Field(default=False, validation_alias=_alias("SIGN_LANE_SKIP_UNSATISFIABLE"))
    SIGN_LANE_OFFSET_FRAC: float = Field(default=1.0, gt=0.0, le=1.0, validation_alias=_alias("SIGN_LANE_OFFSET_FRAC"))
    SIGN_LANE_CORNER_ENTRY_M: float = Field(default=0.50, ge=0.0, validation_alias=_alias("SIGN_LANE_CORNER_ENTRY_M"))
    SIGN_DEFORM_SPEED_THRESHOLD_M: float = Field(default=0.02, validation_alias=_alias("SIGN_DEFORM_SPEED_THRESHOLD_M"))
    PIN_HEADING_GUARD_DEG: float = Field(default=35.0, validation_alias=_alias("PIN_HEADING_GUARD_DEG"))
    DETECTION_MATCH_DIST_M: float = Field(default=0.30, validation_alias=_alias("DETECTION_MATCH_DIST_M"))
    MIN_CONFIDENCE: float = Field(default=0.25, validation_alias=_alias("MIN_CONFIDENCE"))
    SETTLE_TICKS: int = Field(default=150, validation_alias=_alias("SETTLE_TICKS"))
    ESCAPE_MASK_RADIUS_M: float = Field(default=0.12, validation_alias=_alias("ESCAPE_MASK_RADIUS_M"))
    COMMIT_HYSTERESIS: bool = Field(default=True, validation_alias=_alias("COMMIT_HYSTERESIS"))
    CORRIDOR_FLIP_TICKS: int = Field(default=1, ge=1, validation_alias=_alias("CORRIDOR_FLIP_TICKS"))

    def sign_contact_steer_norm(self) -> float:
        """Sign-evade steering as the normalised command the actuator takes.

        Stored as a physical road-wheel angle, so a wider servo yields a
        SMALLER normalised command for the same 19.25 degrees rather than the
        same command meaning a wider swerve. Same rationale as
        :meth:`~shared.config.navigation_tuning.escape.EscapeManeuverParams.rev_steer_norm`.
        """
        return angle_rad_to_steering_norm(math.radians(self.SIGN_CONTACT_STEER_DEG), RobotSpecs.MAX_STEERING_ANGLE)

    def retrace_steer_gain_norm(self, lateral_over_distance: float) -> float:
        """Reverse-pure-pursuit steering for a target ``lateral/distance`` off-axis.

        The caller passes the dimensionless bearing ratio; the gain turns it
        into a road-wheel angle, and only then does the servo's reach enter.
        Clamping happens in normalised space, exactly as the previous inline
        ``max(-1.0, min(1.0, ...))`` did.
        """
        return angle_rad_to_steering_norm(
            math.radians(self.RETRACE_STEER_GAIN_DEG) * lateral_over_distance, RobotSpecs.MAX_STEERING_ANGLE
        )


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
        ROBOT_CORRIDOR_FLIP_TICKS: Consecutive ticks the robot's OWN
            corridor classification must disagree with the settled value
            before ``ObservedSignMap`` accepts the change. Association is
            gated on this settled corridor matching a track's (see
            ``_SignTrack.corridor``), specifically to survive a
            wrong-but-consistent rigid rotation of the believed pose without
            folding one corridor's sign into another's track -- a gate keyed
            on the sign's own (reprojected) position cannot do this, since
            the rotational-lock bug reprojects two DIFFERENT signs'
            positions to coincide by construction. But the robot's own
            per-tick corridor, read raw, flips far more than a clean
            rotation alone would predict -- ordinary localizer jitter near a
            boundary is enough -- and every flip started a brand-new track
            before this existed.

            A continuous distance on the robot's own position was tried as a
            replacement (avoiding the discrete-partition problem entirely)
            and measured WORSE across its whole reasonable threshold range --
            202/256 corpus collisions here vs 227-267/256 there -- so this
            imperfect-but-empirically-better corridor gate is what's shipped.
            Deliberately NOT reusing ``SignRouterParams.CORRIDOR_FLIP_TICKS``
            (shipped at 1, i.e. no hysteresis) -- that value settles a
            SIGN's converged position estimate, which is already fairly
            stable; the robot's own tick-to-tick believed position is not.

            A second, still rotation-equivariant refinement was also tried: at
            a corner, widen the gate to accept the adjacent corridor too
            within a small face-distance slack (a corner-blend), on the
            theory that a sign visible from either of two adjacent corridors
            was being arbitrarily split into two tracks. Measured on the full
            256-scenario blind corpus: EVERY nonzero slack tried (0.03-0.20 m)
            was worse than off (190 collisions in that same sweep -- an
            internally fair comparison, but not the corpus baseline, which
            reproduces at 202/256 standalone) on collisions, laps>=3, and
            in-time -- 195-207/256, non-monotonic. Reverted; not shipped as a
            field here. The corner-boundary case is evidently not the
            dominant remaining failure mode, and blending in a second
            corridor's tracks costs more via bad merges than it recovers.

            Two further attempts dropped this exact-match requirement
            entirely at PUBLICATION time instead of at per-observation
            association time, both targeting the SAME physical sign forking
            a new track every lap (traced on go_obstacles_0003: 18 tracks for
            6 physical signs) and both measured WORSE on the full corpus, so
            neither is shipped as a field here -- see
            ``ObservedSignMap.newly_confirmed``'s docstring for the numbers
            and the mechanism (skip-publish: 209/256, discards the
            duplicate's refinement; fold-into-target: 231/256, worse still --
            letting a cross-corridor position match influence the published
            record at all is evidently the problem, not what happens to the
            duplicate's data once matched).
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    MIN_RELIABLE_BBOX_HEIGHT_PX: int = Field(default=5, validation_alias=_alias("MIN_RELIABLE_BBOX_HEIGHT_PX"))
    MAX_INGEST_RANGE_M: float = Field(default=1.5, validation_alias=_alias("MAX_INGEST_RANGE_M"))
    ASSOCIATION_DIST_M: float = Field(default=0.25, validation_alias=_alias("ASSOCIATION_DIST_M"))
    MIN_HITS: int = Field(default=3, validation_alias=_alias("MIN_HITS"))
    MAX_PILLAR_ASPECT: float = Field(default=1.0, validation_alias=_alias("MAX_PILLAR_ASPECT"))
    """Widest box (width/height) still accepted as a traffic-sign pillar.

    A pillar is taller than it is wide by construction, so a box wider than tall
    is not one. Measured on the 2026-09-05 hardware run `run_20260905_214920`:
    GREEN detections, which have no same-coloured scenery to be confused with,
    show w/h p90 = 0.85 with only 0.6% above 1.0. RED, which shares its hue with
    the magenta parking-lot barrier under motion blur, has **52% of its
    detections wider than tall**, concentrated in the two corridors the parking
    lot occupies (west 63%, south 34%) and absent from north entirely.

    AREA CANNOT DO THIS JOB -- wall-shaped reds average 30251 px against
    pillar-shaped reds' 32016, so a size gate keeps both or drops both.
    Confidence cannot either: the bad boxes are CONFIDENT, p50 0.79. Aspect is
    the only separator the data offers.

    Consequence when they get through: every accepted box can seed a sign, and
    `nav_debug.active_sign_count` climbed 5 -> 50 across that run on a track
    holding at most 8, with the router deforming on 85% of ticks.

    ``0.0`` disables the gate.
    """

    VISION_LATENCY_S: float = Field(default=0.85, ge=0.0, validation_alias=_alias("VISION_LATENCY_S"))
    """Camera capture-to-consumption lag, used only when a frame carries no stamp.

    `/vision/detections` is a `std_msgs/String` with no header, so until
    2026-09-07 every detection was paired with the pose at RECEIPT. Measured on
    run_20260906_232408/_232748 the true gap is **0.85 s** (0.78 and 0.95
    found independently in the two runs), and at 0.3 m/s through a corner that
    is most of a sign's lateral offset.

    It was the whole of the bearing residual left after the mirror fix: the
    residual falls **20.2 deg -> 5.4 deg** once the pose is aligned, and the
    share of detections within 10 deg goes 20% -> 73%. The check that is not
    fitted to the lag: the recovered `cx`-vs-true-bearing slope reads
    **-309 px/rad** at zero lag, which NO real lens can produce -- the physical
    floor is ~620 -- and 0.85 s restores it to -679, inside the physical band.

    The vision node now stamps each frame at capture (`CAPTURED_AT_KEY`), which
    is the real fix; this is the fallback for a payload from an older build, so
    a version skew degrades to a fixed correction rather than to none.
    """

    RANGE_SCALE: float = Field(default=1.95, gt=0.0, validation_alias=_alias("RANGE_SCALE"))
    """Empirical correction on the pinhole range, applied to the RESULT.

    **Ships at 1.0 -- the raw pinhole -- even though the pinhole is measurably
    wrong.** That is a deliberate choice, not an untested default.

    Measured on run_20260906_192424 against pillars located by LIDAR, the
    pinhole UNDER-reads by roughly 2x: radial bias **-43.7 cm**, and all three
    encounters agree on the direction. The defect is real.

    But correcting it with a scalar makes the estimate WORSE where it matters.
    Decomposing the error along and across the robot->sign ray:

    | scale | 2D p50 | radial p50 | lateral p50 |
    |-------|--------|------------|-------------|
    | 1.00  | 47.7cm | 46.2 cm    | **9.8 cm**  |
    | 1.90  | 35.0cm | 12.6 cm    | **18.6 cm** |

    A residual ~12 degree bearing error survives the 2026-09-06 sign
    correction, and it is ANGULAR -- so lengthening the ray lengthens the
    lateral miss in proportion. **Lateral is the component the router acts on**:
    pass side, lane assignment, and whether the estimate lands on a wall. A 2D
    error metric hides this, because a large radial win masks a lateral loss.
    Independently confirmed on a wall-proximity metric over the whole run:
    estimates within 10 cm of a wall go 0% at scale 1.0 to 12% at 1.90.

    So: **fix the bearing error before the range.** Until then this stays at 1.0.

    The coefficient does not generalise either -- fitted per encounter it is
    1.81 / 2.14 / 2.59, and the global least-squares fit is 2.22, from THREE
    encounters on ONE run. When revisiting, fit the OBJECT HEIGHT (the boxes are
    ~1.75x taller than a 0.10 m pillar subtends) rather than a multiplier, and
    use at least three runs spread over the mat. An affine fit was tried and
    rejected: it buys 0.8 cm of rms for a second free parameter.

    **The lens is not the problem.** Focal solved from BEARINGS is 545-645 px,
    consistent with the shipped 621.9. Focal solved from BBOX HEIGHTS is
    1034-1088 px.
    """

    LIDAR_RANGE_FUSION_CLUSTER: bool = Field(
        default=False, validation_alias=_alias("LIDAR_RANGE_FUSION_CLUSTER")
    )
    """Gate ``LIDAR_RANGE_FUSION`` on a pillar-shaped, ISOLATED cluster.

    The unqualified fusion below takes whatever the LIDAR returns at the
    camera's bearing, which is a wall on 51% of detections and cost 28 cm of
    median position error. This is the version its own docstring asks for and
    never got: instead of a single ray, require a free-standing cluster of
    pillar width -- ``lidar_proposer.find_clusters``, which already bounds a
    run on BOTH sides by an isolation step, so a flat wall cannot qualify --
    and require its range to AGREE with the pinhole within
    ``LIDAR_RANGE_FUSION_AGREEMENT``. Failing either test, the pinhole stands.

    The division of labour is the point, and it follows the sensors' measured
    strengths rather than a preference: RANGE from the LIDAR, which measures it
    directly, and COLOUR from the camera, which is the only sensor that has it.
    The camera's range comes from bbox height through a pinhole whose focal
    solved from heights (1034-1088 px) disagrees with the one solved from
    bearings (545-645 px), so it is the weakest number in the chain -- and it
    is the number whose instability makes a believed sign teleport, measured
    2026-09-09 at p90 60.9 cm WITHIN a live commitment.

    Reuses the proposer's cluster finder rather than a second implementation:
    that code is already measured (91% recall, 0.59 m lead) and its precision
    problem is about which clusters are SIGNS, which is exactly the question
    the camera answers here. Pairing them plays each to its strength -- the
    camera says a sign is at this bearing, the LIDAR says how far.

    **Measured offline over the 2026-09-08 obstacles runs, 899 detections
    replayed against their own scans:**

    | version | fires on | range shift vs pinhole p10/p50/p90 |
    |---|---|---|
    | nearest ray (the old one) | ~100% | -0.09 / **+0.50** / **+1.61** m |
    | gated cluster (this) | **55.8%** | -0.28 / **-0.14** / +0.15 m |

    That reproduces the documented failure and removes it. The old version's
    systematic push OUTWARD -- a wall behind a sign is always further -- is
    right there at +0.50 m median with a +1.61 m tail, and gating collapses it
    to -0.14 m with a symmetric spread.

    What this does NOT establish: there is no ground truth in that replay, so
    it shows the gated range no longer carries the SIGNATURE that was measured
    harmful, not that it is more accurate. A run settles that.

    DEFAULTS OFF alongside the mechanism it gates.
    """

    LIDAR_RANGE_FUSION_AGREEMENT: float = Field(
        default=0.5, gt=0.0, validation_alias=_alias("LIDAR_RANGE_FUSION_AGREEMENT")
    )
    """Fractional range disagreement above which the cluster is rejected.

    0.5 means the cluster must sit within +-50% of the pinhole estimate. Wide
    on purpose: the pinhole is known to under-read, so a tight band would
    reject the very corrections this exists to make, while a band this size
    still rejects the wall-behind-the-sign case that broke the first attempt
    (median error there was -69 cm on a sub-metre sign).

    Not yet measured on hardware -- it is a guard, not a tuned value, and the
    first run that exercises it should re-read it.
    """

    LIDAR_RANGE_FUSION: bool = Field(default=False, validation_alias=_alias("LIDAR_RANGE_FUSION"))
    """Take the sign's range from the LIDAR ray at the camera's bearing.

    **Shipped ON until 2026-09-06 and measured to make the estimate WORSE.**

    A single ray at the camera's bearing is not the pillar. On
    run_20260906_192424 the return at that bearing is wall-shaped (implied
    chord > 30 cm) on **51%** of detections and pillar-shaped on **27%**,
    median implied chord **34 cm** against a 5 cm sign. The override fired on
    **92.5%** of detections -- the gate is ``0.05 < r < 10.0``, which is no gate
    at all -- and cost 5 cm of median position error under the old bearing and
    28 cm under the corrected one. A wall behind a sign is always FURTHER, so
    this was the second half of the outward bias that pinned believed signs to
    the walls. Even restricted to pillar-shaped returns, range error is p50
    **-69 cm** with only 16% inside 10 cm.

    Kept because the idea is sound and the implementation is what failed. A
    version requiring a small ISOLATED cluster and agreement with the calibrated
    pinhole is worth measuring -- but cluster shape alone discriminated pillar
    from wall at **54%**, near chance, so it needs its own evidence first.
    """

    FRAME_EDGE_TOLERANCE_PX: float = Field(default=2.0, ge=0.0, validation_alias=_alias("FRAME_EDGE_TOLERANCE_PX"))
    """How close to the frame border a box edge must be to count as CLIPPED.

    A clipped box's aspect ratio is not a measurement of the object's shape, so
    ``MAX_PILLAR_ASPECT`` is not applied to one. A pillar the robot is closing
    on grows until it runs out of frame: its height stops increasing while its
    width keeps going, and the ratio crosses 1.0 with nothing about the pillar
    having changed.

    Measured on run_20260906_145546 (23.4-24.6 s): a red pillar, confidence
    0.47-0.84, x_max pinned at 1536 for every frame, w/h climbing 0.33 -> 1.27
    as it approached -- rejected exactly when nearest. Across that run and
    _145909, 304 of the 500 red detections the aspect gate rejects (61%) are
    frame-clipped, against 22 of 43 for green, which is why the gate cost red
    so much more than green.

    2 px rather than 0: the detector's boxes are floats and land a fraction
    short of the border as often as exactly on it.
    """

    ROBOT_CORRIDOR_FLIP_TICKS: int = Field(default=5, ge=1, validation_alias=_alias("ROBOT_CORRIDOR_FLIP_TICKS"))
