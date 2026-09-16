# 0051. Sign avoidance rewrites the path into a lane

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0045

## Context

The first sign-avoidance mechanism only nudged the pure-pursuit target: it
overrode the lateral coordinate of the carrot within `activation_dist_m` while
the planned polyline never moved. That fails at the source, because
`cross_track_error` is the controller's own error signal and the input
`select_lookahead` gates on, and it was measured against the undeformed path, so
it stayed near zero for the whole pass. The tracker never learned it was supposed
to be somewhere else; it only ever saw a carrot pushed sideways, and the offset
closed only asymptotically, a consistent shortfall of 6.3 to 6.6 cm across four
independently traced scenarios (go_obstacles_0009/0011/0020/0046). Seven levers
that changed when and how hard the chase happens (activation distance, offset
magnitude, lookahead, steering gain, sign-aware lookahead x2, sign-aware speed
x3) were all measured flat or worse.

Two signs 0.50 m apart in a 1.0 m corridor, which is ordinary WRO geometry, also
made the per-tick nearest-wins race flip the chosen sign mid-approach and jump the
commanded lateral line with no runway left.

## Options considered

- (a) Keep nudging the carrot (per-waypoint deform) and tune it harder.
- (b) Rewrite the planned polyline into a lateral lane so crosstrack, the
      lookahead gate and the target search all agree the lane is the path.

## Decision

(b). `apply_sign_lanes` moves the polyline itself: a lateral offset with a ramp
(`sign_lane_ramp_m = 0.9`) into a held plateau (`sign_lane_hold_m = 0.25`) around
each routed sign. The transform is 1:1 and order-preserving (same waypoint count,
same order, only laterals change), so every index-keyed invariant in
`CoreNavigator` (`_waypoint_index` advance, the lap-seam wrap, `replace_path`'s
re-seek) survives it unchanged; the index is deliberately NOT re-seeked after a
lane refresh.

`sign_lane_planner = true` and `sign_lane_suppress_deform = true`: the lane is the
production avoidance path and the older per-waypoint deformation is suppressed.
`sign_lane_offset_frac = 1.0` (full offset). Open Challenge is untouched by
construction, not by flag value: it builds no `SignRouter`, so the transform
returns its input byte-for-byte, pinned by
`test_open_challenge_path_is_untouched_even_with_the_flag_on`.

`sign_lane_gap_centre_frac = 1.0` moves a squeezed plateau toward the midpoint of
the free gap (0.0 is the clamped placement). This REVERSES a 2026-08 refutation;
see History.

The lane inherits the planned centreline rather than re-deriving it: `base_lateral`
is the median of the straight waypoints' laterals, which already carries whichever
centre bias the corridor width selected (ADR 0028). It must never flatten that.

## Consequences

- The lane supplies the runway the sign-commit hysteresis lacks: it spreads the
  lateral travel over the whole corridor straight instead of demanding it in the
  last 1.4 m.
- Lanes are always recomputed from the planned path, never from `_waypoints`, so
  blind-discovery refinements cannot compound the offset.
- Contested corner-runway waypoints belong to exactly one lane (`_assign_owners`);
  before this, 931 of 1413 borrowed waypoints (66 percent) landed where neither
  lane asked, diverging by up to 215 mm.
- `sign_lane_gap_centre_frac = 1.0` is NOT validated on hardware; the whole
  adjustable range is 31 mm of plan and the 6.3 to 6.6 cm tracking argument is
  untouched.

## History

- 877ecee6 2026-08-16: route Obstacles past signs by moving the path, not the
  carrot. Adds the transform and `sign_lane_planner` shipping false. Full 256
  sighted corpus: collisions 234 to 70, laps>=3 22 to 186, in-time 19 to 128;
  corner entry dominates (subset64 collisions 53/42/32/17/22/20/20 across
  0.00/0.20/0.35/0.50/0.65/0.80/0.95), without it the feature is worth ~2 percent.
  Carrot override stays rejected: sighted in-time 139 to 88 for no collision gain.
- 85163513 2026-08-16: give Obstacles its own centreline bias, measured at 0.15 m.
  Full 256 sighted lane on: collisions 70 to 62, laps>=3 186 to 194, in-time
  128 to 139. Explicitly a compensation, to be re-derived downward if drift is
  fixed.
- 032a4c24 2026-08-17: ship the sign lane planner on. Same-run OFF arm: sighted
  collisions 242 to 62, laps>=3 14 to 194, in-time 13 to 139; blind 248 to 231,
  laps>=3 8 to 27. Adds the `SectorRanges.measured` reverse guard.
- c09d1db0 and 7afb065a 2026-08-20: gap-centre a squeezed plateau, REFUTED and
  reverted. Blind 256: fractions 0.25/0.40/0.55/0.70/1.00 gave 213/211/217/220/229
  against 202/256, wall collisions 3 to 61, laps>=3 56 to 32. The lever is 3.1 cm
  against a 6.3 to 6.6 cm crosstrack shortfall.
- 3b36bc49 2026-08-25: `sign_lane_skip_unsatisfiable`, measured and shipped OFF.
  A corner-diagonal sign labelled on the wrong face had its clamped target capped
  past the corridor bound, on the wrong side of the sign (traced green WEST spec
  at x=0.993: lane wants 1.272, clamp gives 0.781, 0.212 m wrong side; specs past
  a corner wrong-side 29 percent against 5 percent inside a straight). Skipping
  confirmed the mechanism (collisions 87 to 71) but pass-side rose 140 to 155, the
  column it exists to improve.
- 7d376325 2026-08-25: relabel a sign whose lane target is unsatisfiable in its
  corridor, fixing the cause upstream via `satisfiable_corridor`. Blind 256 A/B:
  pass-side 140 to 121, laps>=3 33 to 42, dual-corridor 22 to 14; cost sign
  collisions 60 to 77 against wall 27 to 17 (partly survivorship).
- 8f535d10 2026-08-25: `sign_lane_split_overlap`, measured and REFUTED. Nested
  plateau dug a hole in the enclosing hold window; pass-side fell 121 to 110 but
  collisions rose 94 to 106 and sign collisions/lap 0.336 to 0.410. The 4.3x lift
  was correlation from the measured 2.50x discovery duplication.
- a67345a9 2026-08-26: decide a corner sign's corridor by depth, not nearest face.
  `depth_consistent_corridor`; assigned-axis disagreeing with the invariant fell
  42.1 percent to 0.0 percent (0/2777). Corpus A/B: pass-side 121 to 73, laps>=3
  42 to 56, in-time 29 to 40. Unvalidated on hardware.
- 59c926a1 2026-09-10: the corridor-gate axis must be BLIND, and the shipped 5
  already wins. Sighted corpus cannot exercise discovery, so four arms were
  byte-identical and void; run properly, flip_ticks 5 leads on all four headline
  metrics.
- 7618f32b and 4ef36c7b 2026-09-12: reinstate then flip gap-centring to 1.0. The
  2026-08 refutation is declared OBSOLETE, not wrong: it was measured without the
  depth pin, with parking after the final lap and at hold 0.25, on a baseline of
  202 collisions where the new one scores 61. New 2x3 factorial: 0.40/0.55/0.70/
  0.85/1.00 with the shipped inner-wall rule gave 61 to 35 collisions, +30
  laps>=3, +39 in-time.
- e52bca88 2026-09-12: gate the target search on path SENSE, and retract the
  deform half. `target_sense_gate` off: Obstacles sighted in-time 98 to 100,
  blind 99 to 102, collisions 20 to 17; Open 128/128 both arms. Retracts the
  deform-sense evidence: `sign_deform_magnitude_m` is a counterfactual under
  suppression, computed unconditionally and never applied.

## Cross-references

- 0045 is superseded: the lane consumes the router's committed sign set through
  `lane_specs`, so the commit hysteresis is what keeps the lane from churning
  under the two-sign geometry; its corpus-flat caveat applies here too.
- 0028 (waypoint centre bias split) is the centreline the lane inherits; it stays
  separate for now.
- 0034, 0035 and 0043 are the scoring rules every skip/split/relabel decision was
  judged against.
- Stale code docstrings in `sign_lane.py` and `routing.py` still say
  `gap_centre_frac` ships at 0.0; the TOML/schema (1.0) is authoritative.

## Evidence

- Sign collisions are 154 in both sighted and blind: handing the robot the full
  layout changes them by zero, and the real mechanism is tracking error
  (crosstrack p90 20.28 cm against a 17.55 cm planned gap; 79 percent of
  collisions in `normal_drive`).
- `sign_lane_hold_m` is capped near 0.25 m because two signs are 1.00 m apart; at
  0.50 m adjacent plateaux meet and one sign's plan governs another's pass
  (wrong-side 58 percent against a 13 percent base), and a 0.55 sweep collapses
  laps>=3 to 2.
- The sign lane and the clearance guard are mutually over-constrained: there are
  geometries where no lane satisfies both.
- `sign_lane_relabel_unsatisfiable` is INERT on the corpus (bit-identical on and
  off), so its 140 to 121 gain is no longer live evidence; kept True only as a
  backstop for geometry the corpus lacks.
- `_hold_committed_path`, `_apply_path_wall_budget` and `deform_waypoint` are all
  INERT at shipped config, and `_in_corner_zone` is a coordinate box, not a turn
  test.
- All 256 corpus scenarios start dead centre (lateral 0.50) in a 1.0 m corridor,
  so narrow and off-centre starts are never exercised.
- `sign_clearance_margin_m` was raised 0.075 to 0.10 on 2026-09-06 from hardware
  (run_20260906_184717): the lateral shortfall is gone (0.247 m achieved against
  0.239 commanded) and what remains is a scatter tail (p10 0.033 m, 21 percent
  within 10 cm of the pillar). The 256 corpus moved together (laps>=3 149 to 151,
  sign collisions 5 to 4). `activation_dist_m = 1.40` and `passed_dist_m = 1.60`
  (stock 0.80/1.20 gave 47 finishes against 27); `deform_depth_buffer_m = 0.5`;
  `settle_ticks = 150`; `detection_match_dist_m = 0.30`; `min_confidence = 0.25`.
- `sign_aware_speed`, `sign_aware_lookahead`, `depth_pin` (`pin_corner_guard`,
  `pin_heading_guard` at 35 deg), `pair_handoff_span_m = 0.30`,
  `sign_deform_speed_threshold_m = 0.02` and `explore_lap_speed_frac = 1.0` are
  the shipped coupling knobs. `sign_aware_lookahead` ships ON against a flat
  headline because a pass-side violation ends the round (7 round-enders to 1).
  `pair_handoff_span_m` blends to the next sign's line only after the REAR clears
  (blending from the pose ORIGIN cost 12 to 16); its benefit is unprovable on this
  corpus, which has no pass-side assertion.
- `_hold_committed_path` measured on subset64 blind: 10 percent of rebuilds moved
  the path away from the chassis by up to 0.301 m, essentially a whole lane
  offset.
- The lane and the deform answer the same question. Measured 2026-09-11 over 129
  bags: on failed CROSSING passes the lane reached the legal side on 37.4 percent
  of passes while the deform's target was on it on 47.3 percent. Where the deform
  did land legal the pass failed 62.6 percent against 82.1 percent where it did
  not (chi2 = 17.0, p = 4e-5). The deform is used only where the lane is not
  already holding the target on the legal side, so the two can never add.
- `explore_lap_speed_frac` measured on subset64: 78 percent of blind failures
  happen during lap 1, 9 percent in lap 2, none in lap 3, so the cap slows only the
  discovering first lap, when the map does not yet exist.
- `lateral_offset` is chassis half-diagonal + sign half-width + the clearance
  margin, derived rather than restated: `TestLateralOffsetTracksChassis` measured
  that a 0.28 mm chassis-width change was enough to flip a corpus scenario, so the
  offset must track the live chassis geometry.
- `activation_dist_m` MUST stay below `passed_dist_m` (`SignRouterConfig` enforces
  it); inverting the pair engages and retires every sign a metre out and silently
  disables avoidance for the whole run (256 of 256 collisions). The pair is
  deliberately 1.40/1.60 rather than the measured peak 1.60/1.80: 1.80 costs 19
  runs against 1.60, and 1.40 keeps twice the margin for hardware pose error.
- `deform_depth_buffer_m`: too small and avoidance switches OFF during the final
  approach to any sign at grid depth 1.0 or 2.0 (the corners, where two-thirds of
  WRO signs sit); 0.70+ is worse again (38 finishes).
- `commit_hysteresis` is ON since 2026-09-07; the corpus still reads flat but the
  instrument changed: hardware duplicate nearest-neighbour separation is 0.21 m
  against 0.012 m in sim. Replaying recorded 09-07 detections while holding the
  commitment cuts aim-point jumps over 0.15 m from 38 to 21 with the committed-tick
  count unchanged at 1538. It was reported from the track first: the robot lines
  up, keeps correcting and arrives badly placed, and median steering flips during
  an approach is zero, so the target was moving.
- `sign_aware_speed`, measured blind on the 256 corpus off to on: clean 77 to 79,
  in-time 71 to 73, rev-run (9.21) 14 to 9, unscored 14 to 9, collisions 15 to 15
  (sign 6 to 4, wall 9 to 11), timeouts 26 to 27. The mechanism is not established
  (escapes flat at 17293 against 17283); it is Obstacles-only because it gates on a
  deformation the router applied. The prior refutation was measured SIGHTED
  (curvature-limited); the blind run is the information-limited competition case.
- `sign_lane_hold_m` hardware study over 141 bags: the believed pillar sits p50
  +0.122 m past the centreline for crossing failures, +0.044 for successes and
  -0.198 for already-legal passes, with the lookahead on the ramp on 47 percent of
  closest approaches, and every one of those bags was recorded at hold 0.25. On the
  256 corpus, hold 0.40 against 0.25: collisions 25 against 21, laps>=3 222 against
  225, in-time 191 against 218, escapes per lap 7.01 against 4.34.
- `sign_lane_deform_fallback_m`: the intended lane offset is 0.28 m and the
  failures' lane peaks at 0.130 m. It ships off because the simulator's sign map is
  EXACT, so the lane materialises and this branch barely fires; a flat sim A/B
  would measure the sim.
- `sign_aware_lookahead`: crosstrack is measured against the RAW path, so it never
  rises during a sign pass and never arms the short lookahead on its own. 256
  corpus off to on: pass-side 2 to 0, rev-run 5 to 1, collisions 8 to 5 (wall 4 to
  0), stuck 26 to 32, timeouts 65 to 69. The field comment said "extend" until
  2026-09-06; it shortens.
- `pair_handoff_span_m` hardware evidence over four 2026-09-14 rounds and 41
  passes: a pass begun on the wrong side grazes 4.8x more often (23.8 against 5.0
  percent) and finishes wrong-side 2.9x more (14.3 against 5.0 percent);
  commitment lands at p50 0.498 m where the crossing needs about 0.614 m,
  publication costing 0.317 m and the commit criteria 0.266 m of the 1.081 m the
  camera gives.
- `sign_deform_sense_guard` counterfactual measured on run_20260912_064539:
  wrong-sense targets carry deform p50 0.554 m against 0.031 m for right-sense
  (18x), on a quantity that was never applied; the lane is the candidate that acts
  on that wish.
- Holding full offset from activation and fading only on the way out was measured
  over the 256 corpus at ramps 0.20/0.40/0.70 m: byte-identical without the depth
  pin (182 collisions at every value) and slightly worse with it (in-time 137 to
  135); the lateral clamp binds first, so the ramp has nothing to give.
- Widening the `_in_lane_span` lateral test as well (to the far corner-square edge,
  gated to a sign's plateau depth) was catastrophically worse on the full 256
  corpus and reverted; at that depth it pulls in the neighbouring corridor's own
  arc points.
- `split_overlap` traced on a WEST corridor holding three specs: a plateau at
  2.15..2.65 lateral 0.781 contained a neighbour's endpoints at 2.22 and 2.24
  lateral 0.325, and the plan passed the sign on that dip.
- `_prefer_committed`: measured over the 256 corpus, 28 of 229 collisions had the
  winner change during the fatal approach.
- Inverting `activation_dist` and `passed_dist` scored 256 of 256 collisions with
  zero laps completed on the corpus, up from 209.
- Disabling `PinCornerGuard` restored the pin and cost 11 wall collisions in the
  sweep, attributed 2026-08-01 to the pre-guard arm of the same sweep.
- The depth-pin heading guard was traced on `go_obstacles_0049` (subset64,
  sighted): the pin held a commanded point frozen for 46 ticks while the yaw
  rotated 67 deg mid-corner, because `PIN_CORNER_GUARD`'s position-only check
  never tripped (the raw waypoint stayed squarely in its corridor even though the
  chassis had already curved into the turn).
- `lateral_offset` is 0.2786 at the current chassis, derived from the chassis
  half-diagonal.
