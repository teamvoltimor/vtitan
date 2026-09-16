# 0088. Refuted config knobs ship off and stay documented

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0047

## Context

Several mechanisms were built, measured and found not to earn their place. The
risk is re-proposing them from the same reasoning, or reading a `false` as
unfinished work. The contact reverse is the clearest case: on
run_20260906_121254 the robot held +0.152 m/s for ten seconds against a green
pillar, and the back-off was meant to address it.

## Options considered

- (a) Delete the refuted code so it cannot be re-enabled by accident.
- (b) Keep the code, ship it off, and record the measurement so it is not
      rediscovered.

## Decision

(b). The mechanisms stay in config at their off value, with the refutation in the
comment, so a `false` does not read as unfinished and the behaviour can be
revisited without rediscovering it.

`contact_reverse_ticks = 0` with `contact_reverse_cooldown_ticks = 20`: gating on
the pose trail gave back the timeouts and most of the extra collisions and gave
back the stall benefit with them, because the trail rarely confirms 6.1 cm of
covered ground at the moment the chassis is against something. What survives is
+5 wall contacts against a flat headline, so the behaviour is not earned in either
form. The hardware case remains real and unaddressed: this mount has no rear
sensing, so a seeing reverse is impossible, and the simulator says a blind reverse
is not the answer.

The other refuted singles ship off the same way: `retrace_escape = false` (halves
wall strikes 13 to 7 but gives back most of the sign gain 41 to 53);
`side_correction_blends = false` (the qualifying forward nudge is 0.8 percent or
less of hardware ticks, so it cannot move the alternation it was written for);
`tick_router_during_maneuver = false` (off and unvalidated, not refuted);
`replan_blend_ticks = 0` (refuted twice); `sign_lane_split_overlap = false`;
`sign_lane_skip_unsatisfiable = false`; `advance_past_passed_waypoint = false`;
`stale_target_rescue = false`; `sign_contact_evade`, `sign_lidar_align`.

`slot_colour_thaw_evidence` (the slot-map colour A/B) is REFUTED and must not be
shipped on the pre-correction table: re-measured on the corrected LIDAR occlusion
band the old fixture-failure gain disappears and the arm's sign flips. A knob
measured against a broken baseline did not merely overstate.

## Consequences

- A refuted knob cannot silently become live; its measurement is one grep away.
- The contact-reverse hardware case is documented as unsolved rather than closed.

## History

- 6102224f 2026-08-17: retrace measured and rejected (wall 13 to 7, sign 41 to
  53).
- 2a0e9e28, dd0cb0c6, c94e8a31 2026-08-31: the unconfirmed-width bias and deferral
  that superseded the replan blend; confirm `replan_blend_ticks` refuted with both
  confounds removed.
- 946a83e0, bb32958c 2026-09-06: the contact reverse, then its trail gate; three
  arms off/ungated/trail-gated all leave the value 0.
- 40b79421 2026-09-05: write the code-only tuning constants into TOML, marked as
  refuted.
- aa54fb42 2026-09-11: price the reverse-to-buy-road manoeuvre and recommend
  against building it; notes `CONTACT_REVERSE_TICKS = 0` so the centred back-off
  never fires.
- b50b889c 2026-09-11: `side_correction_blends`, refuted by its own scoping.
- 88148831 2026-09-12: `tick_router_during_maneuver`, off and unvalidated.
- b982e109 2026-09-13: create ADR 0047.
- f5fa3a61 2026-09-14: correct what `retrace_escape` is and which evidence refuted
  it.
- `slow_dist` sweep, 256 corpus blind, in-time/clean/laps>=3/wall/park/timeouts/
  stuck: 0.35 65/114/158/25/19/106/28; 0.25 62/112/159/30/19/110/26 (shipped);
  0.20 65/108/153/30/17/108/29; 0.15 67/112/154/29/15/105/32; 0.13
  64/110/155/31/15/107/30. In-time spans 62 to 67 over a 2.7x threshold range with
  no ordering; timeouts and wall flat.
- 2026-09-15: `slot_colour_thaw_evidence` A/B on 16 Obstacles fixtures, fail =
  collision, wrong-side pass or under 3 laps. base 7 failing, ctl 16, `ev=1.5` 9,
  `ev=2.0` 10, `margin=0.5` 6, `thaw=1.5` 7, `thaw=3.0` 5, `thaw=1.5,margin=0.5`
  6. All measured while the simulator's LIDAR occlusion band sat 180 deg from where
  the chassis puts it.
- c8b1ae79 2026-09-16: re-measure the same 16 fixtures and fail rule on the
  corrected band. base 5 (0003 0004 0005 0009 0014), ctl 16, `thaw=3.0` 6,
  `margin=0.5` 6. The arm is REFUTED: its old 7 to 5 was entirely 0000 and 0013,
  which the band correction fixes by another route (both pass at base now); the
  only colour failure left is 0008, which no arm ever fixed (single view at 5 s on
  top of the first pillar), and both arms now turn it from a pass into a failure.

## Cross-references

- 0047 is superseded; its decision is carried above.
- 0055 and 0050 own the escape family the contact reverse belongs to; 0057 owns
  the corridor follow the replan blend belongs to; 0051 owns the sign lane.

## Evidence

- `collision_thickness` never affected the headless sim: `track_model.py` defines
  `_WALL_COLLISION_HALF` and never uses it; only the Go Gazebo SDF generator reads
  it.
- All three `bay_exit_*` constants are refuted and `--known-start` changes nothing,
  so the in-bay trap is not a localization or tuning problem.
- The nine-arm yaw screen is refuted (boundary yaw spanned 0.6 deg against a
  28.2 deg threshold; the 0.12/0.24 lookahead's 5-to-2 sign-collision change is
  Poisson noise; a 1.5x steer-rate worsened collisions).
- Maximin placement and routing is refuted in-tree (sign 199 to 168 but wall 3 to
  61); do not re-try a placement change before reducing tracking error.
- `sign_clearance_margin_m` is inert on this path, while the binding
  `wall_clearance_margin_m` is unreachable because `clamp_lateral` ignores runtime
  context.
- Four inert knobs were found in `diag_sign_sweep.py` (most recently `arc`,
  byte-identical at 0.35 and 0.45); count invocations before trusting any constant
  A/B.
- `slow_dist` was ruled out as the timeout cause; at about 266 escape starts per
  run the manoeuvres dominate the clock, so a ladder retune is secondary.
- Navigation tuning has no environment override (`NAV_TUNING__*` is silently
  ignored); set arms in TOML or via `model_copy`.
- Sweep method: `--corpus` inside a git worktree reports a clean 0/0 (pass
  `--scenarios-dir`), and an equivalence check passed at 16 and 128 scenarios but
  failed at 256.
- The sign-router refuted or unvalidated knobs stay off or inert:
  `retrace_escape`, `sign_contact_evade`, `sign_lidar_align` (steering at a
  LIDAR-resolved pillar turned the robot round: U-turns 19x, rev-runs 7x),
  `sign_lidar_propose`, `stale_target_rescue`, `sign_lane_split_overlap`,
  `sign_lane_skip_unsatisfiable`, `sign_lane_deform_fallback_m`. `slot_sign_map`
  is on; its comment still says "SHIPS OFF", a stale note. `escape_mask_radius_m`
  and its cluster association are in 0056.
- `side_correction_blends` raw split: the qualifying forward nudge is 44/12/0/0
  ticks (0.8/0.3/0.0/0.0 percent) against 550/572/143/88 reversing over the four
  2026-09-11 Obstacles rounds; side corrections dominate 184/198 ticks against the
  K-turn's 22.
- `tick_router_during_maneuver`: three 2026-09-12 rounds gave 183 episodes, 22.3
  percent of all ticks (150 s of 686 s), 179 of 183 (97.8 percent) holding a
  constant steering value up to 44 ticks / 2.2 s; the sim exercises
  `side_correction` on 1.09 percent of ticks against hardware's 19-25 percent.
  Validate with `scripts/bag/diag_bag_planner_silence.py` and `diag_bag_pass_side.py`
  against those rounds.
- `advance_past_passed_waypoint` false: measured to stall the waypoint index at
  corners (measurement in 0057). `forward_only_reseek` also ships false and no ADR
  owns it: the reseek-direction gate is undocumented and should be a deliberate
  decision.
- `sign_lidar_align` full A/B, 256 corpus off to on: in-time 148 to 146, laps>=1
  179 to 172, U-turns 2 to 38 (27 of 256 runs), rev-run 1 to 7, unscored 1 to 7,
  timeouts 69 to 65, stuck 32 to 30. Feeding the return to discovery, or using it
  to bias the classifier ROI, does not require pointing the chassis at the sign.
- `sign_lidar_align_deadband_deg` caution: the U-turn refutation turns on this gain
  and deadband; do not raise them while the law ships off.
- `retrace_escape` shares its session and 640-case sweep with
  `steer_cap_from_commit_distance`, so that sweep is its evidence provenance.
- `side_correction_blends` in the sighted sim corpus: `side_correction` ran 1.09
  percent of 23,290 ticks and the blend gate (`speed >= 0`) was satisfied on ZERO of
  them because every sim `side_correction` is in reverse, so the 256-run A/B was
  void.
- The flat 256-run A/B: in-time 98 against 98, laps>=3 99 against 99, collisions
  16 against 15, pass-side violations 0 in both arms. Void, because the gated branch
  is never reached in the corpus.
- Do not read a non-zero gate as reachability: the first version of the check
  counted 8 ticks in three rounds as REACHABLE; the verdict threshold must be a
  share of the gated manoeuvre.
- Reverse-to-buy-road, priced on 129 bags / 1113 passes: crossings fail 67.8
  against 6.9 percent and are 215 of 248 EXECUTION failures; a failed crossing
  drives 0.41 m against 0.92 m and under 0.25 m of road fails 89.5 percent. The
  ship's trail gate vouches for the shortfall only on a subset of short passes, the
  added time is about 2 legs per firing against the 180 s limit, and approaching at
  0.15 m/s erases most shortfalls: the recommendation not to build it.
- `aim_point_demands_an_impossible_radius`: 58 percent of ticks ask for 0.23 m
  against the floor. The simulator also under-rotates 40-50 percent during
  manoeuvres.
- `sign_lidar_propose` A/B, 16 fixtures x 6 seeds x 2 arms, blind: off in-time 59 /
  laps>=3 71 / collided 18 / pass-side 0; on 58 / 66 / 23 / 0. The off arm
  reproduces the `VISION_RANGE_MODEL` sweep's on arm (59/71/18/0).
- The c8b1ae79 LIDAR band correction moves this A/B's fixture failures 7 to 5
  while 0086 records the same correction moving the corpus 28 to 22; do not
  conflate the two counts.
- The first version of the slot-colour A/B patched
  `sign_discovery.ObservedSignMap`, which the production router no longer uses,
  and returned byte-identical output that read as a clean refutation; that is why
  `ctl` is a mandatory arm.
- The budget put 62 percent of the missing 1.4 m of anticipation in perception
  already (2026-09-11 `ev=X` rationale); 0051 records the 1.4 m decay, not this
  split.
