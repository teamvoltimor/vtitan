# 0050. Escape steering is physical road-wheel degrees and follows the committed pass side

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0022

## Context

Two independent defects lived in the escape steering, and both were about the
value meaning something on one machine and something else on the next.

First, the angles were stored as a normalised fraction of full lock
(`rev_steering_scale = 0.8`, `side_correction_steer = 0.3`). The name said
"scale" while every call site used it as a magnitude (`steering = value *
side_sign`), and a fraction silently means a different physical angle on every
steering geometry: 44 deg at the bench-measured 55 deg limit, 68 deg on the
270 deg servo. ADR 0022 recorded the same hazard for the command limit, where
conflating physics and policy would have recomputed `linkage_ratio` as
55/135 = 0.407 instead of 85/135 = 0.630 and driven the servo 1.55x too far, and
the simulator would not have caught it because it never performs the servo
conversion.

Second, the escape chose its side from "which wall is nearer", while the router
commits to a pass side. They answer different questions and agree only 56 percent
of the time overall and 48 to 49 percent in a corner. On nine Obstacles bags the
K-turn steered OPPOSITE the plan's own intent in 73 of 128 episodes (57 percent),
against 51 of 239 (21 percent) for side correction and 0 of 20 for stuck reverse.
The K-turn is also 88 percent of hardware manoeuvre time. A wrong-side pass ENDS
THE ROUND, so this is not a symmetric cost.

Two smaller defects were fixed alongside. The escape side flipped every attempt,
so consecutive attempts rotated the chassis opposite ways and cancelled: measured
as four escalating escapes over 40 s with zero net translation
(run_20260805_200011). And the stuck K-turn's base side was hardcoded to 1.0 at
every reset, never read from LIDAR, so it opposed the clearer side 57 percent of
the time against 11 percent for side correction.

## Options considered

- (a) Keep the normalised fraction and the nearer-wall side.
- (b) Store physical road-wheel degrees and convert at one point; commit the side
      for a block of attempts; let the escape follow the router's committed side.

## Decision

(b). `rev_steer_deg = 44.0` and `side_correction_steer_deg = 16.5` are stored as
road-wheel degrees and read through `rev_steer_norm()` / `side_correction_steer_norm()`,
which divide by `RobotSpecs.MAX_STEERING_ANGLE`. The physical angles the escape
asks for are now readable in config, not implied by the servo.

`escape_side_commit_attempts = 2` holds one side for a block before trying the
other; 1 is the alternating behaviour this exists to stop. `_escape_steer_sign_for_attempt`
derives the side from `_escape_count` in one place.

`obstacles_escape_side_follows_committed_sign = true` (shared flag false, and
inert on Open which has no committed sign) makes the escape follow the router's
side, decided on the operator's cost function rather than an A/B, because a
wrong-side pass ends the round. `escape_side_override_min_clearance_m = 0.12` is
the absolute clearance the router's side must already have before it may be
forced; a relative band wide enough to catch the problem (0.20 m) also overrules
22 of the 49 episodes that choose correctly.

The same physical-units and side rules drive the neighbouring levers:
`obstacles_escape_mirrors_reverse = true` (208 of 243 leg pairs, 85.6 percent,
held the same sign, which is the bay pendulum outside the bay), 
`obstacles_k_turn_fit_rear_gap = true` (the reverse did not fit in 35 percent of
46 episodes), and the stuck K-turn base seeded once per sequence from a left/right
clearance comparison.

`side_correction_follows_committed_sign` (both variants) ships false: three
formulations cost the same scenario the same 12 to 15, fixing nothing, and the
corpus cannot adjudicate it. Settle it on a counter-clockwise hardware round.

## Consequences

- An escape angle no longer changes meaning when the servo changes; only the
  conversion does.
- The escape can no longer steer against the side the router has committed to,
  which is the failure that ends a round.
- The K-turn and side-correction branches still need the committed-sign override
  to be re-resolved when the router is swapped per challenge: 13,852 driving ticks
  in nine Obstacles rounds commanded an Open tier and zero commanded an Obstacles
  one until the per-challenge params were re-resolved on the router swap.
- The remaining side-correction refusal is unvalidated and off; it needs hardware.

## History

- 5f7f2f68 2026-08-05: stop the CCW position estimate tracking backwards. Add
  `escape_side_commit_attempts` and `_escape_steer_sign_for_attempt`, replacing
  three independent sites that flipped the sign every attempt.
- 4bbef1a7 2026-08-21: express speed and steering tuning in physical units.
  `rev_steering_scale 0.8` to `rev_steer_deg 44.0`, `side_correction_steer 0.3`
  to `side_correction_steer_deg 16.5`, plus the `steering_limit_deg` validator.
- d2599906 2026-09-10: cap the escape reverse at the rear room it can measure.
  `_fit_reverse_to_rear_gap`; 46 episodes, rear gap p50 17 cm / p10 7 cm, reverse
  fits 30/46 today against an expected 46/46.
- d85ba358 2026-09-11: mirror the escape reverse behind a flag, off. 208 of 243
  leg pairs same sign (85.6 percent); 52 intervals gave 0.120 m absolute wheel
  travel against 0.033 m signed (3.6x).
- 33be7da7 2026-09-11: seed the stuck K-turn's steering side from measured
  clearance. The base was hardcoded to 1.0 at every reset and never read LIDAR;
  it opposed the clearer side 57 percent against 11 percent for side correction.
- 7fd3fb24 2026-09-11: ship the mirrored escape reverse for Obstacles. The mirror
  fixes 4 and breaks 0 on go_obstacles_0007 and _0011. With the old hardcoded
  base, mirroring sent reverse opposite a forward leg that was already wrong.
- 226af9aa 2026-09-11: port the K-turn rear-gap cap into Go escape recovery.
- 0baace4a 2026-09-13: let the escape follow the side the router committed to,
  behind a flag, off. 194 hardware episodes: agree 56 percent, 48 to 49 percent
  in a corner; +0.050 m and 87 percent improved when agreeing, -0.017 m and
  16 percent when opposing. Refutes "the K-turn side is a near-tie": 18 of 37
  opposing escapes were decided on a margin of 0.20 m or more.
- 1a6a46c3 2026-09-14: re-resolve the per-challenge params when the sign router
  is swapped. Nine Obstacles rounds dropped `obstacles_k_turn_fit_rear_gap` and
  `obstacles_escape_mirrors_reverse`; escapes burned 15 to 25 percent of a round.
- 6334fd94 2026-09-14: rebuild the challenge controllers on a switch and follow
  the router's pass side for Obstacles. K-turn opposite the plan in 73 of 128
  (57 percent), 88 percent of manoeuvre time.
- 4dce7f17 2026-09-14: pin `obstacles_contact_dist` to 0.10 after it was promoted
  to live by accident: per 141 K-turn latches 0.10 catches 112 (79 percent),
  0.07 catches 28 (20 percent).
- d62560f4 2026-09-15: let a side correction refuse the router's shut side, off.
  Both formulations 12 to 15, failing on go_obstacles_0004[South/CCW].
- ff1292d5 2026-09-15: make the side-correction refusal actually reverse, and
  weigh its A/B. The flag covers 1.09 percent of sim ticks against hardware's
  19 to 25 percent; it is not the instrument.
- 6700b8a1 2026-09-15: the escape pendulum is a HANDOFF, not a mirror that fails
  to fire. Three CCW rounds: 168/172, 322/326, 72/76 reversals cross out of the
  escape (95 to 99 percent are the escape-to-planner handoff).
- Lateral displacement follows the STEERING side in 86 to 88 percent of measured
  episodes and the nose's side in 12 percent. Because a K-turn is 100 percent
  reverse, `_committed_sign_steer_sign` uses the steering side, not the nose's.
- Resolving the speed tier must cover the sign lane's band changes: they demand
  0.335 to 0.371 m of turn radius over the 256-scenario corpus, against the speed
  curve `R = 0.053 + 1.86v`.
- `escape_preferred_sign` written on `self._debug` before the escape branch hands
  the snapshot over published on 0 of 2,867 ticks of a full Obstacles scenario. It
  must be set on the local `debug` object the escape branch reassigns.
- A side correction is a 16.5 deg nudge lasting 0.20 s, not a manoeuvre. Measured
  2026-09-11: a latched manoeuvre supplied 100 percent of the commanded steering
  while `steer_target` went unpublished on 92 to 93 percent of its ticks, so the
  two layers alternated and undid each other at 2.7 to 4.4x absolute over signed
  wheel travel. The correction is ADDED to the plan, not substituted for it.

## Cross-references

- 0022 is superseded: its physics-vs-policy split is the rule this story applies
  to the escape angles.
- 0023 (escape durations in seconds) stays separate; this story does not cover
  duration units.
- 0038 (bay exit mirrors its reverse-leg steering) is the direct ancestor of
  `escape_mirrors_reverse`.
- 0034 and 0035 cover the pass-side rule the committed side obeys; 0047 covers
  contact reverse.
