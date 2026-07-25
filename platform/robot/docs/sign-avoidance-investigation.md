# Obstacles Challenge — sign avoidance investigation

Reference log for the traffic-sign collision problem: what was fixed, what was
tried and rejected, and the geometric limits that constrain any future fix.

Written 2026-07-25. Baseline commits: `01ca617` (SignRouter fixes),
`fd33fd5` (obstacle physics).

## The problem

In closed-loop obstacles scenarios the robot drives over traffic signs it has
already correctly identified and routed around. Before the work logged here it
was invisible: the simulator reported `collided=False, success=True` while
driving straight through them.

## The enabling fix (committed)

`track_model.py` had **zero** references to signs, and it owns both
`raycast_scan` (LIDAR) and `footprint_collides` (collision). Signs and parking
blocks existed only as `SignRouter` waypoint deformations — no collision
geometry, invisible to LIDAR. Every "scenarios complete without collision"
result in the obstacles test battery was therefore blind to the one failure
mode it exists to catch.

`fd33fd5` makes them physical (`ObstacleBox`, `obstacles_from_metadata`,
collision + raycast integration, `lidar_sees_obstacles` flag). **Do not regress
this** — without it no navigation change can be evaluated.

Consequence: 15/16 scenarios now report a collision, and four obstacles tests
fail. Those are true pre-existing failures, not regressions.

## Sign-router bugs found and fixed (committed, `01ca617`)

1. Deformation math keyed to the *robot's* corridor label, not the *sign's*.
   They legitimately disagree at a corner, deforming the wrong world axis.
2. Signs already behind the robot still won the nearest-wins rule. Measured on
   `go_obstacles_0000`: a sign 0.372m *behind* outranked one 0.421m *ahead*.
3. Offset taper keyed only to the lookahead target, which runs 0.2-0.4m ahead —
   so the offset had decayed ~35% at the moment the robot drew level with the
   sign. Now tapers on `min(robot-to-sign, target-to-sign)`.
4. An inapplicable nearest candidate returned the waypoint untouched instead of
   falling through to the next candidate, blanking avoidance across the whole
   corner-exit stretch.

Net effect: worst clearance on `go_obstacles_0000` 0.026m -> 0.060m. Real
improvements, but they do not solve the problem.

## Hypotheses tested and REJECTED

Each was measured across all 16 obstacles scenarios. Do not re-try these
without new information.

### Lower speed alone — no effect (but see the caveat)

Ackermann minimum turn radius is set by steering angle and wheelbase and is
**independent of speed**. Slowing down buys time but not a tighter path.

| Speed (default 0.20/0.40 lookahead) | Result |
|---|---|
| 0.50 / 0.30 / 0.25 m/s | identical clearances; 37 overlaps at every speed |

**Caveat — speed matters once the lookahead is short.** With the default
lookahead the steering-rate limit never binds, so speed is irrelevant. Shorten
the lookahead and it does bind, and a lower top speed then buys more steering
travel per metre advanced. See the combined result below: the same 0.12/0.24
lookahead gives 12/16 at 0.5 m/s but 9/16 at 0.30 m/s. Neither setting is worth
much alone; together they are the best configuration found.

### True pure pursuit — worse, reverted

`waypoint_controller.py` documents pure pursuit but implements a P-controller on
bearing error (`steering_rad = steer_kp * angle_error`), which ignores target
distance and cannot compute the curvature to converge onto an offset line.
Replacing it with `kappa = 2*sin(alpha)/L_d`, `delta = atan(wheelbase*kappa)`:

- sign collisions 15/16 -> 14/16 (essentially unchanged)
- **broke 6 other tests**, including 3 Open Challenge deviation-recovery cases
- gain/speed sweep (kp 1.0/1.5/2.0 x 0.30/0.25 m/s) never beat 13/16, and
  0/16 completed 3 laps

The bearing-P law is theoretically wrong but tuned to this system. Replacing it
is risk without reward *on its own*; any retry must come with retuned lookahead
and a plan for the Open Challenge regressions.

### Larger lateral offset — trades sign hits for wall hits

The corridor is saturated; there is no room to simply push further out.

| `lateral_offset` | Result |
|---|---|
| 0.20 | 14/16 collisions |
| 0.23 / 0.26 | 15/16 |
| 0.29 | 13/16 |
| 0.35 | **wall collisions** |

### Path-level deformation — planning is NOT the bottleneck

Prototyped shifting the waypoint path itself perpendicular to the local tangent
(works on corner arcs, unlike the current world-axis override). The resulting
path is genuinely clean — clears every sign by 0.28-0.47m at offset 0.30 — yet
the sim still collided 15/16.

### LIDAR-visible signs confusing the collision controller — SUPERSEDED

Originally recorded here as "not the cause", on the grounds that toggling
`lidar_sees_obstacles` gave 15/16 either way.

**That conclusion was wrong**, and the error is worth keeping visible: it
compared only the *collision* count. Collisions are not the only way a run
fails. Toggling the flag swaps one failure mode for another — the totals stayed
flat while the behaviour changed completely. See
"Router vs. collision-controller conflict" below for what that flag actually
trades off. When judging a change here, always report collisions, laps
completed, timeouts and success together; any one alone hides the others.

## What the numbers actually show

**Every collision is a sign at grid depth 1.0 or 2.0** — exactly `CORNER_MIN`
and `CORNER_MAX`. Zero collisions at depth 1.5.

```
hit signs on a corner boundary (depth 1.0/2.0): 14
hit signs mid-corridor (depth 1.5):              0
```

The WRO sign grid has three depths (1.0, 1.5, 2.0), so **two-thirds of legal
sign positions sit on a corner boundary**. Mid-corridor avoidance already works.

**Collision geometry:** collisions fire at ~0.18-0.21m centre-to-centre because
the robot hits **corner-first while still turning** — its half-*diagonal* is
0.180m (`sqrt(0.15^2 + 0.10^2)`), not its half-*width* 0.10m. Required clearance
mid-maneuver is therefore ~0.205m, not the 0.125m a straight side-pass needs.
`lateral_offset = 0.20` sits right at that threshold.

**Tracking accuracy vs. available slack — the binding constraint:**

- Measured cross-track error against a known-clean path:
  median **4.7cm**, p90 **8.4cm**, max **11.7cm**.
- For a red sign in the outer lane passed outward, the gap between the wall
  collision boundary and the sign edge is 0.335m. With a 0.20m-wide robot that
  leaves **+-6.7cm of slack**.

Required accuracy (<6.7cm) is tighter than actual (11.7cm). That is the whole
problem. It is not offset, not speed, not planning.

## Best configuration found — lookahead tuning

Shortening lookahead is the tightest-tracking knob and the single biggest
improvement found. Swept at 0.30 m/s:

| lookahead (short/long) | Collisions |
|---|---|
| 0.20 / 0.40 (current default) | 16/16 |
| 0.18 / 0.36 | 13/16 |
| 0.16 / 0.32 | 12/16 |
| **0.14 / 0.28** | **9/16** |
| **0.12 / 0.24** | **9/16** |
| 0.10 / 0.20 | 13/16 (over-tightened, weaves) |

Broad optimum across 0.12-0.14, plateauing at 9/16.

Combined with the speed cap, which only pays off once the lookahead is short:

| Configuration | Collisions |
|---|---|
| 0.20/0.40 lookahead, 0.5 m/s (defaults) | 16/16 |
| 0.12/0.24 lookahead, 0.5 m/s | 12/16 |
| **0.12/0.24 lookahead, 0.30 m/s** | **9/16** |

**Landed** as `NavigationTuning.for_obstacles()`, applied by `ScenarioSimulator`
whenever the challenge is obstacles and no explicit tuning is passed. This keeps
one steering law for both challenges and expresses the difference as tuning,
rather than forking the shared `CoreNavigator`.

Note it is a typed factory, **not** a JSON profile: the existing
`tuning_profiles/*.json` files cannot be loaded at all. They use lowercase keys
(`contact_dist`) while the dataclasses expect uppercase field names
(`CONTACT_DIST`), so `NavigationTuning.load_from_json` raises `TypeError` on
every one of them. That loader bug is unrelated to sign avoidance and still
needs fixing separately.

**Larger offset does not combine with tighter tracking** — it actively hurts.
`lateral_offset = 0.20` remains the optimum:

| lookahead | offset 0.20 | offset 0.23 | offset 0.26 |
|---|---|---|---|
| 0.12 / 0.24 | **9/16** | 15/16 | 16/16 |
| 0.14 / 0.28 | **9/16** | 11/16 | 15/16 |

Interpretation: past ~0.20 the deformed line runs too close to the outer wall,
and the wall-clamp (`_clamp_lateral`) plus collision-avoidance reactions fight
the router. The corridor genuinely has no more room.

### Clamp sized on the chassis half-diagonal (9/16 -> 7/16)

Classifying what each remaining footprint actually overlapped showed the 9
failures were **three different bugs**, not one:

| Failure mode | Count |
|---|---|
| Inner-block collision | 3 |
| Outer-wall collision | 1 |
| Actual sign contact | 5 |

`_WALL_CLEARANCE` was `RobotSpecs.WIDTH / 2 + 0.02` = 0.12 — the chassis
half-*width*. That only bounds a robot travelling parallel to the surface it is
clamped against. A robot still *turning* presents its corner, reaching the
half-*diagonal* 0.180m. Since sign deformations bite hardest right at a corner —
exactly where the robot is mid-turn — the clamp let the corner clip the inner
block while the waypoint itself was still nominally legal.

Resized to `hypot(LENGTH/2, WIDTH/2) + 0.04` = 0.220:

| `_WALL_CLEARANCE` | Collisions |
|---|---|
| 0.12 (half-width, old) | 9/16 |
| 0.16 / 0.20 | 9/16 |
| **0.22 (half-diagonal + 0.04)** | **7/16** |
| 0.24 | 10/16 (over-constrains the deformation) |

Removed one inner-block and one sign collision. Note this makes the clamp bind
at standard grid positions, clipping roughly the last 2cm of offset for
inner/outer-lane signs — `test_sign_router.py` expectations now mirror that.

### Rejected after the retune

- **Path-level deformation, retried with the tighter lookahead.** The original
  rejection could have been a tracking artifact, so it was retested once
  tracking improved. Still worse: 10-16/16 against the 9/16 baseline, with or
  without the runtime router also active.
- **Smaller corner arc radius**, to finish the turn earlier and buy straight
  runway before a corner-adjacent sign. `ARC_RADIUS` 0.45 (current) is already
  best: 0.40 -> 13/16, 0.36 -> 10/16, 0.33 -> 12/16.

**Status: improved 16/16 -> 7/16 on collisions, not solved.** The remaining
failures are still concentrated on corner-boundary signs (depth 1.0/2.0). Tuning
levers are exhausted — lookahead, speed, offset, arc radius and clamp have all
been swept and are at their optima. Note the 7/16 headline counts collisions
only; see below for what the same runs do instead of colliding.

## Router vs. collision-controller conflict (real, but NOT the blocker)

Two subsystems hold incompatible assumptions about how close the robot may
legitimately come to a traffic sign.

* `SignRouter` **deliberately** routes past a sign at ~0.20 m centre-to-centre,
  which is ~0.175 m from the sign's surface. That gap is the entire mechanism —
  `lateral_offset` exists to produce it, and the corridor has no room for more
  (see the offset sweep above).
* `CollisionAvoidanceController.assess_risk` returns `CRITICAL` when the nearest
  range in the forward path drops below `contact_dist` (0.10 m), which triggers
  a reversing escape maneuver. The forward path is a corridor of `path_margin`
  (0.10 m) either side of the heading.

While the robot is **turning** past a corner-adjacent sign, the sign sweeps into
that forward corridor. The collision controller reads a critical threat and
reverses — out of a gap the router aimed for on purpose. The robot re-approaches,
panics again, and oscillates until the run times out.

Observed on `go_obstacles_0000`: the robot wedges at (0.44, 1.10) and again at
(0.84, 0.30), each **0.19 m from a sign** ((0.6, 1.0) and (1.0, 0.4)), thrashing
forward/reverse for 500+ ticks with `laps=0` until the 200 s cap. The repeated
`Robot stuck - triggering escape` log is this loop, **not** a ParkController
problem — parking never engages, because no lap ever completes.

Toggling `lidar_sees_obstacles` trades one failure mode for the other:

| `lidar_sees_obstacles` | Collisions | Completed 3 laps | Timeouts |
|---|---|---|---|
| True (current default) | 7/16 | 0/16 | 9/16 |
| False | 16/16 | 5/16 | 0/16 |

Both fail all 16, differently. Perception is doing its job — seeing signs is what
cuts collisions from 16 to 7. The defect is that the escape logic converts
"legitimately close" into a deadlock instead of a controlled squeeze.

### Resolving the conflict does NOT help — tested

The obvious fixes were prototyped. Both clear the deadlock and both make the
outcome worse, because **the escape maneuver was the only thing preventing those
collisions**, not a spurious panic.

Lowering `contact_dist` (the escape trigger):

| `CONTACT_DIST` | Collisions | Completed 3 laps | Timeouts |
|---|---|---|---|
| 0.10 (current) | 7/16 | 0/16 | 9/16 |
| 0.07 | 15/16 | 5/16 | 0/16 |
| 0.05 | 11/16 | 5/16 | 0/16 |
| 0.03 | 13/16 | 5/16 | 0/16 |

Sign-aware escape suppression — skip the escape maneuver when the near obstacle
is a known sign the router is actively routing past, keeping the 0.10 m guard
for walls. This is the "principled" fix this section previously recommended:

| Suppression radius | Collisions | Completed 3 laps | Timeouts | **Success** |
|---|---|---|---|---|
| 0 (off) | 7/16 | 0/16 | 9/16 | **0/16** |
| 0.30 m | 14/16 | 5/16 | 0/16 | **0/16** |
| 0.40 m | 14/16 | 5/16 | 0/16 | **0/16** |
| 0.50 m | 14/16 | 5/16 | 0/16 | **0/16** |

**Success is 0/16 in every configuration**, including every tuning combination
tried elsewhere in this document. The deadlock is a *symptom*: the commanded
trajectory genuinely aims into the sign, the collision controller catches it at
the last moment, and with nowhere to go it oscillates. Remove the catch and the
robot simply hits the sign instead.

So this conflict is real and worth knowing about, but it is **not** the blocker —
fixing it in isolation buys nothing. Loosening `contact_dist` is separately
unsafe anyway: 0.10 m is the chassis half-width, so lowering it invites real wall
contact. The trajectory has to stop aiming at the sign in the first place; only
then does escape behaviour matter.

## Lane planning — tried, marginal

The trajectory has to be in the correct lane *before* the corner, so avoidance
cannot start inside the corridor. Two versions were prototyped.

**Feasibility first — the maneuver is provably possible.** For a red sign at
(1.0, 0.4) approached from the west corridor, with Ackermann minimum turn radius
0.329 m:

* required south-corridor lane: `y = 0.195` (0.205 m corner-on clearance)
* wall limit: `y >= 0.140` -> feasible, 5.5 cm to spare
* starting the 90-degree arc anywhere in `x = 0.30-0.50` exits at
  `x = 0.63-0.83`, leaving 0.17-0.37 m of straight run before the sign

So the geometry is not the obstacle. The planner simply never produces this path.

**Version 1 (wrong).** Shifted only the straight-segment waypoints onto the
chosen lane and left the corner arcs alone, on the assumption that the arcs
already terminate on the lanes they join. They do not — they terminate on the
corridor *centrelines*, so this injected a lateral step at every arc/straight
junction. Unfollowable: 0/16 completed a lap, 14-16/16 collided.

**Version 2 (correct construction).** The whole path is determined by four
numbers — `north_cy`, `south_cy`, `east_cx`, `west_cx` — and the arc ICRs are
derived from them (`sw_icr = (west_cx + r, south_cy + r)`). Rebuilding the path
with sign-derived lanes therefore moves straights *and* arcs together, with no
discontinuity:

| Config | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| lane 0.20, runtime router on | 12/16 | 0/16 | 0/16 |
| lane 0.20, router off | 14/16 | 2/16 | 1/16 |
| lane 0.28, router off | 14/16 | **3/16** | 0/16 |

Better than nothing (0/16 -> 3/16 completing a lap) but nowhere near enough. Note
the runtime router is actively *harmful* once lanes are planned — the two
deformations compound.

### Why it still fails: reliability has to compound

A lap crosses roughly 4-6 signs. Completing one needs per-sign success chained:
at ~75% per sign, a lap is ~0.75^5 ~ 24%, which matches the observed 3/16. To
finish three laps reliably each sign needs >95%.

With +-6.7 cm of slack and 11.7 cm peak cross-track error, per-sign reliability
cannot get there. **Tracking accuracy is the binding constraint** — the same
conclusion reached from the geometry early on, now confirmed from the other
direction. Lane planning removes the *timing* excuse (the robot is in the right
lane on corridor entry) and the failures persist, which isolates tracking as the
remaining variable.

Closing this needs a genuinely better path tracker. The obvious candidate, true
pure pursuit, is measured above: it regressed the Open Challenge and did not help
here. That makes this a real piece of control work, not a tuning pass.

## Architectural context

`CoreNavigator` is shared by both challenges; they diverge only by which
optional collaborators are injected (`sign_router`, `park_controller`), each
guarded by `is not None`. `ScenarioSimulator` wires these up the same way the
real ROS2 `TrackNavigator` node does, so **findings here reproduce on the
physical robot** — these are not simulation artifacts.

## Next steps

Done: obstacles tuning profile (`NavigationTuning.for_obstacles()`, lookahead
0.12/0.24 + 0.30 m/s) and the half-diagonal clamp.

1. **Corner-adjacent avoidance beginning *during* the preceding arc**, for the
   depth-1.0/2.0 signs that straight-segment tuning cannot reach. This is the
   actual blocker: the commanded trajectory aims into those signs, and every
   downstream symptom (deadlock, collision) follows from that. Note path-level
   deformation has already failed twice — a different approach is needed, not
   another iteration of that one.
2. Do NOT spend further effort on the escape/collision-controller conflict on its
   own; it is measured above and fixing it in isolation yields success 0/16.
   Revisit only once trajectories actually clear the signs.
3. Only then revisit the steering law, with retuned lookahead and a plan for the
   Open Challenge deviation-recovery regressions.

Unrelated but adjacent, found while doing the above: `tuning_profiles/*.json`
cannot be loaded at all (lowercase JSON keys vs uppercase dataclass fields), so
every profile file in that directory is currently dead.

## Measuring changes here

Report **collisions, laps completed and timeouts together**. The
`lidar_sees_obstacles` mistake above came from tracking collisions alone, which
stayed flat while the actual behaviour inverted. A drop in collisions can simply
mean the robot stopped moving.

## Open questions

- Is the outer-lane red-sign squeeze (+-6.7cm) actually achievable on hardware,
  or does the pass-side rule need re-checking for that case?
- ~~Can the C1 see a 0.10m sign from a 0.10m-high mount?~~ **ANSWERED
  (user-confirmed 2026-07-25): yes, the C1 detects the signs.** So
  `lidar_sees_obstacles=True` is the correct model, the sim is representative on
  this point, and the router/collision-controller interaction documented above is
  real on hardware rather than a simulation artifact. This also means tracking
  work here is justified: the numbers it would be tuned against are trustworthy.
- The parking pocket is `ParkingLotSpecs.LENGTH` (0.200 m) deep and the chassis is
  ~0.20 m wide, so full containment has ~zero margin. Pin down the true chassis
  width to the millimetre — at this scale 19.0 mm vs 20.0 mm is the difference
  between a 10 mm margin and none at all.
