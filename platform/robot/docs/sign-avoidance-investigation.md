# Obstacles Challenge — sign avoidance investigation

Reference log for the traffic-sign collision problem: what was fixed, what was
tried and rejected, and the geometric limits that constrain any future fix.

Written 2026-07-25. Baseline commits: `01ca617` (SignRouter fixes),
`fd33fd5` (obstacle physics).

> **The "Re-measured under 4WS" section's central conclusion is wrong.** It
> reports every tuning knob as flat and concludes the router's commanded offset
> never reaches the chassis. The knobs are flat, but not for that reason, and
> the section it points at as the fix ("What this means for the next attempt")
> is not reachable from the current state. See "The escape layer is the gate"
> immediately below, measured 2026-07-27. Its *measurements* still stand; its
> diagnosis does not.

## The escape layer is the gate (2026-07-27)

Baseline unchanged from the section below: **16/16 collisions (15 sign, 1 inner
wall), 0/16 complete even one lap**, and 16/16 drive three clean laps once the
signs are removed. The tracker is fine.

### `lateral_offset` is not flat, it is inert

Swept 0.20 / 0.24 / 0.28 / 0.32 the results are **byte-identical** — same
collision count, same wall/sign/parking split, same laps, same timeouts. A knob
that genuinely reached the trajectory and merely failed to help would move
*something* across a 60% change in magnitude. Nothing moves. (The value was
asserted to reach the live `SignRouter`, so this is not a broken harness.)

Two candidate explanations were tested and **both ruled out**:

* **The corner-arc guard.** `_is_squarely_in_corridor` rejects the target as a
  corner-arc point, and the tick trace of `go_obstacles_0000` shows `def == raw`
  through the entire fatal pass — so it looked like the guard blanks avoidance
  exactly where it decides the outcome. Forcing the guard permanently open
  changes **one** fixture (an inner-wall contact becomes a sign contact) and
  nothing else. Not the gate.
* **Offset magnitude.** Covered by the sweep above.

### What is actually happening

Take the signs away from the *reactive* layer only — `lidar_blind`, so they stay
physical and collisions stay real, but `CollisionAvoidanceController` can no
longer see them — and the identical offset sweep stops being inert:

| `lateral_offset` | lidar sees signs | lidar blind to signs |
|---|---|---|
| 0.20 | 16/16, 0 laps | 16/16, 0 laps |
| 0.24 | 16/16, 0 laps | **15/16, 1 lap** |
| 0.28 | 16/16, 0 laps | **14/16, 2 laps** |
| 0.32 | 16/16, 0 laps | 14/16, 2 laps |

Reproduce with `scripts/diag_sign_sweep.py masked-offset 0.20 0.24 0.28 0.32`.

So the router works. It is **masked**: while the robot turns past a
corner-adjacent sign, the sign enters the collision controller's forward
corridor, `assess_risk` returns `CRITICAL`, and the escape maneuver fires. The
tick trace shows the consequence directly — the robot pins itself ~0.18 m from
the first sign of the run and oscillates forward/reverse (`v` alternating
-0.200 / +0.150) with its target waypoint frozen on the corner arc, until it
clips the sign. **The run is decided by the reactive layer before the router's
aim can matter**, which is why every planning-side knob measures flat.

This is the conflict the "Router vs. collision-controller conflict" section
below describes and then dismisses as "real, but NOT the blocker". It is the
blocker. That section reached its conclusion by reading **`success`**, which is
0/16 in every configuration ever tried because parking is independently blocked
by chassis-vs-pocket geometry — the exact metric this document's own
"Measuring changes here" section says never to read. Its own numbers show
sign-aware escape suppression taking `laps>=3` from 0/16 to 5/16.

### The second ceiling, behind the first

Masking is necessary but not sufficient: with signs hidden the sweep plateaus at
**14/16 from 0.28 onward**, because `_WALL_CLEARANCE` (0.220, the half-diagonal
clamp) binds and further offset produces no further lateral movement. So there
are two independent ceilings stacked, and only the first is currently visible.
Expect to have to clear both.

### Blind changes nothing, which is itself the useful result

Since 2026-07-27 an obstacles scenario can be run properly blind: `blind=True`
withholds the corridor widths, the travel direction *and* the sign positions,
and the router rebuilds the sign layout from the mocked vision node (see
`navigation/planning/sign_discovery.py` and
[blind-navigation-evaluation.md](blind-navigation-evaluation.md)).

| Configuration | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| sighted (signs from metadata) | 16/16 (15 sign, 1 wall) | 0/16 | 0/16 |
| sighted + mocked vision colours | 16/16 (15 sign, 1 wall) | 0/16 | 0/16 |
| **blind (track, direction, signs)** | **16/16 (16 sign)** | **0/16** | **0/16** |

Discovery is accurate — 1.9 cm median position error, every colour correct, no
spurious tracks — so this is not "blind fails because it cannot find the
signs". Both regimes are stopped at the same place by the same thing, which
means **the escape-layer gate is the blocker for the competition
configuration**, not only for the instrumented one. Nothing above needs
re-measuring against blind before it is fixed.

`lidar_blind` is a *diagnostic*, not a fix — the C1 really does see the signs
(user-confirmed 2026-07-25) and blinding the safety layer to a whole obstacle
class is not shippable. The indicated fix is the standard split: a known,
mapped obstacle the planner is already routing around belongs to the planner,
not to the reactive escape trigger, while walls and genuinely unknown returns
keep the full guard. That is a real change to the safety layer and it has not
been made.

---

> **Everything below the "Re-measured under 4WS" section is the ORIGINAL log,
> taken against a front-only bicycle model with roughly half the real robot's
> yaw authority (min turn radius 0.329 m simulated vs 0.165 m actual).** It was
> invalidated by `8eb3c38` ("model the chassis as counter-phase four-wheel
> steer, not front-only") and again by `668e40a` (drive loop closed, sim speed
> clamped to the measured 0.156 m/s ceiling). Treat its *reasoning* as usable
> and every *number* in it as superseded by the section immediately below.

## Re-measured under 4WS (the current numbers)

All figures here are over the 16 Go-generated obstacles fixtures via
`ScenarioSimulator(...).run()`, reported as collisions / laps>=1 / laps>=3 /
timeouts together. Reproduce with `scripts/diag_sign_sweep.py` (see
"Harnesses" at the end). Runs are deterministic: the same config repeated three
times gives byte-identical counts.

`success` is never the headline — it also requires `parked`, and parking is
independently blocked by chassis-vs-pocket geometry, so it is 0/16 in every
configuration ever tried. `laps>=3` is the driving-success metric.

**Measured against this plant.** Everything in this document has now been
invalidated twice by a change to the vehicle model, so pin it explicitly and
re-check these values before trusting any number below:

| Constant | Value |
|---|---|
| `RobotSpecs.LENGTH` / `WIDTH` | 0.30 / 0.20 m |
| `RobotSpecs.WHEELBASE` | 0.19 m |
| `RobotSpecs.MAX_STEERING_ANGLE` | **1.2253 rad** (~70.2°) |
| `rear_steer_ratio` | 1.0 (counter-phase, `L_eff` = wheelbase/2) |
| minimum turn radius | **0.034 m** |
| `kinematics._DEFAULT_MAX_SPEED_MPS` | 0.156 m/s |

Derived from those: chassis half-diagonal 0.1803 m, so a sign pass needs
**0.205 m** centre-to-centre while turning and **0.125 m** square.

If any of those changed, re-run before citing anything here. The steering angle
in particular went 0.5236 -> 1.2253 on 2026-07-25, which alone moved the
minimum turn radius from 0.165 m to 0.034 m.

### The failure is entirely traffic signs, and the path tracker is fine

Making each obstacle class non-physical in turn isolates the cause completely:

| Physical objects | Collisions | laps>=1 | laps>=3 | Timeouts |
|---|---|---|---|---|
| signs + parking (default) | 16/16 (16 sign, 0 wall, 0 parking) | 0/16 | 0/16 | 0/16 |
| parking only | **0/16** | **16/16** | **16/16** | 0/16 |
| signs only | 16/16 (16 sign) | 0/16 | 0/16 | 0/16 |
| neither | **0/16** | **16/16** | **16/16** | 0/16 |

**Parking blocks are never hit, in any configuration.** Remove the signs and
all 16 fixtures drive three clean laps on the identical corridor layout, start
pose and lap count. 100% of the failure is traffic signs.

### Cross-track error — the old headline conclusion is dead, but not because tracking improved

Measured as point-to-*segment* distance from the chassis to its own planned
path (nearest-*waypoint* distance overstates it by up to half the waypoint
spacing), on the clean runs above. This is dominated by lookahead, so it is
reported per setting rather than as one number:

| lookahead (short/long) | median | p90 | max |
|---|---|---|---|
| 0.12 / 0.24 | **1.7 cm** | **5.1 cm** | 8.6 cm |
| 0.16 / 0.32 | 3.4 cm | 8.6 cm | 9.9 cm |
| **0.20 / 0.40 (shipped default)** | **5.4 cm** | **12.9 cm** | 14.3 cm |
| 0.30 / 0.60 | 12.6 cm | 24.5 cm | 26.7 cm |

At the shipped default, tracking is **worse** under 4WS than the old model's
4.7 / 8.4 / 11.7 cm, not better — doubling the yaw authority without retuning
`steer_kp` costs accuracy. So the old number was not simply pessimistic.

The conclusion still dies, by a stronger route. Tracking accuracy *can* be
bought: lookahead 0.12/0.24 puts p90 at 5.1 cm, inside the ±6.7 cm slack the
outer-lane squeeze allows. **Sign collisions at that setting are still 16/16.**
Buying the required accuracy changes nothing, so **"tracking accuracy is the
binding constraint" is false** — not because the tracker got better, but
because making it good enough does not help. The 16/16 clean three-lap result
above says the same thing from the other direction.

What survives is a narrower claim: *lag against a freshly-deformed target*
still matters (see the mechanism section). That is not the same as raw path
accuracy, and it is not fixed by tightening the lookahead.

### Every tuning knob is flat

Each swept independently, all 16 fixtures, collisions shown:

| Knob | Values swept | Result |
|---|---|---|
| `ARC_RADIUS` | 0.20 0.25 0.30 0.35 0.40 0.45 | 16/16 at every value |
| lookahead (short/long=2x) | 0.10 0.12 0.16 0.20 0.30 0.40 | 16/16 at every value |
| `lateral_offset` | 0.20 0.22 0.24 0.26 0.28 0.30 | 16/16 at every value |
| `_DEFORM_DEPTH_BUFFER` | 0.30 0.45 0.60 0.80 1.00 | 16/16 at every value |
| `lidar_sees_obstacles` | True / False | 16/16 either way |

`laps>=3` is 0/16 throughout. Two notes:

* **The `ARC_RADIUS` re-sweep was the one the old log could not do** — 0.33 m
  was *at* the old model's 0.329 m limit, so it measured a physics wall. The
  real robot runs 0.165 m arcs. Tightening the arc to 0.20 m is now feasible
  and still does nothing, so "clear the corner sooner, gain runway" is
  genuinely ruled out rather than merely untested.
* **Both lookahead extremes convert sign contacts into wall contacts**, while
  the total stays pinned at 16/16. Wall hits by lookahead: 0.10 -> 3,
  0.12 -> 1, 0.16 -> 0, 0.20 -> 0, 0.30 -> 1, 0.40 -> 7. This is exactly the
  trap the `lidar_sees_obstacles` mistake fell into — read the split, not the
  total.

### The sign router is inert

| Configuration | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| physical signs, router on | 16/16 (16 sign) | 0/16 | 0/16 |
| physical signs, router **off** (`lateral_offset=0`) | 16/16 (16 sign) | 0/16 | 0/16 |
| ghost signs (routed around, non-collidable), router on | 0/16 | 16/16 | 16/16 |
| ghost signs, router off | 0/16 | 16/16 | 16/16 |

Turning avoidance off changes nothing. And neither ghost row is a success: with
nothing able to stop it the robot simply *drives through* the signs, which is
what the router exists to prevent — the two ghost rows being identical is the
proof that the router is not bending the trajectory in any useful amount.

### Why: the commanded offset never reaches the chassis

Traced tick-by-tick on `go_obstacles_0004` past its red sign at (2.40, 1.00),
grid depth 1.0 in the east corridor — i.e. right at the corner exit
(`scripts/diag_sign_trace.py 4 --around-sign 2 --radius 0.55`). At the collision
tick (t=105) the chassis is at **x = 2.401** while the sign is at x = 2.40:

| | value |
|---|---|
| lateral separation **needed** (half-diagonal 0.180 + sign half 0.025) | 0.205 m |
| lateral separation **commanded** by the router (target x = 2.566) | 0.166 m |
| lateral separation **achieved** by the chassis | **0.001 m** |

The router is asking for 0.166 m and getting essentially none of it. Three
things go wrong, and no single one explains the gap:

1. **`lateral_offset` is derived from the chassis half-*width*.**
   `_SIGN_LATERAL_OFFSET = WIDTH/2 + sign_half + margin` = 0.10 + 0.025 + 0.075
   = 0.20. Correct for a robot travelling *parallel* to the corridor; a robot
   mid-turn presents its corner at 0.180 m, so 0.20 m is short exactly at
   corner-adjacent signs. **This is the same half-width-vs-half-diagonal bug
   already fixed in `_WALL_CLEARANCE` by `47827ca`, never fixed here.**
2. **The taper removes another ~3.4 cm at the worst moment**, cutting the
   commanded 0.200 m to 0.166 m.
3. **The commanded offset is aimed a full lookahead too far ahead — this is the
   dominant term.** The deformation is applied to the *lookahead target*, which
   at the traced tick sits at y = 1.243 while the chassis is at y = 0.804: the
   target is 0.44 m ahead, and 0.24 m *past* the sign. The robot converges onto
   the offset line at the target's position, not its own, so it reaches x=2.566
   only well after the sign is behind it. The bearing error to that target is a
   few degrees, so the P-on-bearing law commands almost no correction — it is
   already pointed at it.

This is why every knob is flat. Shortening the lookahead attacks (3) and does
reduce cross-track error threefold, but the *deformation* is still applied at
the target, so the lag scales with it and the geometry never improves enough.

The escape maneuver then fires at ~0.20 m and thrashes the chassis
forward/reverse into the sign — a *consequence*, not the cause: with signs
invisible to LIDAR (no escape trigger at all) the result is unchanged at 16/16.

Aggregated over all 84 sign encounters in the 16 fixtures
(`scripts/diag_sign_pass.py`): **26 passes are inside the 0.205 m mid-turn
requirement and 16 are inside even the 0.125 m square-pass requirement.**

### Sign depth

Every collision, attributed to the specific sign hit
(`scripts/diag_sign_hits.py`), by that sign's depth along its own corridor:

```
depth 1.00: 10     depth 1.50: 2      depth 2.00: 4
```

14 of 16 are on a corner boundary (`CORNER_MIN`/`CORNER_MAX`), which is the
original log's central observation — but note **depth 1.5 is no longer immune**.
The old "zero at depth 1.5" figure was measured under the now-reverted
`for_obstacles()` lookahead; at the shipped default two mid-corridor signs are
hit too. Corner-adjacent signs remain much the worse case, and two-thirds of
legal WRO sign positions sit there.

### What this means for the next attempt

The offset has to be a function of where the **robot** is, not where its
lookahead target is, and it has to be sized on the chassis half-diagonal.
Note the ordering: fixing the magnitude alone was swept above as
`lateral_offset` 0.20→0.30 and did nothing, because the lag dominates. Fixing
the lag alone (shorter lookahead) also did nothing, because the deformation
moves with the target. Both have to change together.

---

## Original log (superseded — kept for the reasoning and the rejected list)

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

> **SUPERSEDED.** Under 4WS at the shipped lookahead the same measurement is
> *worse* (median 5.4 / p90 12.9 / max 14.3 cm), yet the robot completes 16/16
> three-lap runs on these layouts once the signs are removed. And buying the
> required accuracy — lookahead 0.12/0.24 puts p90 at 5.1 cm, inside the
> ±6.7 cm budget — still leaves sign collisions at 16/16. Accuracy is
> therefore not the binding constraint. See the top of this document.

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

> **REVERTED.** Re-measured under 4WS:
>
> * **The speed cap cannot do anything at all.** `668e40a` clamps the simulated
>   drivetrain to its measured 0.156 m/s ceiling, so FAST_SPEED 0.30 and 0.50
>   saturate to the same value. The "lower top speed buys steering travel per
>   metre" argument above became unreachable the moment the drive loop closed.
> * **The lookahead change does not help sign avoidance.** 16/16 collisions at
>   every value from 0.10 to 0.40.
>
> The lookahead half is not *inert* — 0.12/0.24 genuinely cuts cross-track error
> from p90 12.9 cm to 5.1 cm. It is simply irrelevant to the problem the profile
> claims to solve, and its stated justification ("tracks the deformation more
> tightly [so] the pass succeeds") is falsified: the passes still fail. If it is
> ever re-added it should be on path-quality grounds, with that as the measured
> claim. The profile and its `ScenarioSimulator` wiring were removed; a comment
> where it used to live records why, so it is not re-added from this section.

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
  best: 0.40 -> 13/16, 0.36 -> 10/16, 0.33 -> 12/16. *(These numbers were void —
  0.33 sat at the old model's 0.329 m minimum radius, so the sweep measured a
  physics wall rather than the idea. Re-swept under 4WS down to 0.20 m, which is
  comfortably feasible: flat 16/16 at every radius. The conclusion survives, for
  a different reason.)*

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

> **The compounding argument still holds; the diagnosis does not.** A *better
> tracker* is not the indicated fix: tightening the lookahead until p90
> cross-track is 5.1 cm — inside the ±6.7 cm budget this argument is built on —
> leaves sign collisions at 16/16. The per-sign budget is blown by where the
> offset is *aimed*, not by how well the robot follows it. See the top.

## Architectural context

`CoreNavigator` is shared by both challenges; they diverge only by which
optional collaborators are injected (`sign_router`, `park_controller`), each
guarded by `is not None`. `ScenarioSimulator` wires these up the same way the
real ROS2 `TrackNavigator` node does, so **findings here reproduce on the
physical robot** — these are not simulation artifacts.

## Next steps

Superseded by "The escape layer is the gate" at the top of this document. The
ordering below was built on the diagnosis that the commanded offset never
reaches the chassis; it does reach it, and the reactive layer overrides it.

1. **Stop the escape maneuver firing at known signs.** Nothing else here is
   measurable until this lands — every planning-side knob reads flat because
   the run is decided before planning matters. The fix is the ordinary split
   between mapped and unmapped obstacles, not a blanket suppression radius and
   emphatically not `lidar_blind`, which is a diagnostic only.
2. **Then** revisit the offset. Behind the escape ceiling there is a second one
   at 14/16, where `_WALL_CLEARANCE` binds. Sizing the offset on the chassis
   half-diagonal (the original item 1 here, still a real defect) becomes
   testable at that point and not before.
3. Do not re-try arc radius, lookahead, speed or the depth buffer *as single
   knobs* — all swept flat, and now known to be flat for a reason that has
   nothing to do with them. They deserve one re-sweep after step 1, since every
   number in this document was taken in the masked regime.
4. The steering law is worth revisiting only after 1 and 2, and only with an
   Open Challenge regression gate — true pure pursuit regressed 3
   deviation-recovery tests when tried.

Also unexplained and worth 20 minutes: collisions measured 10/16 at `47827ca`
and 16/16 at `edace67` ("navigate without being handed the corridor layout"),
a 6-fixture regression in the *sighted* path from a blind-navigation change.
`STEER_KP` 1.5 -> 1.2 was the obvious suspect and is not it — restoring 1.5
leaves 16/16.

Unrelated but adjacent: `tuning_profiles/*.json` is dead config. Confirmed by
attempting every loader on every file — all three raise `TypeError` via both
`load_from_json` and `load_from_yaml`, and the schema mismatch is structural
(groups `heading_error`/`lookahead`/`collision`/`steering`/`stuck`/`vision`
have no dataclass counterpart), not just key casing. Nothing references them;
`node.py`'s `--tuning` defaults to unset.

## Measuring changes here

Report **collisions, laps>=1, laps>=3 and timeouts together**. The
`lidar_sees_obstacles` mistake above came from tracking collisions alone, which
stayed flat while the actual behaviour inverted. A drop in collisions can simply
mean the robot stopped moving. Report the wall/sign/parking split too — the
lookahead sweep looks flat on the total while silently trading sign contacts for
wall contacts.

Never read `success`: it also requires `parked`, which is blocked by chassis
geometry, so it is 0/16 regardless of any driving change.

## Harnesses

Run from `platform/robot` with `PYTHONPATH=.` under `pixi run -e dev`.

| Script | What it answers |
|---|---|
| `scripts/diag_sign_sweep.py` | The four metrics over all 16 fixtures. Swept modes take values as arguments (`lookahead` `arc` `speed` `offset` `masked-offset` `buffer` `crosstrack`); fixed comparison modes do not (`baseline` `profile` `diagnose` `ghost` `lidar`). `--verbose` adds per-scenario rows. |
| `scripts/diag_sign_hits.py` | Attributes every collision to the specific sign hit, with its grid depth. |
| `scripts/diag_sign_pass.py` | Achieved vs commanded lateral clearance, and heading relative to the corridor, at closest approach to each sign. |
| `scripts/diag_sign_trace.py` | Per-tick trace of one scenario: lookahead target, deformed target, steering, pose. The only tool here that shows *mechanism* rather than counts. |

`diag_sign_sweep.py diagnose` is the one to run first on any change — it
separates "the tracker broke" from "sign avoidance failed", which no aggregate
collision count can do.

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
