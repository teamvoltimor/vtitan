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

### LIDAR-visible signs confusing the collision controller — not the cause

Tested with `lidar_sees_obstacles` on and off: 15/16 either way.

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

**Status: improved 16/16 -> 9/16, not solved.** The remaining 9 are still
corner-boundary signs (depth 1.0/2.0). The 9/16 plateau across both lookahead
and offset suggests a further binding constraint not yet identified — most
likely that corner-adjacent avoidance must begin *during* the preceding arc,
which no amount of straight-segment tuning can supply.

## Architectural context

`CoreNavigator` is shared by both challenges; they diverge only by which
optional collaborators are injected (`sign_router`, `park_controller`), each
guarded by `is not None`. `ScenarioSimulator` wires these up the same way the
real ROS2 `TrackNavigator` node does, so **findings here reproduce on the
physical robot** — these are not simulation artifacts.

## Next steps

1. Lookahead sweep 0.12-0.18, landed as an obstacles tuning profile.
2. Set speed 0.25-0.30 m/s for hardware realism.
3. Only then revisit the steering law, with retuned lookahead and a plan for the
   Open Challenge deviation-recovery regressions.

## Open questions

- Is the outer-lane red-sign squeeze (+-6.7cm) actually achievable on hardware,
  or does the pass-side rule need re-checking for that case?
- Signs and the chassis are both 0.10m tall, so a deck-mounted C1 scans at their
  top edge; real LIDAR detection is marginal and the camera may be the only
  reliable sensor. `lidar_sees_obstacles` exists to model both cases.
