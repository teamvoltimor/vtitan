# Obstacles Challenge — sign avoidance investigation

Reference log for the traffic-sign collision problem: what was fixed, what was
tried and rejected, and the geometric limits that constrain any future fix.

Written 2026-07-25. Baseline commits: `01ca617` (SignRouter fixes),
`fd33fd5` (obstacle physics).

> **Start here.** The current headline is **137/256 rounds in time (54%),
> 119/256 collisions**, blind, `park=False`, scored under the real push rule.
> It is set by the depth pin and the geometric ceiling behind it — see "The
> depth pin" and "What the remaining 108 sign collisions are" below. Every
> figure in the section immediately following predates the pin and is the
> `pin off` arm (182 collisions, 74 in time).

> **Read "Item 2b re-measured (2026-08-15)" before acting on the Next list.**
> The `A-clamped`/`A-lag` split is judged against the WORST-CASE yaw, which
> over-assigns to `A-clamped`; re-measured against the yaw actually held, only
> about half the Mode A collisions have a commanded line that was genuinely
> short, and the other half were 126 mm off a line that was already adequate.
> "Arrive square" is a real lever for at most half of them, and worth ~19 mm.

## Current state (2026-08-01) — the gate is open, three fixes landed

**Shipped defaults now: 14/16 collisions (14 sign, 0 wall, 0 parking), 2/16
complete all three laps, 0 timeouts — the same numbers SIGHTED and BLIND, on
the same fixtures.** Previously 16/16 and 0/16 in both. This is the first
non-zero `laps>=3` ever recorded here with physical signs and the LIDAR seeing
them. It is an improvement, not a solution.

Fixes 1 and 2 buy the sighted result; fix 3 makes blind match it.

The ORDER of the first two is the point — neither is measurable without the
other, which is why every earlier attempt at the second one read as flat:

1. **The mapped/unmapped split in the reactive layer**
   (`collision_avoidance_controller.mask_mapped_obstacles`, wired in
   `CoreNavigator.step`). A LIDAR return landing within
   `ESCAPE_MASK_RADIUS_M` (0.12) of a sign the `SignRouter` still intends to
   route around is withheld from the CRITICAL escape trigger. Walls, unmapped
   returns, and signs the router has already retired keep the full guard.
   Speed is still governed by the **raw** scan, so the robot goes on slowing
   for a sign — the split removes the escape maneuver, not the caution. This is
   the "ordinary split between mapped and unmapped obstacles" the old next-steps
   list asked for, and emphatically not `lidar_blind`.

2. **`_SIGN_LATERAL_OFFSET` sized on the chassis half-DIAGONAL**, matching what
   `47827ca` already did to `_WALL_CLEARANCE`. Was `WIDTH/2 + sign_half +
   margin` = 0.200; now `hypot(LENGTH/2, WIDTH/2) + sign_half + margin` = 0.280.
   The old value sat *below* the 0.205 m a mid-turn pass needs, at the corner
   positions where two-thirds of legal WRO signs sit.

Measured with the offset knob genuinely connected (see the harness warning
below), the two together:

| `lateral_offset` | split off | split on |
|---|---|---|
| 0.20 | 16/16, 0 laps>=3 | 16/16, 0 laps>=3 |
| 0.24 | 16/16, 0 laps>=3 | **14/16, 2 laps>=3** |
| 0.28 (shipped) | 16/16, 0 laps>=3 | **14/16, 2 laps>=3** |
| 0.32 | 16/16, 0 laps>=3 | **14/16, 2 laps>=3** |

Reproduce with `diag_sign_sweep.py offset ...` and `unsplit-offset ...`. With
the split off the knob is byte-identical across a 60% change in magnitude —
the same inertness the section below diagnosed, now confirmed with a working
knob rather than a disconnected one. **That diagnosis was right.**

3. **The blind layout prior seeded from the challenge's own rules**
   (`CorridorWidthEstimator(assumed_width=...)`). The Obstacles Challenge fixes
   every corridor at 1.0 m; a blind run was assuming 0.6 m. See "Blind now
   matches sighted exactly" below. Regression-gated against the 28 blind Open
   Challenge fixtures: 27/28 pass, 0 collisions, unchanged.

### The corpus, and how to reproduce any of this

```
task gen:corpus CHALLENGE=obstacles              # 256 scenarios, seed 2026
python scripts/sim/diag_sign_sweep.py <mode> --corpus
python scripts/sim/diag_sign_pairs.py --corpus --quiet
```

Lands in `platform/robot/.corpus/` — gitignored, regenerated, byte-identical
from the same generator and seed. `CORPUS_SIZE`/`CORPUS_SEED`/`CORPUS_DIR` are
Taskfile vars.

**Aggregates may be quoted from the committed 16. Attributions may not.** The
16 gave the right overall rate but two wrong diagnoses (below). Anything of the
form "X causes Y" needs the corpus.

**Measurement hazard, hit twice in one session.** Do not edit navigation code
while a sweep is running: `ProcessPoolExecutor` workers are already warm and
you get a mix of old and new code with no error. Put both arms of a comparison
in ONE invocation — that is what `SweepConfig.commit_hysteresis` and the
`hysteresis` mode exist for, rather than editing between two runs.

### Baseline at 256

| Metric | value |
|---|---|
| collisions | 229/256 |
| two signs in play at the fatal approach | 37/229 (16%) |
| ...of which the winner CHANGED mid-approach | **28** |
| collisions on an already-clear line | 175/219 (80%) |

### Re-measured over 200 scenarios (2026-08-01)

Every figure in this document before this section was taken over the SAME 16
committed fixtures. That is one small sample of a large space, so the headline
was re-run against a 200-scenario corpus from the same Go generator:

```
go run ./cmd/simgen generate --challenge obstacles \
    --num-scenarios 200 --seed 2026 --output-dir <dir>
python scripts/sim/diag_sign_sweep.py <mode> --scenarios-dir <dir>
```

Not committed — the 16 are the unit-test battery and have to stay fast; the
corpus is reproducible from the generator and its seed. `scenario_catalog`
takes a `fixtures_dir`, and `diag_sign_sweep.py` / `diag_sign_pairs.py` take
`--scenarios-dir`.

**The aggregate held.** The 16-fixture sample was not misleading about the
overall rate:

| Set | sighted | blind |
|---|---|---|
| 16 fixtures | 14/16 collisions (87.5%), 2/16 laps>=3 | 14/16, 2/16 |
| **200 corpus** | **179/200 (89.5%), 21/200 laps>=3** | **178/200, 22/200** |

Blind/sighted parity survives at scale, so the wide-prior fix is sound.

**The driving is flawless; 100% of the failure is signs.** `diagnose` at 200:

| Physical objects | Collisions | laps>=3 |
|---|---|---|
| signs + parking (default) | 179/200 | 21/200 |
| **no signs, parking physical** | **0/200** | **200/200** |
| nothing physical | 0/200 | 200/200 |

**The router works, and is worth about 10%.** With it off, every single
scenario fails:

| Configuration | Collisions | laps>=3 |
|---|---|---|
| physical signs, router **off** | **200/200** | **0/200** |
| physical signs, router on | 179/200 | 21/200 |

#### Where the corpus changed the ANSWER, not just the denominator

Two attributions taken from the 16 were wrong. The aggregate generalised; the
diagnoses did not.

* **Two signs bracketing a gap is NOT rare.** Reported from the mat, and
  dismissed here off the 16 as "13 of 14 collisions have one sign in play".
  Over 200 (`diag_sign_pairs.py`): **31 of 179 collisions (17%) involve two
  signs**, 24 of them with the nearest-wins winner CHANGING mid-approach —
  roughly triple the rate the small sample implied. Statically the
  configuration is everywhere: 147/200 scenarios (74%) have two signs within
  `activation_dist`. The small-sample undercount was an artefact of runs dying
  early (typically step 100-600 of 3000), so the robot rarely reaches a point
  where two are simultaneously in play — not evidence the layouts lack it.
* **82% of collisions happen on an already-clear line.** 141 of 173, against
  86% on the 16. The router is putting an already-safe robot back into
  contention far more often than it is rescuing an unsafe one.

#### Tried and reverted (again): clearance as a bound

The 82% figure suggests treating `lateral_offset` as a MINIMUM rather than a
target line — `min`/`max` against the required value instead of assignment, so
a target already clear is left alone while a wrong-side one is still moved all
the way across. Two anchorings, both dead:

* **Anchored on the waypoint.** Byte-identical at 200 (179/200, 21 laps), and
  provably so: the planned path runs down the corridor centre, and across all
  **1009 sign passes in the corpus that centreline sits at exactly 0.100 m from
  every sign** against a 0.280 m requirement — `already clear on path = 0/1009`.
  The bound can never bind. It is dead code, not a weak effect.
* **Anchored on the robot.** This one *would* bind — the robot is usually off
  the centreline, because a previous sign's deformation put it there — but it
  broke 36 tests, the whole `TestPassSideRule` matrix, since a robot
  approaching from the wrong side stops being moved across.

That 0.100 m figure is the useful residue: **on the planned path there is never
a straight line through**. The "just go straight" option only exists because
the *previous* deformation moved the robot off-centre. Any fix here has to
reason about the robot's actual line without losing the wrong-side guarantee,
which is what the robot-anchored attempt failed to do.

### The 14/16 plateau is NOT the wall clamp

The section below predicted a second ceiling at `_WALL_CLEARANCE` (0.220).
Swept (`diag_sign_sweep.py wall ...`), it is already at its optimum and the
plateau is something else:

| `_WALL_CLEARANCE` | Collisions | laps>=3 |
|---|---|---|
| 0.16 | 16/16 (wall 4, sign 12) | 0/16 |
| 0.18 | 15/16 (wall 3, sign 12) | 1/16 |
| 0.20 | 14/16 (wall 0, sign 14) | 2/16 |
| **0.22 (shipped)** | **14/16 (wall 0, sign 14)** | **2/16** |
| 0.24 | 16/16 (wall 0, sign 16) | 0/16 |

Loosening it buys wall contacts one-for-one; tightening it over-constrains the
deformation. So the prediction was wrong and **what holds the remaining 14 is
still unidentified.** Do not spend the next session on the clamp.

### Blind now matches sighted exactly (the wide prior)

**Fixed.** `blind` is now 14/16 with 2/16 three-lap finishes — identical to
sighted, and identical fixture-for-fixture (the same two, `0001` and `0003`,
are the ones that finish).

The cause was a prior that is simply wrong for this round. A blind run assumed
every corridor was narrow (0.6 m) and corrected it from LIDAR as it drove. That
is the right default for the Open Challenge, whose corridors are independently
60 or 100 cm — neither value is a better guess and the tighter one fails safe.
**The Obstacles Challenge fixes every corridor at 1.0 m**, which is a rule of
the event and therefore knowable before the robot is placed, exactly like "the
track is 3x3 m and the loop is rectangular". Verified across the fixture sets:

| Fixture set | Corridor widths |
|---|---|
| obstacles (16) | **64/64 at 1.0 m** |
| open/test (28) | 55 at 0.6 m, 57 at 1.0 m |

So the robot was starting every blind obstacles round holding a belief that was
wrong for all four corridors, and paying the ``_MIN_SAMPLES`` (12 valid
readings) latency to correct each one — while driving past traffic signs on a
path built for the wrong geometry. `CorridorWidthEstimator` now takes an
``assumed_width``; `ScenarioSimulator` and `TrackNavigator` seed it from the
challenge type. The estimator still measures and can still override the prior;
this changes only where it starts. `CorridorDimensions.OBSTACLES_WIDTH` already
existed and documented the rule — it just was not wired to the estimator.

> A note on reading vote data. Before this was understood, the obstacles
> fixtures' all-wide voting (`east: [0, 152]`) looked like a measurement bug.
> It was not: every corridor genuinely *is* wide, so the occasional narrow vote
> was the error and the all-wide result was correct. Any future change here
> should be judged against the true widths, not against vote diversity.

### Superseded: the gains did not used to carry to blind

> **Resolved by the wide prior above.** Kept because the attribution below is
> what located the cause, and because the method — split what `blind` withholds
> rather than treating it as one switch — is the reusable part.

As first measured (`diag_sign_sweep.py blind`, ideal sensors, so this is the
equivalent of the open-challenge evaluation's `ideal` row and not its
`bno085 at spec` one):

| Configuration | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| sighted (signs from metadata) | 14/16 | 2/16 | **2/16** |
| blind (track, direction, signs) | 16/16 | 1/16 | **0/16** |

Before these fixes the two regimes were identical (16/16, 0 laps either way),
which is what made "blind costs nothing" true. **It is no longer true**: the
sighted improvement largely does not survive discovery.

What the fixes are worth in blind, isolated within a single tree
(`diag_sign_sweep.py blind-split`). Measure it this way rather than against a
blind figure from an earlier session — unrelated changes move blind lap counts
on their own, `replace_path`'s lap-seam fix among them:

| Blind configuration | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| pre-fix (offset 0.20, split off) | 16/16 | 0/16 | 0/16 |
| split only (offset 0.20) | 16/16 | 0/16 | 0/16 |
| offset only (0.28, split off) | 16/16 | 0/16 | 0/16 |
| **both (shipped defaults)** | 16/16 | **1/16** | 0/16 |

So the gain is real and it is attributable — and it is **one fixture reaching
one lap**. Collisions do not move at all and no fixture finishes three. Note
the combinatorial signature is the same as sighted: neither fix does anything
alone, which is consistent with the split being the gate and the offset being
what the open gate lets through.

#### The shortfall is the corridor layout, not the signs — attributed

`diag_sign_sweep.py blind-source` holds the track and direction back but hands
the sign layout over, which separates the two things `blind` withholds at once:

| Configuration | Collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| sighted (everything known) | 14/16 | 2/16 | 2/16 |
| blind track+direction, **signs known** | 16/16 | 1/16 | 0/16 |
| fully blind (signs discovered) | 16/16 | 1/16 | 0/16 |

The bottom two rows are identical, so **sign discovery costs nothing** and the
entire blind loss is the estimated corridor layout. The discovery-timing
hypothesis (`MIN_HITS` vs `activation_dist`) is dead; do not spend time on it.

`diag_blind_layout.py` then attributes it further, scoring only the corridor the
robot is *in* when it dies (scoring the whole belief would blame the estimator
for corridors the collision itself prevented it from ever visiting):

```
WRONG+UNMEASURED (still on the narrow default) = 9
belief here correct, still collided            = 7
```

**9 of 16 blind collisions happen in a corridor the robot has never measured.**
A blind run assumes every corridor is narrow and corrects from LIDAR; the robot
banks its starting corridor, turns into the next one still holding the narrow
default, and meets a sign there before `_MIN_SAMPLES` (12) valid readings have
accumulated. In the Open Challenge that same latency is harmless — there is
nothing in the corridor to hit — which is why blind open reaches 28/28 while
blind obstacles is stuck at 16/16. **The lever is how fast a newly-entered
corridor is committed, or what the robot does before it is.**

The remaining 7 are collisions with a correct, settled belief for that
corridor — the same unexplained residue as the sighted 14.

##### Tried and reverted: median-window wall distance

`measure_corridor_width` takes the single ray at +-90 deg (`_nearest_ray`),
which during a sign pass measures the sign rather than the wall, biasing the
width *short* — i.e. toward "narrow", which is what a blind run already assumes,
so the error silently confirms the wrong default. Replacing it with a median of
`r*cos(offset)` over a +-25 deg window (robust to a sign occupying a minority of
the window, and unbiased for a flat wall) **changed nothing**: still 16/16 and
9 WRONG+UNMEASURED. It also moved the vote distribution the wrong way — narrow
votes across the fixtures went from a real minority (e.g. `east: [24, 152]`) to
exactly zero — so it was reverted rather than kept as a neutral cleanup. The
window evidently picks up returns past the inner block's end, which is only
1 m long against a 3 m corridor. Note the bias is real; it is just not what is
costing the runs, because the corridors that lose them are never measured at
all.

Still unvalidated on the perception side, independent of the above: the emulator
emits detections every control tick at 20 Hz, which the real Hailo pipeline will
not match, so `MIN_HITS` confirmation costs more distance on the mat than in
sim. That does not affect these numbers (discovery is free here) but it could on
hardware.

Blind is the configuration that matters on the mat, so **this gap is the result
to close, and no sighted-only number here should be claimed for race day.**

### The tracker is still fine

`diagnose` at the new defaults: strip the signs and all 16 fixtures drive three
clean laps, exactly as before. 100% of the remaining failure is traffic signs.

### WARNING: this harness was silently broken, and it voids numbers below

`diag_sign_sweep.py` had **two independent breakages**, both of which failed
silently as "the knob does nothing" rather than as an error. Both are fixed;
both invalidate figures in the sections below.

* **The `lateral_offset` override never reached the router.** It patched
  `sign_router.SignRouterConfig`, but `scenario_simulator` binds that name at
  import time and builds the config via `SignRouterConfig.from_tuning(...)`,
  so neither half of the patch applied. Every `offset` and `masked-offset`
  figure taken after the config moved to `from_tuning` measured the default
  0.20 repeatedly. **This is why the `masked-offset` table below no longer
  reproduces** — that table is from before the breakage and cannot currently be
  confirmed or refuted.
* **Every tuning override raised `TypeError`.** `NavigationTuning` is a
  dataclass but its groups are frozen *pydantic* models, and `tuning()` used
  `dataclasses.replace` on the groups. So `lookahead`, `arc` and `speed` could
  not run at all after that migration.

Practical rule for the next session: **before citing any number in this
document, check it was taken with the knob it claims to sweep actually
connected.** A flat sweep is now the first thing to distrust, not the last —
two of the three "every tuning knob is flat" style conclusions here turned out
to have a disconnected knob behind them.

### Scoring: the real push rule, and the 10-round modelling gap

Until now every number in this document scored **any** chassis-sign contact as a
failure. That is stricter than the event. The real rule: each pillar is placed
inside an **85 mm circle**, and the round stands as long as **any corner of the
pillar's square is still inside that circle** — brushing it, or nudging it a few
millimetres, costs nothing. `TrafficSignSpecs.MAX_LEGAL_DISPLACEMENT_M` turns
that into a displacement bound: **59.4 mm** for an axis-aligned push, which is
the worst case (a diagonal push tolerates 77.9 mm), so the axis figure is used
everywhere as the conservative choice. Scored against the 256-corpus:

| Scoring criterion | Collisions | In time |
|---|---|---|
| strict (any contact = failure) | 194 | 62 |
| real rule, push = magnitude of the advance | 192 | 64 |
| **real rule, push = advance projected onto the sign** | **182** | **74** |

So the correct rule is worth **+12 rounds** over the criterion used above, 24% →
29%.

**Read the spread before quoting the headline.** The two rows differ only in how
the *simulator* converts a contact into a displacement, and they differ by 10
rounds — more than almost any tuning change measured in this document. The rule
is unambiguous; **how far a sign travels when the chassis touches it is not**,
and neither row is a measurement. Both are assumptions about friction and
contact that this simulator does not model:

* *magnitude of the advance* — the sign absorbs the robot's whole forward
  motion. Conservative; over-counts glancing contacts.
* *projected onto the sign* — only the component of the advance directed at the
  sign moves it. Optimistic; this is the row that produces 74.

The 74 is therefore a **projection, not a result**, and 64 is the number to
quote if only one is quoted. Resolving the gap is empirical, not a choice
between models: push a real sign with the real chassis and measure the
displacement per centimetre of advance, then pin whichever model matches. Until
that measurement exists, do not tune against the difference — a change worth
fewer than 10 rounds cannot be distinguished from the modelling choice.

**What none of this changes:** the 182 remaining failures are not borderline.
They are frontal pushes of 60+ mm, past the 59.4 mm bound under either model,
so no push model rescues them and the diagnosis in the sections above is
untouched. Avoidance *does* execute in all 182; it does not arrive in time. That
remains the open question.

### The depth pin — the largest single effect measured here (2026-08-01)

**74 -> 137 rounds in time, +63, 29% -> 54%.** Nothing else in this document
comes close. Attributed with both arms in ONE invocation
(`diag_sign_sweep.py pin --corpus`), blind, `park=False`:

| Arm | Collisions | wall / sign | in time |
|---|---|---|---|
| pin off (the whole document above) | 182/256 | 0 / 182 | 74/256 |
| **pin on (shipped)** | **119/256** | 11 / 108 | **137/256** |

`_apply_deformation` used to pass the lookahead point's depth coordinate
straight through, deforming only the lateral one. So the commanded point held a
fixed lateral value but kept receding 0.2-0.4 m per tick: the slope the chassis
had to follow to reach it flattened every tick and the lateral error closed only
asymptotically. Traced on `go_obstacles_0000`, the chassis needed 0.324 m of
lateral travel over the 0.42 m of runway left and achieved 0.163 m of it,
arriving level with the pillar half a chassis width inside its own commanded
line.

`_pin_depth` holds the commanded point at the SIGN's own depth while the sign
lies between the chassis and the lookahead point. The target stops receding and
becomes a fixed gate abeam the pillar, which the chassis has to be on by the
time it arrives. The condition lapses on its own once the robot is level, so
there is no release to get wrong. Toggle: `SignRouterParams.DEPTH_PIN`.

**It costs 11 wall collisions**, up from 0 — pulling the target back to the
sign's depth near a corner evidently puts the line into a wall. That is a real
regression riding along with a large net win, and it is the cheapest thing left
to fix.

### What the remaining 108 sign collisions are: a geometric ceiling

With the pin on, `diag_failure_split.py --corpus` classifies **108 of 108
remaining sign collisions as `A-clamped`** — the commanded lateral line is
itself closer to the sign than a yawed chassis needs. Zero `A-lag`. **The pin
eliminated lag as a failure mode entirely.** (The split's `A-lag 7` +
`B-no-deform 4` = 11 are the wall collisions, where those labels mean nothing.)

The cause is not tuning. At real WRO grid positions, with `_WALL_CLEARANCE`
0.219 reserving room for the chassis half-diagonal:

| Sign line | Colour | Commanded | After clamp | Achievable | vs 0.204 needed mid-turn |
|---|---|---|---|---|---|
| outer (0.4) | red -> outward | 0.279 | 0.219 | **0.181** | **short** |
| outer (0.4) | green -> inward | 0.279 | 0.679 | 0.279 | ok |
| inner (0.6) | red -> outward | 0.279 | 0.321 | 0.279 | ok |
| inner (0.6) | green -> inward | 0.279 | 0.781 | **0.181** | **short** |

**Half of all legal sign/colour combinations cannot be passed mid-turn at all.**
Whenever the pass-side rule pushes the robot toward the nearer boundary, a 1.0 m
corridor simply does not contain 0.204 m of clearance plus the clamp. No offset,
taper or steering gain reaches this; the clamp is already saturated.

But 0.181 m is not a dead end — **it is a yaw budget**. A square pass needs only
0.122 m, so there is 6 cm of surplus, and the requirement scales with heading as
`(L/2)|sin th| + (W/2)|cos th| + sign_half`:

```
clearance available to the chassis   0.156 m   (0.181 - sign half-width)
MAX YAW that still clears            28.0 deg off the corridor axis
  yaw 20 deg -> needs 0.168 m   clears
  yaw 28 deg -> needs 0.181 m   COLLIDES
```

So these signs are passable **iff the chassis is within ~28 deg of the corridor
axis when it draws level**. That is consistent with the oldest observation in
this document — 14 of 16 collisions on a corner boundary — those being exactly
the signs met while still rotating. **The lever is arriving square, not aiming
wider.**

#### Tried and rejected: ramping the offset in

The taper fades the offset in as well as out, which peaks it *at* the sign: at
`activation_dist` 1.40 against `passed_dist` 1.60 it opens at 0.125, asking for
3.5 cm where a mid-turn pass needs 20.4 cm. Holding full offset from activation
and fading only on the way out, swept at 0.20/0.40/0.70 m:

* **pin off: byte-identical**, 182 collisions at every value.
* **pin on: slightly worse** — 119 -> 117 collisions but 137 -> 135 in time.

The lateral clamp saturates before the taper ever binds, so the ramp has nothing
to give. Removed rather than shipped as an inert tunable. Do not re-try without
new information.

#### Methodological warning: two harness failures in one session

Both produced confident, wrong answers that looked reasonable.

* **The failure split compared Euclidean distances.** `target_gap` was
  `dist(deformed_waypoint, sign)`, which folds in the lookahead's along-track
  lead — so a line clamped to 0.181 m of real clearance still measured >0.205 m
  and `A-clamped` could essentially never fire. It reported **182/182 `A-lag`**
  and "the wall clamp is exonerated". The exact opposite is true. Fixed to
  compare lateral separation on the sign's own corridor axis.
* **The depth pin landed mid-measurement.** Its 182 -> 119 was briefly
  attributed to the classifier fix, because the two numbers came from two
  different invocations. This is the third time this document records that
  mistake. **A number compared across invocations is not a measurement**; that
  is what the `pin`/`hysteresis`/`ramp` fixed-arm modes exist for.

### Next

0. **Run everything against the 200-scenario corpus, not the 16.** Two
   attributions taken off the 16 were wrong (see above) while the aggregate was
   fine, so the rule is: aggregates may be quoted from the 16, *attributions*
   may not.

1. **The two-sign winner switch — IN PROGRESS, UNMEASURED.** 28 of 229
   collisions at 256. `SignRouter` re-ran a pure nearest-wins race every tick
   with no memory, so with two signs in play the commanded lateral line could
   jump from one sign's required value to the other's while the chassis was
   already committed. Both lines are legal; the damage is switching between
   them with no runway left to track the new one.

   `SignRouter._prefer_committed` (behind `COMMIT_HYSTERESIS`, default on) now
   holds the engaged sign until it is genuinely cleared — retired, behind the
   chassis, beyond `activation_dist`, or yielding no applicable deformation.
   That last guard matters: without it a receding sign holds its claim from out
   of range and masks the one coming up, which is the same masking bug the
   nearest-wins ordering already had to fix once.

   Selection-only hysteresis, deliberately: the deformation math and the
   pass-side rule are untouched, which is what sank both clearance-bound
   attempts. All 105 `test_sign_router.py` tests pass.

   **Not yet measured.** Run `diag_sign_sweep.py hysteresis --corpus` (all four
   arms — off/on x sighted/blind — in one invocation) and
   `diag_sign_pairs.py --corpus --quiet` to confirm the switch count actually
   drops rather than reading the aggregate alone. If the aggregate is flat but
   switches fall to ~0, the mechanism is fixed and something else dominates
   those 28; if switches do not fall, the hysteresis is not binding and the
   claim-drop conditions are the place to look.

2. **ANSWERED — it was the receding target, and the residue is geometric.**
   The depth pin took 182 collisions to 119 and 74 in-time rounds to 137, and
   the corpus split now puts 108 of 108 remaining sign collisions at
   `A-clamped` with zero `A-lag`. See the two sections above. What replaces
   this item:

   * **2a. The 11 new wall collisions the pin introduced** (0 -> 11). Cheapest
     open item, and a pure regression: `_pin_depth` pulls the commanded point
     back to the sign's depth, which near a corner can put the line into a
     wall. Likely wants the same corner guard `_is_squarely_in_corridor`
     already applies to the lateral deformation.
   * **2b. Arrive square, do not aim wider.** The 108 are capped at 0.181 m of
     achievable clearance against 0.204 m needed while yawed, but a square pass
     needs only 0.122 m — a **28 deg yaw budget**. Measure the actual yaw at
     the fatal tick first (`diag_sign_trace.py`): if it clusters past 28 deg,
     the fix is finishing the corner arc before the sign or holding heading
     through the pass. Do NOT spend time on offset magnitude, the taper or the
     clamp value — all three are saturated or measured flat.

3. **Re-sweep lookahead, arc radius and speed — now genuinely worth it.**
   Every one of those numbers was taken either in the masked regime or through
   a harness that raised before running, so they are unmeasured rather than
   flat. They are also exactly the knobs that govern *how square the chassis
   is at a corner exit*, which item 2b identifies as the binding constraint —
   arc radius most of all.
4. `ESCAPE_MASK_RADIUS_M` has not been swept — 0.12 is derived (sign
   half-diagonal 0.035 + ~0.085 pose/mapping error), not tuned.
   `diag_sign_sweep.py mask-radius ...` exists for it.

### Item 2b re-measured (2026-08-15) — half the premise does not hold

Item 2b said to measure the yaw at the fatal tick before spending anything on
"arrive square". Done, over the 256 corpus, blind, `park=False`
(`diag_failure_split.py --corpus --yaw`, which now carries the geometry):

```
Mode A collisions                        218 / 248
  yaw off corridor axis   median 23.7 deg, max 83.8
    0-10 deg  44 (20%)   20-28 deg  77 (35%)   40-90 deg  50 (23%)
   10-20 deg  14  (6%)   28-40 deg  33 (15%)

  line ADEQUATE at the held yaw   100 (46%)  <- median 126 mm OFF that line
                                                 20 of them on the wrong side
  line SHORT at the held yaw      118 (54%)  <- median shortfall 19 mm
    of which squaring would fix   118/118
```

Two corrections to the Next list fall out:

* **`A-clamped` is not what it reads as.** It compares the commanded line
  against the chassis half-DIAGONAL — the worst yaw the chassis could present,
  not the one it held. Judged against the actual heading, **46% of Mode A had a
  line with room to spare and simply was not on it**, by a median 126 mm in a
  1.0 m corridor. The doc's "the pin eliminated lag as a failure mode entirely,
  108 of 108 `A-clamped`, zero `A-lag`" is an artefact of that threshold. Lag
  never went away; it stopped being *labelled*.
* **"Arrive square" is worth having but is not the dominant lever.** It applies
  to the 54% whose line was genuinely short, and every one of those would clear
  a square chassis — but the median shortfall is **19 mm**. Sizeable next to the
  clamp, small next to the 126 mm the other half is off by. And 20% of fatal
  ticks are already inside 10 deg of the corridor axis, so a fifth of them have
  no yaw left to recover.

The yaw histogram is bimodal, which is the part worth carrying forward: a
cluster at 20-28 deg (the corner-boundary passes item 2b predicted) and a
second at 40-90 deg that is not a sign-clearance problem at all — a chassis
more than 40 deg off the corridor axis while abreast of a sign is not passing
it, it is still cornering, or lost.

### The 20 Hz axis flip — found, fixed, measured flat, left OFF

Tracing `go_obstacles_0000` blind showed the commanded waypoint alternating
between **two orthogonal targets on every single tick** for the whole approach:

```
t=326 def=(2.645,1.999) sign=0@(2.393,1.999)/EAST   steer=-0.088
t=327 def=(2.370,2.256) sign=0@(2.395,2.003)/NORTH  steer=-0.068
t=328 def=(2.649,1.998) sign=0@(2.398,1.998)/EAST   steer=-0.172
t=329 def=(2.370,2.255) sign=0@(2.400,2.003)/NORTH  steer=-0.068
```

Same committed sign throughout. A sign's corridor selects which world axis its
deformation treats as lateral, `_sign_corridors` is re-derived every tick from
a discovery estimate that keeps moving, and this estimate was wobbling ±5 mm
across **y = 2.00 at x = 2.40** — a corridor boundary. `corridor_for_position`
is a hard partition, so the label flipped EAST/NORTH every tick and the
deformation swapped axes with it. Two-thirds of legal WRO grid positions sit on
a corner boundary, so this is not an exotic case.

Worth noting how the corner tie-break behaves here: it picks the NEAREST inner
face, so (2.40, 2.003) — 400 mm deep in the east band, 3 mm past the north one
— classifies **NORTH**. Moving a point further into the corridor you want can
make that corridor *less* likely, which is why the fix is temporal
(`CORRIDOR_FLIP_TICKS`, N consecutive agreeing ticks) rather than a geometric
dead-band. There is no usable distance-to-decision-surface to threshold on.

The oscillation is real, the fix removes it (traced, and five unit tests in
`TestSignCorridorHysteresis`), and **it does not move the corpus**:

| `corridor_flip_ticks` | collisions | laps>=1 | laps>=3 |
|---|---|---|---|
| 1 (inert, shipped) | 252/256 (0 wall) | 11 | 4 |
| 5 | 254/256 (**5 wall**) | 17 | 2 |
| 10 | 252/256 (3 wall) | 15 | 4 |

Flat on collisions and `laps>=3`, and both damped arms trade sign strikes for
new *wall* strikes — holding a stale corridor keeps deforming on the wrong axis
for longer. **Defaulted to 1 (inert)**, same verdict and same reasoning as
`commit_hysteresis`: a real mechanism that does not earn its place in a safety
path on the evidence. The code and the `corridor-flip` sweep arm stay.

One caveat before writing it off: the sim commands steering with infinite
bandwidth, so a 20 Hz axis flip costs it almost nothing, where a real servo has
to physically slew between the two commands every tick. This is a candidate for
re-testing on hardware rather than in another sweep.

#### Harness bug: the sweep reported every sign strike as `park`

`_classify_collision` paired its probes with the wrong labels. Both are built
by REMOVAL — `_without_signs` leaves the *parking* blocks standing — so testing
the sign-strike case against `_without_signs` and the parking case against
`_without_parking` **inverted the split**. Every `sign N / park M` this harness
has printed is the two numbers swapped. It is why a first pass at the arm above
read `park 16/16` on a pure sign-collision corpus, and it disagreed with
`diag_failure_split.py`, which was right. Fixed. This is the fourth harness
error this document records; the split now agrees across both harnesses.

`diag_sign_trace.py` had also gone stale in a quieter way: it patched
`sign_router._DEFORM_DEPTH_BUFFER`, a module global that moved into
`SignRouterParams` during the constants centralisation. It raised
`AttributeError` when finally used — but before the name disappeared entirely
it would have silently patched nothing, and `--buffer` would have read flat.
Both overrides now go through `NavigationTuning`.

---

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

Reproduce with `scripts/sim/diag_sign_sweep.py masked-offset 0.20 0.24 0.28 0.32`.

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

> **SUPERSEDED (2026-08-01).** True only while both regimes were stuck at the
> same 16/16. Once the escape split and the half-diagonal offset landed, sighted
> moved to 14/16 with 2/16 three-lap finishes and blind did not follow — see
> "The gains do NOT carry to blind" at the top. The closing claim here, that
> "nothing above needs re-measuring against blind before it is fixed", has now
> expired: it is fixed, and blind does need re-measuring.

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
timeouts together. Reproduce with `scripts/sim/diag_sign_sweep.py` (see
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
(`scripts/sim/diag_sign_trace.py 4 --around-sign 2 --radius 0.55`). At the collision
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
(`scripts/sim/diag_sign_pass.py`): **26 passes are inside the 0.205 m mid-turn
requirement and 16 are inside even the 0.125 m square-pass requirement.**

### Sign depth

Every collision, attributed to the specific sign hit
(`scripts/sim/diag_sign_hits.py`), by that sign's depth along its own corridor:

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

## Tuning scope

Any future arc_radius/lookahead retune from this document routes through
`NavigationTuning.load_default(challenge=ScenarioType.OBSTACLES)` and the
`platform/shared/config/navigation-challenges/obstacles/` overlay, never
through `platform/shared/config/navigation/` (the base config Open Challenge
also reads). The old `for_obstacles()` classmethod this document refers to no
longer exists as such — the mechanism it would have used is the challenge
overlay, and `navigation-challenges/open/` stays empty by construction (see
`test_load_default_open_challenge_is_byte_identical_to_no_challenge` in
`test_navigation_tuning.py`). Do not add a value to either overlay without a
measurement backing it — see "Next steps" below for what that measurement is.

On real hardware the active challenge is resolved at runtime from the
GPIO23 jumper (`state_machine_node`, `/challenge_mode/active`), not fixed at
process start — `track_navigator_node` loads both challenge profiles eagerly
and picks the active one, along with a fresh `SignRouter`/`ParkController`,
every time `RACING` is entered (including after a long-press `SYSTEM_RESET`).
See `TrackNavigator.reset()` and `CoreNavigator.replace_sign_router`.

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
| `scripts/sim/diag_sign_sweep.py` | The four metrics over all 16 fixtures. Swept modes take values as arguments (`lookahead` `arc` `speed` `offset` `unsplit-offset` `masked-offset` `buffer` `wall` `mask-radius` `crosstrack`); fixed comparison modes do not (`baseline` `profile` `diagnose` `ghost` `lidar`). `--verbose` adds per-scenario rows. |
| `scripts/sim/diag_escape_mask.py` | Whether the mapped/unmapped split is actually firing, per tick: raw vs masked risk, and which escapes began. Answers "is this knob connected?" — the question two silent harness breakages here turned on. |

Every mode except `blind` runs **sighted**, which is not the competition
configuration. Re-read any result through `diag_sign_sweep.py blind` before
claiming it for the mat; since 2026-08-01 the two no longer agree.
| `scripts/sim/diag_sign_hits.py` | Attributes every collision to the specific sign hit, with its grid depth. |
| `scripts/sim/diag_sign_pass.py` | Achieved vs commanded lateral clearance, and heading relative to the corridor, at closest approach to each sign. |
| `scripts/sim/diag_sign_trace.py` | Per-tick trace of one scenario: lookahead target, deformed target, steering, pose. The only tool here that shows *mechanism* rather than counts. |

`diag_sign_sweep.py diagnose` is the one to run first on any change — it
separates "the tracker broke" from "sign avoidance failed", which no aggregate
collision count can do.

## Commit hysteresis — measured, DEFAULTED OFF (2026-08-01)

Four arms over the 256-scenario corpus, one invocation
(`diag_sign_sweep.py hysteresis --corpus`):

| Configuration | Collisions | laps>=3 |
|---|---|---|
| sighted, hysteresis off | 229/256 | 27/256 |
| sighted, hysteresis on | 229/256 | 27/256 |
| blind, hysteresis off | 227/256 | **29/256** |
| blind, hysteresis on | 228/256 | 28/256 |

Flat sighted, marginally worse blind. `COMMIT_HYSTERESIS` now defaults to
`false` in `sign_router.toml` and `SignRouterParams`; `_prefer_committed` and
the `hysteresis` sweep arm stay so it can be re-tested once the dominant
failure is understood. The winner-switch case it targets is real — 28 corpus
approaches switch mid-approach — it is simply not what decides those runs.

The probe that was supposed to confirm the mechanism fired was itself wrong,
for the third time in this investigation. `diag_sign_pairs.py` wrapped
`_active_sign_candidates`, which runs *before* `_prefer_committed` reorders,
so it measured the raw nearest-wins race and was blind to hysteresis by
construction — it reported an unchanged switch count for a change that was
working exactly as written. It now reads `router._committed`, the sign
`deform_waypoint` actually selected, and takes `--hysteresis on|off` so both
arms come from one harness.

## Parking is not in the way (2026-08-01)

`ScenarioSimulator(..., park=False)` skips the parking maneuver while leaving
the blocks on the mat and collidable; `diag_sign_sweep.py no-park --corpus`
runs it against the parking arm. Parking changes **nothing**:

| Arm | Collisions | laps>=3 |
|---|---|---|
| laps only, sighted | 229/256 | 27/256 |
| with parking, sighted | 229/256 | 27/256 |
| laps only, blind | 228/256 | 28/256 |
| with parking, blind | 228/256 | 28/256 |

Every collision happens during the laps, before parking would engage. So
deferring parking until sign avoidance works costs no information, and a clean
three-lap run now reads as one instead of ending in a `ParkController` give-up.
The switch was verified live (controller present vs `None`) rather than
inferred from the identical aggregates — identical numbers are the signature of
an inert knob here, and twice they have been exactly that.

**Baseline to beat: 27/256 sighted, 28/256 blind (~10.6%) complete three laps.**

## The corner dead zone — root cause found (2026-08-01)

`diag_sign_trace.py` on two failing scenarios shows **two distinct** failure
modes, not one.

**Mode A — deformation active but geometrically insufficient** (fixture 5, red
sign at (1.00, 2.60) on the north outer division line):

```
t=115 pos=(0.832,2.528) raw=(1.243,2.550) def=(1.243,2.780) DEFORM sign=0 steer=-0.035
collided=True laps=0
```

The router engages the right sign, picks the right side, and holds it. But the
target is wall-clamped: it wants `sign_y + 0.280 = 2.880` and `_WALL_CLEARANCE`
caps it at `3.0 - 0.220 = 2.7797`, leaving 0.1797 m to the sign where a yawed
chassis needs 0.2053 (half-diagonal 0.1803 + half-sign 0.025). **The commanded
line is 25.6 mm short before the robot even tries to follow it** — and it never
arrives anyway: impact at y=2.528 is 0.25 m short of the target, because pure
pursuit closes cross-track error over distance and the chassis draws abreast of
the sign first. Impact is 0.183 m from sign centre at 33° of yaw: a corner
strike.

The geometry underneath: an outer-line sign leaves 0.40 m to the outer wall. A
yawed chassis needs 0.3856 m of that plus wall margin (0.4256 total) and does
not fit; **aligned it needs 0.225 m and fits easily.** These passes are only
feasible square to the corridor, and two-thirds of WRO signs sit at corner
positions.

**Mode B — no deformation at all** (corpus scenario 1, red sign at (1.00,
0.60), the one that explains the activation cliff):

```
t=299 pos=(1.175,0.497) raw=(0.728,0.484) def=(0.728,0.484) sign=None
collided=True laps=0
```

`def == raw`, `sign=None`, 0.203 m from the sign. The waypoint has reached
x=0.728 but the south corridor's straight segment is x in [1.0, 2.0], so
`_is_squarely_in_corridor` rejects it, every candidate falls through
`deform_waypoint`'s loop, and the waypoint returns untouched. Signs at grid
depth 1.0 and 2.0 sit *on* the corner boundary, so the lookahead target crosses
into the corner during the final approach and **avoidance switches itself
off** — exactly what `_DEFORM_DEPTH_BUFFER`'s own comment predicts.

## Tuning measured against the corpus (2026-08-01)

All figures are 256 scenarios, laps-only scoring (`park=False` changes nothing).

| Configuration | Collisions | laps>=3 |
|---|---|---|
| baseline (act 0.80, buffer 0.30) | 229 | 27 |
| act 1.00 | 216 | 40 |
| **act 1.00 + buffer 0.50** | **209** | **47** |

Activation distance, at buffer 0.30: 0.80 -> 229/27, 0.90 -> 225/31,
**1.00 -> 216/40**, 1.10 -> 217/39, 1.20 -> 217/39, then a *total* collapse at
1.30/1.40/1.70 -> 256 collisions, 0 laps. The collapse is Mode B: all 39
survivors flip to lap-0 sign collisions.

Corner buffer, at act 1.00: 0.30 -> 216/40, **0.50 -> 209/47**, 0.70 -> 218/38,
0.90 -> 218/38.

Wall clearance, at act 1.00: 0.185 -> 221 (**16 wall**, 205 sign)/35,
0.200 -> 214/42, 0.220 -> 217/39. Lowering it relocates the crash onto walls;
read the wall/sign split, never the total.

**Noise floor:** `wall 0.220 @ act 1.00` gives 217/39 while the untouched
default (0.22027756) gives 216/40 — a **0.28 mm** change flips a scenario. Treat
differences of 1-3 runs as noise. The activation and buffer gains are well
outside it; the wall-clearance gain is not.

## The retirement bug — the real cliff (2026-08-01, FIXED)

The 256/256 collapse at ``activation_dist >= 1.30`` is **not** the corner dead
zone. That was my diagnosis from one trace and it was wrong: widening the
buffer to 0.50 left 1.30 at 256/0, which the dead-zone story cannot explain.

Instrumenting the router at the moment of impact:

```
final corridor south   candidates []
_engaged {2, 3, 4}   _passed {2, 4}
```

Sign 2 — the sign being hit — was already retired. ``_active_sign_candidates``
does this, in this order, on one tick:

```python
if settled and d < self._config.activation_dist:
    self._engaged.add(i)
if d > self._config.passed_dist:
    if settled and i in self._engaged:
        self._passed.add(i)
    continue
```

With ``activation_dist >= passed_dist`` every sign entering the activation
radius is engaged and marked passed in the same breath, a metre out, and stays
retired for the whole run. Nothing logs, nothing raises; avoidance simply stops
existing. The shipped ``passed_dist`` is 1.20, which is exactly where the cliff
sits.

Proof by construction: at ``activation 1.30`` the old pairing gives 256
collisions / 0 laps; moving ``passed`` to 1.50 gives 210 / 46. Same activation
distance, one coupled threshold.

**Fixed** by ``SignRouterConfig.__post_init__``, which raises rather than
clamps — silently repairing the config would hide that the tuning being run is
not the tuning that was asked for. Pinned by ``TestActivationPassedOrdering``.
This is the same defect ``settle_ticks`` guards from the other direction.

## Tuning peak, thresholds moved together (2026-08-01)

Buffer 0.50 throughout, ``passed = activation + 0.20``:

| activation / passed | Collisions | laps>=3 |
|---|---|---|
| 1.00 / 1.20 | 209 | 47 |
| 1.30 / 1.50 | 210 | 46 |
| 1.40 / 1.60 | 209 | 47 |
| **1.60 / 1.80** | **205** | **51** |
| 1.80 / 2.00 | 224 | 32 |
| 2.00 / 2.20 | 218 | 38 |

**Best measured: 205 collisions, 51/256 three-lap finishes — 27 -> 51, 10.5% ->
19.9%.** The peak is not broad: 1.80 costs 19 runs against 1.60. 1.40 holds 47
with twice the margin to that edge, which is the safer pick for hardware where
pose error moves the effective distance.

Still unadopted — every default is untouched. And still four failures in five:
this is tuning, not a geometric fix. Mode A (wall-clamped, 25.6 mm short of the
clearance a yawed chassis needs) is untouched by any of it.

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
