# Blind navigation evaluation

Validating `TrackNavigator` under the conditions it faces at competition:
driving a track it was **not** told about, with sensors nobody can buy perfect.

Status: implemented and measured. This document was a plan; it is now results.
Everything below is simulation only -- see [What is not
validated](#what-is-not-validated).

## The problem this addressed

The sim harness used to build both the simulated world and the navigator's
track model from the *same* scenario metadata, so every passing run assumed
perfect prior knowledge of corridor widths, start section and direction --
knowledge the robot cannot have on the day, because the round's track is
randomised and it is placed on it without being told which layout it is.

"Passes N open scenarios" therefore measured *path following given the answer*,
not *can it drive an unknown track*. Only the second predicts race day.

Two separate things were being given away:

| | Withheld by |
|---|---|
| The track (corridor widths) | `blind=True` |
| The robot's own pose and heading | `SensorErrors` |

## What the robot is still told

Blind does not mean it knows nothing. It keeps what is legitimately knowable
before a round:

- the track is 3x3 m and the loop is rectangular
- every corridor is **either 60 or 100 cm** -- the load-bearing one
- which section it starts in, and its travel direction
- its starting pose, which the operator sets by placing it

It derives the two things that change every round: **which corridors are wide**,
and **where it currently is**.

### Why the symmetry shortcut does not work

An earlier idea was to exploit the track's 4-fold rotational symmetry and assume
a canonical start corridor. That is wrong: corridors are independently 60 or
100 cm, so the four are not interchangeable and the track is not symmetric in
general. Assuming a canonical start produces waypoints for the wrong geometry.

The same fact cuts the other way. Corridor width is large and directly
observable -- 40 cm of separation against ~3 cm of LIDAR noise -- so it is never
*measured*, only classified into one of two values. What cannot be seen is the
other three corridors, since the centre block occludes the opposite one, so the
model is built progressively on entry to each corridor. See
`src/navigation/corridor_estimator.py`.

Width readings are attributed to a corridor **by heading, not position**.
Position comes from matching against a wall model built from the widths being
estimated, so attributing by position is circular -- a wrong belief mis-files
the reading that would have corrected it. Measured: 19/28 by position,
28/28 by heading.

## Reproducing it

### Watch a run in RViz

RViz draws the **true** track and **true** pose while the robot steers on its
estimate, so any divergence you see is state-estimation error reaching control.

Hardware at datasheet spec (the 27/28 row):

```
task sim:navigate:visualize:all -- --challenge open --interactive --blind \
  --place-error 2 --yaw-bias 2 --imu-drift 0.0083 --gyro-scale 0.25 --imu-noise 0.5 --rate 3
```

Realistic, roughly 2x spec (the 23/28 row):

```
task sim:navigate:visualize:all -- --challenge open --interactive --blind \
  --place-error 5 --yaw-bias 3 --imu-drift 0.02 --gyro-scale 0.5 --imu-noise 1.0 --rate 3
```

Swap `--interactive` for `--scenario go_open_0015` to go straight to the fixture
that fails in both profiles. Add `--recover` to make walls solid and let the
robot reverse out of contact instead of ending the run at first touch -- that is
how you tell a graze from a genuine pin.

**What to look for:** scale error grows with corners turned and drift with time,
so lap 1 should look clean and lap 3 visibly worse. Degradation that is present
from the start is the bias term, not the accumulating ones.

### Re-run the numbers

```
python scripts/sim/diag_localization.py perturbed --sweep combined --verbose
python scripts/sim/diag_localization.py perturbed --sweep all --workers 14
```

Sweeps: `placement` `heading` `drift` `drift-fine` `scale` `noise` `combined`.
Budget ~5 min per row (28 fixtures, 14 workers); a full `all` run is over an
hour.

## Results

All figures are the 28 Go-generated Open Challenge fixtures, blind, three laps.

### Combined -- the number that predicts race day

| Profile | Pass | Layout | Collide | Peak yaw |
|---|---|---|---|---|
| ideal | 28/28 | 28/28 | 0/28 | 0 deg |
| **bno085 at spec** | **27/28** | 27/28 | **0/28** | 7.8 deg |
| realistic (~2x spec) | 23/28 | 27/28 | 2/28 | 14.1 deg |
| pessimistic (~5x spec) | 15/28 | 25/28 | 6/28 | 28.7 deg |

At spec: 96%, zero collisions. The single failure completes 1 lap and mis-learns
the layout; it runs out of round rather than hitting anything.

This is a slope, not a plateau. Being at spec is worth ~4 fixtures and all of
the collisions.

### Per axis, where the knee is

| Axis | Safe to | Knee | Hardware sits at |
|---|---|---|---|
| Placement error | 20 cm+ | none found | irrelevant |
| Yaw bias | 2 deg | **5 deg** | placement squareness |
| IMU drift | 0.05 deg/s | 0.1 deg/s | 0.0083 deg/s (**6x margin**) |
| Gyro scale error | 0.25% | **0.5%** | ~0.25-0.5% (**at the knee**) |
| Yaw noise | 1 deg | mild throughout | irrelevant |

**Placement is free** because it is the one error the robot can observe -- the
walls say where it is, and the localizer absorbs up to 40 cm within one scan.
Past ~50 cm it never recovers, because `LidarLocalizer` is a local search seeded
from the previous estimate. It converges, but not from anywhere.

**Scale error is the tightest axis**, and the one with no datasheet figure
(~1-2 deg per revolution is forum anecdote). Three laps is twelve 90-degree
corners, so over 1080 degrees is banked before steering corrections -- a
lap-driving robot maximises exactly the error a robot vacuum averages out.
Worth a bench measurement: drive ten known revolutions, compare reported yaw
against 3600 degrees.

**Peak yaw error does not compare across error types.** 0.5% scale and 5 deg
bias both peak near 5.4 deg, but score 25/28 and 20/28. A bias is wrong from the
first corner; scale error starts at zero and grows, so the robot drives most of
the round well.

**Layout learning holds at 25-28/28 throughout.** Heading error corrupts corridor
attribution before it corrupts driving, so failures are steering failures in a
rotating frame, not lost maps.

## Hardware implications

The BNO085 runs in **UART-RVC mode** (`src/hardware/imu/bno08x/uart_rvc.py`),
which matters more than it looks:

- It is the **6-axis fusion, no magnetometer**, so the datasheet's 0.5 deg/min
  (0.0083 deg/s) drift applies rather than Rotation Vector's 3.5 deg error. That
  is the better side of the trade for this application, and it is immune to the
  motors and steel around a competition mat.
- It is **output-only**. There is no SHTP command channel, so the datasheet's
  own mitigation -- disable gyro auto-calibration when the device lacks tremor
  -- is unavailable. The datasheet warns that slow horizontal rotation around
  gravity with insufficient tremor can fool ZRO calibration, which describes a
  wheeled robot driving a loop on a smooth mat exactly.
- **Yaw is relative to power-on**, with nothing to correct it. The heading
  reference must therefore be zeroed at the **start button**, not at node
  startup -- the robot is carried to the track after boot, and that rotation
  would otherwise fold into its world heading. Fixed; see
  `state_machine/estimator.py::reset_heading_reference`.

With that fix, the entire yaw-bias budget is *how squarely the robot is set
down*. 2 deg passes 27/28, 5 deg drops to 20/28, which argues for a placement
jig rather than placing by eye.

## Contact rules

Each challenge forbids a different wall: the Open Challenge the **outer**, the
Obstacles Challenge the **inner**. `TERMINAL_SURFACES` in `simulation/gateway.py`
maps challenge to the contacts that end a run; contact with the permitted wall
is recorded in `SimResult.contact_count` instead of failing the round.

Two consequences worth knowing:

- Walls are **solid** for any permitted surface. Contact used to be detected but
  never resisted, which was harmless only while every touch ended the run on the
  tick it happened. Once a surface stops being terminal, a passable wall lets the
  robot drive through the inner block and go on counting laps.
- Signs and parking blocks are grouped with the inner wall as terminal. **This is
  an assumption, not a quoted rule.** All four known obstacles-challenge failures
  are sign collisions, so if signs are meant to be a survivable penalty, that one
  line changes those results directly.

## What is not validated

Nothing here has run on the robot.

The simulation still gives away **perfect wheel traction, zero control latency,
and exact steering response**. The IMU was the first place we looked, not the
last thing that can hurt. Every layer of realism removed so far has cost real
pass rate.

The heading-reference fix in particular **cannot** be validated in simulation by
construction: `ScenarioSimulator` seeds the estimator with the true starting yaw
and feeds it ground-truth IMU, so there is no transport phase to get wrong. It
needs a real power-on, carry, place, press cycle.

For a physical run, note that `race.launch.py` requires a `metadata:=` path and
no scenario data exists on the Pi 5. Under the blind design that requirement
should default or disappear, since the navigator no longer consumes a per-round
scenario.

## Scope

The results above are the **Open Challenge**. The Obstacles Challenge needs one
more thing withheld -- the sign layout, which breaks the symmetry and
determines pass side -- and that is now wired up too.

`blind=True` on an obstacles scenario withholds the corridor widths, the travel
direction **and** the sign positions. What replaces the last of these is
`navigation/planning/sign_discovery.py`: the mocked vision node projects each
detection to a world position through the pinhole model the router already
used for colour confirmation, and those observations are accumulated into
persistent sign tracks. Measured over the 16 obstacles fixtures, the signs the
robot drives past are found with **1.9 cm median position error (p90 3.4 cm),
every colour correct, and no spurious tracks**.

Two things that measurement does *not* say:

- **The camera is perfect in sim.** `vision_emulator.py` inverts the same
  projection `_detection_to_world` decodes, with no pixel noise, no
  quantisation and no occlusion, so the residual error above is almost entirely
  robot-pose error rather than perception error. This is the same category of
  giveaway that "perfect wheel traction, zero control latency" is above, and
  every one of those removed so far has cost real pass rate. Treat 1.9 cm as a
  floor.
- **Discovery is not the blocker, and blind is not what makes obstacles fail.**
  Blind and sighted score identically (16/16 collisions, 0/16 laps), because
  both are stopped further upstream by the reactive collision layer -- see
  [sign-avoidance-investigation.md](sign-avoidance-investigation.md).

The same wiring is on the real robot: `node.py` previously built a `SignRouter`
only when metadata supplied the signs, so a blind run on the mat had **no sign
avoidance at all**. It now always builds one and discovers into it.
