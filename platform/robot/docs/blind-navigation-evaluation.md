# Blind navigation evaluation

Plan for validating `TrackNavigator` under the conditions it actually faces at
competition: driving a track it was **not** told about.

## Why the current test doesn't prove much

The sim harness builds both the simulated world and the navigator's track model
from the *same* scenario metadata. So every passing run assumes perfect prior
knowledge of corridor widths, start section and direction -- knowledge the robot
cannot have on the day, because the round's track is randomised and it is placed
on it without being told which layout it is.

"Passes N open scenarios" therefore measures *path following given the answer*,
not *can it drive an unknown track*. Those are different claims, and only the
second one predicts competition behaviour.

## What blind evaluation means

Separate two things `SimulatedHardwareGateway` currently conflates:

| | Source today | Source under blind eval |
|---|---|---|
| Simulated world (ground truth) | scenario metadata | scenario metadata (unchanged) |
| Navigator's track model | the same metadata | a **generic/default** model |

Then: simulate scenario N, hand the navigator the default model, and score laps
completed and collisions. Repeat across the scenario set. The pass rate is the
number that actually predicts race-day behaviour.

## Why the obvious shortcut does not work

An earlier idea was to exploit the track's 4-fold rotational symmetry: assume a
canonical start corridor, since all corridors would be geometrically identical
and the waypoints would be correct regardless of where the robot really is.

**That is wrong.** Corridors are independently either **60 cm or 100 cm** wide,
so the four are not interchangeable and the track is not symmetric in general.
`TrackWalls(corridor_widths_m)` takes per-section widths precisely because they
differ. Assuming a canonical start would produce waypoints for the wrong
geometry.

The same fact cuts the other way, though: corridor width is a large, directly
observable quantity (60 vs 100 cm is far bigger than LIDAR noise), so a
stationary scan can classify the corridor it is sitting in. What it cannot see
is the other three -- the centre block occludes the opposite corridor -- so a
full track model could only be built progressively, on entry to each corridor.

## The question blind evaluation answers first

Whether that progressive measurement is needed **at all**.

If the navigator completes laps with a default track model, then the waypoints
are a coarse guide and the reactive layer (collision avoidance, corridor
centring) is doing the real work -- and no LIDAR width estimation is required.
If it corner-cuts or collides, widths matter and must be measured.

Building LIDAR width estimation before running this test would be solving a
problem that may not exist.

## Prerequisites (do not skip)

1. **Re-tune `steer_kp`.** The kinematics were corrected on 2026-07-25 to
   counter-phase four-wheel steering (see `sensor-verification.md`), which
   doubles the yaw rate for a given steering angle. `steer_kp` was fitted against
   the old front-steer model, so it is roughly 2x too hot. A blind run now would
   fail for steering reasons and the result would say nothing about track
   knowledge. *(In progress in a parallel session as of 2026-07-25.)*
2. **Re-establish the baseline.** Prior "passes N scenarios" figures were
   measured against the old model and no longer hold. Re-run the normal
   (non-blind) evaluation first, so the blind pass rate has something to be
   compared against.

## Then, physical testing

Only after blind sim evaluation passes is a real-track run informative -- a
physical failure would otherwise be ambiguous between navigation logic, track
knowledge, and hardware.

For the physical run, note that `race.launch.py` requires a `metadata:=` path and
no scenario data exists on the Pi 5. Under the blind design that requirement
should disappear (or default), since the whole point is that the navigator does
not consume a per-round scenario.

Also relevant: `TrackNavigator` now holds stopped until `/robot_state` reports
`racing`, so a real run starts on the button press rather than the moment the
node launches.

## Scope note

This applies to the **Open Challenge**. The Obstacle Challenge genuinely needs
layout knowledge -- sign positions break the symmetry and determine pass side --
so blind operation there is a different and harder problem.
