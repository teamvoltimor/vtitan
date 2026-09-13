# 0019. Wheel geometry is split off the simulated track topic

- Status: accepted
- Date: 2026-09-13
- Commit: d0f55ce4

## Context

The chassis/sensor/wheel geometry could have shared the `track` topic with the
simulated track markers. The track markers are cached and re-sent about once a
second, and that cached array leads with a `DELETEALL`, which would wipe a freshly
published wheel if the two shared a topic. The wheels carry the live steering
angle and so must go out every tick.

## Options considered

- (a) Publish the robot model on the `track` topic with the markers.
- (b) Give the robot model its own topic.

## Decision

(b). `robot_model = "/sim/robot_model"` carries the chassis/sensor/wheel geometry
in the `base_link` frame, separate from `track`. The `plan` and `sign_estimates`
topics are kept as beliefs, not ground truth: under the Obstacles Challenge the
plan IS the avoidance manoeuvre (the sign lane is a rewrite of these waypoints),
so watching a blind run without them shows the chassis moving but not the
algorithm moving it.

## Consequences

- A cached DELETEALL on `track` cannot wipe the per-tick wheels.
- The plan and sign estimates make the navigator's live belief observable.
