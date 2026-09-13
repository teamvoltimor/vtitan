# 0002. LIDAR collision geometry sits at the real mount

- Status: accepted
- Date: 2026-08-21
- Commit: 6c727c87

## Context

The simulated LIDAR puck sat behind where the hardware actually mounts, so the
simulator reported clear space where the real sensor would already be inside a
wall. The real unit is flush with the front edge of the chassis.

## Options considered

- (a) Leave the simulated sensor behind the real mount and compensate with an
      inflated collision wall (the 40 mm hack in ADR 0001).
- (b) Place the sensor at the real mount and keep collision geometry honest.

## Decision

(b). The mount offset is derived, not declared: `chassis.length/2 - mesh
radius` (0.15 - 0.0278 = 0.1222), matching the `lidar_link` mesh already
modeled in
`other/apps/gazebo/runtime/robot_description/wro_robot.urdf.xacro`. The puck
geometry in `robot.toml`
(`diameter`, `height`) matches that same mesh so the mount derivation stays
self-consistent with what is rendered.

## Consequences

- Forward ranges shorten to what the hardware really sees. Pass rates measured
  before this change are not comparable to those after (see ADR 0001).
- Collision no longer needs an inflation term, which unblocks parking (ADR
  0001).
- This decision and ADR 0001 are intentionally separate commits: they pull
  opposite ways at the front, so bundling them would make neither measurable.
