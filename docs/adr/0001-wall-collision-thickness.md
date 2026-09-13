# 0001. Wall collision thickness matches the visual wall

- Status: accepted
- Date: 2026-08-21
- Commit: matching LIDAR fix landed separately as 6c727c87

## Context

The simulated wall had a collision body 0.18 m thick against a 0.10 m visual
mesh, i.e. 40 mm inflated per side. It was justified by the claim that "the
LIDAR sits 30 mm ahead of the chassis front, so 40 mm keeps it at least 10 mm
outside the visual face even at full contact".

That premise was false. The C1 is flush with the bumper: `lidar.mount_x_offset`
is derived as `chassis.length/2 - mesh radius` (0.15 - 0.0278), so the puck's
front tangent lands exactly on the front edge and the sensor can never be
inside a wall the chassis has not already entered. The pass-through case the
inflation guarded against could not arise.

## Options considered

- (a) Keep the 40 mm inflation as defensive geometry.
- (b) Collapse collision to the visual thickness and fix the LIDAR model
      separately, so each change is attributable.

## Decision

(b). Collision thickness is 0.10 m, equal to the visual wall, with no
inflation. The matching LIDAR fix moved the simulated sensor forward to where
it really mounts. The two land as separate changes because they pull opposite
ways at the front (the LIDAR fix shortens forward ranges, this one restores
the body's space), and bundling them would make neither measurable.

## Consequences

- Every corridor behaved 8 cm narrower than spec (13 percent of a 0.6 m
  corridor) while the inflation stood.
- Parking was arithmetically impossible: a 0.194 m chassis in a 0.20 m bay
  needs its centre within 0.10 m of the wall, but collision stopped it at
  0.14 m. The long-standing 0/16 park rate was that contradiction, not the
  chassis and not the servo.
- Every pass rate recorded before these two changes had both distortions
  active and is not comparable to anything measured after.
