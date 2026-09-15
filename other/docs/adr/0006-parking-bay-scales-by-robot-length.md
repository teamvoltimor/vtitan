# 0006. Parking bay length scales by the robot's length

- Status: superseded by 0062
- Superseded by: 0062
- Date: 2026-09-13
- Commit: 3b6456d5

## Context

The bay length is set as a multiple of the robot's LENGTH, the dimension that has
to fit inside the bay. Scaling it by width instead once produced a bay exactly one
chassis long, which the robot could never enter.

## Options considered

- (a) Scale the bay by the robot's width.
- (b) Scale the bay by the robot's length.

## Decision

(b). `spacing_factor = 1.5` multiplies the robot length.

## Consequences

- The bay is genuinely enterable for the shipped 0.30 m chassis.
- The factor is meaningless without the robot's length from `robot.toml`.
