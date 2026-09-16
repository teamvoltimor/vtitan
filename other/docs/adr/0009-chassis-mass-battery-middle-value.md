# 0009. Chassis mass is the middle of the battery configurations

- Status: superseded by 0077
- Superseded by: 0077
- Date: 2026-09-07
- Commit: a23d725a

## Context

The assembled car weighs about 1.51 kg with the practice battery (Deans) and about
1.46 kg with the competition battery (XT60 shorty), connectors included
(2026-09-07). The URDF and the Gazebo model sum `chassis.mass` with four wheel
masses to get the total.

## Options considered

- (a) Use the practice-battery mass.
- (b) Use the competition-battery mass.
- (c) Use one deliberate middle value.

## Decision

(c). `chassis.mass = 1.3` kg is the chassis body ALONE, because the consumers sum
it with four wheel masses: 1.3 + 4 x 0.05 = 1.5. That is a deliberate middle value
between both battery configurations, so the simulation never under-predicts
required torque.

## Consequences

- Change chassis and wheel mass together, or the simulated car stops weighing what
  the real one does.
- Mass drives the inertia tensors, so the choice reaches the dynamics.

## Superseded by 0077

Replaced by [0077](0077-power-rails-and-chassis-mass.md): Power rails are sized to
measured load and the chassis mass is the battery midpoint.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
