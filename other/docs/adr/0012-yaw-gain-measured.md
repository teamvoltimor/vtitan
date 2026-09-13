# 0012. Yaw gain is a measured fraction of the kinematic model

- Status: accepted
- Date: 2026-08-29
- Commit: bfd644d4

## Context

The fraction of the yaw rate the kinematic model predicts that the chassis
ACTUALLY delivers is 1.0 in the zero-slip textbook model; anything less is the
tyre slip and linkage compliance that model has no term for. Before this was
measured the simulator ran an implicit 1.0 and turned 1.83x sharper than the real
car over the same command stream: 3512 deg of yaw against the IMU's 1918, so every
corner the sim cleared was a corner the hardware would have run wide.

## Options considered

- (a) Keep the implicit 1.0 zero-slip model.
- (b) Measure the delivered yaw and scale the model.

## Decision

(b). `yaw_gain = 0.55`, measured 2026-08-29 from `run_20260829_140424` (a clean
3-lap Open run) via `scripts/bag/diag_bag_sim_fidelity.py --replay`. The shortfall
is flat (~0.49) from 5 to 30 deg of commanded wheel angle, which is what makes it
a scale factor rather than large-angle saturation.

## Consequences

- Sim yaw matches the hardware over the same command stream.
- It is a property of THESE tyres on THIS surface: re-measure after a tyre, weight
  or mat change, the same way `max_speed_mps` is re-measured after a motor swap.
