# 0030. The servo slew rate is split from the steering-rate policy

- Status: superseded by 0076
- Superseded by: 0076
- Date: 2026-09-11
- Commit: 0fecf09a

## Context

One number was doing two jobs that pull in opposite directions. `MAX_STEERING_RATE`
is a CORNERING POLICY: lowered 2.0 to 1.2 on 2026-08-28 because cornering was
too drastic, and it still binds 8.2-13.1 percent of driving ticks. Bay exit also
budgets its servo standstill from the same number as `ceil(swing / (rate /
CONTROL_HZ))`, so at 1.2 rad/s a full lock-to-lock reversal costs about 50 ticks
of commanded ZERO. The bay spent 92 percent of its ticks there, confirmed on
track before the change (commanded-zero runs p50 2.551/2.556/2.552 s against the
2.50 s the constant predicts).

The model was simply wrong. Measured 2026-09-11 on the bench with the wheel
LOADED (`scripts/hardware/diag_servo_slew.py`): the wheel reached the far lock
at a 1.20 s hold and stopped about 20 deg short at 0.90 s. That shortfall is the
better datum: 150 deg in 0.90 s is 2.91 rad/s, implying a 1.02 s full swing.

## Options considered

- (a) Keep using `max_steering_rate` for both the policy and the physical model,
      and lower the policy to make the bay fast.
- (b) Split the physical slew rate out from the policy.

## Decision

(b). `servo_slew_rate_rad_s` is a model of the hardware, consumed by bay exit's
standstill budget; `max_steering_rate` stays the cornering policy. Shipped at
2.4 rad/s, the conservative end of the measurement: too high under-budgets the
pause and dead reckoning assumes an angle the wheel has not reached, while too
low only costs time. Bay budget 50 ticks to 25 per reversal.

The exit went from 30.5 s to 5.3 s (`run_20260911_211817` against
`run_20260911_225646`), at the same net rotation and displacement, and the
manoeuvre needs 3 reversals instead of 11. A finer ladder between 0.90 and
1.20 s plus a reading of `MAX_WHEEL_ANGLE_DEG` would justify 2.7-2.9; they are
coupled, because a wheel stopping short of 85 deg makes a rate computed from a
170 deg swing overstated.

## Consequences

- Fixing the bay no longer re-heats cornering, and retuning cornering no longer
  silently changes the bay budget.
- The value is a hardware number needing a loaded bench measurement; the
  steering-position topic is an echo of the command and can never settle it.

## Superseded by 0076

This decision was replaced by [0076](0076-drivetrain-and-steering-hardware.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

The traction branch measured about 10 A sustained at 50 percent duty and peaks
near 20 A, against the L298N's 2 A per channel: the old H-bridge was not failing,
it was operating at an order of magnitude over its specification, which produced
brownouts and heating. The Pi Zero 2 W SoC also has exactly two hardware PWM
generators, and a floating H-bridge enable line spun the motor at full reverse
with no software running.

Encoder calibration had similar silent failures: a motor swap invalidated every
constant derived from motor physics, and an unset hardware profile modelled
whichever servo happened to be checked in.

### Options considered

- (a) Keep the L298N, share one PWM channel, inherit encoder constants, and rely
      on passive pull-downs.
- (b) Size the H-bridge on measured current, allocate the scarce PWM engines
      deliberately, sequence enables and drive the GPIO actively, and require
      per-motor calibration.

### Decision

(b). The BTS7960 (43 A, MOSFET, low drop) replaces the L298N; after the swap the
weakest link is the power switch, whose conduction capacity is still unmeasured.
The harness uses genuinely independent RPWM and LPWM with `R_EN` and `L_EN`
permanently HIGH: direction is selected by which channel carries duty. The servo
uses hardware PWM on GPIO12 (absolute position, jitter-intolerant); forward drive
uses the one free hardware engine on GPIO13 (frequent, performance-critical);
reverse rides software PWM on GPIO26 because only parking and recovery use it and
it tolerates jitter. One `pwm-2chan` overlay exports both hardware channels.

Enables are asserted only after both PWM channels are confirmed at 0 duty; the
GPIO lines are actively driven low (`op,dl`, not the ineffective `pd`), re-driven
on service exit via `ExecStopPost`, and re-driven on driver disconnect, because a
plain stop once reproduced full-speed reverse. A physical pull-down on LPWM is the
complete fix and is still not installed.

`counts_per_rev` and `max_rpm` are required fields, never inherited across a motor
swap. `max_duty = 0.5` caps closed-loop drive duty. Steering facts stay physical:
`max_wheel_angle_deg` is the bench-measured linkage travel and must never be
lowered as a tuning action; `steering_limit_deg` is the navigator's policy, and a
validator rejects a limit above the physical maximum (see 0050). `servo_slew_rate_rad_s
= 2.4` is a loaded bench measurement and must not be read from the steering-
position topic, which is only an echo of the command.

Reverse is about 40 percent slower than forward at identical duty (336.6 against
193.7 deg/s), attributed to brushed-motor timing advance, so escape reverse bursts
travel less than a symmetric model predicts.

### Consequences

- The H-bridge operates within spec and the boot spin is removed.
- A motor swap cannot silently reuse another motor's numbers.
- The PWM allocation is a deliberate use of a scarce resource, not an accident.
- Open residual risks: no physical LPWM pull-down; the feedback sign is
  inconsistent with the command sign under a reversed drive; reverse asymmetry has
  not been re-measured since the motor swap.

### History

- 6732680d 2026-07-25: characterise the drive motor on battery power. Deadband
  below about 0.7 m/s; reverse asymmetry 336.6 against 193.7 deg/s.
- c81625c3 2026-07-25: calibrate encoder counts_per_rev against measured travel
  (194 to 676). Duty-based calibration abandoned (43 against 54 cm).
- bddcdf52 2026-07-25: drive the steering servo from hardware PWM to stop jitter.
- dc4051ce 2026-07-28: move the drive motor PWM to hardware sysfs, off gpiozero
  software PWM; switch to `pwm-2chan`.
- b238de37, e78e7fe2, 3b2571cd 2026-08-25: split drive from encoder, add the
  BTS7960 backend, fix R_EN/L_EN pins, make encoder calibration required.
- a67345a9 2026-08-26: the master change to independent RPWM/LPWM (bundled inside
  a nav commit; the branch twin is 6aee1ac2).
- 7ce5f207, f5797c6a, 96020000, 873a1232, 9748b30d, d8791b89 2026-08-27/28: drive
  the GPIO low from earliest boot (`op,dl` refutes `pd`), re-drive on exit and
  disconnect, template the pins, and assert enables only after 0 duty.
- 8fccb633 2026-08-27: flip `drive.reversed` for the BTS7960 rewire.
- 51c9f914 2026-08-27: cap closed-loop drive duty at 50 percent after the motor
  swap.
- 4f254ef4 2026-08-28: recalibrate the encoder (676 to 86, 42.5 to 123) and rescale
  PID gains about 8x.
- 1913975c, 958011fa 2026-08-29: counts_per_rev 86 to 60, live-verified.
- bbf9660d and 27146f7a 2026-08-29: raise max_rpm so the feedforward stops
  saturating (123 to 175 to 348); add the duty deadband (`rpm = 434.6*duty - 86.7`).
- d47b1cbe 2026-08-29: split encoder calibration into per-motor profile overlays.
- bbd6803f 2026-08-30: raise the speed ladder for the new motor.
- 654e57e6 2026-09-02: `max_speed_mps = 0.58` measured, not the feel-based 1.0.
- 0fecf09a and 73693f53 2026-09-11: split the servo's real slew rate from the
  command rate limiter; 2.4 rad/s measured (bay exit 30.5 to 5.3 s).

### Cross-references

- 0010 and 0030 are superseded; their decisions are carried above.
- 0050 owns the escape steering angle policy in physical units.
- 0070 owns the profile overlays that hold per-motor calibration.
- 0060 owns the bay exit that rides on the servo slew rate.

