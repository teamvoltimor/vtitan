# 0030. The servo slew rate is split from the steering-rate policy

- Status: accepted
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
