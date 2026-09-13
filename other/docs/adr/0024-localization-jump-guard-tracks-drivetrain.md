# 0024. The localization jump guard tracks the drivetrain ceiling

- Status: accepted
- Date: 2026-08-29

## Context

`localization.MAX_SPEED_MPS` is an implausible-jump guard: a LIDAR pose update
implying more than this is discarded and the prior pose kept. It shipped at
0.25, sized against the retired motor's 0.156 m/s ceiling, and was never raised
when the profile changed. The current drivetrain measures about 0.58 m/s
closed-loop and ~0.9 m/s open-loop at the 2026-08-29 bench, so the gate sat
below the speeds the robot actually reaches and silently froze pose whenever it
drove quickly.

That was not just a localization bug, it invalidated a measurement: pose was
used as the independent check on encoder distance. On `run_20260829_003233` the
two agreed to 0.5 percent (28.46 m encoder against 28.62 m pose), and both were
under-reporting -- the encoder because `counts_per_rev` was too high, pose
because updates past 0.25 m/s were being thrown away. Two suppressed
measurements agreeing is not corroboration.

## Options considered

- (a) Keep the static 0.25 bound.
- (b) Raise the guard with the drivetrain, at a fixed headroom over the measured
      ceiling.

## Decision

(b). The guard is raised to about 1.5x the measured closed-loop ceiling, and
raised again whenever the drivetrain gets faster. It is a speed bound, not a
quality threshold; the relocalization path (ADR 0025) handles a pose that is
genuinely wrong.

## Consequences

- The guard no longer defers honest scan-match corrections at speed.
- A stale `MAX_SPEED_MPS` silently invalidates any comparison of encoder and
  pose odometry, so it must move with every motor or speed-profile change.
