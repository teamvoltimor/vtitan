# 0033. The per-challenge speed ladders are a deliberate split

- Status: accepted
- Date: 2026-09-03

## Context

The two challenges do not want the same car. First Obstacles speed data ever
taken, full 256 corpus, `diag_sign_sweep.py speed --corpus`:

| fast_mps | collisions | laps>=1 | laps>=3 | in-time | laps driven | sign/lap |
|---|---|---|---|---|---|---|
| 0.156 | 128 | 89 | 45 | 38 | 197.0 | 0.533 |
| 0.500 | 144 | 83 | 17 | 16 | 137.0 | 0.788 |
| 0.600 | 145 | 61 | 10 | 9 | 96.0 | 1.219 |

Slower is decisively better for Obstacles: in-time 38 to 9, laps>=3 45 to 10,
per-lap sign collisions more than double. These gaps are far outside the noise
floor. Open, with the clearance ladder corrected, tolerates 0.80 m/s (117,
beating its baseline). The hardware profile is a per-run env var, so this is a
decision, not a conflict.

## Options considered

- (a) One speed ladder for both challenges.
- (b) Per-challenge tier overrides on top of the shared base ladder.

## Decision

(b). `speed.toml` carries base tiers plus `open_*` and `obstacles_*` overrides.
Absent and zero mean different things: nil falls back to the shared base tier,
while 0 is a tier a `gt=0.0` constraint rejects. A drivetrain with no headroom
declares neither prefix and both challenges share one ladder.

## Consequences

- Hand-updating `max_speed_mps` alone does nothing until the tiers move too
  (ADR 0011); the overrides are an explicit per-challenge edit.
- The reference row in the sweep is confounded; a shipped-config control must be
  run before using any absolute number from it.
