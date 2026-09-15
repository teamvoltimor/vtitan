# 0031. Clearance thresholds are distance, not time

- Status: superseded by 0085
- Superseded by: 0085
- Date: 2026-08-28

## Context

There are seven independent speed caps, composed by `min()` with no
arbitration. The one that caps speed in practice is the clearance ladder:

```
contact 0.10 . slow 0.25 . medium 0.50 . fast 1.00 m
```

These do not scale with speed. Converted to reaction time, `medium_dist = 0.50 m`
is 3.2 s at the 0.156 m/s ceiling they were tuned against, 1.0 s at the shipped
0.50, 0.83 s at 0.60, and 0.63 s at 0.80. Every speed increase silently cuts the
runway the thresholds were designed to provide.

That predicts the 0.80 collapse directly: plain 53/128 against the 0.50
baseline's 114/128. Scaling the ladder with the ceiling (x1.6 at 0.80: contact
0.16 / slow 0.40 / medium 0.80 / fast 1.60) recovers almost the entire collapse:
53 to 111, and with `CRAWL 0.3` to 117, which beats the 0.50 baseline while
running a 60 percent higher ceiling. At 0.60 the thresholds are only about 17
percent off, so scaling adds nothing beyond `CRAWL` alone; the dose-response is
the strongest evidence the mechanism is dimensional rather than incidental.

Braking distance is not the issue: `max_accel_mps2 = 2.0` stops from 0.60 m/s
in 0.09 m. What the thresholds buy is steering runway, not braking runway.

## Options considered

- (a) Ship the x1.6 literals and leave the ladder distance-based.
- (b) Derive the thresholds from current speed (a time-to-collision budget) so
      the envelope travels with the speed profile.

## Decision

(b), not yet implemented. The refactor is the decision because shipping the
literals would leave the same latent bug for the next speed change, and the same
distance-based ladder feeds the `risk` cap and the sign-deformation cap on the
Obstacles path.

## Consequences

- Until the refactor lands, raising `fast_mps` shrinks reaction time with no
  compensating change (raising it 0.35 to 0.42 cuts `medium_dist`'s runway from
  1.43 s to 1.19 s).
- The clearance-ladder result is Open-only; the correction has not been tested
  on Obstacles, where the ladder stacks with the risk and deformation caps.
- The +/-4-case noise floor and the Obstacles "slower is better" data are in
  `other/docs/internal/algorithms/speed-envelope.md`.
