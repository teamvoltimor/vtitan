# 0038. The bay exit mirrors its reverse-leg steering

- Status: superseded by 0060
- Superseded by: 0060
- Date: 2026-09-10
- Commit: ec649f98

## Context

The clearance guard made the exit legal on 2026-09-04 but not effective: it
shuffled 2.78 m for no net outward gain. The diagnosis recorded then was "both
legs hold the same lock and a constant-|steer| cycle provably cannot
accumulate". That was the right diagnosis with the wrong conclusion drawn from
it: the reverse leg HELD the forward lock and therefore retraced the forward
arc, so each cycle returned the chassis to where it started. It was a pendulum,
not a ratchet.

Measured on hardware without mirroring: the steering held the same lock across
every reversal (0 flips in 111 and in 140 legs), forward legs turned +1.94 deg
each and reverse legs -1.74, and the chassis spent 815-1070 deg of rotation to
keep 2-7 with ZERO net travel, 0/2 out of the bay. With the mirror: 0 held / 4
flipped, 0 percent of consecutive legs cancelling, about 75 deg of rotation for
about 71 net (95 percent efficient), 0-4 reversals instead of 111-324, and out
of the bay 3/3 in 2.8 / 13.4 / 21.1 s.

The mirror flag existed in the tree as `bay_exit_reverse_steer_norm`, marked
REFUTED 2026-08-29. That refutation was measured on the pre-guard exit, which the
2026-09-04 banner voided wholesale, so the flag sat at its refuted value on the
strength of a measurement that no longer applied to anything.

## Options considered

- (a) Keep holding the forward lock on the reverse leg.
- (b) Mirror the steering so the arcs curve opposite ways.

## Decision

(b). `bay_exit_mirrors_reverse` / `bay_exit_hold_steer` ship true. Mirroring
makes yaw accumulate instead of cancelling.

## Consequences

- The exit leaves the pocket (3/3) instead of shuffling for no net gain.
- Method lesson: when a banner voids a body of work, re-examine the flags that
  body of work turned off.
- The legacy contact-bounded cycle's `bay_exit_reverse_steer_norm = 0.0` /
  `bay_exit_hold_steer` values still sit in the TOML but govern an unreachable
  path; do not read them as the shipped behaviour.

## Superseded by 0060

Replaced by [0060](0060-bay-exit-clearance-guard.md): The bay exit is bounded by
predicted fin clearance, not by contact.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
