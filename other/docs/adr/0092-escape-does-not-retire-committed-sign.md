# 0092. An escape does not retire the committed sign, and ships off

- Status: accepted
- Date: 2026-09-16

## Context

Most escapes are handed back the same target and still hold the same committed
sign, so the plan the chassis returns to is the one that drove it into the
object. Re-planning cannot fix that, because the sign is still in the map. The
candidate remedy is to retire the committed sign when an escape latches.

## Options considered

- (a) Rely on the router re-planning on the far side of the manoeuvre.
- (b) Retire the sign the router is committed to, once per latched manoeuvre.

## Decision

(b) is implemented but ships off: `escape_retires_committed_sign = false`.
`CoreNavigator._retire_escaped_sign` calls the router's `retire_committed()` once
per latched manoeuvre, not every tick. It is a single key, not split per
challenge, because only Obstacles carries a committed sign.

Retiring a sign the chassis has not actually passed forfeits its pass side, and a
wrong-side pass ends the round. The defence is that an escape only fires once the
pass is already compromised, with the chassis inside contact range of the object
it meant to go around, but that is an argument rather than a measurement.

## Consequences

- Measured 2026-09-15 over 105 escape episodes (five rounds,
  `scripts/bag/diag_bag_escape_convergence.py`): retiring gained a median of
  9.8 cm of forward clearance.
- The gain does not hold: 62 percent of the episodes were followed by another
  escape within two seconds, because 97 percent were handed back the same target
  and 79 percent still held the same committed sign.
- Off means the returned plan can still be the one that drove into the object,
  which is the failure this was written for.

## Cross-references

- 0055 owns the escape manoeuvre selection; 0050 owns the committed pass side.
