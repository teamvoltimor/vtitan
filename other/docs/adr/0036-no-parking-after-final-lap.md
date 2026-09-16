# 0036. Parking is not pursued after the final lap

- Status: superseded by 0062
- Superseded by: 0062
- Date: 2026-09-05
- Commit: 4e061f6f

## Context

The parking bay is geometrically unreachable for this chassis: 0.194 m of
chassis width against a 0.20 m pocket is 6 mm of total slack, +/-3 mm on the
centre, and a maximum heading error of 1.16 deg against the 6.0 deg the rule
allows. Chasing it after the laps are already banked only spends clock and
contact.

Measured blind over the 256 corpus, pursue-OFF against pursue-ON: in-time 158 vs
62, collisions 4 vs 51, timeouts 61 vs 110, with laps>=3 identical at 159.

## Options considered

- (a) Continue to pursue the bay after the final lap.
- (b) Stop in the finish section.

## Decision

(b). `attempt_after_final_lap = false`. The bay is unreachable, so the pursuit
spends the round for no return.

## Consequences

- A round that would have chased the bay stops in the finish section instead.
- This was fixed in Python at `4e061f6f` but Go had never read the key until it
  was mirrored 2026-09-06, so every Go corpus number taken before that was
  measured against the pursuing behaviour.

## Superseded by 0062

Replaced by [0062](0062-sim-contact-model-and-parking.md): The simulator scores the
rulebook contact model, and parking is a RACING phase.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
