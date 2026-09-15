# 0043. Obstacles inner-wall contact scoring is configurable

- Status: superseded by 0059
- Superseded by: 0059
- Date: 2026-09-11

## Context

Operator-confirmed rule set, 2026-09-11:

- OPEN: the OUTER wall may not be touched (unchanged by this flag).
- BOTH: a wall may not be MOVED if it is not fixed, and moving one takes
  considerable force, which a scrape at this chassis's mass and speed does not
  reach. That is what makes relaxing the inner-wall rule sound and not merely
  permitted.
- OBSTACLES: the PARKING LOT may not be touched (9.24.7).

Terminal regardless of this flag: the parking lot, displacing a sign out of its
85 mm circle (9.20), and the wrong pass side (9.19/9.24.5). Those three are the
whole of what ends an Obstacles round.

## Options considered

- (a) Keep inner-wall contact terminal, as every figure in this repo was
      measured under.
- (b) Score the actual rule, with inner-wall contact non-terminal.

## Decision

`obstacles_inner_wall_terminal` is configurable. TRUE keeps the strict scoring
every figure in this repo was measured under; FALSE scores the actual rule. A
result must state which scoring it used -- one taken under each and compared is
meaningless.

## Consequences

- There is now an explicit, recorded difference between the harness baseline and
  the competition rule.
- The flag does not touch the parking lot, sign displacement or pass side, which
  remain terminal in both settings.
