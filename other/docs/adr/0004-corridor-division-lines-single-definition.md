# 0004. Corridor division lines are defined once

- Status: superseded by 0069
- Superseded by: 0069
- Date: 2026-09-13
- Commit: 3b6456d5

## Context

The two division lines painted across every corridor, measured out from the outer
wall, were previously written out four times over: `CorridorDivOuter` and
`CorridorDivInner`, `SignGridWidthOuter` and `SignGridWidthInner`, and their two
Python twins. Four constants holding two numbers.

## Options considered

- (a) Keep the four constants and keep them aligned by hand.
- (b) Store the two division lines once and derive every other lengthwise division
      from them.

## Decision

(b). `division_lines = [0.40, 0.60]` is the single definition. The starting
square's 40/20/40 bands and the sign-grid width lines the pillars are placed on
are derived from it (see `x-derived` in `track.schema.json`).

## Consequences

- Changing a division line updates every dependent measurement at once.
- Readers must follow the derivation rather than reading a literal per consumer.
