# NNNN. Short title in the imperative

- Status: proposed
- Date: YYYY-MM-DD
- Commit: <short sha that landed it>
- Supersedes: <NNNN, comma-separated, or omit>
- Superseded by: <NNNN, once a later ADR replaces this one, or omit>

## Context

What forced a decision: the observation, the measurement, the constraint. State
the premise that was believed, even if it later turned out false.

## Options considered

- (a) ...
- (b) ...

## Decision

What was chosen, and why the alternatives lost.

## Consequences

What the choice buys, what it costs, what it invalidates. Record any earlier
measurement that is no longer comparable.

## History

Optional, and only when one decision was touched repeatedly. List the commits in
order, one line each: `<sha> <date>: what it changed, with the numbers and run
ids, including the attempts that were refuted or reverted`. The current ADR tells
the whole story of a variable; do not spawn a micro-ADR per attempt. When this
ADR absorbs earlier ones, their decisions are carried into Context and Decision
above and their status becomes `superseded by NNNN`.

## Cross-references

Optional. Name the ADRs this one supersedes, the ones that stay separate and why,
and any known inconsistency (stale comment, drifted schema description) found
while writing it.
