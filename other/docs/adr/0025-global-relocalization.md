# 0025. A lost pose is recovered by global relocalization

- Status: superseded by 0084
- Superseded by: 0084
- Date: 2026-09-07

## Context

The local search is a hill-climb reseeded from its own previous answer, with
nothing checking whether that answer is good in absolute terms. Once it lands in
the wrong basin it stays there for the rest of the round. Measured on
`run_20260907_205830` (Open, 0 laps): the estimate latched 1.5-3 m off at t=8.0 s
and never came back, driving the navigator into walls for 20 escape manoeuvres
and 46 s of net-zero travel.

The car cannot leave the track, so a pose whose predicted scan does not match
the real one is provably wrong, and a match landing off-track is evidence the
search is lost rather than a reason to stand still.

A cost over the threshold does not always mean the pose is lost -- it can mean
the wall model is wrong, which during blind operation (corridor widths still
being estimated) it usually does. Searching globally against a wrong model finds
the best explanation of a track that is not there.

## Options considered

- (a) Local search only.
- (b) A global re-solve when a bad-fit streak arms it, accepted only when it
      clearly beats the local estimate.

## Decision

(b). Off-track scans and over-threshold costs both count toward the same streak;
crossing `relocalize_after_scans` re-solves position over the whole free space
instead of the search window. The global winner is accepted only when its cost
beats the local one by at least `relocalize_accept_ratio`; otherwise the wall
model, not the pose, is presumed wrong and no jump happens.

Separation measured by replaying the three 2026-09-07 hardware runs across 1945
scans: neither clean 3-lap run crossed the threshold even once, while the failed
run sat at a median 0.0432. Replayed from its own corrupted pose, one
relocalization took it back to a 0.0101 fit and 0 percent off-track beams. The
accept ratio guards a false positive: without it the balanced-128 Open sweep
went 128/128 to 127/128, and one case lost 17 s.

## Consequences

- A pose lost in the wrong basin can be recovered mid-round.
- The accept ratio keeps a wrong wall model from triggering a spurious jump.
