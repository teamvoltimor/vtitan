# 0029. The Open Challenge uses its own lookahead

- Status: superseded by 0052
- Superseded by: 0052
- Date: 2026-09-03

## Context

The two challenges want different values for the straight lookahead, and
everything else in `pursuit.toml` is shared, so one number costs whichever
challenge does not get it. Measured 2026-09-03, both arms paired against the
same seeded cases:

| corpus | result |
|---|---|
| Open 640 | mean sim time -5.39 s/case (530 faster, 94 slower); one verdict flips ok to incomplete |
| Obstacles 256 | WORSE: clean 21 to 19, timeouts 35 to 40, escapes/lap 59.6 to 66.4; sign-pass crosstrack identical at 10.32 cm median |

The Open tracking gain does not reproduce on Obstacles.

## Options considered

- (a) One shared lookahead.
- (b) A per-challenge override that replaces the base value on the challenge it
      names.

## Decision

(b). `open_lookahead_long` replaces `lookahead_long` on an Open run only.
Obstacles is untouched, so this cannot shadow the base constant on an Obstacles
sweep. `obstacles_yaw_gain_compensation` is the mirror decision for the yaw
compensation term: it replaces the base on an Obstacles run only, where the sim
plant is exactly `yaw_gain = 0.55` but the on-robot 1.8x is still unresolved
between `rear_steer_ratio` and `linkage_ratio`.

## Consequences

- The challenge that must not regress (Open 640) keeps its gained time without
  moving Obstacles.
- The per-challenge keys are named by prefix; a challenge with no override
  shares the base ladder, which is the design rather than a degenerate case.

## Superseded by 0052

Replaced by [0052](0052-pursuit-target-selection.md): Pursuit selects its target by arc
length and path sense, and arms the lookahead on the corner ahead.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
