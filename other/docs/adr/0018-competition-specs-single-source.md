# 0018. Competition round rules live in config, not Python literals

- Status: superseded by 0069
- Superseded by: 0069
- Date: 2026-09-10
- Commit: 00047e4c

## Context

Round timing and lap counts were hand-maintained literals in
`src/python/shared/src/shared/config/constants/simulation.py`.

## Options considered

- (a) Keep them as Python literals.
- (b) Move the rules to TOML and read them as config.

## Decision

(b). `src/config/competition_specs.toml` is the source of truth:
`round_time_limit_s = 180.0` (the official 3 minute round),
`open_challenge_laps = 3`, `obstacle_challenge_laps = 3`.

## Consequences

- The rule values are documented once with units and checked against their schema.
- The dead simulation constants were deleted with the move.
