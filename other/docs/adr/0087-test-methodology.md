# 0087. Testing uses a four-level ladder and a fixed A/B protocol

- Status: accepted
- Date: 2026-09-15

## Context

The most expensive mistakes were not programming errors but unverified
assumptions. Fixtures misled repeatedly, a corpus pinned one start cell, and a
simulator that did not model contact, sensing or the servo was trusted to approve
control-law changes.

## Options considered

- (a) Trust the simulator and fixtures.
- (b) A four-level ladder with an explicit rule that a level-2 conclusion is not a
      conclusion, a fixed-seed corpus, and a fixed A/B protocol.

## Decision

(b). The ladder is: level 1 unit tests (95 Python files, 162 Go files) for broken
logic and contracts; level 2 sim corpus for navigation behaviour over hundreds of
tracks; level 3 hardware bench for what the sim does not model (servo, motor, real
LIDAR); level 4 track rounds for the truth. A conclusion that exists only at level
2 is not a conclusion; the simulator discards fast and compares, it does not
approve.

The corpus is fixed-seed (`task gen:corpus:all` = 640 Open and 256 Obstacles, seed
2026), and the same revision on the same corpus gives the same number. Open
screening uses `balanced128`, which varies the start cell rather than pinning
`start_cell = 0` as the old `open128` did; `open128` overstated the shipped config
by about six cases and hid one fix while inflating another. Never quote an Open
figure without its corpus AND seed: the baseline swings nine points from start-cell
assignment alone.

The full metric set is read together: laps>=3, in-time, timeouts, collisions, and
(wrong-side passes on Obstacles, scored against ground truth). `SimResult.success`
is degenerate on Obstacles because it requires `parked`, which is false in 256 of
256; split by collision, not by success. Never read a config value from a literal;
load `load_default()` and print it, and count invocations at the real call site
before an A/B. Never validate a label against the function that derives it; use
independent ground truth. Do not compare sweeps across invocations if concurrent
edits occurred.

The A/B protocol has five named traps: screen with 128, decide with 640; the
reference is the parent commit, not an earlier measurement; results are not
comparable across time; both arms must run identical test modules; prove the two
arms differ before believing the result. `--tuning` replaces the config tree, it
does not merge. Changes that break comparability carry `!` in the commit type.

## Consequences

- The simulator cannot approve a change on its own.
- A measurement without commit, corpus, seed and profile is meaningless.
- CI being red and ignored is worse than no CI.

## History

- 06b7fc1a 2026-08-01: let the chassis graze a wall instead of freezing; every
  pass rate before it needs re-measuring.
- 7b4a50d6 2026-08-01: score the 180 s round limit as a pass criterion.
- 54f326c1, 3b04fcc5 2026-08-22: model and generate the full 640 Open space.
- 551b7f9a 2026-08-30: vary the start cell by default (`balanced128`).
- 59fbcaee 2026-09-04: port the balanced128 screening corpus to Go with a
  reimplemented CPython `random`.
- 3b6456d5, b982e109 2026-09-13: config schemas, `config:check`, and `x-journal`
  rationale links.
- d7e24594 2026-09-14: document the testing workflow and the `!` convention in
  `tests.md` and `CHANGELOG.md`.

## Cross-references

- 0086 owns the fidelity changes the `!` convention marks; 0069 owns the config
  `config:check` that closes the documentation loop.

## Evidence

- Go tests: fuzz the serial frame decoders, use `testing/synctest` for the
  watchdog, and run an in-process NATS (no Docker) behind `//go:build integration`;
  bag-replay and sim-corpus parity are the go/no-go gates.
- `contact_dist = 0.05` can only be settled end-to-end (scan to tick to PID to
  decel) and has no env override; sweep arms are set by editing `clearance.toml`.
- Pytest runs via `pixi run -e dev test` plain, never bare `pytest` (which loses
  the RMW pin and causes false failures). `-n 6 --dist loadscope` is the confirmed
  cap; `-n 12` re-dispatches a dead worker and is slower.
- Parallelise all test and sim sweeps by default (5.6x on 24 cases) and never pipe
  a long run through `tail`.
- Windows is a first-class constraint: pin `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  on win-64 pytest tasks, cap xdist at 6, and do not assume dependency, Docker or
  subprocess parity.
