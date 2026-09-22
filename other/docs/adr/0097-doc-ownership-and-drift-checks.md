# 0097. Tracked prose owns only the present it can be checked against, and a docs check enforces it

- Status: accepted
- Date: 2026-09-22

## Context

Config drift has a mechanism: every TOML key carries a description and an
`x-journal` ref, and `task config:check` fails when either is missing. Prose had
nothing, and in one working session (2026-09-21/22) it drifted in four separate
ways, none of them caught by reading:

- ADR 0094 was written with a Decision saying the camera driver "did not move".
  It moved later the same session, and the sentence stayed false for a day.
- ADR 0068 said Go does not read `sign_discovery.toml`. It does, through
  `signrouter.DiscoveryConfigFor`, and the claim had been false for six days.
- Path-scoped exclusions in `src/go/.golangci.yml` were silently disarmed by
  package moves three times. Each time the only symptom was the lint count.
- `task go:todo` printed a hand-written migration punch list dated 2026-09-04.
  By 2026-09-22 half of its eight items were false and several of its paths no
  longer existed, and it still printed with full confidence.

Separately, the migration plan and `go-future.md` were moved under the ignored
`docs/internal/` trees, while a Taskfile and two `.proto` files still cited the
plan. A fresh clone gets the citation and not the document.

The common shape: a statement about the present, written once, with nothing that
fails when the present changes. Writing more carefully does not fix it; each of
the four was written carefully.

## Options considered

- (a) A review habit: re-read the affected docs on every move.
- (b) Stop writing present-tense status in tracked prose, and mechanically check
      what remains.
- (c) Move everything into the ignored working docs and track only code.

## Decision

(b).

**Ownership.** Tracked ADRs are the normative record, and the only prose a clone
is guaranteed to get. The ignored `docs/internal/` trees are working notes: they
may be stale, and tracked files must not cite them as their rationale. A tracked
file cites an ADR or the code.

**Status is derived, never written.** Anything answering "what is left" or "what
state is this in" computes the answer when asked. `go:todo` now greps the tree
for `TODO`/`FIXME` and unimplemented stubs instead of printing a list. A
checkbox list of open work in a tracked file is a defect of the same kind as a
stale pin.

**The check.** `src/tools/doc_audit.py check` (`task docs:check`, and a step in
the CI `config` job) covers only statements that claim to describe the present:

- `adr-paths`: in an *accepted* ADR, every backticked path under a live root in
  Decision or Consequences must exist. Context and History describe the past and
  are exempt, as are superseded and deprecated ADRs. Its first run found 0094's
  camera sentence.
- `lint-exclusions`: every path-scoped exclusion in `src/go/.golangci.yml` must
  match a tracked file.
- `doc-refs`: a tracked file citing a `docs/**.md` path must point at a file that
  exists and is tracked. The 16 citations that predate the check are frozen in
  `src/tools/doc_audit_baseline.txt`, which may only shrink: a new offender
  fails, and so does a baseline entry that has been fixed but not deleted.

The check was confirmed to fire by planting one violation of each kind, and was
green before it joined CI.

## Consequences

- An ADR's Decision can no longer name a path that later moves without the move
  failing CI. Moving a package now means amending the ADRs that describe it,
  in the same change. That cost is the point.
- The check does not read meaning. ADR 0068's `sign_discovery.toml` claim would
  pass it: the file exists, the sentence about it was false. Claims about
  behaviour are still only as good as the pin tests behind them. This ADR closes
  the path half of the problem, not the semantic half.
- Paths in prose must be backticked to be checked. An unquoted path is invisible
  to `adr-paths`, which is an escape hatch, not a feature.
- The 16 baselined citations are real rot in Python scripts, tests and deploy
  files. They are left for whoever next touches those files, and the baseline
  tells them which.

## History

- 2026-09-22: written with the tool. The number was first reserved as 0096 in
  the migration plan, and 0096 was taken by the `vt` CLI ADR the same day: a
  plan's written reservation had gone stale before the ADR it reserved was
  written.

## Cross-references

- 0068 (the migration whose plan and consequence list drifted)
- 0087 (the four-level test ladder; "landing a red job and normalizing it" is
  why each check had to be green before joining CI)
- 0094 (the ADR whose Decision the first run corrected)
- 0096 (`vt` wraps the Taskfiles; `go:todo` stays a task)
