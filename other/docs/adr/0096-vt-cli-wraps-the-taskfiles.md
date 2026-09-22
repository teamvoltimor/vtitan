# 0096. `vt` wraps the Taskfiles, and migrates to Go only what has real logic

- Status: accepted
- Date: 2026-09-22

## Context

`task --list-all --json` returns 303 tasks across 15 Taskfiles, and the root
flattens most of them into one namespace (`flatten: true` in 10 of the 15
includes). `task --list` is therefore a 303-line wall, and discoverability had
already broken once: `other/tasks/platform.yml` carries a hand-written `help`
task, a multi-screen `echo` of a curated menu. Nothing fails when a task is
added and the menu is not, so it drifts silently.

Three facts shaped the decision:

- **Cobra was already in.** `github.com/spf13/cobra` is a direct dependency of
  `src/go`, 11 of the 13 binaries in `cmd/` use it, and `internal/cmdkit`
  already centralises shared flags. A CLI extends a convention; it does not
  invent one.
- **Task's JSON gives the tree for free, but not the parameters.** It exposes
  `name`, `desc`, `aliases` and `location`, not `vars`, `requires` or defaults.
  Only 30 tasks declare `requires.vars`; the rest encode their parameters in
  free text inside `desc` (`"usage: task windows:ping PING_HOST=address"`).
  Typed flags and their defaults have to be declared by hand, command by
  command. That is the real cost, not the cobra wiring.
- **Most tasks are thin wrappers** over `uv`, `pixi`, `ruff`, `docker`,
  `ansible`, `ssh`, `npx`, often with `platforms: [windows]` /
  `[linux, darwin]` variants of the same command. Rewriting those bodies in Go
  loses the per-platform variants and buys nothing.

## Options considered

- (a) **Thin wrapper.** A cobra tree generated from the JSON, every leaf runs
      `task <name>`, no typed flags.
- (b) **Hybrid.** Wrap now, with typed flags hand-declared for the daily
      flows; later reimplement natively in Go only what has real logic.
- (c) **Full migration.** Rewrite the 303 task bodies in Go.

And, for the first cut:

- (d) Generate the whole tree from the JSON.
- (e) Curate the daily flows by hand; reach the rest through a catch-all.

## Decision

(b) with (e).

`src/go/cmd/vt` is a development-machine tool (the boards are not assumed to
have Go and keep using `task`). Task's `module:verb` naming maps to nested
subcommands (`task go:test:hw` is `vt go test hw`). Each wrapped leaf runs
`task <name> VAR=... -- <args>` as a child process with stdin/stdout/stderr
attached and **propagates the exit code untouched**; the Taskfiles stay the
only source of truth on *how* something runs. Domains may be renamed where the
Task name reads badly (`windows:*` is `vt fleet ...`), which is why the curated
layer is a declarative table in `internal/vtcli/spec.go` and not a generator.
Every other task is reachable through `vt run <task> [VAR=value ...]`,
validated against the live inventory.

(c) lost on cost and on the `platforms:` variants. (a) lost because it adds
nothing `task --list` does not already give, and (d) for the same reason: the
value is in the typed flags, and those cannot be generated.

A task becomes a candidate for native Go (a later phase) only when all three
hold: it has real control logic (loops, host selection, retries), it runs from
the development machine and not on a Pi, and it does not depend on Task's
`platforms:` variants to exist. The candidates that pass today are the chained
SSH hops in `other/tasks/fleet.yml`, the `SSH_HOST | default` host selection
repeated across ~15 tasks, and the ping retries.

Choices made along the way:

- **Anti-drift is a build failure, not a habit.** Three tests in
  `spec_test.go`, run against the real `task --list-all --json`: every task the
  spec names exists; every task in a curated domain is either in the spec or
  in an explicit, justified exclusion list (the test injects a fake task to
  prove it bites); and the catch-all accepts all tasks and rejects an invented
  one. Same pattern as `task config:check` for `x-journal` references.
- **Declared defaults are shown, not pinned.** Task does not expose defaults,
  so the spec copies them for `--help` and to prefill the form, but only values
  the user changes are forwarded. A Task default that moves leaves stale help
  text, never a wrong invocation.
- **Presentation only in lipgloss.** `NO_COLOR` and a non-TTY stdout give plain
  text, so CI and pipes are clean.
- **An interactive picker, reversing the plan.** The plan excluded a TUI. On a
  TTY, `vt` with no arguments opens a bubbletea drill-down picker (domains, then
  their commands) and a form per leaf with its args and flags. The form ends
  on a confirmation screen showing the exact `task` invocation, with a warning
  for commands marked `Heavy` (those that launch a simulator or a GUI). The
  confirmation was added after a filtered selection in a test session launched
  `sim test` unintentionally. Without a TTY, or with `VT_NO_PICKER=1`, `vt`
  prints the static menu instead.
- **All user-facing text is in English**, matching the Taskfiles.

Fixed versions: cobra v1.10.2, lipgloss v1.1.0, bubbletea v1.3.10,
bubbles v0.21.0.

## Consequences

- Two paths coexist on purpose: `task X` keeps working unchanged, and CI keeps
  calling `task`. `vt` adds no behaviour a Taskfile does not already define.
- The spec is a second copy of task names and defaults. The three tests are the
  price of entry for that copy; defaults are the one part they cannot check,
  hence the don't-pin rule.
- Each curated domain costs hand-written flags and an exclusion list. At the
  first cut the spec had 20 wrapped commands and two curated domains (`go`,
  `fleet`); `sim` joined as the third (see History).
- `vt` has to be built (`task cli:build`, to `src/go/bin/vt`) or run with
  `task cli:run -- <args>`. `go:build:static` cross-compiles it along with the
  rest of `./cmd/...`, which is harmless.
- The hand-written `help` task in `other/tasks/platform.yml` is retired: `vt
  run` lists the full inventory and the picker filters it. There is one menu,
  and it is generated.

## History

- `59a7caab` 2026-09-22: phase 0. `cmd/vt`, the spec, the catch-all, the three
  anti-drift tests, `cli:build`/`cli:test`/`cli:lint`/`cli:run`, the lipgloss
  menu, the bubbletea picker with form and confirmation, completions.
- `4c9a6349` 2026-09-22: drill-down submenus in the picker, replacing the flat
  list.
- `c19dafd2` 2026-09-22: all user-facing text in English.
- `98cbf8c2` 2026-09-22: Task defaults surfaced in help and the form, without
  pinning them.
- `495970df` 2026-09-22: `sim` curated, all 14 `sim:*` tasks decided (13
  leaves, one exclusion: the `sim:rviz:navigate:visualize:all` alias). The
  spec is now 29 commands across three curated domains.
- `14f36931` 2026-09-22: `vt run` with no task lists the inventory, and the
  picker's escape hatch browses it instead of asking for a typed name. The
  confirmation screen showed flag names (`host=`) where Task receives the
  variable (`HOST=`); it now renders what is forwarded.
- `5b02bf79` 2026-09-22: `sim test` and `sim navigate` lose `Heavy`. Both are
  headless pytest runs; the first version of this ADR said `sim test` starts
  Gazebo and RViz, which was wrong.
- `docs(cli): close phase 2` 2026-09-22: phase 2 closed. The `help` task is deleted and the
  README documents `vt`, with `task cli:build` as the recommended path and
  `task cli:run` as the no-build alternative.

## Cross-references

- 0094: `vt` lives in `internal/vtcli`, not `pkg/`; it is not public surface.
- The working plan is `other/docs/development/plan-cli-unificada.md`; it holds
  the phases and is deleted once it no longer describes pending work.
