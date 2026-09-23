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
subcommands (`task go:hw:build` is `vt go hw build`). Each wrapped leaf runs
`task <name> VAR=... -- <args>` as a child process with stdin/stdout/stderr
attached and **propagates the exit code untouched**; the Taskfiles stay the
only source of truth on *how* something runs. Domains may be renamed where the
Task name reads badly (`windows:*` is `vt fleet ...`), which is why the curated
layer is a declarative table in `internal/vtcli/spec.go` and not a generator.
Every other task is reachable through `vt task <name> [VAR=value ...]`
(`vt run` was its first name and stays an alias), validated against the live
inventory.

(c) lost on cost and on the `platforms:` variants. (a) lost because it adds
nothing `task --list` does not already give, and (d) for the same reason: the
value is in the typed flags, and those cannot be generated.

A task becomes a candidate for native Go (a later phase) only when all three
hold: it has real control logic (loops, host selection, retries), it runs from
the development machine and not on a Pi, and it does not depend on Task's
`platforms:` variants to exist. The three candidates first named for it (the
chained SSH hops, the repeated `SSH_HOST | default`, the ping retries) all
failed the first test once measured; see History. No task is native today.

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
- **The picker is a session, not a launcher.** Picker, form and run pane are
  one bubbletea program. A confirmed command runs on a pseudo-terminal
  (`charmbracelet/x/xpty`: a Unix PTY, ConPTY on Windows) and its output
  streams into a scrollable pane under the equivalent `vt` line and a live
  status. After it ends, enter goes back to the picker level it was picked
  from, `r` runs it again and `q` quits. A PTY and not pipes because the
  tools only behave as in a terminal when they see one: colors, line-by-line
  flushing from Python, progress bars that redraw in place. While the task
  runs, keys are typed into it and ctrl+c interrupts it (a second one kills
  its process group) instead of ending vt. A command marked `Terminal` (an
  SSH shell, `cli run`) gets the whole terminal instead and the session
  resumes when it exits. The exit code of each run is shown in the pane and
  printed after the session ends, one line per run; the session itself exits
  0. The typed CLI (`vt <command>`) is unchanged: terminal attached, exit code
  propagated.
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
  `fleet`); by the end of phase 1 it has 65 across `go`, `fleet`, `sim`,
  `robot`, `rpi` and the umbrella verbs (see History).
- Tasks that run on a board are excluded from the typed tree rather than
  wrapped, with that reason recorded: `vt` is not on the boards, so a typed
  command for them would only ever fail. That is all of `rpi:*` but
  `rpi:migrate-data`, and seven `robot:*` tasks.
- `vt` has to be built (`task cli:build`, to `src/go/bin/vt`) or run with
  `task cli:run -- <args>`. `go:build:static` cross-compiles it along with the
  rest of `./cmd/...`, which is harmless.
- The hand-written `help` task in `other/tasks/platform.yml` is retired: `vt
  task` lists the full inventory and the picker filters it. There is one menu,
  and it is generated.
- vt depends on a Task rule nobody had written down: a global var of an
  INCLUDED Taskfile beats a CLI `VAR=x` unless it is written as its own
  default (`'{{.VAR | default "x"}}'`); the root Taskfile's globals do yield.
  Before this was found, `--profile` did nothing on every run, and
  `task gen:corpus CHALLENGE=obstacles` regenerated the open corpus. Anti-drift
  check 4 now fails on any var vt forwards, or any `NAME=` a task description
  advertises, that is a plain global in an included file.
- Commands limited to some OSes are hidden elsewhere, because Task skips
  their commands and exits 0, which reads as success. Check 5 derives each
  task's platforms from the Taskfile YAML (following calls and deps) and fails
  if the spec disagrees.

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
- `a28a0cf6` 2026-09-22: phase 2 closed. The `help` task is deleted and the
  README documents `vt`, with `task cli:build` as the recommended path and
  `task cli:run` as the no-build alternative.
- `c0a5710e` 2026-09-22: phase 1 closed. `robot` (19 dev-side leaves, 7
  on-board exclusions), `rpi` (all 28 on-board, `rpi:migrate-data` moved to
  `vt fleet migrate-data`), the eight deferred `fleet` tasks, and the
  umbrellas; `ownerDomain` also owns the bare verb (`lint` for `lint:`).
- `ee9a9092` 2026-09-22: phase 3 closed without migrating anything. The ping
  retries were two variables nothing read (deleted). The host selection was 9
  fixed per-task defaults, not ~15 and not a choice. The nested SSH hops had a
  real bug, but a quoting one: a quote in the WiFi SSID or password broke the
  command and `$` was expanded on the Pi 5, setting a wrong password silently.
  Task's `shellQuote` fixes it in the Taskfile, for `task` and CI as well,
  which a Go rewrite would not have. Phase 4 therefore does not apply.
- `467c5edb`, `f4f4aecb` 2026-09-22: the Task var rule above. PROFILE,
  OUTPUT_DIR, CHALLENGE, SCENARIOS, METADATA, CORPUS_SIZE and CORPUS_SEED are
  written as their own default; measured with `--dry`, the overrides went from
  ignored 12 of 12 to honoured 6 of 6. `robot:deploy` exports HEF only when it
  was passed, so `HEF=` (code only) finally reaches the script.
- `9b96fee2` 2026-09-22: usability pass. One resolver (`planInvocation`)
  behind the CLI, the form preview, `--dry-run` and the echoed `vt` line;
  variant switches (`lint --fix`, `clean --cache|--all`, `gen corpus
  --both`...) and task-picking arguments (`fleet set-wifi|audit
  <pi5|zero>`) replace one-child subcommands; secret flags; platform hiding
  with check 5; root groups instead of alphabetical order; the catch-all
  renamed `task`; `fleet setup`, `sim view`, `setup`, `go hw`, `robot vision`
  and a curated `gen` domain; the last five picks at the top of the picker,
  and the equivalent `vt` command printed after a form run.

- 2026-09-22, this session: the curated surface grew from 64 to 161 commands
  and the boundary rule was made explicit. The domains `frontend`, `backend`,
  `config`, `workflow`, `cli`, `annotator`, `hailo`, `simgen`, `docs`,
  `shared`, `openapi`, `proto`, `docker` and `models` joined `sim`, `robot`,
  `go`, `fleet` and `gen`. The two specialist apps (`auto-annotator`, `hailo`)
  expose only their daily verbs; their container, contract and pipeline trees
  are excluded by name with a reason, because they are long Docker-bound
  workflows that `vt task` already reaches. Two domains were reallocated
  rather than invented: `record:*` moved under `gen record` (it generates the
  training videos) and `vpn`/`cloudflare` under `fleet` (they are board and
  network ops, and they live in `other/tasks/infra.yml`).
  `BASH` and `EXE` moved to the root Taskfile, which made `BASH=...` override
  work for the first time: it was defined in `platform.yml` and in
  `src/go/Taskfile.yml`, and used but never defined in `src/python`, where it
  only resolved because `platform.yml` is flattened first. `go:test:hw`
  became `go:hw:build`, since it cross-compiles with `-c -o` and never runs
  anything. Check 7 (`TestNoTaskNameCollisions`) was added: Task's namespace
  is flat, so two includes defining `lint` collide and include order decides
  the winner -- invisible today only because the four colliding files happen
  to be included without `flatten: true`.
- 2026-09-22, navigation pass: the root is grouped by use (robot and
  simulation, apps, contracts and models, across the repo, tooling) and the
  picker now follows that order; it used to follow spec concatenation order,
  so `--help` and the picker disagreed. Every domain orders its commands the
  same way: run it, then the quality gates, then install and clean.
  Single-task namespaces were flattened (`hailo test`, `proto <action>`),
  sibling cleans and dev modes became switches (`hailo clean --calib`,
  `annotator clean --deep`, `annotator dev --install`), and backend's 14 flat
  rows became 12 with `backend gen` and `backend mod`. Namespace descriptions
  are keyed by full path, since `build` under `go` and under `backend` mean
  different things; the old per-segment table had left eight namespaces
  blank. Checks 8 and 9 (`TestNamespacesDescribed`, `TestRootGroups`) fail on
  a namespace without a description, a stale one, or a first-level command
  outside the root groups.
- 2026-09-22, the picker stays open: runs show inline in a run pane on a
  PTY and return to the menu, instead of ending vt; see the session bullet
  under Decision. The output buffer (`termbuf.go`) is deliberately not a
  terminal emulator: scrollback plus the redraws tools use in place (carriage
  return, erase line, cursor up, column), colors per cell, everything else
  dropped.
- 2026-09-22, Taskfile clean-up: the last globals shared by more than one
  include moved to the root Taskfile (PROFILE, ROBOT_PIXI, CHALLENGE,
  OUTPUT_DIR, METADATA, and the board IPs RPI_LOCAL_IP/ZERO_WIFI_IP/
  RPI_ZERO_GADGET_IP), so no global is defined twice. OUTPUT_DIR and METADATA
  are anchored with `printf "%s/..." .ROOT_DIR`: the includes run from
  different working dirs (gazebo's record:* from other/apps/gazebo/runtime),
  and the old relative defaults resolved differently in each, which had
  silently pointed `sim:analyze` and `record:*` at a training_data dir that did
  not exist. The gazebo Taskfile moved into runtime/, beside the pyproject its
  tasks run against, so its include's `taskfile:` and `dir:` name one
  directory. `simgen:install`, which only called `simgen:build`, is deleted.
  Check 7 now also requires every raw task key defined by more than one include
  to be listed in `namespacedDuplicateTasks` with a reason, so a new cross-file
  duplicate is a decision rather than something include order hides.

- The boundary rule, now written down: **Taskfiles own what runs, `vt` owns
  how it is spelled.** A change that alters what a task does belongs in the
  Taskfile, so `task` and CI get it too; a change to discovery, flags or
  confirmation belongs in `vt`. Phase 3 is why this is a rule and not a
  preference: the WiFi quoting bug was fixed once, in the Taskfile, and
  `task`, CI and `vt` all got the fix.

## Cross-references

- 0094: `vt` lives in `internal/vtcli`, not `pkg/`; it is not public surface.
- The working plan that held the phases has been deleted, now that all of them
  have closed and it no longer described pending work.
