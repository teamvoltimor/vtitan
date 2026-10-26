# vtitan — agent guide

Team Voltimor's robot for WRO 2026, Future Engineers: the ROS 2/Python stack that
runs on the Pi 5, a Go reimplementation and simulator of the same navigation, one
TOML config tree both read, and the tooling around them (telemetry, Gazebo,
auto-annotator, MCP contracts). The root mirrors the folder layout the WRO
submission requires. The engineering prose in `README.md` is Spanish; code,
identifiers and commit messages are English.

## Commands (repo root; `task --list` shows the rest)

- Setup: `mise install`, then `task install`; one-time `task init:dev` (install + lint)
- Test: `task test` (Python + Go), `task robot:test SCOPE=unit|navigation|hardware|all`,
  `task sim:test`, `task go:test`
- Lint: `task lint` (ruff, golangci-lint, ESLint, buf), `task lint:fix`
- Config: `task config:check`, `task config:verify` (generated artifacts stale?)
- Docs: `task docs:check` (prose points at real paths), `task docs:adr-refs`
- Simulate: `task gen:corpus:all` (pinned seed), then
  `task sim:navigate:visualize:all -- --challenge open --interactive`
- Robot: `task robot:deploy` (code + detector HEF, rebuild colcon, restart),
  `task rpi:stack ACTION=status`, `task rpi:health`, `task robot:calibrate-encoder`
- `vt` CLI over the same tasks: `task cli:build` → `src/go/bin/vt`, or
  `task cli:run -- sim --help`

## Layout

```
src/python/   competition stack: src/ (real logic), ros2_ws/src/ (thin node wrappers),
              shared/ (pydantic config loaders), tests/{unit,ros2,hardware}
src/go/       Go port, sim runner, Pico firmware, systemd units, the vt CLI
src/config/   the TOML the robot reads (robot.toml, track.toml, hardware/, navigation/,
              profiles/) — the single source for both stacks
src/model/    JSON Schemas validating that TOML
other/        apps/ (telemetry backend + frontend, gazebo, auto-annotator, hugo-docs),
              contracts/ (proto, openapi), ml/ (hailo training, tracked weights),
              deploy/ansible, docs/, scripts/, tasks/ (Taskfile includes), data/ (run output)
schemes/ models/ v-photos/ t-photos/ video/   WRO deliverables, CAD and photos
```

## Conventions

- The root `Taskfile.yml` flattens ~16 includes; a new task belongs in the owning
  Taskfile (`other/tasks/*.yml`, `src/*/Taskfile.yml`, `other/apps/*/Taskfile.yml`),
  not at the root. Vars every include needs (`BASH`, `PROFILE`, `ROBOT_PIXI`, board
  addresses) are declared once at the root because includes do not share vars.
- `src/python/src/` is the implementation; `ros2_ws/src/` holds only wrappers that
  adapt it to topics/params/lifecycle. New logic goes in `src/` so it stays
  testable without ROS 2.
- `src/config/*.toml` is the source both stacks read; the `PROFILE` var must name
  one servo and one motor profile because the base `robot.toml` declares neither.
- Model weights are versioned, not dropped in: `task models:promote NAME=… HEF=…`
  creates `other/ml/weights/<name>/vN/`, `task models:deploy` copies one into the
  annotator.
- Conventional Commits, GPG-signed. `!` in the type (`perf(nav)!:`) means the
  change breaks comparability with earlier measurements, not just an interface.
- Every measured number in the docs must trace to code or a run; never estimate
  one, and never compare across such a `!` commit.

## Boundaries

- Always: `task test` and `task lint` before done; add `task config:verify` and
  `task docs:check` when config or prose changes.
- Ask first: `task robot:deploy`; any `task rpi:*` that reboots, reprovisions or
  shuts down a board; bumping `mise.toml` pins; tagging a release.
- Never: commit `other/data/` runtime output (bags, videos, photos) or
  `training_data/`; run hardware tasks (`robot:test-motors`,
  `robot:calibrate-encoder`) anywhere but the robot on the bench; put a new board
  address outside the root Taskfile vars.
