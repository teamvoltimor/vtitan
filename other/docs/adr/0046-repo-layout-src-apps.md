# 0046. The repository layout moves code into src/ and apps/

- Status: accepted
- Date: 2026-09-10
- Commit: 39d8e679

## Context

The monorepo lived under `platform/`, with `src/` and `other/` as pointer
folders holding a README that redirected there. Someone opening `src/` expecting
code (a judge, a new collaborator) found a redirect instead. `platform/robot`
also had a live path dependency on `platform/shared` (`pixi.toml`), so it could
not move alone without breaking its pixi environment.

## Options considered

- (a) Keep `platform/` and the pointer folders.
- (b) Physically move the code to the names the WRO folder requirement expects.

## Decision

(b). Scope was expanded mid-execution from `robot`+`shared` to all of
`platform/`:

- `platform/robot` to `src/`, then split into `src/python/` and `src/go/`.
- `platform/robot-go`, `backend`, `frontend`, `gazebo` to `apps/*`.
- `platform/proto`, `openapi` to `contracts/*`.
- `platform/Taskfile.yml` to `tasks/platform.yml`, keeping the documented
  `platform:*` namespace so existing commands still work.
- systemd units, Ansible inventory, CI working-directories and ~500 path
  references updated in the same change.

## Consequences

- `src/` and `other/` are real directories rather than pointers.
- The `platform:*` task namespace no longer matches the folder layout; it was
  kept deliberately for command compatibility.
- The move was validated by lint/build, not by a real hardware deploy: the Pi 5
  systemd units were not exercised in the session that landed it.
