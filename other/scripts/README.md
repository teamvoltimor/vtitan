# Dev-machine helpers

Two Windows PowerShell helpers used by the root `Taskfile.yml`. This folder
used to hold the Pi provisioning scripts (`setup_common.sh` / `setup_pi_5.sh` /
`setup_pi_zero.sh`); those were ported to Ansible on 2026-07-30 and live under
`../deploy/ansible/` now. Nothing here flashes, partitions or provisions a Pi.

| file | run by | what it does |
|---|---|---|
| `platform-info.ps1` | `task info` | prints toolchain versions (`uv`, `pixi`, `go`, `buf`) and the main output directories |
| `setup-ssh-config.ps1` | `task windows:ssh:setup-config` | appends the `rpi-*` host blocks to `~/.ssh/config` |

## `platform-info.ps1`

```
task info
```

Takes the output directory as `$args[0]` (the Taskfile passes `OUTPUT_DIR`).
Purely informational; it does not change anything.

## `setup-ssh-config.ps1`

```
task windows:ssh:setup-config SSH_USER=... SSH_KEY_PATH=... [SSH_DOMAIN=...] [SSH_ZERO_HOSTNAME=...]
```

Writes host aliases for the dev Pi 5 (over the direct Ethernet link and over
the LAN), the Pi Zero, and the public remote host. You can then reach them as
`ssh rpi-5-local`, `ssh rpi-5-direct`, `ssh rpi-zero-local`, and so on. Every value is a parameter with a
placeholder default (`YOUR_USERNAME`, `YOUR_KEY_FILE`, ...), so the script is
safe to read as a template: it refuses nothing, but an unset placeholder
produces an unusable host block. Re-running appends a fresh block; it does not
deduplicate an existing one, so trim `~/.ssh/config` by hand if you run it more
than once.

## Provisioning a Pi

Not here. See `../deploy/ansible/README.md` and the `rpi:provision:*` tasks.
The diagnostics that run on the robot or over a recorded bag are a separate
tree with their own README: `../src/python/scripts/README.md`.
