# Ansible provisioning

Ports `../scripts/setup_pi_5.sh` and `../scripts/setup_pi_zero.sh` into
idempotent, re-runnable Ansible roles. Runs from the **dev Pi 5** (same place
the bash scripts run today), not from Windows — Ansible's control node needs
a POSIX shell.

## Layout

- `inventory/hosts.yml` — `robot_pi5` / `robot_pi_zero` groups. `ansible_host`
  defaults mirror `Taskfile.yml`'s `RPI_LOCAL_IP`/`ZERO_WIFI_IP`; override with
  `-e ansible_host=<ip>` per run.
- `group_vars/all.yml` — shared vars (repo, paths, USB gadget subnet, WiFi
  band-lock BSSID). Mirrors the equivalent `Taskfile.yml` vars.
- `roles/common/` — GitHub auth, apt upgrade, hardware groups, pixi install,
  private repo clone, `.env`, udev rules. Shared by both boards.
- `roles/pi5/` — raspi-config interfaces, Hailo runtime (`apt install
  hailo-all`), `usb0` host static IP, `vision` pixi env, LiDAR driver,
  `vtitan-pi5`/`vtitan-lidar` systemd units.
- `roles/pi_zero/` — `config.txt`/`cmdline.txt` overlays, USB gadget MACs,
  `usb0` client static IP, swap bump, `dev` pixi env, `vtitan-pi-zero`
  systemd unit.
- `playbooks/provision_pi5.yml`, `playbooks/provision_pi_zero.yml`, `site.yml`
  (both, Pi 5 first).

## Usage

Prefer the `task rpi:provision:*` / `task rpi:ansible:*` wrappers in the root
`Taskfile.yml` — they set `GH_TOKEN` and pick the right playbook. Direct
invocation:

```bash
ansible-galaxy collection install -r requirements.yml   # one-time
GH_TOKEN=$(gh auth token) ansible-playbook playbooks/provision_pi5.yml -e pi5_ip=192.168.0.50 --ask-become-pass
GH_TOKEN=$(gh auth token) ansible-playbook playbooks/provision_pi_zero.yml --check --diff --ask-become-pass  # dry run
```

Use `-e pi5_ip=<ip>` / `-e zero_ip=<ip>` to override a target's address — NOT
`-e ansible_host=<ip>`, which is a global extra-var and would silently
re-target both hosts at once (breaking the Zero playbook's `delegate_to: pi5`
tasks: WiFi band lock, SSH-trust bootstrap).

## Notes

- systemd units under `platform/robot/systemd/*.service` are the single
  source of truth — Ansible substitutes their `__TARGET_USER__`/
  `__TARGET_HOME__` placeholders the same way the bash scripts' `sed` does,
  rather than duplicating them as `.j2` templates.
- The Pi Zero play locks the Pi 5's WiFi to its 2.4GHz BSSID for
  first-contact reachability, wrapped in `block`/`always` so the lock is
  released even if provisioning fails partway (a real gap in the bash
  version, which leaves it locked on a mid-script failure).
- `--check --diff` (`task rpi:ansible:check`) is a real dry run against an
  already-provisioned Pi — expect no changes if nothing has drifted.

## Pre-rename checkout migration

Real hardware as of 2026-07-29 still has the pre-rename checkout(s)
(`~/voldemorbot`, remote `teamvoldemor/voldemorbot`, systemd units named
`voldemorbot-pi5.service`/`voldemorbot-lidar.service`/`voldemorbot-pi-zero.service`)
— the org/repo rename to `teamvoltimor/vtitan` never landed on the robots.

The `common` role always does a **fresh `gh repo clone`** into `~/vtitan`
(`old_repo_dir` in `group_vars/all.yml` names the old location), regardless
of whether that old location is a git checkout (Pi 5) or a tarball-deployed
directory with no git history at all (Pi Zero, via
`scripts/deploy-dev-env-to-zero.sh`). The only thing carried over from the
old deployment is the real gitignored `.env` (steering offsets, LiDAR yaw,
Hailo model path) — a fresh clone can't reproduce that, so it's copied
across explicitly before the `.env.example` fallback could otherwise stomp
it. Once that copy is confirmed, the old deployment directory is removed.

This used to be an in-place `mv` + `git remote set-url` + `git pull
--rebase` for git checkouts, to preserve reflog/stash history — changed to a
fresh clone on 2026-07-30 after confirming (`git log --branches --not
--remotes`, empty on every branch) that nothing on the Pi 5's checkouts was
unpushed, so there was no history worth the extra complexity of preserving.

The Pi 5 also had 5 sibling `git worktree` checkouts of other branches
(`voldemorbot-auto-annotator`, `-docs`, `-hailo`, `-hugo-docs`, `-platform`)
plus a stale manual `voldemorbot-session-backup` dir from an older rename.
All confirmed clean the same way — the `pi5` role removes them
(`old_worktree_dirs` in `group_vars/robot_pi5.yml`) once the main migration
succeeds, leaving a single `~/vtitan` checkout. `ansible/` and `scripts/`
both live on `master` now, so nothing else needs that separate worktree —
`ANSIBLE_DIR`/`PLATFORM_SCRIPTS` in the root `Taskfile.yml` point straight
at `~/vtitan`.

The old `voldemorbot-*.service` units are **not** touched by provisioning —
they keep running side by side with the new `vtitan-*.service` ones until
you explicitly run `task rpi:cleanup:old-services` after confirming the new
services work. Don't skip this step: two services fighting over the same
GPIO/PWM/serial hardware is a real failure mode, not a theoretical one.
