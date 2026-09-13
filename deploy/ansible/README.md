# Ansible provisioning

Idempotent, re-runnable Ansible roles, ported from the old `setup_pi_5.sh`/
`setup_pi_zero.sh` scp+ssh scripts (removed 2026-07-30). Runs from the
**dev Pi 5**, not from Windows - Ansible's control node needs a POSIX shell.

## Layout

- `inventory/hosts.yml` - `robot_pi5` / `robot_pi_zero` groups. `ansible_host`
  defaults mirror `Taskfile.yml`'s `RPI_LOCAL_IP`/`ZERO_WIFI_IP`; override with
  `-e ansible_host=<ip>` per run.
- `inventory/group_vars/all.yml` - shared vars (repo, paths, USB gadget subnet, WiFi
  band-lock BSSID). Mirrors the equivalent `Taskfile.yml` vars.
- `roles/common/` - GitHub auth, apt upgrade, hardware groups, pixi install,
  private repo clone, `.env`, udev rules. Shared by both boards.
- `roles/pi5/` - raspi-config interfaces, Hailo runtime (`apt install
  hailo-all`), `usb0` host static IP, `vision` pixi env, LiDAR driver,
  `vtitan-pi5`/`vtitan-lidar` systemd units.
- `roles/pi_zero/` - `config.txt`/`cmdline.txt` overlays, USB gadget MACs,
  `usb0` client static IP, swap bump, `dev` pixi env, `vtitan-pi-zero`
  systemd unit.
- `playbooks/provision_pi5.yml`, `playbooks/provision_pi_zero.yml`, `site.yml`
  (both, Pi 5 first).

## Usage

Prefer the `task rpi:provision:*` / `task rpi:ansible:*` wrappers in the root
`Taskfile.yml` - they set `GH_TOKEN` and pick the right playbook. Direct
invocation:

```bash
ansible-galaxy collection install -r requirements.yml   # one-time
GH_TOKEN=$(gh auth token) ansible-playbook playbooks/provision_pi5.yml -e pi5_ip=192.168.0.50 --ask-become-pass
GH_TOKEN=$(gh auth token) ansible-playbook playbooks/provision_pi_zero.yml --check --diff --ask-become-pass  # dry run
```

Use `-e pi5_ip=<ip>` / `-e zero_ip=<ip>` to override a target's address - NOT
`-e ansible_host=<ip>`, which is a global extra-var and would silently
re-target both hosts at once (breaking the Zero playbook's `delegate_to: pi5`
tasks: WiFi band lock, SSH-trust bootstrap).

## Notes

- systemd units under `src/systemd/*.service` are the single
  source of truth - Ansible substitutes their `__TARGET_USER__`/
  `__TARGET_HOME__` placeholders the same way the bash scripts' `sed` does,
  rather than duplicating them as `.j2` templates.
- The Pi Zero play locks the Pi 5's WiFi to its 2.4GHz BSSID for
  first-contact reachability, wrapped in `block`/`always` so the lock is
  released even if provisioning fails partway (a real gap in the bash
  version, which leaves it locked on a mid-script failure).
- `--check --diff` (`task rpi:ansible:check`) is a real dry run against an
  already-provisioned Pi - expect no changes if nothing has drifted.

## Carrying the calibrated `.env` across the restructure

The checkout lives at `~/vtitan` on both boards, and the `common` role brings it
to `repo_ref` in place: `gh repo clone` is `creates:`-guarded, so on a board that
already has one it does nothing and the `fetch` / `checkout` / `merge --ff-only`
below it do the work.

The one thing a checkout cannot carry itself is the real gitignored `.env`
(steering offsets, LiDAR yaw, Hailo model path). The 2026-09-10 restructure
dissolved `platform/` into `src/`, which moves `robot_dir` out from under it and
strands it at `platform/robot/.env` (`legacy_env_path` in
`inventory/group_vars/all.yml`). The role copies it across before the
`.env.example` fallback could stomp it.

### The removal this replaced, and why it was dangerous

Until 2026-09-13 the same block also removed an `old_repo_dir` once the `.env`
copy was confirmed, written for the 2026-07 `teamvoldemor -> teamvoltimor`
rename. **That rename renamed the GitHub org, not the checkout directory**, so
`old_repo_dir` was defined as `/home/{{ ansible_user }}/vtitan` - byte-identical
to `repo_dir`. Its guard was `old_repo_dir_stat.stat.exists and
final_env_stat.stat.exists`, with no comparison against `repo_dir`, so on any
board that already had a checkout and a landed `.env` it would have deleted the
live repo, including the calibrated `.env` just copied into it. Verified on
2026-09-13 that **both** Pis satisfied that guard. The removal is gone; only the
`.env` migration it wrapped survives.

The Pi 5 also had 5 sibling `git worktree` checkouts of other branches
(`vtitan-auto-annotator`, `-docs`, `-hailo`, `-hugo-docs`, `-platform`)
plus a stale manual `vtitan-session-backup` dir from an older rename.
All confirmed clean the same way - the `pi5` role removes them
(`old_worktree_dirs` in `inventory/group_vars/robot_pi5.yml`), leaving a single
`~/vtitan` checkout. Every entry there is a SIBLING of `~/vtitan`, never
`~/vtitan` itself. `deploy/ansible/` and `scripts/`
both live on `master` now, so nothing else needs that separate worktree -
`ANSIBLE_DIR`/`PLATFORM_SCRIPTS` in the root `Taskfile.yml` point straight
at `~/vtitan`.

The old `vtitan-*.service` units are **not** touched by provisioning -
they keep running side by side with the new `vtitan-*.service` ones until
you explicitly run `task rpi:cleanup:old-services` after confirming the new
services work. Don't skip this step: two services fighting over the same
GPIO/PWM/serial hardware is a real failure mode, not a theoretical one.
