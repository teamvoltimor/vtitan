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
