# Raspberry Pi setup

Two phases. **Raspberry Pi Imager** writes a bootable, reachable OS; Phase 2
adds the vtitan layer (interfaces, USB-gadget link, ROS2 workspace, systemd
services) on top. Neither phase 2 mechanism flashes or partitions anything.

**Phase 2 is now driven by Ansible** (`../ansible/`, via `task rpi:provision:pi5`
/ `task rpi:provision:zero`) — idempotent, re-runnable, dry-runnable
(`task rpi:ansible:check`). The `setup_pi_*.sh` scripts below are what the
Ansible roles were ported from; they still work standalone and are documented
here as the manual fallback (e.g. no Ansible on the dev Pi 5 yet — see
`task rpi:ansible:setup`).

## Phase 1 — Raspberry Pi Imager (Windows)

Use **Raspberry Pi OS Lite (64-bit)** for *both* boards. The Pi Zero 2W is
64-bit capable, and pixi/conda-forge has **no 32-bit ARM packages** — a 32-bit
image cannot build the robot stack.

In Imager's OS customization (the gear / "Edit settings"):

- **Hostname** — e.g. `ralvarezdev-raspberrypi-zero` / `...-pi5`
- **Enable SSH** → "Allow public-key authentication" → paste your public key
  (e.g. `~/.ssh/id_vtitan_pi_zero.pub`)
- **Username** — any name (the scripts derive it from `$SUDO_USER` and the
  systemd units get it substituted in at install time — it doesn't have to be
  `pi`)
- **Password** — set one
- **WiFi** — SSID, password, and country (first contact is over WiFi)
- **Locale** — timezone / keyboard

Write the card, boot the Pi, and let it join WiFi.

## Phase 2 — provision over SSH

### Ansible (recommended)

From the dev Pi 5 (one-time: `task rpi:ansible:setup`):

```bash
task rpi:provision:pi5 PI5_IP=x.x.x.x
task rpi:provision:zero               # defaults to ZERO_WIFI_IP
task rpi:ansible:check TARGET=pi5     # dry-run + diff against an already-provisioned Pi
```

GitHub auth (private repo) and the Pi Zero's WiFi 2.4GHz band lock are handled
the same way as the manual scripts below, just automatically. See
`../ansible/README.md` for the role/playbook layout.

### Manual fallback (`setup_pi_*.sh`)

Find the Pi on WiFi (`ssh <user>@<hostname>.local` or its `192.168.x.y` lease)
and copy the scripts over. The vtitan repo is **private**, so the script
clones it via `gh` and needs GitHub auth for that user. Two ways:

**A. Token (one pass, non-interactive):**
```bash
scp -r scripts <user>@<ip>:/tmp/
ssh <user>@<ip> 'sudo GH_TOKEN=ghp_xxx bash /tmp/scripts/setup_pi_5.sh --hailo-deb /path/hailort.deb'
```

**B. Interactive `gh auth login` (two passes):** run the script once — it
installs `gh`, then stops at the auth check. Log in as that user, then re-run:
```bash
scp -r scripts <user>@<ip>:/tmp/
ssh <user>@<ip> 'sudo bash /tmp/scripts/setup_pi_5.sh'   # installs gh, then stops
ssh -t <user>@<ip> 'gh auth login'                       # authenticate (one time)
ssh <user>@<ip> 'sudo bash /tmp/scripts/setup_pi_5.sh'   # now proceeds to completion
```

Use `setup_pi_zero.sh` for the Pi Zero 2W (USB gadget @ 192.168.250.1) and
`setup_pi_5.sh` for the Pi 5 (usb0 host @ 192.168.250.2). The auth check runs
*before* any heavy work, so a missing login fails fast.

Each script updates the OS, enables hardware interfaces, clones the repo, builds
the ROS2 workspace with pixi, installs + enables the systemd service(s), and
reboots. Provisioning is heavy (especially the Zero) — run it attended.

## Networking

The Pi Zero ↔ Pi 5 link is a USB Ethernet gadget on **192.168.250.0/24**
(Zero = `.1`, Pi 5 = `.2`), owned entirely by NetworkManager — no netplan, no
manual `ip` services, no ICS switcher. WiFi (a separate subnet) carries internet
and first-contact SSH. After provisioning, ROS2 DDS auto-discovers over `usb0`.
