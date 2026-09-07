# Raspberry Pi setup

Two phases. **Raspberry Pi Imager** writes a bootable, reachable OS; Phase 2
adds the vtitan layer (interfaces, USB-gadget link, ROS2 workspace, systemd
services) on top. Neither phase 2 mechanism flashes or partitions anything.

**Phase 2 is driven by Ansible** (`../ansible/`, via `task rpi:provision:pi5`
/ `task rpi:provision:zero`) - idempotent, re-runnable, dry-runnable
(`task rpi:ansible:check`). The old `setup_common.sh`/`setup_pi_5.sh`/
`setup_pi_zero.sh` scp+ssh scripts the Ansible roles were ported from have
been removed (2026-07-30) - Ansible is installed and reachable on the dev
Pi 5, though the playbook itself is still pending its first real run (see
`../ansible/README.md`).

## Phase 1 - Raspberry Pi Imager (Windows)

Use **Raspberry Pi OS Lite (64-bit)** for *both* boards. The Pi Zero 2W is
64-bit capable, and pixi/conda-forge has **no 32-bit ARM packages** - a 32-bit
image cannot build the robot stack.

In Imager's OS customization (the gear / "Edit settings"):

- **Hostname** - e.g. `ralvarezdev-raspberrypi-zero` / `...-pi5`
- **Enable SSH** → "Allow public-key authentication" → paste your public key
  (e.g. `~/.ssh/id_vtitan_pi_zero.pub`)
- **Username** - any name (the scripts derive it from `$SUDO_USER` and the
  systemd units get it substituted in at install time - it doesn't have to be
  `pi`)
- **Password** - set one
- **WiFi** - SSID, password, and country (first contact is over WiFi)
- **Locale** - timezone / keyboard

Write the card, boot the Pi, and let it join WiFi.

## Phase 2 - provision over SSH

From the dev Pi 5 (one-time: `task rpi:ansible:setup`):

```bash
task rpi:provision:pi5 PI5_IP=x.x.x.x
task rpi:provision:zero               # defaults to ZERO_WIFI_IP
task rpi:ansible:check TARGET=pi5     # dry-run + diff against an already-provisioned Pi
```

GitHub auth (private repo) and the Pi Zero's WiFi 2.4GHz band lock are
handled automatically. See `../ansible/README.md` for the role/playbook
layout.

## Networking

The Pi Zero ↔ Pi 5 link is a USB Ethernet gadget on **192.168.250.0/24**
(Zero = `.1`, Pi 5 = `.2`), owned entirely by NetworkManager - no netplan, no
manual `ip` services, no ICS switcher. WiFi (a separate subnet) carries internet
and first-contact SSH. After provisioning, ROS2 DDS auto-discovers over `usb0`.
