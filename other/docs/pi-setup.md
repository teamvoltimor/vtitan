# Raspberry Pi setup & provisioning

Canonical record of how the three Raspberry Pis are configured: the developer /
build machine (set up by hand) and the two robot nodes (provisioned by
Ansible, `deploy/ansible/` - via `task rpi:provision:pi5` / `task rpi:provision:zero`
- which replaced the old `scripts/setup_pi_*.sh` scp+ssh scripts, removed
2026-07-30). Also lists what provisioning **doesn't** cover, so nothing is
silently missing.

## Machines & roles

| Machine | Role | Provisioned by |
|---|---|---|
| **Dev / build Pi 5** (`ralvarezdev-raspberrypi`, user `ralvarezdev`) | Developer workstation: edits, builds, flashes, remote access | by hand (documented below) |
| **Robot Pi 5** (user `ralvarezdev`) | State machine, vision (Hailo), IMU, LiDAR, telemetry bridge | `deploy/ansible/roles/pi5` (`task rpi:provision:pi5`) |
| **Robot Pi Zero 2W** (user `ralvarezdev`) | Motors (servo + DC encoder), button, OLED | `deploy/ansible/roles/pi_zero` (`task rpi:provision:zero`) |

All three run **Raspberry Pi OS Lite (64-bit)** / Debian 13 (trixie), arm64.
64-bit is mandatory - pixi/conda-forge has no 32-bit ARM packages.

---

## Dev / build Pi 5 - installed tooling

Set up manually; recorded here so it's reproducible. (AI coding assistants are
intentionally not listed.)

**Base:** Raspberry Pi 5 Model B, Debian 13 trixie arm64, 2 GB zram swap, user in
groups `gpio i2c spi dialout render video`.

**Source control & GitHub:**
- `git` with **GPG commit signing** (`user.signingkey=D2661E4AD09ED1CD`,
  `commit.gpgsign=true`); GPG key `rsa4096/D2661E4AD09ED1CD`
- `gh` (GitHub CLI) authenticated; configured as git credential helper
- `gpg` (gnupg)

**Remote access:**
- `cloudflared` - installed, `cloudflared.service` enabled, tunnel configured in
  `/etc/cloudflared/` (`config.yml` + tunnel credentials JSON)
- **WireGuard VPN** (`utun420`, `100.64.0.0/10` CGNAT range - Tailscale)

**Build / language toolchains:**
- `pixi` (per-user) - drives the ROS2 robot environment
- `task` (go-task) - runs the repo Taskfiles
- `ansible` + `community.general` collection - provisions the two robot Pis
  (`task rpi:ansible:setup` installs both; see `deploy/ansible/README.md`)
- Go (`/usr/local/go`), Node (bundled by the Zed editor)

**AI accelerator (Hailo AI HAT+):** full stack via apt - `hailo-all 5.1.1`
(`hailort 4.23.0`, `hailo-tappas-core`, `hailort-pcie-driver`,
`python3-hailort`, `python3-hailo-tappas`, `rpicam-apps-hailo-postprocess`);
`hailort.service` enabled.

**Robot stack:** repo checked out as per-branch git **worktrees** under
`~/vtitan*` (master, platform, docs, hailo, hugo-docs, auto-annotator);
robot pixi env and `ros2_ws` built.

> Interfaces (i2c/spi/serial) are **disabled** on this box - it builds and
> flashes, it isn't wired to sensors. The robot nodes enable them.

---

## Robot nodes - two-phase provisioning

### Phase 1 - Raspberry Pi Imager (Windows)

Writes the OS and first-contact config. In OS customization: hostname, enable
SSH + public key, username (any name - the scripts derive it from
`$SUDO_USER`), password, WiFi (SSID/password/country), locale. See
`scripts/README.md`.

### Phase 2 - Ansible (run from the dev Pi 5, over SSH)

`task rpi:provision:pi5 PI5_IP=x.x.x.x` / `task rpi:provision:zero` - see
`deploy/ansible/README.md` for the role layout. Idempotent (`task rpi:ansible:check`
dry-runs with `--check --diff`), ported from and matching `scripts/setup_pi_*.sh`
(kept as a manual fallback, see `scripts/README.md`):

Both roles (`common` + board role): verify GitHub auth up front (`gh`, private
repo) → apt full-upgrade → hardware groups → deps → clone repo via `gh` →
pixi env + `build-ws` → install/enable systemd service(s) + udev rules →
reboot only if boot config actually changed.

**`pi_zero` role additionally:** config.txt overlays (`dwc2`, `pwm-2chan`,
`i2c`, `uart`), `cmdline.txt` `modules-load=dwc2,g_ether` + fixed gadget MACs,
NM-only `usb0` static profile **192.168.250.1/24**, swap bump for the build,
Pi 5 WiFi 2.4GHz band lock for first-contact reachability (released even on
failure).

**`pi5` role additionally:** raspi-config interfaces (i2c/spi/serial/camera),
Hailo runtime (`apt install hailo-all`), NM `usb0` host IP
**192.168.250.2/24**, LiDAR driver fetch + `vtitan-lidar.service`.

### Networking

Pi Zero ↔ Pi 5 USB-Ethernet gadget on **192.168.250.0/24** (Zero `.1` gadget,
Pi 5 `.2` host), **NetworkManager-only** - no netplan, no manual `ip` service, no
ICS switcher. WiFi (separate subnet) carries internet + first-contact SSH.
(Migrated off `10.250.250.0/24` - a VPN was intercepting all `10.0.0.0/8`
traffic and silently blackholing the direct link.)

### Getting internet on the road (away from the home network)

Three ways to *reach* the boards from Windows, only one of which gives the
boards themselves internet:

| Link | SSH alias | Subnet | Gives the Pi internet? |
|---|---|---|---|
| Home WiFi | `rpi-5-local`, `rpi-zero-local` | home router's LAN | Yes, if that WiFi has internet |
| Direct cable/USB-gadget | `rpi-5-direct`, `rpi-zero-direct` | `192.168.251.0/24` (Windows↔Pi5), `192.168.250.0/24` (Pi5↔Zero) | **No** - private point-to-point link only |
| Cloudflare Tunnel | `rpi-5-remote` | over the internet | N/A - this is *outbound from* the Pi, so it only works once the Pi already has internet via its own WiFi |

The direct links exist so you can always reach a board even with **no WiFi at
all** - that's also what makes them useful for bootstrapping WiFi at a new
location:

1. Plug the Ethernet cable Windows↔Pi 5 (`task windows:ethernet:setup-link` once,
   if `rpi-5-direct`'s static IP isn't already configured on this Windows
   machine).
2. Set the Pi 5's new WiFi credentials over that cable - works even though
   the Pi 5 has no WiFi connection yet:
   `task windows:set-wifi:pi5 SSID=name PASSWORD=pass`
3. Set the Pi Zero's new WiFi credentials the same way, hopping through the
   Pi 5 over the USB gadget link - works even though the Zero has no WiFi
   connection yet:
   `task windows:set-wifi:zero SSID=name PASSWORD=pass`
4. Both boards now have internet through the new network. `rpi-5-local` /
   `rpi-zero-local` will only resolve once you know each board's new IP on
   that network (DHCP-assigned, so it can differ from the home values baked
   into `Taskfile.yml`'s `RPI_LOCAL_IP` / `ZERO_WIFI_IP`) - check the new
   router's client list, or SSH in over the direct link and run
   `task rpi:iface-ip IFACE=wlan0`.

**Gotcha:** the Pi Zero is **2.4 GHz-only**. A phone hotspot or router that's
5 GHz-only (or dual-band under one SSID that prefers 5 GHz) will connect the
Pi 5 but leave the Zero unable to join - pick/force a 2.4 GHz network or SSID
for the Zero.

Remote SSH from anywhere (`rpi-5-remote`, via `cloudflared`) only reaches the
Pi 5, and only after step 2 has given it working internet; it doesn't help
bootstrap a brand-new location and doesn't reach the Zero directly (hop
through Pi 5 once connected, same as `rpi-zero-direct` does over the gadget
link).

---

## Coverage matrix - what provisioning does / doesn't cover

**✅ Covered (robot-runtime layer):** OS upgrade · hardware groups · interfaces ·
deps · private-repo clone via `gh` · pixi env + ROS2 build (+ LiDAR driver) ·
Hailo runtime (`hailo-all` via apt) · USB-gadget networking · systemd services
+ udev.

**➖ Intentionally omitted (dev/ops only - on the build box, not robot nodes):**
GPG commit signing · personal git identity · `task` (go-task) · cloudflared
tunnel · Tailscale/WireGuard.

**🔧 Not covered by anyone - Imager or manual:** hostname / username / WiFi /
SSH keys (Imager) · robot `.env` tuning (GPIO pins, `STEERING_BACKEND`,
`DRIVE_BACKEND`) · on-device steering calibration (`find_limits_interactive`,
see `src/python/KNOWN_ISSUES.md`) · physical sensor wiring.
