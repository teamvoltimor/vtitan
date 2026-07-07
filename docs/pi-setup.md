# Raspberry Pi setup & provisioning

Canonical record of how the three Raspberry Pis are configured: the developer /
build machine (set up by hand) and the two robot nodes (set up by the
`scripts/setup_pi_*.sh` provisioners). Also lists what the scripts **don't**
cover, so nothing is silently missing.

## Machines & roles

| Machine | Role | Provisioned by |
|---|---|---|
| **Dev / build Pi 5** (`ralvarezdev-raspberrypi`, user `ralvarezdev`) | Developer workstation: edits, builds, flashes, remote access | by hand (documented below) |
| **Robot Pi 5** (user `ralvarezdev`) | State machine, vision (Hailo), IMU, LiDAR, telemetry bridge | `scripts/setup_pi_5.sh` |
| **Robot Pi Zero 2W** (user `ralvarezdev`) | Motors (servo + DC encoder), button, OLED | `scripts/setup_pi_zero.sh` |

All three run **Raspberry Pi OS Lite (64-bit)** / Debian 13 (trixie), arm64.
64-bit is mandatory — pixi/conda-forge has no 32-bit ARM packages.

---

## Dev / build Pi 5 — installed tooling

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
- `cloudflared` — installed, `cloudflared.service` enabled, tunnel configured in
  `/etc/cloudflared/` (`config.yml` + tunnel credentials JSON)
- **WireGuard VPN** (`utun420`, `100.64.0.0/10` CGNAT range — Tailscale)

**Build / language toolchains:**
- `pixi` (per-user) — drives the ROS2 robot environment
- `task` (go-task) — runs the repo Taskfiles
- Go (`/usr/local/go`), Node (bundled by the Zed editor)

**AI accelerator (Hailo AI HAT+):** full stack via apt — `hailo-all 5.1.1`
(`hailort 4.23.0`, `hailo-tappas-core`, `hailort-pcie-driver`,
`python3-hailort`, `python3-hailo-tappas`, `rpicam-apps-hailo-postprocess`);
`hailort.service` enabled.

**Robot stack:** repo checked out as per-branch git **worktrees** under
`~/voldemorbot*` (master, platform, docs, hailo, hugo-docs, auto-annotator);
robot pixi env and `ros2_ws` built.

> Interfaces (i2c/spi/serial) are **disabled** on this box — it builds and
> flashes, it isn't wired to sensors. The robot nodes enable them.

---

## Robot nodes — two-phase provisioning

### Phase 1 — Raspberry Pi Imager (Windows)

Writes the OS and first-contact config. In OS customization: hostname, enable
SSH + public key, username (any name — the scripts derive it from
`$SUDO_USER`), password, WiFi (SSID/password/country), locale. See
`scripts/README.md`.

### Phase 2 — `scripts/setup_pi_*.sh` (run on the Pi over SSH)

Both scripts: verify GitHub auth up front (`gh`, private repo) → apt full-upgrade
→ hardware groups → deps → clone repo via `gh` → pixi env + `build-ws` →
install/enable systemd service(s) + udev rules → reboot.

**`setup_pi_zero.sh` additionally:** config.txt overlays (`dwc2`, `pwm-2chan`,
`i2c`, `uart`), `cmdline.txt` `modules-load=dwc2,g_ether` + fixed gadget MACs,
NM-only `usb0` static profile **192.168.250.1/24**, swap bump for the build.

**`setup_pi_5.sh` additionally:** raspi-config interfaces (i2c/spi/serial/camera),
Hailo runtime, NM `usb0` host IP **192.168.250.2/24**, LiDAR driver fetch +
`voldemorbot-lidar.service`.

### Networking

Pi Zero ↔ Pi 5 USB-Ethernet gadget on **192.168.250.0/24** (Zero `.1` gadget,
Pi 5 `.2` host), **NetworkManager-only** — no netplan, no manual `ip` service, no
ICS switcher. WiFi (separate subnet) carries internet + first-contact SSH.
(Migrated off `10.250.250.0/24` — a VPN was intercepting all `10.0.0.0/8`
traffic and silently blackholing the direct link.)

---

## Coverage matrix — what the scripts do / don't

**✅ Covered (robot-runtime layer):** OS upgrade · hardware groups · interfaces ·
deps · private-repo clone via `gh` · pixi env + ROS2 build (+ LiDAR driver) ·
Hailo runtime · USB-gadget networking · systemd services + udev.

**➖ Intentionally omitted (dev/ops only — on the build box, not robot nodes):**
GPG commit signing · personal git identity · `task` (go-task) · cloudflared
tunnel · Tailscale/WireGuard.

**⚠️ Known script gaps (TODO):**
- **Hailo:** script installs from a manual `--hailo-deb`; the real/official path
  is `apt install hailo-all` (as on the dev box). Should switch.

**🔧 Not covered by anyone — Imager or manual:** hostname / username / WiFi /
SSH keys (Imager) · robot `.env` tuning (GPIO pins, `STEERING_BACKEND`,
`DRIVE_BACKEND`) · on-device steering calibration (`find_limits_interactive`,
see `platform/robot/KNOWN_ISSUES.md`) · physical sensor wiring.
