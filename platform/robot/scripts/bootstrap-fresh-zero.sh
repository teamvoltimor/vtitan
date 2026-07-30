#!/usr/bin/env bash
# One-shot setup for a freshly-flashed Pi Zero, before deploy-dev-env-to-zero.sh
# can be used. Safe to re-run (idempotent) -- e.g. after a re-flash, or to
# reassert state on an existing Zero.
#
# What this covers (previously done by hand, see docs/sensor-verification.md's
# "Setting up a freshly-flashed Pi Zero" section for the full narrative):
#   1. Trust Pi 5's SSH key into the Zero's authorized_keys
#   2. Verify passwordless sudo on both boards
#   3. Copy platform/robot AND platform/shared (a sibling dependency pulled in
#      via pixi.toml's `vtitan-shared = { path = "../shared" }`) onto the
#      Zero -- git archive over SSH, no GitHub auth needed on the Zero itself
#   4. Create .env from .env.example if missing (never overwrites an existing one)
#   5. Install the pixi CLI on the Zero (just the binary -- NOT `pixi install`,
#      that heavy resolve/build step is what deploy-dev-env-to-zero.sh avoids
#      running on the Zero at all; see that script's header)
#   6. Enable I2C (dtparam=i2c_arm=on + i2c-dev kernel module) -- off by
#      default on a fresh Raspberry Pi OS image, required for the OLED display
#   6b. Map the two-channel hardware PWM overlay onto the servo and drive
#      motor pins, so both are driven by the SoC peripheral instead of
#      gpiozero's software PWM (see the step's inline comment and
#      docs/sensor-verification.md)
#   6c. Configure the USB-gadget link to Pi 5 (dwc2 + g_ether + fixed MACs +
#      a static usb0 profile) -- the competition-critical path, since WiFi may
#      not be available at the venue. A fresh flash sets up none of it.
#   7. Template and install the vtitan-pi-zero.service systemd unit
#      (left DISABLED -- confirm it works standalone first, see
#      docs/sensor-verification.md's Pi Zero deployment section)
#
# After this script: run deploy-dev-env-to-zero.sh to build+ship the dev pixi
# env and ros2_ws, then `sudo systemctl start vtitan-pi-zero.service` to
# test it manually before enabling it to auto-start on boot.
#
# Run this ON Pi 5. Requires Pi 5 already has SOME trusted way into the Zero
# once (e.g. the SSH keys Raspberry Pi Imager's OS customization pre-installs)
# -- this script cannot bootstrap first contact from nothing.
#
# Usage: bash scripts/bootstrap-fresh-zero.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.0.51 bash scripts/bootstrap-fresh-zero.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROBOT_DIR"

ZERO_HOST="${ZERO_HOST:-ralvarezdev@192.168.250.1}"
ZERO_USER="${ZERO_HOST%@*}"
SSH_OPTS=(-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new)
# Must match SERVO_GPIO_PIN / motor_pwm_pin in .env -- the overlay decides
# which pins the PWM peripheral drives, and the drivers just open the
# resulting pwmchip channels (0 = SERVO_PWM_PIN, 1 = MOTOR_PWM_PIN).
SERVO_PWM_PIN="${SERVO_PWM_PIN:-12}"
MOTOR_PWM_PIN="${MOTOR_PWM_PIN:-13}"
# USB-gadget link addressing. Fixed MACs so NetworkManager sees the same device
# across reboots; the .1/.2 split matches ZERO_HOST's default above.
ZERO_USB_IP="${ZERO_USB_IP:-192.168.250.1}"
ZERO_USB_MAC="${ZERO_USB_MAC:-02:00:00:00:ce:01}"
PI5_USB_MAC="${PI5_USB_MAC:-02:00:00:00:ce:02}"

log() { echo "[bootstrap-fresh-zero] $*"; }

log "Target: $ZERO_HOST"
# shellcheck source=scripts/_ssh_preflight.sh
. "$SCRIPT_DIR/_ssh_preflight.sh"
SSH_PREFLIGHT_INTERACTIVE=0 SSH_PREFLIGHT_HINT="This script trusts Pi 5's key INTO the Zero, but needs some existing
trusted path in first (e.g. Raspberry Pi Imager's own SSH key customization)."   ssh_preflight "$ZERO_HOST" "${SSH_OPTS[@]}" || exit 1

log "1/7 Trusting Pi 5's SSH key into the Zero's authorized_keys"
if [ ! -f ~/.ssh/id_ed25519.pub ]; then
  ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
fi
PI5_PUBKEY="$(cat ~/.ssh/id_ed25519.pub)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "mkdir -p ~/.ssh && chmod 700 ~/.ssh && grep -qxF '$PI5_PUBKEY' ~/.ssh/authorized_keys 2>/dev/null || echo '$PI5_PUBKEY' >> ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys"

log "2/7 Verifying passwordless sudo on both boards"
sudo -n true 2>/dev/null || { echo "ERROR: Pi 5 itself needs passwordless sudo set up (see docs/sensor-verification.md Prerequisites)." >&2; exit 1; }
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "sudo -n true" 2>/dev/null || { echo "ERROR: Zero needs passwordless sudo set up (see docs/sensor-verification.md Prerequisites)." >&2; exit 1; }

log "3/7 Copying platform/robot + platform/shared onto the Zero (git archive over SSH, no auth needed on the Zero)"
cd "$ROBOT_DIR/../.."  # repo root
git archive HEAD -- platform/robot platform/shared | ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "mkdir -p ~/vtitan && tar -x -C ~/vtitan"
cd "$ROBOT_DIR"

log "4/7 Creating .env from .env.example on the Zero (only if missing -- never overwrites)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "cd ~/vtitan/platform/robot && [ -f .env ] || cp .env.example .env"

log "5/7 Installing pixi CLI on the Zero (just the binary, not the dev env)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" '[ -x ~/.pixi/bin/pixi ] || curl -fsSL https://pixi.sh/install.sh | sh'

log "6/7 Enabling I2C (needed for the OLED display, off by default on a fresh image)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  sudo sed -i 's/^#dtparam=i2c_arm=on/dtparam=i2c_arm=on/' /boot/firmware/config.txt
  grep -qxF 'i2c-dev' /etc/modules || echo 'i2c-dev' | sudo tee -a /etc/modules >/dev/null
  sudo modprobe i2c-dev || true
  sudo usermod -a -G i2c,gpio,spi,dialout,video '$ZERO_USER'
"

log "6b/7 Enabling hardware PWM on the servo (GPIO $SERVO_PWM_PIN) and drive motor (GPIO $MOTOR_PWM_PIN) pins"
# Both PWM lines must be driven by the SoC's PWM peripheral, not gpiozero's
# software PWM: under LGPIOFactory (what the Zero uses) gpiozero generates the
# pulse train in software, so scheduling jitter lands on the duty cycle and
# the software-PWM thread runs continuously regardless of tick rate. On the
# servo this showed up as visible twitching even while holding a fixed angle
# (confirmed on hardware -- the twitching survived removing all PWM rewrites
# and swapping in a fresh battery); on the drive motor it showed up as
# continuous CPU cost from the background bit-bang thread on an
# already-overloaded Pi Zero, plus PID-visible speed noise. func=4 selects
# ALT0 on both pins (GPIO 12 = PWM0, GPIO 13 = PWM1).
#
# Two-channel overlay: pin/func is channel 0 (servo), pin2/func2 is channel 1
# (drive motor). See src/hardware/motors/servo/config.py and
# src/hardware/motors/dc_encoder/config.py for the pwmchip/channel mapping.
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  sudo sed -i '/^dtoverlay=pwm\(-2chan\)\?\(,\|\$\)/d' /boot/firmware/config.txt
  echo 'dtoverlay=pwm-2chan,pin=$SERVO_PWM_PIN,func=4,pin2=$MOTOR_PWM_PIN,func2=4' | sudo tee -a /boot/firmware/config.txt >/dev/null
"

log "6c/7 Configuring the USB-gadget (Ethernet-over-USB) link to Pi 5"
# This is the competition-critical path: WiFi may not be available at the
# venue, so Pi 5 <-> Zero ROS2 traffic has to work over the USB cable alone.
# A fresh Raspberry Pi Imager flash does NOT set any of this up, and losing it
# is silent -- the Zero simply never appears on 192.168.250.1.
#
# Four separate pieces, all required:
#   - dtoverlay=dwc2      : put the USB controller in a mode that can act as a
#                           peripheral at all (a stock image leaves it host-only)
#   - modules-load=...    : load dwc2 + g_ether from initramfs, early enough that
#                           the host sees the gadget on its first enumeration
#   - fixed MACs          : without these the gadget gets a random MAC each boot,
#                           so NetworkManager treats it as a new device and the
#                           static profile below never binds
#   - static usb0 profile : the link is point-to-point with no DHCP server on it
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  grep -qxF 'dtoverlay=dwc2' /boot/firmware/config.txt \
    || echo 'dtoverlay=dwc2' | sudo tee -a /boot/firmware/config.txt >/dev/null

  # cmdline.txt must stay a SINGLE line -- appending a newline makes the kernel
  # silently ignore everything after it, including rootwait.
  grep -q 'modules-load=dwc2,g_ether' /boot/firmware/cmdline.txt \
    || sudo sed -i 's/\$/ modules-load=dwc2,g_ether/' /boot/firmware/cmdline.txt

  echo 'options g_ether dev_addr=$ZERO_USB_MAC host_addr=$PI5_USB_MAC' \
    | sudo tee /etc/modprobe.d/g_ether.conf >/dev/null

  sudo nmcli connection show usb0 >/dev/null 2>&1 || sudo nmcli connection add \
    type ethernet ifname usb0 con-name usb0 \
    ipv4.method manual ipv4.addresses '$ZERO_USB_IP/24' ipv6.method disabled
"

log "7/7 Templating and installing the systemd service unit (left DISABLED)"
ZERO_HOME="$(ssh "${SSH_OPTS[@]}" "$ZERO_HOST" 'echo $HOME')"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  sed -e 's|__TARGET_USER__|$ZERO_USER|g' -e 's|__TARGET_HOME__|$ZERO_HOME|g' \
    ~/vtitan/platform/robot/systemd/vtitan-pi-zero.service \
    | sudo tee /etc/systemd/system/vtitan-pi-zero.service >/dev/null
  sudo systemctl daemon-reload
"

log "Done. A reboot is needed for the I2C and PWM overlay changes to take effect (use safe-shutdown-zero.sh, don't pull power)."
log "Next: bash scripts/deploy-dev-env-to-zero.sh, then after reboot:"
log "  ssh $ZERO_HOST 'sudo systemctl start vtitan-pi-zero.service'"
