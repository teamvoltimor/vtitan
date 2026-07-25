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
#      via pixi.toml's `voldemorbot-shared = { path = "../shared" }`) onto the
#      Zero -- git archive over SSH, no GitHub auth needed on the Zero itself
#   4. Create .env from .env.example if missing (never overwrites an existing one)
#   5. Install the pixi CLI on the Zero (just the binary -- NOT `pixi install`,
#      that heavy resolve/build step is what deploy-dev-env-to-zero.sh avoids
#      running on the Zero at all; see that script's header)
#   6. Enable I2C (dtparam=i2c_arm=on + i2c-dev kernel module) -- off by
#      default on a fresh Raspberry Pi OS image, required for the OLED display
#   7. Template and install the voldemorbot-pi-zero.service systemd unit
#      (left DISABLED -- confirm it works standalone first, see
#      docs/sensor-verification.md's Pi Zero deployment section)
#
# After this script: run deploy-dev-env-to-zero.sh to build+ship the dev pixi
# env and ros2_ws, then `sudo systemctl start voldemorbot-pi-zero.service` to
# test it manually before enabling it to auto-start on boot.
#
# Run this ON Pi 5. Requires Pi 5 already has SOME trusted way into the Zero
# once (e.g. the SSH keys Raspberry Pi Imager's OS customization pre-installs)
# -- this script cannot bootstrap first contact from nothing.
#
# Usage: bash scripts/bootstrap-fresh-zero.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.0.51 bash scripts/bootstrap-fresh-zero.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROBOT_DIR"

ZERO_HOST="${ZERO_HOST:-ralvarezdev@192.168.250.1}"
ZERO_USER="${ZERO_HOST%@*}"
SSH_OPTS=(-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new)

log() { echo "[bootstrap-fresh-zero] $*"; }

log "Target: $ZERO_HOST"
if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$ZERO_HOST" "echo ok" >/dev/null 2>&1; then
  echo "ERROR: cannot reach $ZERO_HOST over SSH with key auth yet." >&2
  echo "This script trusts Pi 5's key INTO the Zero, but needs some existing" >&2
  echo "trusted path in first (e.g. Raspberry Pi Imager's own SSH key customization)." >&2
  exit 1
fi

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
git archive HEAD -- platform/robot platform/shared | ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "mkdir -p ~/voldemorbot && tar -x -C ~/voldemorbot"
cd "$ROBOT_DIR"

log "4/7 Creating .env from .env.example on the Zero (only if missing -- never overwrites)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "cd ~/voldemorbot/platform/robot && [ -f .env ] || cp .env.example .env"

log "5/7 Installing pixi CLI on the Zero (just the binary, not the dev env)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" '[ -x ~/.pixi/bin/pixi ] || curl -fsSL https://pixi.sh/install.sh | sh'

log "6/7 Enabling I2C (needed for the OLED display, off by default on a fresh image)"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  sudo sed -i 's/^#dtparam=i2c_arm=on/dtparam=i2c_arm=on/' /boot/firmware/config.txt
  grep -qxF 'i2c-dev' /etc/modules || echo 'i2c-dev' | sudo tee -a /etc/modules >/dev/null
  sudo modprobe i2c-dev || true
  sudo usermod -a -G i2c,gpio,spi,dialout,video '$ZERO_USER'
"

log "7/7 Templating and installing the systemd service unit (left DISABLED)"
ZERO_HOME="$(ssh "${SSH_OPTS[@]}" "$ZERO_HOST" 'echo $HOME')"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "
  sed -e 's|__TARGET_USER__|$ZERO_USER|g' -e 's|__TARGET_HOME__|$ZERO_HOME|g' \
    ~/voldemorbot/platform/robot/systemd/voldemorbot-pi-zero.service \
    | sudo tee /etc/systemd/system/voldemorbot-pi-zero.service >/dev/null
  sudo systemctl daemon-reload
"

log "Done. A reboot is needed for the I2C change to take effect (use safe-shutdown-zero.sh, don't pull power)."
log "Next: bash scripts/deploy-dev-env-to-zero.sh, then after reboot:"
log "  ssh $ZERO_HOST 'sudo systemctl start voldemorbot-pi-zero.service'"
