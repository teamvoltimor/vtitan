#!/usr/bin/env bash
# Cleanly shut down the Pi Zero, then Pi 5 itself -- in that order -- before
# cutting power to either (e.g. before switching the robot from wall/USB
# power to battery).
#
# Why this order: the Zero is reached over USB from Pi 5, so it must be shut
# down first while Pi 5 is still up to talk to it. Pi 5 shuts itself down
# last, once the Zero is confirmed offline.
#
# Run this ON Pi 5 (same SSH setup as safe-shutdown-zero.sh /
# deploy-dev-env-to-zero.sh).
#
# Usage: bash scripts/safe-shutdown-both.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.250.1 bash scripts/safe-shutdown-both.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

log() { echo "[safe-shutdown-both] $*"; }

log "1/2 Shutting down the Pi Zero"
bash "$ROBOT_DIR/scripts/safe-shutdown-zero.sh"

log "2/2 Zero confirmed offline -- shutting down Pi 5 (this machine) now"
sync
sudo shutdown -h now
