#!/usr/bin/env bash
# One-shot bench-mode vision/HUD recording test: start, record for
# SECONDS_TO_RECORD, stop, and pull the video back. Equivalent to running
# bench-hud-start.sh, sleeping, then bench-hud-stop.sh by hand.
#
# Usage:
#   bash scripts/bench-hud-record.sh
#   SECONDS_TO_RECORD=45 bash scripts/bench-hud-record.sh
#   PI5_HOST=rpi-5-direct bash scripts/bench-hud-record.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

SECONDS_TO_RECORD="${SECONDS_TO_RECORD:-20}"

log() { echo "[bench-hud-record] $*"; }

bash "$ROBOT_DIR/scripts/bench-hud-start.sh"

log "Recording for ${SECONDS_TO_RECORD}s..."
sleep "$SECONDS_TO_RECORD"

bash "$ROBOT_DIR/scripts/bench-hud-stop.sh"
