#!/usr/bin/env bash
# Cleanly shut down the Pi Zero before cutting its power.
#
# Why this exists: pulling power on a running Zero (instead of shutting it
# down first) leaves the ext4 journal uncommitted. That's shown up in
# practice as free-block/inode count mismatches and a stale orphan-file flag
# on the next boot -- not fatal, but it can hang the boot waiting on an
# interactive fsck prompt with no display attached, and repeat occurrences
# are a real corruption risk (see docs/sensor-verification.md's Pi Zero
# deployment section). Always run this before removing power.
#
# Run this ON Pi 5 (same SSH setup as deploy-dev-env-to-zero.sh).
#
# Usage: bash scripts/safe-shutdown-zero.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.250.1 bash scripts/safe-shutdown-zero.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ZERO_HOST="${ZERO_HOST:-ralvarezdev@192.168.250.1}"
SSH_OPTS=(-o ConnectTimeout=10)

log() { echo "[safe-shutdown-zero] $*"; }

log "Target: $ZERO_HOST"
# shellcheck source=scripts/_ssh_preflight.sh
. "$SCRIPT_DIR/_ssh_preflight.sh"
SSH_PREFLIGHT_INTERACTIVE=0   ssh_preflight "$ZERO_HOST" "${SSH_OPTS[@]}" || exit 1

log "Stopping voldemorbot-pi-zero.service and syncing disks before shutdown"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "sudo systemctl stop voldemorbot-pi-zero.service; sync; sudo shutdown -h now" || true

log "Waiting for the Zero to go offline (safe to cut power once this reports it's down)"
for _ in $(seq 1 30); do
  if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes -o ConnectTimeout=3 "$ZERO_HOST" "echo ok" >/dev/null 2>&1; then
    log "Zero is offline -- safe to remove power now."
    exit 0
  fi
  sleep 2
done

echo "WARNING: Zero still reachable after 60s -- do not cut power yet, check manually." >&2
exit 1
