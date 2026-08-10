#!/usr/bin/env bash
# Pull rosbag runs recorded on the Pi 5 (see bag_recorder_node.py, default
# ~/vtitan_runs on the robot, one dir per race: run_<timestamp>/) down into
# this folder for local diagnosis. The runs themselves are gitignored --
# only this script (and the .gitkeep placeholder) are tracked.
#
# Usage:
#   bash pull-runs-from-pi5.sh                       # pull every run
#   bash pull-runs-from-pi5.sh run_20260804_213147    # pull one run by name
#   bash pull-runs-from-pi5.sh run_2026080            # pull every run matching a prefix
#   PI5_HOST=rpi-5-direct bash pull-runs-from-pi5.sh  # use a different configured host
#   PI5_HOST=user@1.2.3.4 bash pull-runs-from-pi5.sh  # bypass ~/.ssh/config entirely

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$HERE/.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-~/vtitan_runs}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[pull-runs] $*"; }
die() { echo "[pull-runs] ERROR: $*" >&2; exit 1; }

# shellcheck source=../scripts/provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

PATTERN="${1:-run_*}"
[ "$PATTERN" = "${PATTERN%\**}" ] && [[ "$PATTERN" != run_* ]] && PATTERN="${PATTERN}*"

mapfile -t RUNS < <(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "cd $REMOTE_BAG_DIR 2>/dev/null && ls -d $PATTERN 2>/dev/null" || true)

[ "${#RUNS[@]}" -eq 0 ] && die "no runs matching '$PATTERN' found in $REMOTE_BAG_DIR on $PI5_HOST"

log "Pulling ${#RUNS[@]} run(s) from $PI5_HOST:$REMOTE_BAG_DIR -> $HERE"
for run in "${RUNS[@]}"; do
  if [ -d "$HERE/$run" ]; then
    log "  $run (already present locally, skipping)"
    continue
  fi
  log "  $run"
  scp -r "${SSH_OPTS[@]}" -q "$PI5_HOST:$REMOTE_BAG_DIR/$run" "$HERE/" ||
    die "copy of $run failed"
done

log "Done."
