#!/usr/bin/env bash
# Pull rosbag runs recorded on the Pi 5 (see bag_recorder_node.py, default
# ~/vtitan/data/live/runs on the robot, one dir per race: run_<timestamp>/) down into
# the shared repo-root data/live/runs tree for local diagnosis (see
# src/go/internal/recording/root.go for the Go side of this same
# tree). The runs themselves are gitignored -- only this script is tracked.
#
# Usage:
#   bash pull-runs-from-pi5.sh                       # pull every run
#   bash pull-runs-from-pi5.sh run_20260804_213147    # pull one run by name
#   bash pull-runs-from-pi5.sh run_2026080            # pull every run matching a prefix
#   PI5_HOST=rpi-5-direct bash pull-runs-from-pi5.sh  # use a different configured host
#   PI5_HOST=user@1.2.3.4 bash pull-runs-from-pi5.sh  # bypass ~/.ssh/config entirely
#   RUNS_DIR=/tmp/runs bash pull-runs-from-pi5.sh     # pull into a different local dir

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$HERE/../.." && pwd)"
REPO_ROOT="$(cd "$ROBOT_DIR/../.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-~/vtitan/data/live/runs}"
RUNS_DIR="${RUNS_DIR:-$REPO_ROOT/data/live/runs}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[pull-runs] $*"; }
die() { echo "[pull-runs] ERROR: $*" >&2; exit 1; }

# shellcheck source=../provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

PATTERN="${1:-run_*}"
# Append a glob unless the caller already supplied one. The documented usage is
# a run name or timestamp PREFIX (PATTERN=run_20260830), which is a literal and
# matches no directory on its own; an earlier form skipped the append for
# anything starting with "run_", i.e. for exactly the prefixes the Taskfile tells
# you to pass.
case "$PATTERN" in
  *\**) ;;
  *) PATTERN="${PATTERN}*" ;;
esac

mapfile -t RUNS < <(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "cd $REMOTE_BAG_DIR 2>/dev/null && ls -d $PATTERN 2>/dev/null" || true)

[ "${#RUNS[@]}" -eq 0 ] && die "no runs matching '$PATTERN' found in $REMOTE_BAG_DIR on $PI5_HOST"

mkdir -p "$RUNS_DIR"
log "Pulling ${#RUNS[@]} run(s) from $PI5_HOST:$REMOTE_BAG_DIR -> $RUNS_DIR"
for run in "${RUNS[@]}"; do
  if [ -d "$RUNS_DIR/$run" ]; then
    log "  $run (already present locally, skipping)"
    continue
  fi
  log "  $run"
  scp -r "${SSH_OPTS[@]}" -q "$PI5_HOST:$REMOTE_BAG_DIR/$run" "$RUNS_DIR/" ||
    die "copy of $run failed"
done

log "Done."
