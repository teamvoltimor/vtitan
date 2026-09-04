#!/usr/bin/env bash
# Push locally-held rosbag runs (pulled via pull-runs-from-pi5.sh, or recorded
# elsewhere) back up into ~/vtitan_runs on the Pi 5, so a reformat/reflash
# doesn't strand the only copy on whichever side happens to have it.
#
# Usage:
#   bash push-runs-to-pi5.sh                       # push every local run
#   bash push-runs-to-pi5.sh run_20260804_213147    # push one run by name
#   bash push-runs-to-pi5.sh run_2026080            # push every run matching a prefix
#   PI5_HOST=rpi-5-direct bash push-runs-to-pi5.sh  # use a different configured host
#   PI5_HOST=user@1.2.3.4 bash push-runs-to-pi5.sh  # bypass ~/.ssh/config entirely

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$HERE/.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-~/vtitan_runs}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[push-runs] $*"; }
die() { echo "[push-runs] ERROR: $*" >&2; exit 1; }

# shellcheck source=../scripts/provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

PATTERN="${1:-run_*}"
[ "$PATTERN" = "${PATTERN%\**}" ] && [[ "$PATTERN" != run_* ]] && PATTERN="${PATTERN}*"

shopt -s nullglob
RUNS=("$HERE"/$PATTERN/)
shopt -u nullglob

[ "${#RUNS[@]}" -eq 0 ] && die "no local runs matching '$PATTERN' found in $HERE"

ssh "${SSH_OPTS[@]}" "$PI5_HOST" "mkdir -p $REMOTE_BAG_DIR" ||
  die "could not create $REMOTE_BAG_DIR on $PI5_HOST"

mapfile -t REMOTE_RUNS < <(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "cd $REMOTE_BAG_DIR 2>/dev/null && ls -d run_* 2>/dev/null" || true)

log "Pushing ${#RUNS[@]} local run(s) -> $PI5_HOST:$REMOTE_BAG_DIR"
for run_path in "${RUNS[@]}"; do
  run="$(basename "$run_path")"
  if printf '%s\n' "${REMOTE_RUNS[@]}" | grep -qx "$run"; then
    log "  $run (already present on Pi 5, skipping)"
    continue
  fi
  log "  $run"
  scp -r "${SSH_OPTS[@]}" -q "$run_path" "$PI5_HOST:$REMOTE_BAG_DIR/" ||
    die "copy of $run failed"
done

log "Done."
