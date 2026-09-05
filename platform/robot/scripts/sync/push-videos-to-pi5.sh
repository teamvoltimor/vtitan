#!/usr/bin/env bash
# Push locally-held per-run videos (pulled via pull-videos-from-pi5.sh, in the
# shared repo-root data/videos_pulled tree) back up into their matching
# ~/vtitan/data/runs_pulled/run_<timestamp>/ on the Pi 5 -- e.g. after re-encoding/
# trimming one locally and wanting the copy alongside its run's mcap bag back
# in sync.
#
# Usage:
#   bash push-videos-to-pi5.sh                       # push every local video
#   bash push-videos-to-pi5.sh run_20260804_213147    # push one run by name
#   bash push-videos-to-pi5.sh run_2026080            # push every run matching a prefix
#   PI5_HOST=rpi-5-direct bash push-videos-to-pi5.sh  # use a different configured host
#   PI5_HOST=user@1.2.3.4 bash push-videos-to-pi5.sh  # bypass ~/.ssh/config entirely
#   VIDEOS_DIR=/tmp/videos bash push-videos-to-pi5.sh # push from a different local dir

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$HERE/../.." && pwd)"
REPO_ROOT="$(cd "$ROBOT_DIR/../.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-~/vtitan/data/runs_pulled}"
VIDEOS_DIR="${VIDEOS_DIR:-$REPO_ROOT/data/videos_pulled}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[push-videos] $*"; }
die() { echo "[push-videos] ERROR: $*" >&2; exit 1; }

# shellcheck source=../provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

PATTERN="${1:-run_*}"
[ "$PATTERN" = "${PATTERN%\**}" ] && [[ "$PATTERN" != run_* ]] && PATTERN="${PATTERN}*"

shopt -s nullglob
VIDEOS=("$VIDEOS_DIR"/$PATTERN/video.mp4)
shopt -u nullglob

[ "${#VIDEOS[@]}" -eq 0 ] && die "no local video.mp4 matching '$PATTERN' found in $VIDEOS_DIR"

# Each video's run directory must already exist on the Pi (it's pushed there
# by bag_recorder_node when the race actually ran) -- this only adds the
# video file into it, never creates a bare run directory with no bag.
mapfile -t REMOTE_RUNS < <(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "cd $REMOTE_BAG_DIR 2>/dev/null && ls -d run_* 2>/dev/null" || true)

log "Pushing ${#VIDEOS[@]} local video(s) -> $PI5_HOST:$REMOTE_BAG_DIR"
for video_path in "${VIDEOS[@]}"; do
  run="$(basename "$(dirname "$video_path")")"
  if ! printf '%s\n' "${REMOTE_RUNS[@]}" | grep -qx "$run"; then
    log "  $run (no matching run directory on Pi 5, skipping -- push the bag first)"
    continue
  fi
  if ssh "${SSH_OPTS[@]}" "$PI5_HOST" "[ -f $REMOTE_BAG_DIR/$run/video.mp4 ]"; then
    log "  $run (already present on Pi 5, skipping)"
    continue
  fi
  log "  $run"
  scp "${SSH_OPTS[@]}" -q "$video_path" "$PI5_HOST:$REMOTE_BAG_DIR/$run/video.mp4" ||
    die "copy of $run/video.mp4 failed"
done

log "Done."
