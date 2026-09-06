#!/usr/bin/env bash
# Pull recorded per-run videos (see src/vision/video_recorder.py, written as
# ~/vtitan/data/live/runs/run_<timestamp>/video.mp4 alongside that run's mcap bag) down
# into the shared repo-root data/live/videos tree for local review, WITHOUT
# the (often much larger) bag.
#
# Deliberately a separate tree from data/live/runs/, not a filter added to
# pull-runs-from-pi5.sh: that script's skip logic is "does run_<timestamp>/
# already exist locally", and a video-only pull creating that same directory
# would make a later full run pull skip the whole run -- silently never
# fetching the mcap. Two independent trees, two independent skip checks
# (file-level here, directory-level there), zero interaction.
#
# Usage:
#   bash pull-videos-from-pi5.sh                       # pull every run's video
#   bash pull-videos-from-pi5.sh run_20260804_213147    # pull one run by name
#   bash pull-videos-from-pi5.sh run_2026080            # pull every run matching a prefix
#   PI5_HOST=rpi-5-direct bash pull-videos-from-pi5.sh  # use a different configured host
#   PI5_HOST=user@1.2.3.4 bash pull-videos-from-pi5.sh  # bypass ~/.ssh/config entirely
#   VIDEOS_DIR=/tmp/videos bash pull-videos-from-pi5.sh # pull into a different local dir

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$HERE/../.." && pwd)"
REPO_ROOT="$(cd "$ROBOT_DIR/../.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-~/vtitan/data/live/runs}"
VIDEOS_DIR="${VIDEOS_DIR:-$REPO_ROOT/data/live/videos}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[pull-videos] $*"; }
die() { echo "[pull-videos] ERROR: $*" >&2; exit 1; }

# shellcheck source=../provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

PATTERN="${1:-run_*}"
[ "$PATTERN" = "${PATTERN%\**}" ] && [[ "$PATTERN" != run_* ]] && PATTERN="${PATTERN}*"

# Only runs that actually recorded a video (Open Challenge before this year's
# feature, record_video=false, or a run that never made it past the
# poll-for-the-bag-directory window all have none) -- listed by testing each
# candidate directory for video.mp4 rather than assuming every run has one.
mapfile -t RUNS < <(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "cd $REMOTE_BAG_DIR 2>/dev/null && for d in $PATTERN; do [ -f \"\$d/video.mp4\" ] && echo \"\$d\"; done" || true)

[ "${#RUNS[@]}" -eq 0 ] && die "no run under '$PATTERN' with a video.mp4 found in $REMOTE_BAG_DIR on $PI5_HOST"

log "Pulling ${#RUNS[@]} video(s) from $PI5_HOST:$REMOTE_BAG_DIR -> $VIDEOS_DIR"
for run in "${RUNS[@]}"; do
  if [ -f "$VIDEOS_DIR/$run/video.mp4" ]; then
    log "  $run (already present locally, skipping)"
    continue
  fi
  log "  $run"
  mkdir -p "$VIDEOS_DIR/$run"
  scp "${SSH_OPTS[@]}" -q "$PI5_HOST:$REMOTE_BAG_DIR/$run/video.mp4" "$VIDEOS_DIR/$run/video.mp4" ||
    die "copy of $run/video.mp4 failed"
done

log "Done."
