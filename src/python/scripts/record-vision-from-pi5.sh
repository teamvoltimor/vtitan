#!/usr/bin/env bash
# Record the annotated detection video on the Pi 5 and copy it back here.
#
# Run this FROM a dev machine. It handles the awkward parts: the annotated
# stream only exists when debug video is enabled, enabling it needs a service
# restart, and leaving it enabled costs bandwidth during a run -- so unless
# KEEP_DEBUG=1 the previous setting is restored on the way out, including if the
# recording fails.
#
# Usage:
#   bash scripts/record-vision-from-pi5.sh                    # 20s -> ./vision-<timestamp>.mp4
#   SECONDS_TO_RECORD=45 bash scripts/record-vision-from-pi5.sh
#   OUT=/tmp/run.mp4 bash scripts/record-vision-from-pi5.sh
#   KEEP_DEBUG=1 bash scripts/record-vision-from-pi5.sh       # leave debug video on
#   PI5_HOST=user@host bash scripts/record-vision-from-pi5.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
PI5_REPO="${PI5_REPO:-~/vtitan}"
SERVICE="${SERVICE:-vtitan-pi5.service}"
SECONDS_TO_RECORD="${SECONDS_TO_RECORD:-20}"
REMOTE_OUT="/tmp/vision-record.mp4"
OUT="${OUT:-$ROBOT_DIR/vision-$(date +%Y%m%d-%H%M%S).mp4}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[record-vision] $*"; }
die() { echo "[record-vision] ERROR: $*" >&2; exit 1; }

# shellcheck source=provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

REMOTE_ROBOT="$PI5_REPO/src/python"
PREVIOUS="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "grep -m1 '^VISION_DEBUG_VIDEO=' $REMOTE_ROBOT/.env 2>/dev/null | cut -d= -f2" || true)"
PREVIOUS="${PREVIOUS:-0}"

restore() {
  if [ -n "${KEEP_DEBUG:-}" ]; then
    log "KEEP_DEBUG set; leaving debug video enabled"
    return
  fi
  if [ "$PREVIOUS" = "1" ]; then
    return
  fi
  log "Restoring VISION_DEBUG_VIDEO=$PREVIOUS and restarting..."
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $REMOTE_ROBOT \
    && sed -i 's|^VISION_DEBUG_VIDEO=.*|VISION_DEBUG_VIDEO=$PREVIOUS|' .env \
    && sudo systemctl restart '$SERVICE'" || true
}

if [ "$PREVIOUS" != "1" ]; then
  log "Enabling debug video (was $PREVIOUS) and restarting $SERVICE..."
  trap restore EXIT
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $REMOTE_ROBOT \
    && (grep -q '^VISION_DEBUG_VIDEO=' .env || echo 'VISION_DEBUG_VIDEO=0' >> .env) \
    && sed -i 's|^VISION_DEBUG_VIDEO=.*|VISION_DEBUG_VIDEO=1|' .env \
    && sudo systemctl reset-failed '$SERVICE' 2>/dev/null; sudo systemctl restart '$SERVICE'"
  # The node has to come up and open the camera before the topic exists.
  sleep 22
else
  log "Debug video already enabled"
fi

log "Recording ${SECONDS_TO_RECORD}s -- put a block in front of the camera now."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "export PATH=\"\$HOME/.pixi/bin:\$PATH\" \
  && cd $REMOTE_ROBOT \
  && bash -c '. ros2_ws/install/setup.bash && PYTHONPATH=. python scripts/hardware/record_vision_video.py \
       --seconds $SECONDS_TO_RECORD --out $REMOTE_OUT'" ||
  die "recording failed on the Pi"

log "Fetching -> $OUT"
scp "${SSH_OPTS[@]}" -q "$PI5_HOST:$REMOTE_OUT" "$OUT" || die "copy back failed"
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "rm -f $REMOTE_OUT" || true

log "Saved $(du -h "$OUT" | cut -f1) to $OUT"
