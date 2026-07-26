#!/usr/bin/env bash
# Ship code and the compiled detector HEF to the Pi 5, rebuild the ROS
# workspace, and restart the robot service.
#
# Run this FROM a dev machine (Windows/Git Bash, Linux or macOS). It only needs
# SSH to the Pi; nothing here is built locally.
#
# Every step exists because leaving it out produced a silent failure on the
# real robot, not because it seemed tidy:
#
#   * The HEF is gitignored, so a `git pull` alone leaves the Pi running
#     whatever model was there before -- or none.
#   * ros2_ws entry points carry an absolute shebang baked in at build time. A
#     workspace built under `-e dev` makes every node re-exec under dev's
#     interpreter, which has neither `hailort` nor `ultralytics` -- so the
#     vision node crash-loops on import no matter which env the service names.
#     It must be rebuilt under `-e vision`.
#   * HAILO_MODEL_PATH in the Pi's .env wins over the code default, because
#     src/logger/config.py calls load_dotenv() at import. A stale entry there
#     points the driver at a file that does not exist and surfaces only as
#     HAILO_OPEN_FILE_FAILURE.
#
# Usage:
#   bash scripts/deploy-to-pi5.sh                 # code + model + rebuild + restart
#   HEF= bash scripts/deploy-to-pi5.sh            # skip the model, code only
#   PI5_HOST=user@host bash scripts/deploy-to-pi5.sh
#   SKIP_RESTART=1 bash scripts/deploy-to-pi5.sh  # leave the service down
#
# Verify afterwards (on the Pi, with the stack running):
#   pixi run -e vision python scripts/diag_hailo_detector.py IMAGE...
#   pixi run -e vision python scripts/diag_vision_topic.py IMAGE...

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$(cd "$ROBOT_DIR/../.." && pwd)"
cd "$REPO_DIR"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
PI5_REPO="${PI5_REPO:-~/voldemorbot}"
BRANCH="${BRANCH:-$(git rev-parse --abbrev-ref HEAD)}"
HEF="${HEF-auto-annotator/ml-service/models/gmr/gmr.hef}"
HEF_DEST="${HEF_DEST:-/usr/local/hailo/models/gmr.hef}"
SERVICE="${SERVICE:-voldemorbot-pi5.service}"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[deploy-pi5] $*"; }
die() { echo "[deploy-pi5] ERROR: $*" >&2; exit 1; }

# shellcheck source=scripts/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1
log "Target: $PI5_HOST  branch: $BRANCH"

# 1. Code. Push through a bundle rather than a remote: this works when the
#    branch has never been pushed, and never publishes anything.
log "Transferring commits..."
REMOTE_HEAD="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $PI5_REPO && git rev-parse HEAD")"
if [ "$REMOTE_HEAD" = "$(git rev-parse "$BRANCH")" ]; then
  log "  already at $(git rev-parse --short "$BRANCH"), nothing to send"
else
  BUNDLE="$(mktemp -t deploy-XXXXXX.bundle)"
  trap 'rm -f "$BUNDLE"' EXIT
  # A bundle based on the Pi's HEAD keeps the transfer to the new commits, but
  # only works while that commit is an ancestor. Fall back to the whole branch.
  if git merge-base --is-ancestor "$REMOTE_HEAD" "$BRANCH" 2>/dev/null; then
    git bundle create "$BUNDLE" "$REMOTE_HEAD..$BRANCH" >/dev/null 2>&1
  else
    log "  Pi HEAD is not an ancestor; sending the full branch"
    git bundle create "$BUNDLE" "$BRANCH" >/dev/null 2>&1
  fi
  scp "${SSH_OPTS[@]}" -q "$BUNDLE" "$PI5_HOST:/tmp/deploy.bundle"
  # Fetch to FETCH_HEAD rather than straight into the branch ref: git refuses
  # the latter when that branch is the one checked out, which it usually is.
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $PI5_REPO \
    && git fetch -q /tmp/deploy.bundle '$BRANCH' \
    && if git rev-parse --verify -q '$BRANCH' >/dev/null; then \
         git checkout -q '$BRANCH' && git merge -q --ff-only FETCH_HEAD; \
       else \
         git checkout -q -b '$BRANCH' FETCH_HEAD; \
       fi \
    && rm -f /tmp/deploy.bundle"
fi
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $PI5_REPO && git log --oneline -1"

# 2. Model.
if [ -n "$HEF" ]; then
  [ -f "$HEF" ] || die "HEF not found: $HEF (compile it with 'task gmr:workflow' in hailo/)"
  log "Deploying $(basename "$HEF") -> $HEF_DEST"
  scp "${SSH_OPTS[@]}" -q "$HEF" "$PI5_HOST:/tmp/$(basename "$HEF_DEST")"
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "sudo mkdir -p '$(dirname "$HEF_DEST")' \
    && sudo mv '/tmp/$(basename "$HEF_DEST")' '$HEF_DEST' && sudo chmod 644 '$HEF_DEST'"

  LOCAL_SUM="$(md5sum "$HEF" | cut -d' ' -f1)"
  REMOTE_SUM="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "md5sum '$HEF_DEST' | cut -d' ' -f1")"
  [ "$LOCAL_SUM" = "$REMOTE_SUM" ] || die "checksum mismatch after copy ($LOCAL_SUM != $REMOTE_SUM)"
  log "  checksum ok: $LOCAL_SUM"

  # The .env value overrides the code default, so a stale one silently wins.
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cd $PI5_REPO/platform/robot \
    && if [ -f .env ]; then \
         if grep -q '^HAILO_MODEL_PATH=' .env; then \
           sed -i 's|^HAILO_MODEL_PATH=.*|HAILO_MODEL_PATH=$HEF_DEST|' .env; \
         else echo 'HAILO_MODEL_PATH=$HEF_DEST' >> .env; fi; \
         grep '^HAILO_MODEL_PATH=' .env; \
       else echo '(no .env; code default applies)'; fi"
fi

# 3. Workspace. Must be the vision env -- see the shebang note above.
log "Rebuilding ros2_ws under the vision env..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "export PATH=\"\$HOME/.pixi/bin:\$PATH\" \
  && cd $PI5_REPO/platform/robot && pixi run -e vision build-ws" >/dev/null ||
  die "colcon build failed; re-run by hand for the log"

SHEBANG="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" \
  "head -1 $PI5_REPO/platform/robot/ros2_ws/install/lib/voldemorbot_vision/vision_node")"
case "$SHEBANG" in
  *envs/vision/*) log "  entry point interpreter: vision env, ok" ;;
  *) die "vision_node shebang is '$SHEBANG' -- expected the vision env" ;;
esac

# 4. Service.
if [ -n "${SKIP_RESTART:-}" ]; then
  log "SKIP_RESTART set; leaving $SERVICE alone"
  exit 0
fi
log "Restarting $SERVICE..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "sudo systemctl reset-failed '$SERVICE' 2>/dev/null; \
  sudo systemctl restart '$SERVICE'"
sleep 15

STATE="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "systemctl is-active '$SERVICE'")"
log "  service: $STATE"
[ "$STATE" = "active" ] || die "$SERVICE is $STATE"

log "Vision node startup:"
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "journalctl -u '$SERVICE' --since '1 min ago' --no-pager 2>/dev/null \
  | grep -iE 'HEF model loaded|Vision Node ready|vision.*(Error|died)' | tail -5" || true

log "Done."
