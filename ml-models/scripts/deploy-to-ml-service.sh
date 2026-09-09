#!/usr/bin/env bash
# Copy a tracked model version into auto-annotator/ml-service/models/<name>/,
# the untracked working directory the ml-service and
# platform/robot/scripts/provisioning/deploy-to-pi5.sh actually read from.
#
# Usage: bash ml-models/scripts/deploy-to-ml-service.sh <name> [version]
#   bash ml-models/scripts/deploy-to-ml-service.sh gmr          # uses LATEST
#   bash ml-models/scripts/deploy-to-ml-service.sh gmr v1        # pin a specific version

set -euo pipefail

NAME="${1:?usage: deploy-to-ml-service.sh <name> [version]}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$(cd "$HERE/.." && pwd)"
MODEL_DIR="$HERE/$NAME"
DEST_DIR="$REPO_DIR/auto-annotator/ml-service/models/$NAME"

log() { echo "[deploy-to-ml-service] $*"; }
die() { echo "[deploy-to-ml-service] ERROR: $*" >&2; exit 1; }

[ -d "$MODEL_DIR" ] || die "no tracked model at $MODEL_DIR"

VERSION="${2:-}"
if [ -z "$VERSION" ]; then
  [ -f "$MODEL_DIR/LATEST" ] || die "$MODEL_DIR/LATEST missing and no version given"
  VERSION="$(cat "$MODEL_DIR/LATEST")"
fi

SRC_DIR="$MODEL_DIR/$VERSION"
[ -d "$SRC_DIR" ] || die "no such version: $SRC_DIR"

mkdir -p "$DEST_DIR"
log "Deploying $NAME $VERSION -> $DEST_DIR"
for f in "$SRC_DIR"/*; do
  base="$(basename "$f")"
  [ "$base" = "metadata.yaml" ] && continue
  cp "$f" "$DEST_DIR/$base"
  log "  $base"
done

log "Done. Deploy to the Pi 5 with:"
log "  bash platform/robot/scripts/provisioning/deploy-to-pi5.sh"
