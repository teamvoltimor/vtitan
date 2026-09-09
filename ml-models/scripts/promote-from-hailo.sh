#!/usr/bin/env bash
# Promote a compiled model from hailo/shared_with_docker/ (the compiler's
# scratch directory) into a new tracked models/<name>/vN/.
#
# The .hef file must be named explicitly, not guessed: hailo/shared_with_docker/
# routinely holds multiple compiled variants of the same model (e.g.
# gmr_cpu_opt0.hef vs gmr_gpu_qat.hef -- different optimization passes,
# different bytes, only a human can judge which one is "the" model to ship).
#
# Usage: bash ml-models/scripts/promote-from-hailo.sh <name> <hef-file> [onnx-file]
#   bash ml-models/scripts/promote-from-hailo.sh gmr gmr_cpu_opt0.hef gmr.onnx

set -euo pipefail

NAME="${1:?usage: promote-from-hailo.sh <name> <hef-file> [onnx-file]}"
HEF_FILE="${2:?usage: promote-from-hailo.sh <name> <hef-file> [onnx-file]}"
ONNX_FILE="${3:-}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$(cd "$HERE/.." && pwd)"
SRC_DIR="$REPO_DIR/hailo/shared_with_docker"
MODEL_DIR="$HERE/$NAME"

log() { echo "[promote-from-hailo] $*"; }
die() { echo "[promote-from-hailo] ERROR: $*" >&2; exit 1; }

[ -f "$SRC_DIR/$HEF_FILE" ] || die "not found: $SRC_DIR/$HEF_FILE"
if [ -n "$ONNX_FILE" ]; then
  [ -f "$SRC_DIR/$ONNX_FILE" ] || die "not found: $SRC_DIR/$ONNX_FILE"
fi

mkdir -p "$MODEL_DIR"
last_version=0
if [ -d "$MODEL_DIR" ]; then
  for d in "$MODEL_DIR"/v*/; do
    [ -d "$d" ] || continue
    n="$(basename "$d")"
    n="${n#v}"
    [[ "$n" =~ ^[0-9]+$ ]] && [ "$n" -gt "$last_version" ] && last_version="$n"
  done
fi
new_version="v$((last_version + 1))"
DEST_DIR="$MODEL_DIR/$new_version"

[ -e "$DEST_DIR" ] && die "$DEST_DIR already exists (unexpected -- not overwriting)"
mkdir -p "$DEST_DIR"

cp "$SRC_DIR/$HEF_FILE" "$DEST_DIR/$NAME.hef"
log "  $HEF_FILE -> $DEST_DIR/$NAME.hef"
if [ -n "$ONNX_FILE" ]; then
  cp "$SRC_DIR/$ONNX_FILE" "$DEST_DIR/$(basename "$ONNX_FILE")"
  log "  $ONNX_FILE -> $DEST_DIR/$(basename "$ONNX_FILE")"
fi

cat > "$DEST_DIR/metadata.yaml" <<EOF
model: $NAME
version: $new_version
source: hailo/shared_with_docker/$HEF_FILE
$([ -n "$ONNX_FILE" ] && echo "source_onnx: hailo/shared_with_docker/$ONNX_FILE")
notes: >-
  Promoted from the compiler's scratch directory. Fill in what changed
  from the previous version and edit this file before committing.
EOF

echo "$new_version" > "$MODEL_DIR/LATEST"
log "LATEST -> $new_version"
log "Review $DEST_DIR/metadata.yaml (fill in the notes), then commit."
