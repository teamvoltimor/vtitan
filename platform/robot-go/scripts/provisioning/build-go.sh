#!/usr/bin/env bash
# Cross-compile the static Go robot binaries and ship them to a target Pi,
# alongside the profile TOMLs. No pixi/conda/ROS2 involved -- pure static
# binary + systemd.
#
# Run this FROM a dev machine (Windows/Git Bash, Linux or macOS). It only needs
# Go locally and SSH/scp to the Pi.
#
# Why it exists (each step removed a real failure mode on hardware):
#   * CGO_ENABLED=0 keeps the binary self-contained; a glibc-linked binary
#     silently fails to exec on a Pi whose libc predates the build host's.
#   * Profiles are gitignored-runtime config, so copying them explicitly (not
#     relying on a git pull) avoids the Pi running a stale/missing profile.
#   * Per-binary copy keeps systemd unit paths stable regardless of how many
#     cmd/* packages exist.
#
# Usage:
#   bash scripts/provisioning/build-go.sh user@pi5-host
#   TARGET_HOST=user@pi5-host INSTALL_DIR=/opt/vtitan-go bash scripts/provisioning/build-go.sh
#   SKIP_RESTART=1 bash scripts/provisioning/build-go.sh user@pi5-host
#
# Verify afterwards (on the Pi):
#   ls -l /opt/vtitan-go/bin
#   sudo systemctl status vtitan-go-pi5.service

set -euo pipefail

GO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$GO_ROOT"

TARGET_HOST="${TARGET_HOST:-${1:-rpi-5-local}}"
INSTALL_DIR="${INSTALL_DIR:-/opt/vtitan-go}"
STAGING_DIR="${STAGING_DIR:-$GO_ROOT/dist}"
BIN_DIR="$STAGING_DIR/bin"
PROFILE_SRC="$GO_ROOT/configs/profiles"
SSH_OPTS=(-o ConnectTimeout=15)

log() { echo "[build-go] $*"; }
die() { echo "[build-go] ERROR: $*" >&2; exit 1; }

if ! command -v go >/dev/null 2>&1; then
  die "go toolchain not found on PATH"
fi

# 1. Cross-compile static arm64 binaries for every cmd/* package.
log "Cross-compiling (CGO_ENABLED=0 GOOS=linux GOARCH=arm64)..."
rm -rf "$BIN_DIR"
mkdir -p "$BIN_DIR"
CGO_ENABLED=0 GOOS=linux GOARCH=arm64 go build -o "$BIN_DIR/" ./cmd/...
log "  built: $(ls -1 "$BIN_DIR" | tr '\n' ' ')"

# 2. Stage the profile TOMLs.
log "Staging profile TOMLs from $PROFILE_SRC..."
mkdir -p "$STAGING_DIR/configs/profiles"
if compgen -G "$PROFILE_SRC/*.toml" >/dev/null; then
  cp "$PROFILE_SRC"/*.toml "$STAGING_DIR/configs/profiles/"
  log "  copied: $(ls -1 "$STAGING_DIR/configs/profiles" | tr '\n' ' ')"
else
  log "  (no .toml files found; profile dir left empty)"
fi

# 3. Ship to the Pi.
log "Transferring to $TARGET_HOST:$INSTALL_DIR ..."
ssh "${SSH_OPTS[@]}" "$TARGET_HOST" "sudo mkdir -p '$INSTALL_DIR/bin' '$INSTALL_DIR/configs/profiles' \
  && sudo chown -R __TARGET_USER__:__TARGET_USER__ '$INSTALL_DIR'"
scp "${SSH_OPTS[@]}" -q -r "$BIN_DIR/." "$TARGET_HOST:$INSTALL_DIR/bin/"
scp "${SSH_OPTS[@]}" -q -r "$STAGING_DIR/configs/profiles/." "$TARGET_HOST:$INSTALL_DIR/configs/profiles/"
ssh "${SSH_OPTS[@]}" "$TARGET_HOST" "sudo chmod 755 '$INSTALL_DIR/bin/'* 2>/dev/null || true"
log "  transfer complete"

# 4. Restart the relevant services.
if [ -n "${SKIP_RESTART:-}" ]; then
  log "SKIP_RESTART set; leaving services alone"
  exit 0
fi

for svc in vtitan-go-pi5.service vtitan-go-pi-zero.service; do
  log "Restarting $svc..."
  ssh "${SSH_OPTS[@]}" "$TARGET_HOST" \
    "sudo systemctl reset-failed '$svc' 2>/dev/null; sudo systemctl restart '$svc'" || \
    log "  (could not restart $svc -- start it by hand)"
done

log "Done."
