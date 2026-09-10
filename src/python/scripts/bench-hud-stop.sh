#!/usr/bin/env bash
# Stop a bench-mode vision/HUD recording test session started by
# bench-hud-start.sh.
#
# Simulates a long_press button event to drive state_machine_node from
# RACING into FINISHED, which is what actually finalizes the per-run video
# (see vision_node's _on_robot_state -- recording stops on the RACING ->
# not-RACING edge, not on process exit). Then tears down the bench launch,
# restores vtitan-pi5.service to whatever state bench-hud-start.sh found it
# in, and pulls the finished video back into the repo-root data/live/videos/.
#
# Usage:
#   bash scripts/bench-hud-stop.sh
#   PI5_HOST=rpi-5-direct bash scripts/bench-hud-stop.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
PI5_REPO="${PI5_REPO:-~/vtitan}"
SSH_OPTS=(-o ConnectTimeout=15)

STATE_DIR=/tmp/vtitan_bench_hud
LOG_FILE="$STATE_DIR/launch.log"
WAS_ACTIVE_FILE="$STATE_DIR/service_was_active"
# Must match bench-hud-start.sh's marker exactly -- see its own comment for
# why "rpi5_nodes.launch.py" alone isn't specific enough (it also matches
# the production vtitan-pi5.service launch, which never passes
# is_simulation:=true).
LAUNCH_MARKER="rpi5_nodes.launch.py is_simulation:=true"

log() { echo "[bench-hud-stop] $*"; }
die() { echo "[bench-hud-stop] ERROR: $*" >&2; exit 1; }

# shellcheck source=provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

REMOTE_ROBOT="$PI5_REPO/src/python"

# -A (ignore-ancestors): see bench-hud-start.sh's matching check for why a
# plain pgrep -f here would always false-positive against its own wrapper.
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "pgrep -Af '$LAUNCH_MARKER' >/dev/null" ||
  die "no bench-hud session found on $PI5_HOST -- nothing to stop"

log "Triggering FINISHED (long_press)..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  cd $REMOTE_ROBOT &&
  export PATH=\"\$HOME/.pixi/bin:\$PATH\" &&
  pixi run -e vision ros2 topic pub /button/event std_msgs/msg/String '{data: long_press}' --once
" >/dev/null || die "failed to publish long_press"

log "Waiting for the recorder to finalize..."
sleep 3

run_name="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  grep -o \"_on_run_path: '[^']*'\" $LOG_FILE | tail -1 | sed \"s/.*run_/run_/; s/'\$//\"
" 2>/dev/null || true)"

log "Tearing down the bench launch..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  # See bench-hud-start.sh's prep step for why this can't be a bare pkill -f:
  # this multi-command remote shell's own argv contains the search pattern
  # as literal text (both \$LAUNCH_MARKER and the node names below), so
  # pkill -f would kill its own wrapping shell mid-script instead of just
  # the targets. -A (ignore-ancestors) excludes that wrapper from the match.
  pids=\$(pgrep -Af '$LAUNCH_MARKER' 2>/dev/null; pgrep -Af 'state_machine_node|vision_node|bno08x_uart_rvc_node|telemetry_bridge_node' 2>/dev/null)
  pids=\$(echo \"\$pids\" | sort -u)
  [ -n \"\$pids\" ] && kill -9 \$pids 2>/dev/null
" || true

was_active="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "cat $WAS_ACTIVE_FILE 2>/dev/null || echo false")"
if [ "$was_active" = "true" ]; then
  log "Restoring vtitan-pi5.service..."
  ssh "${SSH_OPTS[@]}" "$PI5_HOST" "sudo systemctl start vtitan-pi5.service" || true
else
  log "vtitan-pi5.service was not running before this session; leaving it stopped."
fi
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "rm -f $WAS_ACTIVE_FILE" || true

if [ -z "$run_name" ]; then
  log "Could not determine the run name from $LOG_FILE on $PI5_HOST -- skipping video pull."
  log "Pull it manually once you have the run name: task robot:pull-videos PATTERN=<run_name>"
  exit 0
fi

log "Pulling $run_name..."
PI5_HOST="$PI5_HOST" bash "$ROBOT_DIR/scripts/sync/pull-videos-from-pi5.sh" "$run_name" ||
  die "video pull failed -- retry with: task robot:pull-videos PATTERN=$run_name"

log "Done. See data/live/videos/$run_name/video.mp4"
