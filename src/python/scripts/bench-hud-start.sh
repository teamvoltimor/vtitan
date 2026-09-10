#!/usr/bin/env bash
# Start a bench-mode vision/HUD recording test session on the Pi 5.
#
# Launches SET=rpi5-bench (state_machine + vision + imu + telemetry_bridge,
# bypassing BOOT_CHECK's IMU/LiDAR/Hailo/jumper checks -- see
# rpi5_nodes.launch.py's is_simulation) and simulates a short_press button
# event to drive state_machine_node into RACING, the only state that arms
# vision_node's per-run video recorder (_maybe_start_recording in
# src/ros2/vision/node.py). Stops vtitan-pi5.service first since it launches
# the same node set and would collide -- see robot:launch's own description.
#
# Pairs with bench-hud-stop.sh, which ends the race, pulls the video back,
# and restores vtitan-pi5.service to whatever state it was in before this
# ran. Do not leave a session started by this script running unattended: the
# production service is down until stop.sh runs.
#
# Usage:
#   bash scripts/bench-hud-start.sh
#   PI5_HOST=rpi-5-direct bash scripts/bench-hud-start.sh

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PI5_HOST="${PI5_HOST:-rpi-5-local}"
PI5_REPO="${PI5_REPO:-~/vtitan}"
SSH_OPTS=(-o ConnectTimeout=15)

STATE_DIR=/tmp/vtitan_bench_hud
LOG_FILE="$STATE_DIR/launch.log"
WAS_ACTIVE_FILE="$STATE_DIR/service_was_active"
# Identifies this launch's supervisor process (not the node executables,
# which are respawn=True and would just come back -- see bench-hud-stop.sh's
# teardown for why that matters). Must be specific to the BENCH invocation,
# not just "rpi5_nodes.launch.py" -- vtitan-pi5.service runs that exact same
# launch file in production (with is_simulation defaulting to false, so the
# arg is simply omitted from argv), and a marker that also matched
# production made this script refuse to start while the production service
# was up -- exactly the normal case this script exists to take over from.
LAUNCH_MARKER="rpi5_nodes.launch.py is_simulation:=true"

log() { echo "[bench-hud-start] $*"; }
die() { echo "[bench-hud-start] ERROR: $*" >&2; exit 1; }

# shellcheck source=provisioning/_ssh_preflight.sh
. "$ROBOT_DIR/scripts/provisioning/_ssh_preflight.sh"
pi5_preflight "$PI5_HOST" "${SSH_OPTS[@]}" || exit 1

REMOTE_ROBOT="$PI5_REPO/src/python"

# -A (ignore-ancestors) matters here: with a plain -f, the >/dev/null
# redirect forces bash to fork instead of exec-replacing itself for this
# single command, so the surviving wrapper shell -- whose own argv is
# literally "pgrep -f $LAUNCH_MARKER >/dev/null" -- matches its own pattern
# and pgrep reports a false positive every time, whether or not a real
# session is running.
if ssh "${SSH_OPTS[@]}" "$PI5_HOST" "pgrep -Af '$LAUNCH_MARKER' >/dev/null"; then
  die "a bench-hud session already looks to be running on $PI5_HOST -- run 'task robot:bench-hud:stop' first"
fi

log "Stopping vtitan-pi5.service (if running) and clearing stray nodes..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  mkdir -p $STATE_DIR
  if systemctl is-active --quiet vtitan-pi5.service; then echo true; else echo false; fi > $WAS_ACTIVE_FILE
  sudo systemctl stop vtitan-pi5.service 2>/dev/null || true
  # pgrep -f matches the FULL cmdline of every process, including this
  # multi-command remote bash -c invocation itself -- its own argv is this
  # entire script, which contains the search pattern as literal text. A
  # bare pkill -f here kills its own wrapping shell mid-script (confirmed:
  # exit 255, script aborts before the target processes ever die). -A
  # (ignore-ancestors) excludes that wrapper -- and everything above it --
  # from the match.
  pids=\$(pgrep -Af 'state_machine_node|vision_node|bno08x_uart_rvc_node|telemetry_bridge_node' 2>/dev/null || true)
  [ -n \"\$pids\" ] && kill -9 \$pids 2>/dev/null
  sleep 1
" || die "failed to prep $PI5_HOST"

log "Launching SET=rpi5-bench..."
# setsid -f (not just nohup + background + disown) -- confirmed on hardware
# that plain backgrounding still leaves the ssh channel hanging open even
# with every std fd redirected away, because the child stays in the same
# session as the login shell. setsid forks a session leader that fully
# detaches; we don't need its pid back since bench-hud-stop.sh finds the
# session via pgrep -Af, not a recorded pid.
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  cd $PI5_REPO &&
  export PATH=\"\$HOME/.pixi/bin:\$PATH\" &&
  setsid -f task platform:robot:launch SET=rpi5-bench > $LOG_FILE 2>&1 < /dev/null
" || die "failed to start the launch on $PI5_HOST"

log "Waiting for vision_node to come up..."
up=false
for _ in $(seq 1 20); do
  if ssh "${SSH_OPTS[@]}" "$PI5_HOST" "pgrep -Af vtitan_vision/vision_node >/dev/null"; then
    up=true
    break
  fi
  sleep 1
done
[ "$up" = true ] || die "vision_node never came up -- check $LOG_FILE on $PI5_HOST"

log "Triggering RACING (short_press)..."
ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  cd $REMOTE_ROBOT &&
  export PATH=\"\$HOME/.pixi/bin:\$PATH\" &&
  pixi run -e vision ros2 topic pub /button/event std_msgs/msg/String '{data: short_press}' --once
" >/dev/null || die "failed to publish short_press"

sleep 2
state="$(ssh "${SSH_OPTS[@]}" "$PI5_HOST" "
  cd $REMOTE_ROBOT &&
  export PATH=\"\$HOME/.pixi/bin:\$PATH\" &&
  timeout 8 pixi run -e vision ros2 topic echo /robot_state --once --qos-reliability best_effort --qos-durability transient_local
" 2>/dev/null | grep -o 'data:.*' | sed "s/data: //; s/[\"']//g")"

[ "$state" = "racing" ] || die "state is '${state:-<empty>}', not 'racing' -- check $LOG_FILE on $PI5_HOST"

log "RACING confirmed, recording is live."
log "Run 'task robot:bench-hud:stop' when done, or use robot:bench-hud:record for a timed one-shot next time."
