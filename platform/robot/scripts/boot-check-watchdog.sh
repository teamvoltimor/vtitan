#!/usr/bin/env bash
# Recover the ROS2 stack when it gets stuck in BOOT_CHECK with matched-but-
# silent DDS participants.
#
# The failure this exists for, observed on hardware 2026-08-09: state_machine
# and sllidar_node both discover and MATCH each other correctly on /scan --
# `ros2 topic info /scan --verbose` shows the subscription listed, no error on
# either side -- but no data ever actually crosses. Same signature as the
# already-documented SHM stale-lock bug (docs/dds-shm-transport-disabled.md),
# reproduced even with SHM disabled: confirmed both on a partial restart
# (only vtitan-pi5.service restarted, vtitan-lidar.service left running) and
# on a genuinely fresh simultaneous cold boot of both boards, so it is not
# purely a "partial restart left stale state" bug -- FastDDS participants on
# this hardware can just get stuck matched-but-silent on their own. The only
# fix found that reliably clears it is restarting both ends together.
#
# This runs on the Pi 5, not the Zero: BOOT_CHECK and /robot_state are owned
# by state_machine_node here.
#
# Usage: boot-check-watchdog.sh <robot_dir> <user>
# e.g.:  boot-check-watchdog.sh /home/ralvarezdev/vtitan/platform/robot ralvarezdev

set -uo pipefail

ROBOT_DIR="${1:?usage: boot-check-watchdog.sh <robot_dir> <user>}"
RUN_AS_USER="${2:?usage: boot-check-watchdog.sh <robot_dir> <user>}"

# TRANSIENT_LOCAL means a late subscriber still gets the last published
# value even if BOOT_CHECK finished seconds or minutes ago -- this is not
# racing the publish, a stuck state genuinely has nothing to return.
ROBOT_STATE_TIMEOUT_SEC="${ROBOT_STATE_TIMEOUT_SEC:-8}"

# Three consecutive misses before acting -- mirrors usb-link-watchdog.sh's
# reasoning: a single slow tick under load is not the same as genuinely stuck,
# and restarting the ROS2 stack on a healthy run would cause the very outage
# this prevents.
FAILURES_BEFORE_RECOVERY="${FAILURES_BEFORE_RECOVERY:-3}"

# If something is keeping the stack from ever reaching READY at all (a real
# hardware fault, not a DDS hiccup), every check fails forever -- the cooldown
# keeps that case to one restart attempt every few minutes instead of one per
# tick, so it doesn't itself become the thing pinning the CPU.
COOLDOWN_SEC="${COOLDOWN_SEC:-180}"

# BOOT_CHECK is legitimately still running for the first tens of seconds
# after boot (state_machine_node's own jumper-read timeout alone is 60s) --
# acting during that window would fight the normal startup instead of
# repairing a genuinely stuck one.
MIN_UPTIME_SEC="${MIN_UPTIME_SEC:-90}"

STATE_DIR=/run/vtitan-boot-check-watchdog
FAIL_FILE="$STATE_DIR/failures"
LAST_RECOVERY_FILE="$STATE_DIR/last_recovery"

mkdir -p "$STATE_DIR"

log() { echo "[boot-check-watchdog] $*"; }

now_sec() { awk '{print int($1)}' /proc/uptime; }

read_counter() {
  [ -r "$1" ] && cat "$1" 2>/dev/null || echo 0
}

uptime_sec="$(now_sec)"
if [ "$uptime_sec" -lt "$MIN_UPTIME_SEC" ]; then
  exit 0
fi

if ! systemctl is-active --quiet vtitan-pi5.service; then
  # Not our problem to fix -- vtitan-pi5.service's own Restart=on-failure
  # handles a crashed process. Nothing to check if it isn't even running.
  echo 0 > "$FAIL_FILE"
  exit 0
fi

# pixi run (not a bare PATH-dependent `ros2`) -- matches how every systemd
# unit in this repo invokes ros2, and avoids depending on ambient PATH/profile
# sourcing that runuser does not replicate the same way an interactive SSH
# session does.
state="$(runuser -u "$RUN_AS_USER" -- bash -c "
  cd '$ROBOT_DIR' && \
  ~/.pixi/bin/pixi run -e vision bash -c 'source ros2_ws/install/setup.bash && timeout ${ROBOT_STATE_TIMEOUT_SEC} ros2 topic echo /robot_state --once --qos-reliability best_effort --qos-durability transient_local' 2>/dev/null
" | grep -o "data:.*" | sed "s/data: //; s/[\"']//g")"

if [ "$state" != "boot_check" ]; then
  # Empty/unreadable is treated as "not stuck" here, not "stuck" -- a
  # transient CLI/tooling failure (confirmed elsewhere this session to be
  # unreliable on its own) should not trigger a restart on its own false
  # positive. Only a CONFIRMED boot_check reading counts as evidence.
  echo 0 > "$FAIL_FILE"
  exit 0
fi

failures=$(( $(read_counter "$FAIL_FILE") + 1 ))
echo "$failures" > "$FAIL_FILE"
log "still in boot_check ($failures/$FAILURES_BEFORE_RECOVERY)"

if [ "$failures" -lt "$FAILURES_BEFORE_RECOVERY" ]; then
  exit 0
fi

last_recovery="$(read_counter "$LAST_RECOVERY_FILE")"
if [ "$last_recovery" -gt 0 ] && [ $(( uptime_sec - last_recovery )) -lt "$COOLDOWN_SEC" ]; then
  log "recovery attempted $(( uptime_sec - last_recovery ))s ago, waiting out the ${COOLDOWN_SEC}s cooldown"
  exit 0
fi

log "restarting vtitan-lidar.service + vtitan-pi5.service together (matches the only fix confirmed to clear this on hardware)"
echo "$uptime_sec" > "$LAST_RECOVERY_FILE"
echo 0 > "$FAIL_FILE"
systemctl restart vtitan-lidar.service vtitan-pi5.service
