#!/usr/bin/env bash
# One-shot setup for a Bluetooth gamepad (e.g. 8BitDo Ultimate 2) used for
# joystick bench-testing via joy_teleop_node -- see docs/joystick-teleop.md.
# Safe to re-run (idempotent): trust/pairing steps are skipped if already done.
#
# What this covers:
#   1. Ensure BlueZ (bluetoothctl) is installed
#   2. Add this user to the `input` group (needed to read /dev/input/js* without sudo)
#   3. Power on the Bluetooth radio and register the pairing agent
#   4. Pair + trust + connect the controller (scans and prompts for a match
#      unless CONTROLLER_MAC is given, which skips scanning entirely)
#   5. Verify a joystick device node appears
#
# Run this ON Pi 5 (the board with a Bluetooth radio the controller pairs to).
#
# Usage:
#   Put the controller in Bluetooth pairing mode first (8BitDo Ultimate 2:
#   hold the pair button until the LED flashes rapidly), then:
#     bash scripts/setup-joystick.sh
#   Or skip the scan if you already know the MAC (see `bluetoothctl devices`):
#     CONTROLLER_MAC=AA:BB:CC:DD:EE:FF bash scripts/setup-joystick.sh
#
# After this script: `pixi run -e dev build-ws` (only if you haven't already
# built since joy_teleop_node was added), then `pixi run drive-controller`.

set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROBOT_DIR"

SCAN_TIMEOUT_S="${SCAN_TIMEOUT_S:-20}"
DEVICE_WAIT_TIMEOUT_S="${DEVICE_WAIT_TIMEOUT_S:-15}"

log() { echo "[setup-joystick] $*"; }

log "1/5 Checking BlueZ (bluetoothctl) is installed"
if ! command -v bluetoothctl >/dev/null 2>&1; then
  log "Installing bluez..."
  sudo apt update && sudo apt install -y bluez
fi

log "2/5 Ensuring $USER is in the 'input' group (read access to /dev/input/js*)"
if ! id -nG "$USER" | grep -qw input; then
  sudo usermod -a -G input "$USER"
  log "Added -- log out/in (or reboot) for the group change to take effect."
else
  log "Already in 'input' group."
fi

log "3/5 Powering on Bluetooth radio + registering pairing agent"
bluetoothctl power on
bluetoothctl agent on
bluetoothctl default-agent

if [ -z "${CONTROLLER_MAC:-}" ]; then
  log "4/5 Scanning for the controller (${SCAN_TIMEOUT_S}s) -- make sure it's in pairing mode now"
  bluetoothctl --timeout "$SCAN_TIMEOUT_S" scan on || true
  echo
  echo "Devices seen:"
  bluetoothctl devices
  echo
  read -rp "Enter the controller's MAC address from the list above: " CONTROLLER_MAC
fi

if [ -z "$CONTROLLER_MAC" ]; then
  echo "ERROR: no MAC address given -- re-run with the controller in pairing mode, or set CONTROLLER_MAC=AA:BB:CC:DD:EE:FF" >&2
  exit 1
fi

log "4/5 Pairing + trusting + connecting $CONTROLLER_MAC"
bluetoothctl pair "$CONTROLLER_MAC" || log "  (already paired, continuing)"
bluetoothctl trust "$CONTROLLER_MAC"
bluetoothctl connect "$CONTROLLER_MAC"

log "5/5 Waiting up to ${DEVICE_WAIT_TIMEOUT_S}s for a joystick device node (/dev/input/js*)"
waited=0
until compgen -G "/dev/input/js*" >/dev/null 2>&1; do
  if [ "$waited" -ge "$DEVICE_WAIT_TIMEOUT_S" ]; then
    echo "ERROR: no /dev/input/js* appeared. Check 'bluetoothctl info $CONTROLLER_MAC' shows Connected: yes." >&2
    exit 1
  fi
  sleep 1
  waited=$((waited + 1))
done

log "Done. Found: $(compgen -G '/dev/input/js*')"
log "Next: pixi run -e dev build-ws (if not already built since joy_teleop_node was added),"
log "then pixi run drive-controller. Verify axis/button indices with:"
log "  ros2 launch voldemorbot_bringup joy_teleop_launch.py &  ros2 topic echo /joy"
