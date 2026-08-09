#!/usr/bin/env bash
# Waits (bounded) for the usb0 gadget link to the Pi Zero to actually pass
# traffic before returning, so vtitan-pi5.service's ROS2 stack doesn't start
# racing the Zero's own boot/USB re-enumeration.
#
# Why this exists: a Pi 5 reboot power-cycles the Zero too -- it draws power
# over the same USB gadget cable, so there is no such thing as "just the
# Pi 5 rebooted" on this hardware. Every Pi 5 boot is a simultaneous cold
# boot of both boards, and the gadget link has to fully re-enumerate from
# nothing while both ROS2 stacks are also starting. Without this wait,
# vtitan-pi5.service's state_machine_node begins its own 60s jumper-read
# timeout immediately, racing (and often losing to) the link's own settle
# time -- confirmed on hardware 2026-08-09: the jumper read timed out and
# defaulted on a cold dual-boot even though the exact same setup worked
# cleanly seconds later once the link was already stable.
#
# Deliberately does NOT fail/block startup forever if the link never comes
# up: this is a head start for the common case, not a hard dependency. The
# Zero's own peripherals (motors, button, OLED) are correctly independent of
# network per vtitan-pi-zero.service's own comment -- this script only ever
# runs on the Pi 5 side, which is the one that actually needs the Zero's data.

set -uo pipefail

PEER="${PEER:-192.168.250.1}"
IFACE="${IFACE:-usb0}"
MAX_WAIT_SEC="${MAX_WAIT_SEC:-55}"
CHECK_INTERVAL_SEC="${CHECK_INTERVAL_SEC:-2}"

log() { echo "[wait-for-gadget-link] $*"; }

start=$(awk '{print int($1)}' /proc/uptime)
while true; do
  if ping -c1 -W2 -I "$IFACE" "$PEER" >/dev/null 2>&1; then
    log "$PEER reachable over $IFACE, continuing"
    exit 0
  fi
  now=$(awk '{print int($1)}' /proc/uptime)
  if [ $(( now - start )) -ge "$MAX_WAIT_SEC" ]; then
    log "$PEER still unreachable over $IFACE after ${MAX_WAIT_SEC}s -- continuing anyway, not blocking startup"
    exit 0
  fi
  sleep "$CHECK_INTERVAL_SEC"
done
