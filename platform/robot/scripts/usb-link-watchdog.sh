#!/usr/bin/env bash
# Recover the usb0 gadget link when it comes up (or falls) half-dead.
#
# The failure this exists for, observed on hardware 2026-08-01: the Pi 5's
# cdc_ether transmit queue stalls --
#
#   cdc_ether 3-2:1.0 usb0: NETDEV WATCHDOG: transmit queue 0 timed out 5052 ms
#
# -- leaving the link carrying traffic gadget->host while host->gadget is dead.
# Both ends still show the interface UP with carrier and correct addressing, so
# nothing short of actually passing traffic detects it. It is nondeterministic:
# of five cold boots observed, four came up fine and one did not.
#
# This runs on the ZERO, not the Pi 5, and that is deliberate. The obvious
# design -- a watchdog on the Pi 5 that SSHes into the Zero to fix it -- needs a
# working network path to the Zero, which in competition may not exist. Here
# the Zero repairs itself with no external dependency at all.
#
# Reloading g_ether on the gadget side is the ONLY remedy that was found to
# work. Verified on hardware, in this order: bouncing usb0 on the Pi 5 (no
# effect), unbinding/rebinding cdc_ether (no effect), forcing USB
# re-enumeration from the host (made it worse -- the device came back but
# `can't set config #1, error -110` left no interface at all). That error is
# the tell: it is the gadget's control endpoint that wedges, so only the gadget
# can clear it. The host side then recovers on its own -- NetworkManager
# reapplies usb0's static address when the device re-enumerates.

set -euo pipefail

PEER="${PEER:-192.168.250.2}"
IFACE="${IFACE:-usb0}"

# Three consecutive misses before acting: a single dropped ping under load (the
# Zero is a 2W and does stall) is not a dead link, and reloading the gadget
# module on a healthy link would cause the very outage this prevents.
FAILURES_BEFORE_RECOVERY="${FAILURES_BEFORE_RECOVERY:-3}"

# If the Pi 5 is simply powered off, every check fails forever. The cooldown
# keeps that case to one reload every few minutes instead of one per tick.
COOLDOWN_SEC="${COOLDOWN_SEC:-180}"

# The link legitimately does not exist for the first moments after boot; acting
# during that window would fight the normal bring-up rather than repair it.
MIN_UPTIME_SEC="${MIN_UPTIME_SEC:-90}"

STATE_DIR=/run/vtitan-usb-link-watchdog
FAIL_FILE="$STATE_DIR/failures"
LAST_RECOVERY_FILE="$STATE_DIR/last_recovery"

mkdir -p "$STATE_DIR"

log() { echo "[usb-link-watchdog] $*"; }

now_sec() { awk '{print int($1)}' /proc/uptime; }

read_counter() {
  [ -r "$1" ] && cat "$1" 2>/dev/null || echo 0
}

uptime_sec="$(now_sec)"
if [ "$uptime_sec" -lt "$MIN_UPTIME_SEC" ]; then
  exit 0
fi

if [ ! -e "/sys/class/net/$IFACE" ]; then
  # No interface at all is a different fault (module not loaded, cable out);
  # a reload is still the best available move, so fall through rather than
  # exiting quietly.
  log "$IFACE does not exist"
else
  if ping -c1 -W2 -I "$IFACE" "$PEER" >/dev/null 2>&1; then
    echo 0 > "$FAIL_FILE"
    exit 0
  fi
fi

failures=$(( $(read_counter "$FAIL_FILE") + 1 ))
echo "$failures" > "$FAIL_FILE"
log "no reply from $PEER over $IFACE ($failures/$FAILURES_BEFORE_RECOVERY)"

if [ "$failures" -lt "$FAILURES_BEFORE_RECOVERY" ]; then
  exit 0
fi

last_recovery="$(read_counter "$LAST_RECOVERY_FILE")"
if [ "$last_recovery" -gt 0 ] && [ $(( uptime_sec - last_recovery )) -lt "$COOLDOWN_SEC" ]; then
  log "recovery attempted $(( uptime_sec - last_recovery ))s ago, waiting out the ${COOLDOWN_SEC}s cooldown"
  exit 0
fi

log "reloading g_ether to clear the wedged gadget"
echo "$uptime_sec" > "$LAST_RECOVERY_FILE"
echo 0 > "$FAIL_FILE"

modprobe -r g_ether || log "modprobe -r g_ether failed (continuing to reload anyway)"
sleep 3
modprobe g_ether

# The host needs a moment to re-enumerate the device and reapply usb0's
# address before the link is testable again.
sleep 8
if ping -c2 -W2 -I "$IFACE" "$PEER" >/dev/null 2>&1; then
  log "link recovered"
else
  log "link still down after reload -- will retry after the cooldown"
fi
