#!/usr/bin/env bash
# End-to-end health check for the Pi 5 <-> Pi Zero integration: USB-gadget
# link, systemd service, ROS2 cross-board discovery, and an actual
# AckermannDriveStamped publish -> motor driver -> feedback round-trip.
#
# Run this ON Pi 5, after bootstrap-fresh-zero.sh + deploy-dev-env-to-zero.sh
# have set up and shipped code to the Zero, and the service is running.
#
# Safe to run with motors unpowered -- the round-trip check only verifies the
# software position-tracking value updates correctly, not physical motion.
#
# Usage: bash scripts/provisioning/verify-zero-integration.sh
# Override targets: ZERO_HOST_USB=ralvarezdev@192.168.250.1 ZERO_HOST_WIFI=ralvarezdev@192.168.0.51 bash scripts/provisioning/verify-zero-integration.sh

set -uo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROBOT_DIR"

ZERO_HOST_USB="${ZERO_HOST_USB:-ralvarezdev@192.168.250.1}"
ZERO_HOST_WIFI="${ZERO_HOST_WIFI:-ralvarezdev@192.168.0.51}"
SSH_OPTS=(-o ConnectTimeout=8 -o BatchMode=yes -o StrictHostKeyChecking=accept-new)

PASS=0
FAIL=0

ok()   { echo "  [PASS] $*"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
step() { echo ""; echo "== $* =="; }

# 1. SSH reachability -- both paths, USB is what matters for competition
step "1/5 SSH reachability"
usb_reachable=false
wifi_reachable=false
if ssh "${SSH_OPTS[@]}" "$ZERO_HOST_USB" "echo ok" >/dev/null 2>&1; then
  ok "Zero reachable over USB-gadget link ($ZERO_HOST_USB)"
  usb_reachable=true
else
  bad "Zero NOT reachable over USB-gadget link ($ZERO_HOST_USB) -- this is the competition path"
fi
if ssh "${SSH_OPTS[@]}" "$ZERO_HOST_WIFI" "echo ok" >/dev/null 2>&1; then
  ok "Zero reachable over WiFi ($ZERO_HOST_WIFI)"
  wifi_reachable=true
else
  bad "Zero NOT reachable over WiFi ($ZERO_HOST_WIFI)"
fi

# Pick whichever host actually works for the rest of the checks that need SSH
if $usb_reachable; then
  ZERO_HOST="$ZERO_HOST_USB"
elif $wifi_reachable; then
  ZERO_HOST="$ZERO_HOST_WIFI"
else
  echo ""
  echo "Cannot reach the Zero over any path -- skipping remaining checks."
  echo "Result: $PASS passed, $FAIL failed"
  exit 1
fi

# 2. USB-gadget link health (even if WiFi is what's reachable, report USB status)
step "2/5 USB-gadget link health"
usb0_state="$(ip addr show usb0 2>&1)"
if echo "$usb0_state" | grep -q "LOWER_UP"; then
  ok "usb0 interface is UP with carrier (LOWER_UP) on Pi 5"
else
  bad "usb0 interface is down or has no carrier on Pi 5 -- check cable/power"
fi
if ping -c 3 -W 3 192.168.250.1 >/dev/null 2>&1; then
  ok "Zero answers ping on USB-gadget IP (192.168.250.1)"
else
  bad "Zero does NOT answer ping on USB-gadget IP -- link is up but not passing traffic"
fi
# Only count watchdog entries from the CURRENT link session. The gadget link
# is intermittent across Zero reboots -- a dead one leaves a burst of these in
# the ring buffer, and grepping all of dmesg then reports a long-since-fixed
# fault forever (seen: 97 stale entries flagged while the link was demonstrably
# passing traffic at 0.3ms). journalctl -k --since gives a real time window;
# dmesg's own timestamps are seconds-since-boot and awkward to compare.
WATCHDOG_WINDOW="${WATCHDOG_WINDOW:-5 minutes ago}"
recent_watchdog="$(journalctl -k --since "$WATCHDOG_WINDOW" --no-pager 2>/dev/null \
  | grep -c 'cdc_ether.*NETDEV WATCHDOG' || true)"
if [ "$recent_watchdog" -gt 0 ]; then
  bad "Found $recent_watchdog cdc_ether NETDEV WATCHDOG (TX timeout) entries since '$WATCHDOG_WINDOW' -- link-layer fault, not a config issue"
else
  ok "No cdc_ether NETDEV WATCHDOG entries since '$WATCHDOG_WINDOW'"
fi

# 3. systemd service state on the Zero
step "3/5 vtitan-pi-zero.service state"
svc_active="$(ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "sudo systemctl is-active vtitan-pi-zero.service" 2>/dev/null || true)"
svc_enabled="$(ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "sudo systemctl is-enabled vtitan-pi-zero.service" 2>/dev/null || true)"
[ "$svc_active" = "active" ] && ok "Service is active" || bad "Service is NOT active (state: $svc_active)"
[ "$svc_enabled" = "enabled" ] && ok "Service is enabled (auto-starts on boot)" || bad "Service is NOT enabled (state: $svc_enabled)"
exec_line="$(ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "grep ExecStart /etc/systemd/system/vtitan-pi-zero.service" 2>/dev/null || true)"
if echo "$exec_line" | grep -q -- "--as-is"; then
  ok "ExecStart uses --as-is (no fetch/verify overhead on boot)"
else
  bad "ExecStart missing --as-is -- $exec_line"
fi

# 4. ROS2 cross-board discovery
step "4/5 ROS2 topic discovery"
# ROS2/colcon setup scripts reference unset variables internally and are not
# set -u safe, so disable it just for the source.
set +u
# shellcheck disable=SC1091
source ros2_ws/install/setup.bash 2>/dev/null
set -u
topics="$(ROS_DOMAIN_ID=0 timeout 8 ros2 topic list 2>/dev/null || true)"
for t in /ackermann_cmd /motor/steering_position /motor/drive_speed /button/event; do
  if echo "$topics" | grep -qxF "$t"; then
    ok "Topic $t discovered"
  else
    bad "Topic $t NOT discovered -- check ROS_DOMAIN_ID and DDS multicast on the active link"
  fi
done

# 5. Real publish -> motor driver -> feedback round-trip
step "5/5 AckermannDriveStamped round-trip (software-level, safe with motors unpowered)"
TEST_ANGLE_RAD=0.2
TEST_ANGLE_DEG="$(python3 -c "import math; print(round(math.degrees($TEST_ANGLE_RAD), 4))" 2>/dev/null || echo "11.4592")"
ROS_DOMAIN_ID=0 timeout 4 ros2 topic pub -r 5 /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: $TEST_ANGLE_RAD, speed: 0.0}}" >/dev/null 2>&1 &
pub_pid=$!
sleep 3
feedback="$(ROS_DOMAIN_ID=0 timeout 4 ros2 topic echo /motor/steering_position --once 2>/dev/null | grep 'data:' | awk '{print $2}')"
wait "$pub_pid" 2>/dev/null || true

if [ -n "$feedback" ]; then
  diff="$(python3 -c "print(abs($feedback - $TEST_ANGLE_DEG))" 2>/dev/null || echo "999")"
  within_tol="$(python3 -c "print(1 if $diff < 0.5 else 0)" 2>/dev/null || echo "0")"
  if [ "$within_tol" = "1" ]; then
    ok "Feedback matches: commanded ${TEST_ANGLE_DEG}deg, got ${feedback}deg"
  else
    bad "Feedback mismatch: commanded ${TEST_ANGLE_DEG}deg, got ${feedback}deg (diff ${diff})"
  fi
else
  bad "No feedback received on /motor/steering_position -- publish never reached the motor node"
fi

echo ""
echo "=================================="
echo "Result: $PASS passed, $FAIL failed"
echo "=================================="
[ "$FAIL" -eq 0 ]
