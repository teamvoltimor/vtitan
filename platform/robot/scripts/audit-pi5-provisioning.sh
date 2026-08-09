#!/usr/bin/env bash
# One-off audit: compares the Pi 5's live state against everything the
# common + pi5 Ansible roles are supposed to have configured. Not part of
# the provisioning flow itself -- run by hand to find drift/gaps, the way
# the netplan-eth0/usb0 conflict on the Zero was found (looked healthy at
# every layer except `nmcli device status`, which nothing else checks).
#
# Usage: bash audit-pi5-provisioning.sh   (run ON the Pi 5)

set -uo pipefail

PASS=0
FAIL=0

ok()   { echo "[OK]   $1"; PASS=$((PASS+1)); }
bad()  { echo "[MISS] $1"; FAIL=$((FAIL+1)); }
info() { echo "       $1"; }

section() { echo; echo "=== $1 ==="; }

USER_NAME="${SUDO_USER:-$USER}"

section "common role: GitHub CLI"
command -v gh >/dev/null 2>&1 && ok "gh installed" || bad "gh not installed"
gh auth status >/dev/null 2>&1 && ok "gh authenticated" || bad "gh not authenticated"

section "common role: apt packages"
for pkg in git build-essential python3-pip i2c-tools network-manager tmux; do
  dpkg -s "$pkg" >/dev/null 2>&1 && ok "apt package $pkg" || bad "apt package $pkg missing"
done

section "common role: hardware groups"
for grp in gpio i2c spi dialout video; do
  id -nG "$USER_NAME" | grep -qw "$grp" && ok "user in group $grp" || bad "user NOT in group $grp"
done

section "common role: pixi"
[ -x "$HOME/.pixi/bin/pixi" ] && ok "pixi installed" || bad "pixi missing at ~/.pixi/bin/pixi"

section "common role: repo"
if [ -d "$HOME/vtitan/.git" ]; then
  ok "~/vtitan repo cloned"
  origin=$(git -C "$HOME/vtitan" remote get-url origin 2>/dev/null)
  [ "$origin" = "https://github.com/teamvoltimor/vtitan.git" ] && ok "origin URL correct ($origin)" || bad "origin URL wrong: $origin"
  branch=$(git -C "$HOME/vtitan" rev-parse --abbrev-ref HEAD 2>/dev/null)
  info "on branch/ref: $branch"
else
  bad "~/vtitan repo NOT present"
fi
[ -f "$HOME/vtitan/platform/robot/.env" ] && ok ".env present" || bad ".env missing"

section "common role: udev + journald"
[ -f /etc/udev/rules.d/99-vtitan-gpio.rules ] && ok "udev GPIO rules installed" || bad "udev GPIO rules missing"
[ -f /etc/systemd/journald.conf.d/persistent.conf ] && ok "persistent journald config" || bad "persistent journald config missing"

section "common role: linger (tmux/background survives SSH drop)"
[ -f "/var/lib/systemd/linger/$USER_NAME" ] && ok "linger enabled for $USER_NAME" || bad "linger NOT enabled for $USER_NAME"

section "pi5 role: raspi-config interfaces"
command -v raspi-config >/dev/null 2>&1 && {
  grep -q '^camera_auto_detect=1' /boot/firmware/config.txt 2>/dev/null && ok "camera enabled (config.txt)" || bad "camera not confirmed enabled in config.txt"
  grep -q '^dtparam=i2c_arm=on' /boot/firmware/config.txt 2>/dev/null && ok "i2c enabled (config.txt)" || bad "i2c not confirmed enabled in config.txt"
  grep -q '^dtparam=spi=on' /boot/firmware/config.txt 2>/dev/null && ok "spi enabled (config.txt)" || bad "spi not confirmed enabled in config.txt"
}

section "pi5 role: Hailo"
dpkg -s hailo-all >/dev/null 2>&1 && ok "hailo-all package installed" || bad "hailo-all package missing"
[ -d /usr/local/hailo/models ] && ok "/usr/local/hailo/models exists" || bad "/usr/local/hailo/models missing"
systemctl is-enabled hailort >/dev/null 2>&1 && ok "hailort service enabled" || bad "hailort service not enabled"
systemctl is-active hailort >/dev/null 2>&1 && ok "hailort service active" || bad "hailort service not active"
lsmod | grep -q '^hailo_pci' && ok "hailo_pci kernel module loaded" || bad "hailo_pci kernel module not loaded"
info "kernel: $(uname -r)"
dmesg 2>/dev/null | grep -q 'find_vma.*hailo_vdma_buffer_map\|WARNING.*hailo' && bad "hailo_pci kernel WARNING present in dmesg (driver/kernel version mismatch -- see session notes)"

section "pi5 role: usb0 gadget link"
nmcli -t -f NAME,DEVICE,STATE con show --active 2>/dev/null | grep -q '^usb0:usb0:activated' && ok "usb0 connection active" || bad "usb0 connection NOT active"
usb0_ip=$(nmcli -t -f ipv4.addresses con show usb0 2>/dev/null)
info "usb0 ipv4.addresses: $usb0_ip"
prio=$(nmcli -t -f connection.autoconnect-priority con show usb0 2>/dev/null | cut -d: -f2)
[ "$prio" = "10" ] && ok "usb0 autoconnect-priority set (10)" || bad "usb0 autoconnect-priority not 10 (got: $prio) -- re-run provisioning to apply the netplan-conflict fix"
nmcli -t -f NAME con show 2>/dev/null | grep -qx netplan-eth0 && {
  na=$(nmcli -t -f connection.autoconnect con show netplan-eth0 2>/dev/null | cut -d: -f2)
  [ "$na" = "no" ] && ok "netplan-eth0 autoconnect disabled" || bad "netplan-eth0 STILL set to autoconnect -- can steal usb0, same bug as the Zero"
}

section "pi5 role: LiDAR driver + workspace build"
[ -d "$HOME/vtitan/platform/robot/ros2_ws/src/sllidar_ros2" ] && ok "sllidar_ros2 driver fetched" || bad "sllidar_ros2 driver not found (check ros2_ws/src path if this looks wrong)"
[ -d "$HOME/vtitan/platform/robot/ros2_ws/install" ] && ok "ROS2 workspace built (install/ exists)" || bad "ROS2 workspace not built"

section "pi5 role: mise + Go + backend"
[ -x "$HOME/.local/bin/mise" ] && ok "mise installed" || bad "mise missing"
[ -x "$HOME/vtitan/platform/backend/bin/server" ] && ok "telemetry backend binary built" || bad "telemetry backend binary missing"

section "pi5 role: systemd units"
for unit in vtitan-pi5.service vtitan-lidar.service vtitan-backend.service vtitan-race.service; do
  systemctl is-enabled "$unit" >/dev/null 2>&1 && ok "$unit enabled" || bad "$unit not enabled"
done

section "pi5 role: old worktree cleanup"
for d in voldemorbot-auto-annotator voldemorbot-docs voldemorbot-hailo voldemorbot-hugo-docs voldemorbot-platform voldemorbot-session-backup; do
  [ -d "$HOME/$d" ] && bad "stale worktree ~/$d still present" || ok "no stale ~/$d"
done

section "power (not ansible-managed, but relevant this session)"
throttled=$(vcgencmd get_throttled 2>/dev/null)
info "vcgencmd get_throttled: $throttled"
[ "$throttled" = "throttled=0x0" ] && ok "no under-voltage/throttle history" || bad "under-voltage or throttling occurred at some point (see session notes on PSU/cable)"

echo
echo "=== Summary: $PASS OK, $FAIL missing/attention ==="
