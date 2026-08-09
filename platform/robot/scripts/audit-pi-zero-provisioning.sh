#!/usr/bin/env bash
# One-off audit: compares the Pi Zero's live state against everything the
# common + pi_zero Ansible roles are supposed to have configured. Mirrors
# audit-pi5-provisioning.sh's approach and rationale -- run by hand to find
# drift/gaps that look healthy at every layer except the one that actually
# matters (see the netplan-eth0/usb0 conflict this was written to catch).
#
# Usage: bash audit-pi-zero-provisioning.sh   (run ON the Pi Zero)

set -uo pipefail

PASS=0
FAIL=0

ok()   { echo "[OK]   $1"; PASS=$((PASS+1)); }
bad()  { echo "[MISS] $1"; FAIL=$((FAIL+1)); }
info() { echo "       $1"; }

section() { echo; echo "=== $1 ==="; }

USER_NAME="${SUDO_USER:-$USER}"
BOOT_DIR="/boot/firmware"
[ -d "$BOOT_DIR" ] || BOOT_DIR="/boot"

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

section "common role: UDP socket buffers (DDS cross-board delivery)"
for setting in net.core.rmem_max net.core.rmem_default net.core.wmem_max net.core.wmem_default; do
  val=$(cat "/proc/sys/${setting//./\/}" 2>/dev/null)
  [ "$val" = "8388608" ] && ok "$setting = 8388608" || bad "$setting not raised (got: $val) -- Pi5<->Zero DDS delivery will silently fail under load, see ansible/roles/common/tasks/main.yml"
done

section "pi_zero role: apt package"
dpkg -s python3-lgpio >/dev/null 2>&1 && ok "python3-lgpio installed" || bad "python3-lgpio missing"

section "pi_zero role: gadget-link overlay + cmdline"
grep -q '^dtoverlay=dwc2,dr_mode=peripheral$' "$BOOT_DIR/config.txt" 2>/dev/null && ok "dwc2 peripheral overlay present" || bad "dwc2 peripheral overlay missing from $BOOT_DIR/config.txt"
grep '^dtoverlay=dwc2' "$BOOT_DIR/config.txt" 2>/dev/null | grep -qv 'dr_mode=peripheral$' && bad "a non-peripheral dtoverlay=dwc2 line also present -- should have been stripped"
grep -q 'modules-load=dwc2,g_ether' "$BOOT_DIR/cmdline.txt" 2>/dev/null && ok "modules-load=dwc2,g_ether present in cmdline.txt" || bad "modules-load=dwc2,g_ether missing from cmdline.txt"
[ "$(tr -d '\n' < "$BOOT_DIR/cmdline.txt" 2>/dev/null | wc -l)" = "0" ] && ok "cmdline.txt is a single line" || bad "cmdline.txt has multiple lines -- kernel silently ignores everything after the first"

section "pi_zero role: g_ether fixed MACs"
if [ -f /etc/modprobe.d/g_ether.conf ]; then
  ok "/etc/modprobe.d/g_ether.conf present"
  info "$(cat /etc/modprobe.d/g_ether.conf)"
else
  bad "/etc/modprobe.d/g_ether.conf MISSING -- MAC will randomize every boot (confirmed root cause of a session-long link outage)"
fi
dev_addr=$(cat /sys/module/g_ether/parameters/dev_addr 2>/dev/null)
[ "$dev_addr" = "02:00:00:00:ce:01" ] && ok "g_ether dev_addr applied (02:00:00:00:ce:01)" || bad "g_ether dev_addr not applied (got: $dev_addr) -- module needs reload or reboot"

section "pi_zero role: usb0 gadget static IP"
lsmod | grep -q '^g_ether' && ok "g_ether module loaded" || bad "g_ether module not loaded"
nmcli -t -f NAME,DEVICE,STATE con show --active 2>/dev/null | grep -q '^usb0:usb0:activated' && ok "usb0 connection active" || bad "usb0 connection NOT active -- check for a netplan-eth0 (or similar) profile claiming the device instead"
usb0_ip=$(nmcli -t -f ipv4.addresses con show usb0 2>/dev/null | cut -d: -f2)
info "usb0 ipv4.addresses: $usb0_ip"
[ "$usb0_ip" = "192.168.250.1/24" ] && ok "usb0 static IP correct" || bad "usb0 static IP wrong or unset (got: $usb0_ip)"
prio=$(nmcli -t -f connection.autoconnect-priority con show usb0 2>/dev/null | cut -d: -f2)
[ "$prio" = "10" ] && ok "usb0 autoconnect-priority set (10)" || bad "usb0 autoconnect-priority not 10 (got: $prio)"
nmcli -t -f NAME con show 2>/dev/null | grep -qx netplan-eth0 && {
  na=$(nmcli -t -f connection.autoconnect con show netplan-eth0 2>/dev/null | cut -d: -f2)
  [ "$na" = "no" ] && ok "netplan-eth0 autoconnect disabled" || bad "netplan-eth0 STILL set to autoconnect -- this is the exact bug that caused the session-long half-dead-link investigation"
}

section "pi_zero role: PWM + I2C overlays"
grep -q '^dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4$' "$BOOT_DIR/config.txt" 2>/dev/null && ok "hardware PWM overlay present" || bad "hardware PWM overlay missing/mismatched in config.txt"
grep -q '^dtparam=i2c_arm=on$' "$BOOT_DIR/config.txt" 2>/dev/null && ok "i2c_arm overlay present" || bad "i2c_arm overlay missing"
grep -q '^enable_uart=1$' "$BOOT_DIR/config.txt" 2>/dev/null && ok "enable_uart overlay present" || bad "enable_uart overlay missing"
[ -f /etc/modules-load.d/i2c-dev.conf ] && ok "i2c-dev modules-load.d entry present" || bad "i2c-dev modules-load.d entry missing"
lsmod | grep -q '^i2c_dev' && ok "i2c-dev module loaded" || bad "i2c-dev module not loaded"
[ -d /sys/class/pwm/pwmchip0 ] && ok "PWM chip present" || bad "PWM chip not present (needs a reboot after the overlay lands)"

section "pi_zero role: swap"
if [ -f /etc/dphys-swapfile ]; then
  swapsize=$(grep '^CONF_SWAPSIZE=' /etc/dphys-swapfile 2>/dev/null | cut -d= -f2)
  [ "$swapsize" = "2048" ] && ok "dphys-swapfile CONF_SWAPSIZE=2048" || bad "dphys-swapfile CONF_SWAPSIZE wrong (got: $swapsize)"
elif [ -f /swapfile ]; then
  ok "plain /swapfile present"
else
  bad "no swap configured at all (dphys-swapfile or /swapfile)"
fi
# /proc/swaps directly, not `swapon --show` -- swapon lives in /sbin, which
# isn't on a plain non-interactive SSH PATH, so the command silently
# "succeeds" with no output and this reported a false "swap not active"
# on a board where swap was genuinely on (confirmed via `free -h`).
[ "$(tail -n +2 /proc/swaps 2>/dev/null | wc -l)" -gt 0 ] && ok "swap is active" || bad "swap not active"

section "pi_zero role: SSH trust with the Pi 5"
[ -f "$HOME/.ssh/id_ed25519.pub" ] && ok "Zero has its own SSH keypair" || bad "Zero has no SSH keypair"
grep -q "ralvarezdev-raspberrypi$" "$HOME/.ssh/authorized_keys" 2>/dev/null && ok "Pi 5's key trusted in Zero's authorized_keys" || bad "Pi 5's key NOT found in Zero's authorized_keys"

section "pi_zero role: dev env deployed from the Pi 5"
[ -d "$HOME/vtitan/platform/robot/ros2_ws/install" ] && ok "ROS2 workspace present (shipped from Pi 5)" || bad "ROS2 workspace missing -- deploy-dev-env-to-zero.sh has not run/succeeded"
[ -d "$HOME/.pixi" ] && ok "pixi env directory present" || bad "pixi env directory missing"

section "pi_zero role: watchdog + service units"
for unit in vtitan-usb-link-watchdog.service vtitan-usb-link-watchdog.timer vtitan-pi-zero.service; do
  systemctl list-unit-files "$unit" >/dev/null 2>&1 && [ -n "$(systemctl list-unit-files "$unit" 2>/dev/null | grep "$unit")" ] && ok "$unit installed" || bad "$unit not installed"
done
systemctl is-enabled vtitan-usb-link-watchdog.timer >/dev/null 2>&1 && ok "usb-link-watchdog timer enabled" || bad "usb-link-watchdog timer not enabled"
systemctl is-enabled vtitan-pi-zero.service >/dev/null 2>&1 && ok "vtitan-pi-zero.service enabled" || bad "vtitan-pi-zero.service not enabled"

echo
echo "=== Summary: $PASS OK, $FAIL missing/attention ==="
