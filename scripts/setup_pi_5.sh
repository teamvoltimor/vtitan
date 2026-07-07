#!/usr/bin/env bash
# Pi 5 provisioning — run ON the Pi over SSH.
#
# Prerequisite: flash with Raspberry Pi Imager (Raspberry Pi OS Lite 64-bit),
# setting hostname, SSH + your public key, a username + password (any name —
# it's auto-detected via $SUDO_USER, doesn't have to be `pi`), and WiFi.
# Boot, then:
#   ssh <user>@<host> 'sudo bash /path/to/setup_pi_5.sh [--hailo-deb /path/hailort.deb]'
#
# The Pi 5 is the USB *host* for the Pi Zero gadget (it just sees usb0 appear),
# so it needs no dwc2/g_ether overlays — only a static IP on usb0.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./setup_common.sh
source "$SCRIPT_DIR/setup_common.sh"

REPO_DIR="$TARGET_HOME/voldemorbot"
ROBOT_DIR="$REPO_DIR/platform/robot"

hailo_deb=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --hailo-deb) hailo_deb="$2"; shift 2 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

# Verify GitHub access up front (private repo) before any heavy work.
require_github_auth

log "Updating packages..."
apt-get update -qq
apt-get full-upgrade -y -qq

log "Enabling interfaces via raspi-config (camera, i2c, spi, serial)..."
raspi-config nonint do_camera 0
raspi-config nonint do_i2c 0
raspi-config nonint do_spi 0
raspi-config nonint do_serial 2   # serial hardware on, login shell off (IMU UART-RVC)

log "Adding $TARGET_USER to hardware groups..."
usermod -aG gpio,i2c,spi,dialout "$TARGET_USER"

log "Installing dependencies..."
apt-get install -y -qq git build-essential python3-pip i2c-tools network-manager

# Hailo AI HAT+ system runtime (firmware + libhailort + service).
if [[ -n "$hailo_deb" ]]; then
    log "Installing HailoRT runtime from $hailo_deb..."
    dpkg -i "$hailo_deb" || apt-get install -y -f -qq
    systemctl enable hailort
fi

# usb0 host side: static IP matching the Pi Zero gadget (Zero=.1, Pi5=.2).
log "Configuring usb0 host static IP (192.168.250.2)..."
nmcli con add type ethernet ifname usb0 \
    ipv4.method manual \
    ipv4.addresses 192.168.250.2/24 \
    ipv6.method disabled \
    connection.id usb0 2>/dev/null || log "usb0 connection already exists"

install_pixi
clone_repo "$REPO_DIR"
install_ros_workspace "$ROBOT_DIR" lidar
copy_env "$ROBOT_DIR"

# Hailo python bindings go INTO the pixi env (where vision_node actually runs),
# not system python — that also sidesteps Trixie's externally-managed pip.
if [[ -n "$hailo_deb" ]]; then
    log "Installing HailoRT python wheel into the pixi dev env..."
    run_as_pi bash -c "cd '$ROBOT_DIR' && '$PIXI_BIN' run -e dev python -m pip install libs/linux_aarch64/hailort-*.whl" \
        || log "WARNING: hailort wheel install failed — check libs/linux_aarch64/"
fi

log "Installing systemd services + udev rules..."
install_systemd_unit "$ROBOT_DIR/systemd/voldemorbot-pi5.service"
install_systemd_unit "$ROBOT_DIR/systemd/voldemorbot-lidar.service"
cp "$ROBOT_DIR/udev/99-voldemorbot-gpio.rules" /etc/udev/rules.d/
systemctl daemon-reload
systemctl enable voldemorbot-lidar.service voldemorbot-pi5.service
udevadm control --reload-rules
udevadm trigger

log "Provisioning complete. Rebooting..."
reboot
