#!/usr/bin/env bash
# Pi 5 provisioning script.
#
# Usage (run on Pi 5 as root, or via SSH):
#   sudo bash setup_pi_5.sh [--hailo-deb /path/to/hailort.deb]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./setup_common.sh
source "$SCRIPT_DIR/setup_common.sh"

REPO_DIR=/home/pi/voldemorbot
ROBOT_DIR="$REPO_DIR/platform/robot"

require_root

hailo_deb=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --hailo-deb) hailo_deb="$2"; shift 2 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

log "Updating packages..."
apt-get update -qq
apt-get full-upgrade -y -qq

log "Enabling interfaces via raspi-config..."
raspi-config nonint do_camera 0
raspi-config nonint do_i2c 0
raspi-config nonint do_spi 0
raspi-config nonint do_serial 2

log "Adding pi to hardware groups..."
usermod -aG gpio,i2c,spi,dialout pi

log "Installing dependencies..."
apt-get install -y -qq git build-essential i2c-tools network-manager rpi-usb-gadget

# Hailo AI HAT+ runtime
if [[ -n "$hailo_deb" ]]; then
    log "Installing HailoRT from $hailo_deb..."
    dpkg -i "$hailo_deb"
    pip install "$ROBOT_DIR/libs/linux_aarch64/hailort-"*.whl
    systemctl enable hailort
else
    log "No --hailo-deb provided — skipping HailoRT install"
fi

# USB gadget ethernet (Pi Zero at 10.250.250.1)
log "Configuring USB gadget ethernet (usb0 @ 10.250.250.2)..."
nmcli con add type ethernet ifname usb0 \
    ipv4.method manual \
    ipv4.addresses 10.250.250.2/24 \
    connection.id usb-gadget 2>/dev/null || log "usb-gadget connection already exists"

install_pixi
clone_repo "$REPO_DIR"
install_ros_workspace "$ROBOT_DIR"
copy_env "$ROBOT_DIR"

log "Installing systemd services..."
cp "$ROBOT_DIR/systemd/voldemorbot-pi5.service" /etc/systemd/system/
cp "$ROBOT_DIR/systemd/voldemorbot-lidar.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable voldemorbot-lidar.service voldemorbot-pi5.service

log "Provisioning complete. Rebooting..."
reboot
