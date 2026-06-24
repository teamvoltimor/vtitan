#!/usr/bin/env bash
# Pi Zero 2W setup: SD card creation and first-boot provisioning.
#
# Usage:
#   create  -- Flash RPi OS Lite to SD card (run from dev machine):
#     sudo bash setup_pi_zero.sh create --device /dev/sdX [--wifi-ssid "..." --wifi-password "..."]
#
#   provision -- Run first-boot setup (run on Pi Zero via SSH, or via firstboot.sh):
#     sudo bash setup_pi_zero.sh provision
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./setup_common.sh
source "$SCRIPT_DIR/setup_common.sh"

REPO_DIR=/home/pi/voldemorbot
ROBOT_DIR="$REPO_DIR/platform/robot"
HOSTNAME_TARGET=ralvarezdev-raspberrypi-zero

# SD card creation (run from dev machine as root)
cmd_create() {
    local device="" wifi_ssid="" wifi_pass=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --device)        device="$2";    shift 2 ;;
            --wifi-ssid)     wifi_ssid="$2"; shift 2 ;;
            --wifi-password) wifi_pass="$2"; shift 2 ;;
            *) echo "Unknown arg: $1" >&2; exit 1 ;;
        esac
    done
    [[ -z "$device" ]] && { echo "--device required" >&2; exit 1; }

    log "Flashing RPi OS Lite (32-bit Bookworm) to $device..."
    log "WARNING: This will ERASE $device. Press Ctrl-C within 5s to abort."
    sleep 5

    # Flash with rpi-imager CLI or direct dd (image must be pre-downloaded)
    if command -v rpi-imager &>/dev/null; then
        rpi-imager --cli --os raspios_lite_armhf --storage "$device"
    else
        echo "rpi-imager not found — flash the image manually, then re-run to configure boot partition." >&2
        exit 1
    fi

    # Mount boot partition
    local boot_mount
    boot_mount=$(mktemp -d)
    mount "${device}1" "$boot_mount"

    # Enable SSH
    touch "$boot_mount/ssh"

    # Hostname
    echo "$HOSTNAME_TARGET" > "$boot_mount/hostname"

    # config.txt additions
    cat >> "$boot_mount/config.txt" <<'EOF'
dtparam=i2c_arm=on
dtoverlay=dwc2
enable_uart=1
dtoverlay=pwm-2chan
EOF

    # cmdline.txt: add USB gadget modules
    sed -i 's/$/ modules-load=dwc2,g_ether/' "$boot_mount/cmdline.txt"

    # WiFi (optional)
    if [[ -n "$wifi_ssid" ]]; then
        cat > "$boot_mount/wpa_supplicant.conf" <<EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=ES

network={
    ssid="$wifi_ssid"
    psk="$wifi_pass"
}
EOF
    fi

    # Copy firstboot script
    cp "$SCRIPT_DIR/setup_pi_zero.sh" "$boot_mount/firstboot.sh"

    umount "$boot_mount"
    rmdir "$boot_mount"

    log "SD card ready. Insert into Pi Zero and boot — it will provision itself on first boot."
}

# First-boot provisioning (run on Pi Zero as root)
cmd_provision() {
    require_root

    log "Expanding root filesystem..."
    raspi-config --expand-rootfs || true

    log "Updating packages..."
    apt-get update -qq
    apt-get full-upgrade -y -qq

    log "Adding pi to hardware groups..."
    usermod -aG gpio,i2c,spi,dialout pi

    log "Installing dependencies..."
    apt-get install -y -qq git build-essential python3-lgpio i2c-tools python3-pip

    log "Verifying PWM..."
    ls /sys/class/pwm/pwmchip0/ || log "WARNING: PWM not available — check dtoverlay=pwm-2chan in config.txt"

    install_pixi
    clone_repo "$REPO_DIR"
    install_ros_workspace "$ROBOT_DIR"
    copy_env "$ROBOT_DIR"

    log "Installing systemd service..."
    cp "$ROBOT_DIR/systemd/voldemorbot-pi-zero.service" /etc/systemd/system/
    cp "$ROBOT_DIR/udev/99-voldemorbot-gpio.rules" /etc/udev/rules.d/
    systemctl daemon-reload
    systemctl enable voldemorbot-pi-zero.service
    udevadm control --reload-rules
    udevadm trigger

    log "Cleaning up firstboot script..."
    rm -f /boot/firstboot.sh

    log "Provisioning complete. Rebooting..."
    reboot
}

case "${1:-}" in
    create)    shift; cmd_create "$@" ;;
    provision) cmd_provision ;;
    *)
        echo "Usage: $0 {create|provision}" >&2
        exit 1
        ;;
esac
