#!/usr/bin/env bash
# Pi Zero 2W provisioning — run ON the Pi over SSH.
#
# Prerequisite: flash the card with Raspberry Pi Imager and, in its OS
# customization, set hostname, enable SSH + your public key, set a username +
# password (any name — it's auto-detected via $SUDO_USER, doesn't have to be
# `pi`), and configure WiFi (SSID/password/country). Use the 64-bit image
# (Raspberry Pi OS Lite 64-bit) — pixi/conda-forge has no 32-bit ARM packages.
# Boot the Pi, let it join WiFi, then:
#
#   scp -r scripts <user>@<wifi-ip>:/tmp/        # or git clone first
#   ssh <user>@<wifi-ip> 'sudo bash /tmp/scripts/setup_pi_zero.sh'
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./setup_common.sh
source "$SCRIPT_DIR/setup_common.sh"

REPO_DIR="$TARGET_HOME/voldemorbot"
ROBOT_DIR="$REPO_DIR/platform/robot"
USB_GADGET_IP=192.168.250.1/24   # Pi Zero is the USB gadget; Pi 5 host is .2

# Verify GitHub access up front (private repo) before any heavy work.
require_github_auth

# RPi OS Bookworm/Trixie keep boot config under /boot/firmware.
BOOT_DIR=/boot/firmware
[[ -d "$BOOT_DIR" ]] || BOOT_DIR=/boot

log "Updating packages..."
apt-get update -qq
apt-get full-upgrade -y -qq

log "Adding $TARGET_USER to hardware groups..."
usermod -aG gpio,i2c,spi,dialout "$TARGET_USER"

log "Installing dependencies..."
apt-get install -y -qq git build-essential python3-lgpio python3-pip i2c-tools network-manager

# ---- Hardware interfaces: I2C, hardware PWM, UART, USB-gadget (dwc2) ----
log "Configuring $BOOT_DIR/config.txt overlays..."
config="$BOOT_DIR/config.txt"
# Strip any prior dwc2 overlay first (e.g. a stock dr_mode=host image default) —
# otherwise it survives alongside the plain "dtoverlay=dwc2" appended below.
sed -i '/^dtoverlay=dwc2/d' "$config"
for line in "dtparam=i2c_arm=on" "dtoverlay=dwc2" "enable_uart=1" "dtoverlay=pwm-2chan"; do
    grep -qxF "$line" "$config" || echo "$line" >> "$config"
done

log "Ensuring g_ether USB gadget module loads at boot..."
cmdline="$BOOT_DIR/cmdline.txt"
grep -q "modules-load=dwc2,g_ether" "$cmdline" || sed -i 's/[[:space:]]*$/ modules-load=dwc2,g_ether/' "$cmdline"
# Stable, locally-administered MACs so the host side sees a consistent link.
cat > /etc/modprobe.d/g_ether.conf <<'EOF'
options g_ether dev_addr=02:00:00:00:ce:01 host_addr=02:00:00:00:ce:02
EOF

# ---- usb0 owned solely by NetworkManager (one manager per interface) ----
log "Writing NetworkManager static profile for usb0 ($USB_GADGET_IP)..."
cat > /etc/NetworkManager/system-connections/usb0.nmconnection <<EOF
[connection]
id=usb0
type=ethernet
interface-name=usb0
autoconnect=true

[ipv4]
method=manual
address1=$USB_GADGET_IP

[ipv6]
method=disabled
EOF
chmod 600 /etc/NetworkManager/system-connections/usb0.nmconnection
chown root:root /etc/NetworkManager/system-connections/usb0.nmconnection
nmcli connection reload 2>/dev/null || true

# ---- More swap so pixi/the ROS2 colcon build survives on 512MB RAM ----
if [[ -f /etc/dphys-swapfile ]]; then
    log "Increasing swap to 2GB for the ROS2 build (dphys-swapfile)..."
    sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
    grep -q '^CONF_MAXSWAP=' /etc/dphys-swapfile \
        && sed -i 's/^CONF_MAXSWAP=.*/CONF_MAXSWAP=2048/' /etc/dphys-swapfile \
        || echo 'CONF_MAXSWAP=2048' >> /etc/dphys-swapfile
    dphys-swapfile setup >/dev/null && dphys-swapfile swapon || true
elif [[ ! -f /swapfile ]]; then
    # Newer Pi OS (Bookworm/Trixie) uses rpi-swap's zram+file hybrid instead of
    # dphys-swapfile, but its file-backing only writes back after a long delay
    # (hours), so it doesn't help a short memory-intensive burst like `pixi
    # install`/colcon — a plain always-active swapfile is simpler and reliable.
    log "No dphys-swapfile on this image — adding a plain 2GB swapfile instead..."
    fallocate -l 2G /swapfile
    chmod 600 /swapfile
    mkswap /swapfile >/dev/null
    swapon /swapfile
    grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' >> /etc/fstab
fi

install_pixi
clone_repo "$REPO_DIR"
install_ros_workspace "$ROBOT_DIR"
copy_env "$ROBOT_DIR"

log "Verifying PWM availability..."
ls /sys/class/pwm/pwmchip0/ >/dev/null 2>&1 \
    || log "WARNING: PWM chip not present yet — it appears after the reboot below."

log "Installing systemd service + udev rules..."
install_systemd_unit "$ROBOT_DIR/systemd/voldemorbot-pi-zero.service"
cp "$ROBOT_DIR/udev/99-voldemorbot-gpio.rules" /etc/udev/rules.d/
systemctl daemon-reload
systemctl enable voldemorbot-pi-zero.service
udevadm control --reload-rules
udevadm trigger

log "Provisioning complete. Rebooting to apply overlays + USB gadget..."
reboot
