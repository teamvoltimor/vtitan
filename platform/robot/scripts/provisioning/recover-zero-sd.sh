#!/usr/bin/env bash
# Repair the Pi Zero's SD card filesystem after an unclean power-off (battery
# died, power was cut without safe-shutdown-zero, or the Zero was crash-
# looping on its own hardware watchdog) -- without a full reformat.
#
# Why this exists: safe-shutdown-zero.sh's own header already documents the
# failure mode -- pulling power on a running Zero leaves the ext4 journal
# uncommitted, showing up next boot as free-block/inode count mismatches and
# a stale orphan-file flag. Confirmed on hardware 2026-08-20: a Zero stuck
# showing BOOT_CHECK forever, with no USB-gadget link, WiFi, or Raspberry Pi
# Connect reachability, turned out to be exactly this -- `e2fsck -f -n` on
# the pulled SD card showed real errors ("Inode bitmap differences",
# "Orphan file ... is not clean"), and `e2fsck -f -y` cleared them without
# needing to reflash. This script automates that recovery.
#
# The same incident also cost the Zero its /etc/NetworkManager/system-
# connections/<ssid>.nmconnection profile despite ~100 prior successful
# boots on this exact image -- not a first-boot issue, the same corruption
# claimed that file too. Recovered it by regenerating it from
# /boot/firmware/network-config, which Raspberry Pi Imager's first-boot
# customization writes and leaves in place permanently (unlike the
# nmconnection file, which is themselves runtime state, not the source of
# truth). This script now does that regeneration automatically so a future
# occurrence does not need doing it by hand again. Deliberately does NOT
# hardcode the SSID/PSK anywhere in this script or the repo -- it re-derives
# the profile from network-config on the card itself every time, so no WiFi
# credential material ever lands in git history.
#
# Run this ON Pi 5, with the Zero powered off and its SD card removed and
# plugged into the Pi 5 via a USB microSD adapter (not over SSH to the Zero --
# the whole point is the Zero may not be able to boot far enough to reach).
#
# Usage: bash scripts/provisioning/recover-zero-sd.sh
# Override device: DEVICE=/dev/sdb bash scripts/provisioning/recover-zero-sd.sh

set -uo pipefail

log() { echo "[recover-zero-sd] $*"; }

# Auto-detect: the first removable (RM=1) disk that is NOT the Pi 5's own
# boot media (mmcblk0/nvme), with exactly the boot(vfat)+root(ext4) partition
# layout every Raspberry Pi OS image uses. Override with DEVICE= if more than
# one removable disk is attached (e.g. another USB drive plugged in too).
if [ -n "${DEVICE:-}" ]; then
  device="$DEVICE"
else
  device=""
  while read -r name rm type; do
    [ "$type" = "disk" ] || continue
    [ "$rm" = "1" ] || continue
    case "$name" in
      mmcblk*|nvme*) continue ;;
    esac
    device="/dev/$name"
    break
  done < <(lsblk -d -n -o NAME,RM,TYPE)
fi

if [ -z "$device" ]; then
  echo "ERROR: no removable SD card device found. Is the Zero's card plugged in via a USB adapter?" >&2
  echo "Override with DEVICE=/dev/sdX if it's attached but not auto-detected." >&2
  exit 1
fi

boot_part="${device}1"
root_part="${device}2"

if [ ! -b "$boot_part" ] || [ ! -b "$root_part" ]; then
  echo "ERROR: $device does not have the expected boot+root partition layout ($boot_part, $root_part not found)." >&2
  echo "Refusing to guess -- pass the right device with DEVICE=/dev/sdX." >&2
  exit 1
fi

log "Target device: $device ($boot_part=boot, $root_part=root)"

# fsck refuses to touch a mounted filesystem -- unmount anything this script
# (or an earlier manual check) left mounted before running.
for part in "$boot_part" "$root_part"; do
  if mount | grep -q "^$part "; then
    log "Unmounting $part first"
    sudo umount "$part"
  fi
done

log "Checking boot partition ($boot_part, FAT32)"
sudo fsck.fat -a "$boot_part"
boot_status=$?

log "Checking root partition ($root_part, ext4) -- this recovers the journal and clears orphaned inodes"
sudo e2fsck -f -y "$root_part"
root_status=$?

echo ""
log "Result: boot fsck exit=$boot_status, root fsck exit=$root_status"

# e2fsck exit code bit 4 (value 4) means errors were left uncorrected even
# with -y -- everything below that (0/1/2) means clean or already fixed.
if [ $((root_status & 4)) -ne 0 ] || [ $((boot_status & 4)) -ne 0 ]; then
  echo "WARNING: uncorrected errors remain -- this card may need a full reformat/reflash." >&2
  exit 1
fi

log "Filesystem is consistent."

# Restore any WiFi connection profile(s) missing from system-connections/ --
# regenerated from network-config, never from a value stored in this repo.
BOOT_MNT="/mnt/vtitan-zero-boot-recover"
ROOT_MNT="/mnt/vtitan-zero-root-recover"
sudo mkdir -p "$BOOT_MNT" "$ROOT_MNT"
sudo mount -o ro "$boot_part" "$BOOT_MNT"
sudo mount -o rw "$root_part" "$ROOT_MNT"

network_config="$BOOT_MNT/network-config"
conn_dir="$ROOT_MNT/etc/NetworkManager/system-connections"

if [ -f "$network_config" ] && [ -d "$conn_dir" ]; then
  log "Checking for missing WiFi connection profiles against $network_config"
  sudo python3 - "$network_config" "$conn_dir" <<'PYEOF'
import re
import sys
import uuid

network_config_path, conn_dir = sys.argv[1], sys.argv[2]
text = open(network_config_path, encoding="utf-8").read()

# Deliberately a small hand-rolled parser, not PyYAML (not guaranteed present
# on a bare Pi image) -- matches the fixed shape Raspberry Pi Imager writes:
# wifis: <ifname>: access-points: "<ssid>": password: "<psk-or-passphrase>"
ssid_re = re.compile(r'^\s+"([^"]+)":\s*$')
password_re = re.compile(r'^\s+password:\s*"([^"]*)"\s*$')

pairs = []
in_access_points = False
pending_ssid = None
for line in text.splitlines():
    if re.match(r"^\s*access-points:\s*$", line):
        in_access_points = True
        continue
    if not in_access_points:
        continue
    m = ssid_re.match(line)
    if m:
        pending_ssid = m.group(1)
        continue
    m = password_re.match(line)
    if m and pending_ssid is not None:
        pairs.append((pending_ssid, m.group(1)))
        pending_ssid = None
        continue
    # Any less-indented line ends the access-points block.
    if line and not line[0].isspace():
        in_access_points = False

if not pairs:
    print("No wifis/access-points found in network-config -- nothing to restore.")
    sys.exit(0)

for ssid, psk in pairs:
    existing = None
    try:
        import os
        for fname in os.listdir(conn_dir):
            if not fname.endswith(".nmconnection"):
                continue
            contents = open(os.path.join(conn_dir, fname), encoding="utf-8").read()
            if f"ssid={ssid}" in contents:
                existing = fname
                break
    except FileNotFoundError:
        pass

    if existing:
        print(f"SSID '{ssid}' already has a profile ({existing}) -- leaving it alone.")
        continue

    profile_path = f"{conn_dir}/{ssid}.nmconnection"
    profile = f"""[connection]
id={ssid}
uuid={uuid.uuid4()}
type=wifi
autoconnect=true
interface-name=wlan0

[wifi]
mode=infrastructure
ssid={ssid}

[wifi-security]
key-mgmt=wpa-psk
psk={psk}

[ipv4]
method=auto

[ipv6]
addr-gen-mode=default
method=auto

[proxy]
"""
    with open(profile_path, "w", encoding="utf-8") as f:
        f.write(profile)
    import os
    os.chmod(profile_path, 0o600)
    print(f"Restored missing profile for SSID '{ssid}' -> {profile_path}")
PYEOF
else
  log "No network-config or system-connections dir found -- skipping WiFi profile check"
fi

# Restore the udev override that keeps usb0 out of NetworkManager's default
# "unmanage every USB gadget device" rule (/usr/lib/udev/rules.d/85-nm-
# unmanaged.rules). Confirmed on hardware 2026-08-20: without this override,
# usb0 sits permanently unmanaged/down -- dwc2/g_ether enumerate cleanly on
# both boards, there is no USB-level fault, but NetworkManager never
# activates the usb0 connection profile, so the link looks completely dead
# from either side. Same corruption pattern as the WiFi profile above: this
# override existed once (see ansible/roles/pi_zero/tasks/main.yml, which now
# also provisions it on a fresh flash) and was lost the same way.
udev_rules_dir="$ROOT_MNT/etc/udev/rules.d"
udev_override="$udev_rules_dir/99-usb0-managed.rules"
if [ -d "$udev_rules_dir" ] && [ ! -f "$udev_override" ]; then
  log "Restoring missing usb0 NetworkManager-managed udev override"
  sudo tee "$udev_override" > /dev/null <<'EOF'
# Override 85-nm-unmanaged.rules' blanket USB-gadget rule (ENV{DEVTYPE}=="gadget"
# -> NM_UNMANAGED=1) for this specific interface, so NetworkManager actually
# activates the usb0 connection profile instead of leaving the device
# permanently unmanaged/down.
SUBSYSTEM=="net", ACTION=="add|change|move", ENV{INTERFACE}=="usb0", ENV{NM_UNMANAGED}="0"
EOF
  sudo chmod 644 "$udev_override"
elif [ -f "$udev_override" ]; then
  log "usb0 udev override already present -- leaving it alone"
else
  log "No /etc/udev/rules.d found -- skipping usb0 udev override check"
fi

sync
sudo umount "$BOOT_MNT" "$ROOT_MNT"
sudo rmdir "$BOOT_MNT" "$ROOT_MNT"

log "Done. Safe to unmount this adapter and put the card back in the Pi Zero."
exit 0
