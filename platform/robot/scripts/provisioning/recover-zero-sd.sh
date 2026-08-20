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

log "Filesystem is consistent. Safe to unmount this adapter and put the card back in the Pi Zero."
exit 0
