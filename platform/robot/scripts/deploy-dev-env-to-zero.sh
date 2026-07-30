#!/usr/bin/env bash
# Build the `dev` pixi env + ros2_ws on Pi 5 and ship it to the Pi Zero as
# tarballs, instead of running `pixi install`/`colcon build` directly on the
# Zero.
#
# Why this exists: the Pi Zero 2 W has ~415MB usable RAM. Resolving/installing
# a full ROS2 Kilted + robostack `dev` environment there directly swap-thrashes
# the SD card hard enough to spike load average into the double digits, make
# SSH unresponsive for many minutes at a time, and in the worst case crash the
# board with an unclean shutdown (real filesystem-corruption risk on repeat).
# Pi 5 has 15GB+ RAM and does the same work in under a minute. See
# docs/sensor-verification.md's "Pi Zero deployment" section for the full
# incident writeup.
#
# Run this ON Pi 5 (it needs the local `dev` pixi env + ros2_ws source tree).
# Requires a one-time SSH key from Pi 5's user into the Pi Zero's
# authorized_keys (see docs/sensor-verification.md) -- this script will not
# set that up for you since it needs an interactive password once.
#
# Usage: bash scripts/deploy-dev-env-to-zero.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.250.1 bash scripts/deploy-dev-env-to-zero.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROBOT_DIR"

ZERO_HOST="${ZERO_HOST:-ralvarezdev@192.168.250.1}"
SSH_OPTS=(-o ConnectTimeout=10)

log() { echo "[deploy-zero] $*"; }

log "Target: $ZERO_HOST"
# shellcheck source=scripts/_ssh_preflight.sh
. "$SCRIPT_DIR/_ssh_preflight.sh"
# No interactive fallback: this runs on the Pi 5, sometimes unattended, where a
# password prompt would hang rather than fail. The retries cover the USB-gadget
# link coming up a moment after the Zero boots.
SSH_PREFLIGHT_INTERACTIVE=0 SSH_PREFLIGHT_HINT="One-time setup: copy this Pi's ~/.ssh/id_ed25519.pub into the Zero's ~/.ssh/authorized_keys
(see docs/sensor-verification.md's Pi Zero deployment section)."   ssh_preflight "$ZERO_HOST" "${SSH_OPTS[@]}" || exit 1

log "Warning: if vtitan-pi-zero.service is currently enabled/running on the Zero, stop it first --"
log "it will fight this transfer for CPU/disk I/O on the same constrained hardware."
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "systemctl is-active vtitan-pi-zero.service 2>&1" || true

log "1/6 pixi install -e dev (this machine)"
~/.pixi/bin/pixi install -e dev

log "2/6 colcon build -> ros2_ws/build-zero, ros2_ws/install-zero (isolated from this Pi's own live build)"
rm -rf ros2_ws/build-zero ros2_ws/install-zero
~/.pixi/bin/pixi run -e dev colcon build --symlink-install --merge-install \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build-zero \
  --install-base ros2_ws/install-zero

log "3/6 tar dev env + ros2_ws build for transfer"
tar czf ~/dev_env.tar.gz -C .pixi/envs dev
# -h/--dereference: colcon --symlink-install makes data_files (launch files,
# package.xml, etc.) symlinks back into ros2_ws/src using an *absolute* path.
# Since both Pis share the same absolute repo path, an un-dereferenced
# symlink extracted on the Zero silently resolves to the Zero's own (stale)
# src tree instead of the content just built here -- packaging real file
# content instead of the symlink is what makes this tarball actually
# self-contained regardless of whether the Zero's src happens to be in sync.
tar czhf ~/ros2_ws_zero.tar.gz -C ros2_ws build-zero install-zero

log "4/6 transfer tarballs to Zero (retrying on drops -- the Zero's link can flake under its own load)"
for f in ros2_ws_zero.tar.gz dev_env.tar.gz; do
  until scp "${SSH_OPTS[@]}" ~/"$f" "$ZERO_HOST:~/$f"; do
    log "  $f transfer failed, retrying in 8s..."
    sleep 8
  done
done

log "5/6 extract + atomically swap in on the Zero"
# shellcheck disable=SC2087
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" bash -s <<'REMOTE'
set -euo pipefail
cd ~/vtitan/platform/robot
rm -rf .pixi/envs/dev_new ros2_ws/build_new ros2_ws/install_new
mkdir -p .pixi/envs/dev_new
tar xzf ~/dev_env.tar.gz -C .pixi/envs/dev_new --strip-components=1
tar xzf ~/ros2_ws_zero.tar.gz -C ros2_ws

ts=$(date +%s)
[ -e .pixi/envs/dev ] && mv .pixi/envs/dev ".pixi/envs/dev_old_$ts"
mv .pixi/envs/dev_new .pixi/envs/dev

[ -e ros2_ws/build ] && mv ros2_ws/build "ros2_ws/build_old_$ts"
[ -e ros2_ws/install ] && mv ros2_ws/install "ros2_ws/install_old_$ts"
mv ros2_ws/build-zero ros2_ws/build
mv ros2_ws/install-zero ros2_ws/install

# colcon bakes relative "../build-zero/..." paths into some develop-mode
# hooks at build time -- keep both names resolvable post-rename.
ln -sfn build ros2_ws/build-zero
ln -sfn install ros2_ws/install-zero

rm -f ~/dev_env.tar.gz ~/ros2_ws_zero.tar.gz
echo "Swap complete. Old dirs kept as *_old_$ts -- remove manually once verified."
REMOTE

log "6/6 verifying package discovery on the Zero"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "cd ~/vtitan/platform/robot && bash -c '. ros2_ws/install/setup.bash && ros2 pkg list | grep vtitan'"

rm -f ~/dev_env.tar.gz ~/ros2_ws_zero.tar.gz
log "Done. Re-enable/start vtitan-pi-zero.service on the Zero when ready:"
log "  ssh $ZERO_HOST 'sudo systemctl enable --now vtitan-pi-zero.service'"
