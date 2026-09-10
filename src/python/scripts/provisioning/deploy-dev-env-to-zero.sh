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
# The env and the workspace are shipped INDEPENDENTLY, each gated on its own
# fingerprint, because they are wildly asymmetric: the env tarball is ~859MB
# and changes only when pixi.lock does, while the workspace tarball is ~1MB and
# changes whenever you touch source. Shipping both every time meant a code
# change one line long cost a full transfer and a ~20 minute extraction on the
# Zero. Now the common case moves 1MB.
#
# Run this ON Pi 5 (it needs the local `dev` pixi env + ros2_ws source tree).
# Requires a one-time SSH key from Pi 5's user into the Pi Zero's
# authorized_keys (the pi_zero Ansible role sets this up).
#
# Usage: bash scripts/provisioning/deploy-dev-env-to-zero.sh
# Override target: ZERO_HOST=ralvarezdev@192.168.250.1 bash scripts/provisioning/deploy-dev-env-to-zero.sh
# Force a redeploy even if fingerprints match: DEPLOY_FORCE=1 bash scripts/...

set -euo pipefail

# Ansible's command/shell modules buffer all output until the task finishes
# -- ansible-playbook shows nothing for this step until it's fully done or
# fully failed, which is unhelpful for the slowest step in provisioning
# (colcon build + an ~859MB transfer). Mirroring everything to a log file
# lets a second session `tail -f` real progress independent of however this
# script is invoked (Ansible, or standalone by hand).
LOG_FILE="${LOG_FILE:-$HOME/vtitan-deploy-zero.log}"
exec > >(tee "$LOG_FILE") 2>&1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROBOT_DIR"

ZERO_HOST="${ZERO_HOST:-ralvarezdev@192.168.250.1}"
SSH_OPTS=(-o ConnectTimeout=10)
DEPLOY_FORCE="${DEPLOY_FORCE:-0}"

# Where the Zero records what it currently has. Deliberately in $HOME and not
# inside the checkout: anything written into the repo shows up as an untracked
# file forever, and the provisioning role now does a `git checkout` that is
# happier with a clean tree.
STAMP_ENV="\$HOME/.vtitan-deploy-stamp-env"
STAMP_WS="\$HOME/.vtitan-deploy-stamp-ws"

log() { echo "[deploy-zero] $*"; }

log "Target: $ZERO_HOST"
# shellcheck source=_ssh_preflight.sh
. "$SCRIPT_DIR/_ssh_preflight.sh"
# No interactive fallback: this runs on the Pi 5, sometimes unattended, where a
# password prompt would hang rather than fail. The retries cover the USB-gadget
# link coming up a moment after the Zero boots.
SSH_PREFLIGHT_INTERACTIVE=0 SSH_PREFLIGHT_HINT="One-time setup: copy this Pi's ~/.ssh/id_ed25519.pub into the Zero's ~/.ssh/authorized_keys
(the pi_zero Ansible role does this for you)."   ssh_preflight "$ZERO_HOST" "${SSH_OPTS[@]}" || exit 1

# Fingerprints are taken over file CONTENT, not over the git commit. A commit
# hash would be simpler and wrong: it cannot see uncommitted edits, and
# deploying work-in-progress source is exactly what this script gets used for.
env_fingerprint() {
  sha256sum pixi.lock | cut -d' ' -f1
}

workspace_fingerprint() {
  find ros2_ws/src -type f -not -path '*/__pycache__/*' -print0 \
    | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1
}

remote_stamp() {
  ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "cat $1 2>/dev/null || true"
}

ENV_FP="$(env_fingerprint)"
WS_FP="$(workspace_fingerprint)"
ENV_FP_REMOTE="$(remote_stamp "$STAMP_ENV" | tr -d '[:space:]')"
WS_FP_REMOTE="$(remote_stamp "$STAMP_WS" | tr -d '[:space:]')"

deploy_env=1
deploy_ws=1
if [ "$DEPLOY_FORCE" != "1" ]; then
  [ "$ENV_FP" = "$ENV_FP_REMOTE" ] && deploy_env=0
  [ "$WS_FP" = "$WS_FP_REMOTE" ] && deploy_ws=0
fi

# A new env means new interpreter paths under .pixi/envs/dev, and colcon bakes
# the building env's python into every generated executable's shebang. Ship the
# workspace alongside it rather than leaving one half a generation behind.
if [ "$deploy_env" = "1" ]; then
  deploy_ws=1
fi

if [ "$deploy_env" = "0" ] && [ "$deploy_ws" = "0" ]; then
  log "ALREADY-CURRENT: env and workspace fingerprints both match the Zero, nothing to do"
  exit 0
fi

log "env: $([ "$deploy_env" = 1 ] && echo 'CHANGED, will ship (~859MB)' || echo 'unchanged, skipping (~859MB saved)')"
log "workspace: $([ "$deploy_ws" = 1 ] && echo 'CHANGED, will ship' || echo 'unchanged, skipping')"

log "Note: if vtitan-pi-zero.service is running on the Zero it will compete for CPU/disk with this transfer."
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "systemctl is-active vtitan-pi-zero.service 2>&1" || true

if [ "$deploy_env" = "1" ]; then
  log "pixi install -e dev (this machine)"
  ~/.pixi/bin/pixi install -e dev
fi

if [ "$deploy_ws" = "1" ]; then
  log "colcon build -> ros2_ws/build-zero, ros2_ws/install-zero (isolated from this Pi's own live build)"
  rm -rf ros2_ws/build-zero ros2_ws/install-zero
  ~/.pixi/bin/pixi run -e dev colcon build --symlink-install --merge-install \
    --base-paths ros2_ws/src \
    --build-base ros2_ws/build-zero \
    --install-base ros2_ws/install-zero
fi

TRANSFER=()
if [ "$deploy_env" = "1" ]; then
  log "tar dev env"
  tar czf ~/dev_env.tar.gz -C .pixi/envs dev
  TRANSFER+=(dev_env.tar.gz)
fi
if [ "$deploy_ws" = "1" ]; then
  log "tar ros2_ws build"
  # -h/--dereference: colcon --symlink-install makes data_files (launch files,
  # package.xml, etc.) symlinks back into ros2_ws/src using an *absolute* path.
  # Since both Pis share the same absolute repo path, an un-dereferenced
  # symlink extracted on the Zero silently resolves to the Zero's own (stale)
  # src tree instead of the content just built here -- packaging real file
  # content instead of the symlink is what makes this tarball actually
  # self-contained regardless of whether the Zero's src happens to be in sync.
  tar czhf ~/ros2_ws_zero.tar.gz -C ros2_ws build-zero install-zero
  TRANSFER+=(ros2_ws_zero.tar.gz)
fi

log "transfer to Zero (retrying on drops -- the Zero's link can flake under its own load)"
# Workspace first: it is the small one, so a flaky link gets the cheap transfer
# out of the way before committing to the expensive one.
for f in "${TRANSFER[@]}"; do
  until scp "${SSH_OPTS[@]}" ~/"$f" "$ZERO_HOST:~/$f"; do
    log "  $f transfer failed, retrying in 8s..."
    sleep 8
  done
done

log "extract + atomically swap in on the Zero"
# shellcheck disable=SC2087
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" bash -s -- "$deploy_env" "$deploy_ws" "$ENV_FP" "$WS_FP" <<'REMOTE'
set -euo pipefail
do_env="$1"; do_ws="$2"; env_fp="$3"; ws_fp="$4"
cd ~/vtitan/src/python
ts=$(date +%s)

if [ "$do_env" = "1" ]; then
  rm -rf .pixi/envs/dev_new
  mkdir -p .pixi/envs/dev_new
  tar xzf ~/dev_env.tar.gz -C .pixi/envs/dev_new --strip-components=1
  [ -e .pixi/envs/dev ] && mv .pixi/envs/dev ".pixi/envs/dev_old_$ts"
  mv .pixi/envs/dev_new .pixi/envs/dev
  rm -f ~/dev_env.tar.gz
fi

if [ "$do_ws" = "1" ]; then
  rm -rf ros2_ws/build_new ros2_ws/install_new
  tar xzf ~/ros2_ws_zero.tar.gz -C ros2_ws
  [ -e ros2_ws/build ] && mv ros2_ws/build "ros2_ws/build_old_$ts"
  [ -e ros2_ws/install ] && mv ros2_ws/install "ros2_ws/install_old_$ts"
  mv ros2_ws/build-zero ros2_ws/build
  mv ros2_ws/install-zero ros2_ws/install
  # colcon bakes relative "../build-zero/..." paths into some develop-mode
  # hooks at build time -- keep both names resolvable post-rename.
  ln -sfn build ros2_ws/build-zero
  ln -sfn install ros2_ws/install-zero
  rm -f ~/ros2_ws_zero.tar.gz
fi

# Keep exactly one previous generation. These used to accumulate forever: three
# deploys in one afternoon left 9.3GB of stale dev envs on a 29GB card, at 3.1GB
# each. One generation is enough to roll back by hand; the rest is just the card
# filling up quietly.
for pattern in ".pixi/envs/dev_old_*" "ros2_ws/build_old_*" "ros2_ws/install_old_*"; do
  # shellcheck disable=SC2086
  ls -dt $pattern 2>/dev/null | tail -n +2 | xargs -r rm -rf
done

# Stamps are written LAST, and only here. Writing them before the swap would
# mean a run that died mid-extraction left the Zero half-updated but marked as
# current, so the next run would skip the repair it needed.
[ "$do_env" = "1" ] && printf '%s\n' "$env_fp" > "$HOME/.vtitan-deploy-stamp-env"
[ "$do_ws" = "1" ] && printf '%s\n' "$ws_fp" > "$HOME/.vtitan-deploy-stamp-ws"
echo "Swap complete."
REMOTE

log "verifying package discovery on the Zero"
ssh "${SSH_OPTS[@]}" "$ZERO_HOST" "cd ~/vtitan/src/python && bash -c '. ros2_ws/install/setup.bash && ros2 pkg list | grep vtitan'"

rm -f ~/dev_env.tar.gz ~/ros2_ws_zero.tar.gz
log "DEPLOYED. Restart the Zero's service when ready:"
log "  ssh $ZERO_HOST 'sudo systemctl restart vtitan-pi-zero.service'"
