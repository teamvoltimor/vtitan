#!/usr/bin/env bash
# Shared helpers for setup_pi_zero.sh and setup_pi_5.sh.
#
# Model: the OS image, hostname, SSH key, user account and WiFi are written by
# Raspberry Pi Imager (Windows). These scripts run ON the target Pi over SSH and
# only configure the voldemorbot-specific layer (interfaces, USB gadget, repo,
# ROS2 workspace, services). They never flash or partition anything.
set -euo pipefail

# pixi is installed per-user under the pi account; reference it by absolute path
# so it works from non-interactive (sudo) shells that don't source ~/.bashrc.
PIXI_BIN=/home/pi/.pixi/bin/pixi

log() { echo "[setup] $*"; }

require_root() {
    if [[ $EUID -ne 0 ]]; then
        echo "This script must be run as root (sudo)." >&2
        exit 1
    fi
}

# Run a command as the pi user with pi's HOME (so ~/.pixi etc. resolve).
run_as_pi() {
    if [[ $EUID -eq 0 ]]; then
        sudo -H -u pi "$@"
    else
        "$@"
    fi
}

install_pixi() {
    if [[ -x "$PIXI_BIN" ]]; then
        log "pixi already installed: $(run_as_pi "$PIXI_BIN" --version)"
        return
    fi
    log "Installing pixi for user pi..."
    run_as_pi bash -c 'curl -fsSL https://pixi.sh/install.sh | bash'
}

install_gh() {
    if command -v gh >/dev/null 2>&1; then
        return
    fi
    log "Installing GitHub CLI (gh)..."
    apt-get install -y -qq curl gnupg
    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        | gpg --dearmor -o /etc/apt/keyrings/githubcli-archive-keyring.gpg
    chmod 0644 /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        > /etc/apt/sources.list.d/github-cli.list
    apt-get update -qq
    apt-get install -y -qq gh
}

# Ensure gh is installed and authenticated for the pi user (the voldemorbot repo
# is private). Auth comes from an existing `gh auth login` or a GH_TOKEN env var.
# Called early so we fail fast instead of after the long apt/pixi build.
require_github_auth() {
    install_gh
    if [[ -n "${GH_TOKEN:-}" ]]; then
        log "Authenticating gh for user pi via GH_TOKEN..."
        printf '%s' "$GH_TOKEN" | run_as_pi gh auth login --with-token
    fi
    if ! run_as_pi gh auth status >/dev/null 2>&1; then
        echo "ERROR: gh is not authenticated for user 'pi' (private repo)." >&2
        echo "  Run:  sudo -u pi gh auth login        (or pass GH_TOKEN=...)" >&2
        echo "  then re-run this script." >&2
        exit 1
    fi
    run_as_pi gh auth setup-git
    log "gh authenticated: $(run_as_pi gh api user --jq .login 2>/dev/null || echo ok)"
}

clone_repo() {
    local dest="${1:-/home/pi/voldemorbot}"
    if [[ -d "$dest/.git" ]]; then
        log "Repo already cloned at $dest"
        return
    fi
    log "Cloning voldemorbot repo to $dest via gh..."
    run_as_pi gh repo clone teamvoldemor/voldemorbot "$dest"
}

install_ros_workspace() {
    local robot_dir="$1"
    local with_lidar="${2:-}"
    if [[ "$with_lidar" == "lidar" ]]; then
        log "Fetching LiDAR driver (sllidar_ros2) before build..."
        run_as_pi bash -c "cd '$robot_dir' && '$PIXI_BIN' run fetch-lidar-driver"
    fi
    log "Installing pixi env and building ROS2 workspace in $robot_dir..."
    run_as_pi bash -c "cd '$robot_dir' && '$PIXI_BIN' install && '$PIXI_BIN' run -e dev build-ws"
}

copy_env() {
    local robot_dir="$1"
    if [[ -f "$robot_dir/.env" ]]; then
        log ".env already exists — skipping copy"
        return
    fi
    cp "$robot_dir/.env.example" "$robot_dir/.env"
    chown pi:pi "$robot_dir/.env"
    log "Copied .env.example to .env — edit before running services"
}
