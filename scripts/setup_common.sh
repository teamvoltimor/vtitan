#!/usr/bin/env bash
# Shared helpers used by setup_pi_zero.sh and setup_pi_5.sh.
set -euo pipefail

log() { echo "[setup] $*"; }

require_root() {
    if [[ $EUID -ne 0 ]]; then
        echo "This script must be run as root (sudo)." >&2
        exit 1
    fi
}

run_as_pi() {
    if [[ $EUID -eq 0 ]]; then
        sudo -u pi "$@"
    else
        "$@"
    fi
}

install_pixi() {
    if run_as_pi bash -c 'command -v pixi &>/dev/null'; then
        log "pixi already installed: $(run_as_pi pixi --version)"
        return
    fi
    log "Installing pixi..."
    run_as_pi bash -c 'curl -fsSL https://pixi.sh/install.sh | bash'
    export PATH="$HOME/.pixi/bin:$PATH"
}

clone_repo() {
    local dest="${1:-/home/pi/voldemorbot}"
    if [[ -d "$dest/.git" ]]; then
        log "Repo already cloned at $dest"
        return
    fi
    log "Cloning voldemorbot repo to $dest..."
    run_as_pi git clone https://github.com/teamvoldemor/voldemorbot.git "$dest"
}

install_ros_workspace() {
    local robot_dir="$1"
    log "Installing pixi env and building ROS2 workspace in $robot_dir..."
    run_as_pi bash -c "cd '$robot_dir' && pixi install && pixi run -e dev build-ws"
}

copy_env() {
    local robot_dir="$1"
    if [[ -f "$robot_dir/.env" ]]; then
        log ".env already exists — skipping copy"
        return
    fi
    cp "$robot_dir/.env.example" "$robot_dir/.env"
    log "Copied .env.example to .env — edit before running services"
    chown pi:pi "$robot_dir/.env"
}
