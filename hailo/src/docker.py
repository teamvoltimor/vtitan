"""Hailo AI Software Suite Docker abstraction.

Encapsulates Docker container lifecycle, image identity, and docker exec orchestration.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

from src.common import get_logger

# Docker container identity
DOCKER_CONTAINER = "hailo8_ai_sw_suite_2025-10_container"
DOCKER_IMAGE = "hailo8_ai_sw_suite_2025-10:1"
DOCKER_SHARED_MOUNT = "/local/shared_with_docker"

log = get_logger(__name__)


def _run_or_print(cmd: list[str], docker: str | None) -> None:
    """Execute *cmd* via ``docker exec`` or print it for manual use.

    Args:
        cmd: The ``hailomz`` command and its arguments.
        docker: Container name. When ``None`` the command is printed.
    """
    if docker:
        full = ["docker", "exec", docker, *cmd]
        log.info("Running: %s", " ".join(full))
        subprocess.run(full, check=True)  # noqa: S603
    else:
        pretty = " \\\n  ".join(cmd)
        log.info("Paste inside Docker:\n%s", pretty)


@dataclass(slots=True, frozen=True)
class DockerRunConfig:
    """Parameters for starting the Hailo AI Software Suite container.

    Args:
        shared_dir: Host path mounted as ``/local/shared_with_docker`` inside
            the container. Resolved to an absolute path at runtime.
        container: Name assigned to the running container instance.
        display: X11 ``DISPLAY`` variable forwarded into the container.
        dry_run: When ``True``, print the command instead of executing it.
    """

    shared_dir: str
    container: str = DOCKER_CONTAINER
    display: str = ":0"
    dry_run: bool = False


def docker_run(config: DockerRunConfig) -> None:
    """Start the Hailo AI Software Suite Docker container.

    Reconstructs the full ``docker run`` command including all required
    device mounts, GPU access, X11 forwarding, and the shared volume.

    Args:
        config: Container startup parameters.
    """
    shared_abs = str(Path(config.shared_dir).resolve())

    cmd = [
        "docker", "run",
        "--privileged",
        "--net=host",
        "--gpus", "all",
        "-e", f"DISPLAY={config.display}",
        "-e", "XDG_RUNTIME_DIR=/run/user/1000/",
        "--device=/dev/dri:/dev/dri",
        "--ipc=host",
        "--group-add", "44",
        "-v", "/dev:/dev",
        "-v", "/lib/firmware:/lib/firmware",
        "-v", "/lib/modules:/lib/modules",
        "-v", "/lib/udev/rules.d:/lib/udev/rules.d",
        "-v", "/usr/src:/usr/src",
        "-v", "/tmp/hailo_docker.xauth:/home/hailo/.Xauthority",  # noqa: S108
        "-v", "/tmp/.X11-unix/:/tmp/.X11-unix/",  # noqa: S108
        "--name", config.container,
        "-v", "/var/run/docker.sock:/var/run/docker.sock",
        "-v", "/etc/machine-id:/etc/machine-id:ro",
        "-v", "/var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket",
        "-v", f"{shared_abs}:{DOCKER_SHARED_MOUNT}:rw",
        "-v", "/etc/timezone:/etc/timezone:ro",
        "-v", "/etc/localtime:/etc/localtime:ro",
        "-ti",
        DOCKER_IMAGE,
    ]

    if config.dry_run:
        pretty = " \\\n  ".join(cmd)
        log.info("Docker run command:\n%s", pretty)
    else:
        log.info("Starting container %s from image %s", config.container, DOCKER_IMAGE)
        subprocess.run(cmd, check=True)  # noqa: S603
