"""Hailo AI Software Suite Docker abstraction.

Encapsulates Docker container lifecycle, image identity, and docker exec orchestration.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

from src.log import get_logger

# Fixed contract with the Hailo AI Software Suite image's own internal mount
# expectations -- not host-specific, so not sourced from HailoSettings.
DOCKER_SHARED_MOUNT = "/local/shared_with_docker"

log = get_logger(__name__)


def run_or_print(cmd: list[str], docker: str | None, workdir: str | None = None) -> None:
    """Execute *cmd* via ``docker exec`` or print it for manual use.

    Args:
        cmd: The ``hailomz`` command and its arguments.
        docker: Container name. When ``None`` the command is printed.
        workdir: Working directory inside the container. When set, the command
            runs there (``docker exec -w``) so any output artifacts land in that
            directory; the printed form is prefixed with a matching ``cd``.
    """
    if docker:
        full = ["docker", "exec", *(["-w", workdir] if workdir else []), docker, *cmd]
        log.info("Running: %s", " ".join(full))
        subprocess.run(full, check=True)  # noqa: S603
    else:
        prefix = f"cd {workdir} && \\\n  " if workdir else ""
        pretty = prefix + " \\\n  ".join(cmd)
        log.info("Paste inside Docker:\n%s", pretty)


@dataclass(slots=True, frozen=True)
class DockerRunConfig:
    """Parameters for starting the Hailo AI Software Suite container.

    Field defaults mirror :class:`~src.settings.HailoSettings`'s defaults;
    ``main.py`` populates them from settings/env vars at CLI-parser
    construction time, so these are only a fallback for direct/programmatic use.

    Args:
        shared_dir: Host path mounted as ``/local/shared_with_docker`` inside
            the container. Resolved to an absolute path at runtime.
        container: Name assigned to the running container instance.
        image: Suite image ``repo:tag`` to launch.
        host_uid: Host UID for the container's ``XDG_RUNTIME_DIR`` mount.
        video_gid: Host video group GID for ``--group-add``.
        display: X11 ``DISPLAY`` variable forwarded into the container.
        dry_run: When ``True``, print the command instead of executing it.
    """

    shared_dir: str
    container: str = "hailo8_ai_sw_suite_2025-10_container"
    image: str = "hailo8_ai_sw_suite_2025-10:1"
    host_uid: int = 1000
    video_gid: int = 44
    display: str = ":0"
    dry_run: bool = False


def _container_exists(name: str) -> bool:
    """Return ``True`` if a container named *name* already exists (any state)."""
    probe = ["docker", "ps", "-aq", "-f", f"name=^{name}$"]
    result = subprocess.run(probe, check=True, capture_output=True, text=True)  # noqa: S603
    return bool(result.stdout.strip())


def docker_run(config: DockerRunConfig) -> None:
    """Start the Hailo AI Software Suite Docker container.

    Reconstructs the full ``docker run`` command including all required
    device mounts, GPU access, X11 forwarding, and the shared volume. When a
    container of the same name already exists it is restarted and re-attached
    instead of creating a duplicate.

    Args:
        config: Container startup parameters.
    """
    shared_abs = str(Path(config.shared_dir).resolve())

    cmd = [
        "docker",
        "run",
        "--privileged",
        "--net=host",
        "--gpus",
        "all",
        "-e",
        f"DISPLAY={config.display}",
        "-e",
        f"XDG_RUNTIME_DIR=/run/user/{config.host_uid}/",
        "--device=/dev/dri:/dev/dri",
        "--ipc=host",
        "--group-add",
        str(config.video_gid),
        "-v",
        "/dev:/dev",
        "-v",
        "/lib/firmware:/lib/firmware",
        "-v",
        "/lib/modules:/lib/modules",
        "-v",
        "/lib/udev/rules.d:/lib/udev/rules.d",
        "-v",
        "/usr/src:/usr/src",
        "-v",
        "/tmp/hailo_docker.xauth:/home/hailo/.Xauthority",  # noqa: S108
        "-v",
        "/tmp/.X11-unix/:/tmp/.X11-unix/",  # noqa: S108
        "--name",
        config.container,
        "-v",
        "/var/run/docker.sock:/var/run/docker.sock",
        "-v",
        "/etc/machine-id:/etc/machine-id:ro",
        "-v",
        "/var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket",
        "-v",
        f"{shared_abs}:{DOCKER_SHARED_MOUNT}:rw",
        "-v",
        "/etc/timezone:/etc/timezone:ro",
        "-v",
        "/etc/localtime:/etc/localtime:ro",
        "-ti",
        config.image,
    ]

    if config.dry_run:
        pretty = " \\\n  ".join(cmd)
        log.info("Docker run command:\n%s", pretty)
        return

    if _container_exists(config.container):
        log.info("Container %s already exists — starting and attaching", config.container)
        start = ["docker", "start", "-ai", config.container]
        subprocess.run(start, check=True)  # noqa: S603
        return

    log.info("Starting container %s from image %s", config.container, config.image)
    subprocess.run(cmd, check=True)  # noqa: S603
