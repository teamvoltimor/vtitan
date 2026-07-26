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
        compile_only: Start a minimal, detached container carrying nothing but
            the shared mount. The full mount set (``/dev``, ``/lib/modules``,
            X11 sockets, dbus) only exists on a Linux host, so it is the sole
            way to run on Docker Desktop. Compilation never touches the NPU --
            only ``hailomz eval --target hardware`` does -- so dropping the
            device passthrough costs nothing for an ONNX-to-HEF run.
        gpu: Request ``--gpus all``. Set ``False`` on hosts without an NVIDIA
            GPU, where the flag makes ``docker run`` fail outright.
        cuda_device: Value for ``CUDA_VISIBLE_DEVICES``. The optimizer picks a
            device with ``select_least_used_gpu()``, which rejects any GPU more
            than 5% full and so never selects a laptop GPU that is also driving
            a desktop. Setting this skips that selector (the SDK only probes
            when the variable is absent) and is the difference between
            optimization level 0 and a fully optimized quantization.
    """

    shared_dir: str
    container: str = "hailo8_ai_sw_suite_2025-10_container"
    image: str = "hailo8_ai_sw_suite_2025-10:1"
    host_uid: int = 1000
    video_gid: int = 44
    display: str = ":0"
    dry_run: bool = False
    compile_only: bool = False
    gpu: bool = True
    cuda_device: str | None = None


def _container_exists(name: str) -> bool:
    """Return ``True`` if a container named *name* already exists (any state)."""
    probe = ["docker", "ps", "-aq", "-f", f"name=^{name}$"]
    result = subprocess.run(probe, check=True, capture_output=True, text=True)  # noqa: S603
    return bool(result.stdout.strip())


def _gpu_args(config: DockerRunConfig) -> list[str]:
    """Return the GPU-related ``docker run`` flags for *config*."""
    if not config.gpu:
        return []
    # The 2025-10 image pins NVIDIA_REQUIRE_CUDA to "driver>=470,driver<471".
    # Any current driver fails that check and the NVIDIA runtime refuses to
    # start the container, so the requirement has to be waived explicitly.
    args = ["--gpus", "all", "-e", "NVIDIA_DISABLE_REQUIRE=1"]
    if config.cuda_device is not None:
        args += ["-e", f"CUDA_VISIBLE_DEVICES={config.cuda_device}"]
    return args


def _compile_only_args(config: DockerRunConfig, shared_abs: str) -> list[str]:
    """Return a minimal, detached ``docker run`` command line.

    Carries the shared mount and nothing else, so it works on Docker Desktop
    where the Linux host paths do not exist. ``sleep infinity`` keeps the
    container alive for the ``docker exec`` calls that :func:`run_or_print`
    issues, since the image's own command is an interactive shell that would
    exit immediately when detached.
    """
    return [
        "docker",
        "run",
        "-d",
        *_gpu_args(config),
        "-v",
        f"{shared_abs}:{DOCKER_SHARED_MOUNT}:rw",
        "--name",
        config.container,
        config.image,
        "sleep",
        "infinity",
    ]


def _launch(cmd: list[str], config: DockerRunConfig, *, attach: bool) -> None:
    """Print *cmd*, restart an existing container, or start a new one.

    Args:
        cmd: Fully built ``docker run`` command line.
        config: Container startup parameters.
        attach: Attach to the container's TTY when restarting an existing one.
            A detached (``compile_only``) container has no TTY to attach to.
    """
    if config.dry_run:
        pretty = " \\\n  ".join(cmd)
        log.info("Docker run command:\n%s", pretty)
        return

    if _container_exists(config.container):
        log.info("Container %s already exists — starting it", config.container)
        start = ["docker", "start", *(["-ai"] if attach else []), config.container]
        subprocess.run(start, check=True)  # noqa: S603
        return

    log.info("Starting container %s from image %s", config.container, config.image)
    subprocess.run(cmd, check=True)  # noqa: S603


def docker_run(config: DockerRunConfig) -> None:
    """Start the Hailo AI Software Suite Docker container.

    Reconstructs the full ``docker run`` command including all required
    device mounts, GPU access, X11 forwarding, and the shared volume. When a
    container of the same name already exists it is restarted and re-attached
    instead of creating a duplicate.

    With ``compile_only`` set, a minimal detached container is started instead
    -- the only form that works on a non-Linux host.

    Args:
        config: Container startup parameters.
    """
    # as_posix() keeps the drive-letter form Docker Desktop accepts on Windows
    # ("D:/path"); on Linux it is identical to str().
    shared_abs = Path(config.shared_dir).resolve().as_posix()

    if config.compile_only:
        _launch(_compile_only_args(config, shared_abs), config, attach=False)
        return

    cmd = [
        "docker",
        "run",
        "--privileged",
        "--net=host",
        *_gpu_args(config),
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

    _launch(cmd, config, attach=True)
