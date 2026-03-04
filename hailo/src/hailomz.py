"""Hailo AI Software Suite Docker bridge.

Workflow
--------
0. ``docker_run`` — start the Hailo AI Software Suite container.
1. ``stage``      — copy ONNX + calibration data into ``shared_with_docker/``
                    so the container sees them at ``/local/shared_with_docker/``.
2. ``compile``    — run (or print) ``hailomz compile`` inside the container.
3. ``eval``       — run (or print) ``hailomz eval``    inside the container.
4. ``profile``    — run (or print) ``hailomz profile`` inside the container.

When ``--docker <container>`` is supplied the hailomz commands are executed
via ``docker exec``. Without it the shell command is printed for manual use.
"""
from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

from src.common import (
    DOCKER_CONTAINER,
    DOCKER_IMAGE,
    DOCKER_SHARED_MOUNT,
    MODEL_REGISTRY,
    SHARED_WITH_DOCKER,
    HailoError,
    HWArch,
    get_logger,
)

log = get_logger(__name__)


# Config dataclasses

@dataclass(slots=True, frozen=True)
class StageConfig:
    """Parameters for staging files into ``shared_with_docker/``.

    Args:
        model: Registry key of the model whose ONNX should be staged.
        calib: Local calibration image directory to copy across.
            Pass ``None`` to skip copying calibration data.
        shared_dir: Host path of the Docker-shared volume
            (default: ``shared_with_docker``).
    """

    model: str
    calib: str | None
    shared_dir: str = SHARED_WITH_DOCKER


@dataclass(slots=True, frozen=True)
class CompileConfig:
    """Parameters for ``hailomz compile``.

    Args:
        model: Registry key used to resolve the zoo name and ONNX filename.
        zoo_name: Override the Hailo Model Zoo identifier. Required when the
            model has no ``zoo_name`` entry in the registry.
        hw: Target hardware architecture.
        calib_path: Calibration data path *inside Docker*
            (default: ``/local/shared_with_docker/calib_data``).
        docker: Docker container name for ``docker exec``.
            When ``None`` the command is printed instead of executed.
    """

    model: str
    zoo_name: str | None
    hw: HWArch
    calib_path: str
    docker: str | None


@dataclass(slots=True, frozen=True)
class EvalConfig:
    """Parameters for ``hailomz eval``.

    Args:
        model: Registry key used to resolve the zoo name and default HAR path.
        zoo_name: Override the Hailo Model Zoo identifier.
        har: HAR path *inside Docker*. Defaults to
            ``/local/shared_with_docker/<zoo_name>.har``.
        target: Evaluation target — ``"emulator"`` or ``"hailo8"``.
        data_count: Number of samples to evaluate.
        visualize: Emit ``--visualize`` flag.
        docker: Docker container name. ``None`` → print command.
    """

    model: str
    zoo_name: str | None
    har: str | None
    target: str
    data_count: int
    visualize: bool
    docker: str | None


@dataclass(slots=True, frozen=True)
class ProfileConfig:
    """Parameters for ``hailomz profile``.

    Args:
        model: Registry key used to resolve the zoo name and default HEF path.
        zoo_name: Override the Hailo Model Zoo identifier.
        hef: HEF path *inside Docker*. Defaults to
            ``/local/shared_with_docker/<zoo_name>.hef``.
        docker: Docker container name. ``None`` → print command.
    """

    model: str
    zoo_name: str | None
    hef: str | None
    docker: str | None


# Helpers

def _resolve_zoo_name(model: str, override: str | None) -> str:
    """Return the hailomz model zoo identifier.

    Args:
        model: Registry key.
        override: Explicit zoo name from the CLI.

    Returns:
        Resolved zoo name string.

    Raises:
        HailoError: When neither the registry entry nor the override
            provides a zoo name.
    """
    if override:
        return override
    entry = MODEL_REGISTRY.get(model)
    if entry and entry.zoo_name:
        return entry.zoo_name
    msg = f"Model {model!r} has no Hailo Model Zoo name. Supply --zoo-name explicitly (e.g. --zoo-name yolov11s)."
    raise HailoError(msg)


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


# Public commands

def stage(config: StageConfig) -> None:
    """Copy ONNX and calibration data into ``shared_with_docker/``.

    Args:
        config: Stage parameters.

    Raises:
        HailoError: If the model is not in the registry or its ONNX file
            is missing.
    """
    entry = MODEL_REGISTRY.get(config.model)
    if entry is None:
        msg = f"Unknown model {config.model!r}. Valid options: {list(MODEL_REGISTRY)}"
        raise HailoError(msg)

    onnx_src = Path(entry.onnx_file)
    if not onnx_src.exists():
        msg = f"ONNX file not found: {onnx_src}. Run `hailo export` first."
        raise HailoError(msg)

    shared = Path(config.shared_dir)
    shared.mkdir(parents=True, exist_ok=True)

    dest = shared / onnx_src.name
    shutil.copy2(onnx_src, dest)
    log.info("Staged %s → %s", onnx_src, dest)

    if config.calib:
        calib_src = Path(config.calib)
        if not calib_src.exists():
            msg = f"Calibration directory not found: {calib_src}. Run `hailo calib download` first."
            raise HailoError(msg)
        calib_dest = shared / "calib_data"
        shutil.copytree(calib_src, calib_dest, dirs_exist_ok=True)
        log.info("Staged calibration data → %s", calib_dest)

    log.info(
        "Stage complete. Files are at %s (%s inside Docker).",
        shared,
        DOCKER_SHARED_MOUNT,
    )


def compile_model(config: CompileConfig) -> None:
    """Run ``hailomz compile`` for the given model.

    Args:
        config: Compile parameters.

    Raises:
        HailoError: When the zoo name cannot be resolved.
    """
    zoo_name = _resolve_zoo_name(config.model, config.zoo_name)
    entry = MODEL_REGISTRY.get(config.model)
    onnx_name = Path(entry.onnx_file).name if entry else f"{config.model}.onnx"
    ckpt = f"{DOCKER_SHARED_MOUNT}/{onnx_name}"

    cmd = [
        "hailomz", "compile", zoo_name,
        "--ckpt", ckpt,
        "--calib-path", config.calib_path,
        "--hw-arch", str(config.hw),
    ]
    _run_or_print(cmd, config.docker)


def eval_model(config: EvalConfig) -> None:
    """Run ``hailomz eval`` for the given model.

    Args:
        config: Eval parameters.

    Raises:
        HailoError: When the zoo name cannot be resolved.
    """
    zoo_name = _resolve_zoo_name(config.model, config.zoo_name)
    har = config.har or f"{DOCKER_SHARED_MOUNT}/{zoo_name}.har"

    cmd = [
        "hailomz", "eval", zoo_name,
        "--har", har,
        "--target", config.target,
        "--data-count", str(config.data_count),
    ]
    if config.visualize:
        cmd.append("--visualize")

    _run_or_print(cmd, config.docker)


def profile_model(config: ProfileConfig) -> None:
    """Run ``hailomz profile`` for the given model.

    Args:
        config: Profile parameters.

    Raises:
        HailoError: When the zoo name cannot be resolved.
    """
    zoo_name = _resolve_zoo_name(config.model, config.zoo_name)
    hef = config.hef or f"{DOCKER_SHARED_MOUNT}/{zoo_name}.hef"

    cmd = ["hailomz", "profile", "--hef", hef, zoo_name]
    _run_or_print(cmd, config.docker)


# Docker container lifecycle

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

    shared_dir: str = SHARED_WITH_DOCKER
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
