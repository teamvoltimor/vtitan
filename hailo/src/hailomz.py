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
from pathlib import Path

from src.config import CompileConfig, EvalConfig, ProfileConfig, StageConfig  # noqa: TC001
from src.constants import IMAGE_EXTENSIONS
from src.docker import (
    DOCKER_SHARED_MOUNT,
    run_or_print,
)
from src.errors import HailoError
from src.log import get_logger
from src.registry import MODEL_REGISTRY, get_entry

log = get_logger(__name__)


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


# Public commands


IMAGE_SUFFIXES = frozenset(IMAGE_EXTENSIONS)
LABEL_SUFFIXES = frozenset({".txt"})


def _stage_flat(source: Path, dest: Path, suffixes: frozenset[str]) -> int:
    """Copy every matching file under *source* into a flat *dest* directory.

    ``hailomz`` reads calibration images from a single directory, so nested
    dataset layouts (``images/<class>/*.jpg``) are flattened. Names are
    prefixed with their relative parent to keep collisions apart -- and because
    labels are flattened with the same rule, an image and its label keep a
    matching stem on the other side.

    Args:
        source: Root directory to walk.
        dest: Flat destination directory, created if absent.
        suffixes: Lowercased file extensions to copy.

    Returns:
        Number of files copied.
    """
    dest.mkdir(parents=True, exist_ok=True)
    count = 0
    for path in sorted(source.rglob("*")):
        if path.suffix.lower() not in suffixes:
            continue
        relative_parent = path.parent.relative_to(source)
        prefix = "_".join(relative_parent.parts)
        name = f"{prefix}_{path.name}" if prefix else path.name
        shutil.copy2(path, dest / name)
        count += 1
    return count


def stage(config: StageConfig) -> None:
    """Copy ONNX and calibration data into ``shared_with_docker/``.

    Args:
        config: Stage parameters.

    Raises:
        HailoError: If its ONNX file is missing.
    """
    entry = get_entry(config.model)

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
        calib_dest = shared / config.calib_name
        staged = _stage_flat(calib_src, calib_dest, IMAGE_SUFFIXES)
        if staged == 0:
            msg = f"No calibration images found under {calib_src}."
            raise HailoError(msg)
        log.info("Staged %d calibration images → %s", staged, calib_dest)

    if config.labels:
        labels_src = Path(config.labels)
        if not labels_src.exists():
            msg = f"Label directory not found: {labels_src}."
            raise HailoError(msg)
        labels_dest = shared / config.labels_name
        staged = _stage_flat(labels_src, labels_dest, LABEL_SUFFIXES)
        if staged == 0:
            msg = f"No label files found under {labels_src}."
            raise HailoError(msg)
        log.info("Staged %d label files → %s", staged, labels_dest)

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
        HailoError: When the zoo name cannot be resolved, or when both
            ``model_script`` and ``performance`` are requested.
    """
    zoo_name = _resolve_zoo_name(config.model, config.zoo_name)
    entry = MODEL_REGISTRY.get(config.model)
    onnx_name = Path(entry.onnx_file).name if entry else f"{config.model}.onnx"
    ckpt = f"{DOCKER_SHARED_MOUNT}/{onnx_name}"

    cmd = [
        "hailomz",
        "compile",
        zoo_name,
        "--ckpt",
        ckpt,
        "--calib-path",
        config.calib_path,
        "--hw-arch",
        str(config.hw),
    ]

    # A retrained checkpoint keeps the zoo model's graph but not its class
    # count, so the NMS config has to be regenerated for the real one.
    classes = config.classes if config.classes is not None else (entry.classes if entry else None)
    if classes is not None:
        cmd += ["--classes", str(classes)]

    # hailomz rejects these two together -- they are one argparse mutually
    # exclusive group -- so guard rather than let the container fail late.
    if config.model_script and config.performance:
        msg = "Pass either --model-script or --performance, not both."
        raise HailoError(msg)
    if config.model_script:
        cmd += ["--model-script", config.model_script]
    elif config.performance:
        cmd.append("--performance")
    # Pin the working directory to the shared mount so the resulting HAR/HEF
    # land where `eval`/`profile` look for them by default.
    run_or_print(cmd, config.docker, workdir=DOCKER_SHARED_MOUNT)


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
        "hailomz",
        "eval",
        zoo_name,
        "--har",
        har,
        "--target",
        config.target,
        "--data-count",
        str(config.data_count),
    ]
    if config.visualize:
        cmd.append("--visualize")

    run_or_print(cmd, config.docker)


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
    run_or_print(cmd, config.docker)
