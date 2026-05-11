"""Shared types, enums, exceptions, model registry, logger, and image utilities."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from datetime import UTC, datetime
from enum import StrEnum
from typing import Any, NewType

# Domain types
ModelName = NewType("ModelName", str)


# Logging


class _JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, str] = {
            "ts": datetime.now(UTC).isoformat(timespec="seconds"),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload)


def get_logger(name: str) -> logging.Logger:
    """Return a module-level logger.

    Args:
        name: Typically ``__name__`` from the calling module.

    Returns:
        A standard :class:`logging.Logger` bound to the JSON handler
        configured in :func:`configure_logging`.
    """
    return logging.getLogger(name)


def configure_logging(level: str = "INFO") -> None:
    """Wire the root logger to emit structured JSON lines.

    Args:
        level: One of ``DEBUG``, ``INFO``, ``WARNING``, ``ERROR``.
    """
    handler = logging.StreamHandler()
    handler.setFormatter(_JsonFormatter())
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        handlers=[handler],
        force=True,
    )


# Enums


class Backend(StrEnum):
    """Inference backend for the ``test`` command."""

    PT = "pt"
    ONNX = "onnx"
    ULTRAONNX = "ultraonnx"


class Task(StrEnum):
    """Inference task type."""

    DETECT = "detect"
    SEGMENT = "segment"


class HWArch(StrEnum):
    """Hailo target hardware architecture."""

    HAILO8 = "hailo8"
    HAILO8L = "hailo8l"


# Exceptions


class HailoError(Exception):
    """Base error for the Hailo pipeline."""


class ModelNotFoundError(HailoError):
    """Raised when a requested model file does not exist on disk."""


class CalibrationDataError(HailoError):
    """Raised when calibration data is missing or malformed."""


class CompileError(HailoError):
    """Raised when the Hailo DFC fails to compile a model."""


# Directory constants

DATA_DIR = "data"
SHARED_WITH_DOCKER = "shared_with_docker"


# Model registry


@dataclass(slots=True, frozen=True)
class ModelEntry:
    """Immutable descriptor for a registered YOLO model variant.

    Args:
        pt_file: Path to the PyTorch checkpoint (under ``data/``).
        onnx_file: Path to the exported ONNX graph (under ``data/``).
        task: Whether this model performs detection or segmentation.
        opset: Default ONNX opset for Hailo compatibility.
        export_extras: Additional keyword arguments forwarded to
            ``YOLO.export()``, stored as an immutable tuple of pairs.
        zoo_name: Hailo Model Zoo identifier used by ``hailomz`` CLI
            (e.g. ``"yolov11s"``). ``None`` if the model is not in the zoo.
    """

    pt_file: str
    onnx_file: str
    task: Task
    opset: int
    export_extras: tuple[tuple[str, Any], ...]
    zoo_name: str | None = None

    def extra_kwargs(self) -> dict[str, Any]:
        """Materialise ``export_extras`` back into a plain dict."""
        return dict(self.export_extras)


MODEL_REGISTRY: dict[str, ModelEntry] = {
    "yolo11n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11n.pt",
        onnx_file=f"{DATA_DIR}/yolo11n.onnx",
        task=Task.DETECT,
        opset=13,
        export_extras=(),
        zoo_name="yolov11n",
    ),
    "yolo11s": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11s.pt",
        onnx_file=f"{DATA_DIR}/yolo11s.onnx",
        task=Task.DETECT,
        opset=13,
        export_extras=(),
        zoo_name="yolov11s",
    ),
    "yolo12n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo12n.pt",
        onnx_file=f"{DATA_DIR}/yolo12n.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False), ("optimize", False)),
        zoo_name="yolov12n",
    ),
    "yolo26n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26n.pt",
        onnx_file=f"{DATA_DIR}/yolo26n.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False), ("optimize", False)),
        zoo_name=None,
    ),
    "yolo26l": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l.pt",
        onnx_file=f"{DATA_DIR}/yolo26l.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False)),
        zoo_name=None,
    ),
    "yolo26l-seg": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l-seg.pt",
        onnx_file=f"{DATA_DIR}/yolo26l-seg.onnx",
        task=Task.SEGMENT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False)),
        zoo_name=None,
    ),
}


def get_entry(model_key: str) -> ModelEntry:
    """Retrieve a model entry from the registry, raising on miss.

    Args:
        model_key: Registry key for the model.

    Returns:
        The ModelEntry for this key.

    Raises:
        HailoError: If the key is not in the registry.
    """
    entry = MODEL_REGISTRY.get(model_key)
    if entry is None:
        msg = f"Unknown model {model_key!r}. Valid options: {list(MODEL_REGISTRY)}"
        raise HailoError(msg) from KeyError(model_key)
    return entry


def _require_dep(module: object | None, package_name: str) -> None:
    """Raise HailoError if an optional dependency is not installed.

    Args:
        module: The imported module object, or None if import failed.
        package_name: Name of the package for the error message.

    Raises:
        HailoError: If module is None.
    """
    if module is None:
        msg = f"{package_name} is not installed. Run: uv add {package_name}"
        raise HailoError(msg)
