"""YOLO PyTorch → ONNX export for Hailo compilation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from src.common import (
    MODEL_REGISTRY,
    HailoError,
    ModelName,
    ModelNotFoundError,
    get_logger,
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore[assignment, misc]

log = get_logger(__name__)


@dataclass(slots=True, frozen=True)
class ExportConfig:
    """Parameters for a single model export run.

    Args:
        model: Registry key identifying the model variant.
        imgsz: Square input resolution passed to ``YOLO.export()``.
        opset: Override the per-model default ONNX opset when set.
        no_simplify: When ``True``, suppress the ``simplify=True`` flag
            even if the model registry requests it.
    """

    model: ModelName
    imgsz: int
    opset: int | None
    no_simplify: bool


def run(config: ExportConfig) -> None:
    """Export a registered YOLO model to ONNX.

    Args:
        config: Export parameters.

    Raises:
        HailoError: If ``config.model`` is not in the registry.
        ModelNotFoundError: If the ``.pt`` checkpoint is missing on disk.
    """
    if YOLO is None:
        msg = "ultralytics is not installed."
        raise HailoError(msg)

    entry = MODEL_REGISTRY.get(config.model)
    if entry is None:
        msg = f"Unknown model {config.model!r}. Valid options: {list(MODEL_REGISTRY)}"
        raise HailoError(msg)

    if not Path(entry.pt_file).exists():
        msg = (
            f"Checkpoint not found: {entry.pt_file}. "
            "Download it or place it in the working directory."
        )
        raise ModelNotFoundError(msg)

    opset = config.opset if config.opset is not None else entry.opset
    extra = entry.extra_kwargs()
    if config.no_simplify:
        extra.pop("simplify", None)

    export_kwargs = {
        "format": "onnx",
        "imgsz": config.imgsz,
        "opset": opset,
        "dynamic": False,
        **extra,
    }

    log.info("Loading %s", entry.pt_file)
    model = YOLO(entry.pt_file)
    log.info(
        "Exporting %s → ONNX  opset=%d  kwargs=%s",
        config.model,
        opset,
        export_kwargs,
    )
    model.export(**export_kwargs)
    log.info("Export complete → %s", entry.onnx_file)
