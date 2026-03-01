"""YOLO PyTorch → ONNX export for Hailo compilation."""
from __future__ import annotations

from dataclasses import dataclass

from src.common import (
    MODEL_REGISTRY,
    ModelName,
    HailoError,
    ModelNotFoundError,
    get_logger,
)

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
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise HailoError("ultralytics is not installed.") from exc

    entry = MODEL_REGISTRY.get(config.model)
    if entry is None:
        raise HailoError(
            f"Unknown model {config.model!r}. "
            f"Valid options: {list(MODEL_REGISTRY)}"
        )

    from pathlib import Path
    if not Path(entry.pt_file).exists():
        raise ModelNotFoundError(
            f"Checkpoint not found: {entry.pt_file}. "
            "Download it or place it in the working directory."
        )

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
