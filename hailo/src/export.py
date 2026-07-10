"""YOLO PyTorch → ONNX export for Hailo compilation."""

from __future__ import annotations

from pathlib import Path

from src.config import ExportConfig  # noqa: TC001
from src.errors import ModelNotFoundError, require_dep
from src.log import get_logger
from src.registry import get_entry

_yolo_import_err: ImportError | None = None
try:
    from ultralytics import YOLO
except ImportError as _exc:
    YOLO = None  # type: ignore[assignment, misc]
    _yolo_import_err = _exc

log = get_logger(__name__)


def run(config: ExportConfig) -> None:
    """Export a registered YOLO model to ONNX.

    Args:
        config: Export parameters.

    Raises:
        ModelNotFoundError: If the ``.pt`` checkpoint is missing on disk.
    """
    require_dep(YOLO, "ultralytics", cause=_yolo_import_err)
    entry = get_entry(config.model)

    if not Path(entry.pt_file).exists():
        msg = f"Checkpoint not found: {entry.pt_file}. Download it or place it in the working directory."
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
