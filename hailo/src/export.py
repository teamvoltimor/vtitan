"""YOLO PyTorch → ONNX export for Hailo compilation."""

from __future__ import annotations

import shutil
from pathlib import Path

from src.config import ExportConfig  # noqa: TC001
from src.constants import EXPORT_FORMAT_ONNX
from src.enums import ExportExtra
from src.deps import YOLO, _yolo_import_err
from src.errors import ModelNotFoundError, require_dep
from src.log import get_logger
from src.registry import get_entry

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
        extra.pop(ExportExtra.SIMPLIFY.value, None)

    export_kwargs = {
        "format": EXPORT_FORMAT_ONNX,
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
    produced = Path(model.export(**export_kwargs))

    # Ultralytics writes the ONNX next to the checkpoint. For checkpoints that
    # live outside `data/` (retrained models kept with their training run) that
    # is not where the rest of the pipeline looks, so move it into place.
    destination = Path(entry.onnx_file)
    if produced.resolve() != destination.resolve():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(produced), destination)
        log.info("Moved %s → %s", produced, destination)

    log.info("Export complete → %s", entry.onnx_file)
