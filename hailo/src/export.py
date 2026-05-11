"""YOLO PyTorch → ONNX export for Hailo compilation."""
from __future__ import annotations

from pathlib import Path

from src.common import (
    HailoError,
    ModelNotFoundError,
    _require_dep,
    get_entry,
    get_logger,
)
from src.config import ExportConfig  # noqa: TC001

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore[assignment, misc]

try:
    import onnx
except ImportError:
    onnx = None  # type: ignore[assignment]

log = get_logger(__name__)


def run(config: ExportConfig) -> None:
    """Export a registered YOLO model to ONNX.

    Args:
        config: Export parameters.

    Raises:
        ModelNotFoundError: If the ``.pt`` checkpoint is missing on disk.
    """
    _require_dep(YOLO, "ultralytics")
    entry = get_entry(config.model)

    if not Path(entry.pt_file).exists():
        msg = (
            f"Checkpoint not found: {entry.pt_file}. "
            "Download it or place it in the working directory."
        )
        raise ModelNotFoundError(msg) from FileNotFoundError(entry.pt_file)

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


def inspect(model_path: str) -> None:
    """Print the ONNX graph structure, input names, and output names.

    Args:
        model_path: Path to the ``.onnx`` file to inspect.

    Raises:
        HailoError: If the file cannot be loaded.
    """
    _require_dep(onnx, "onnx")

    if not Path(model_path).exists():
        msg = f"ONNX file not found: {model_path}"
        raise HailoError(msg) from FileNotFoundError(model_path)

    log.info("Loading %s", model_path)
    model = onnx.load(model_path)

    log.info("%s", onnx.helper.printable_graph(model.graph))

    inputs = [inp.name for inp in model.graph.input]
    outputs = [out.name for out in model.graph.output]
    log.info("Inputs:  %s", inputs)
    log.info("Outputs: %s", outputs)
