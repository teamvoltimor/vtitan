"""ONNX graph inspection utilities."""

from __future__ import annotations

from pathlib import Path

from src.errors import HailoError, require_dep
from src.log import get_logger

_onnx_import_err: ImportError | None = None
try:
    import onnx
except ImportError as _exc:
    onnx = None  # type: ignore[assignment]
    _onnx_import_err = _exc

log = get_logger(__name__)


def inspect(model_path: str) -> None:
    """Print the ONNX graph structure, input names, and output names.

    Args:
        model_path: Path to the ``.onnx`` file to inspect.

    Raises:
        HailoError: If ``onnx`` is not installed or the file cannot be loaded.
    """
    require_dep(onnx, "onnx", cause=_onnx_import_err)

    if not Path(model_path).exists():
        msg = f"ONNX file not found: {model_path}"
        raise HailoError(msg)

    log.info("Loading %s", model_path)
    model = onnx.load(model_path)

    log.info("%s", onnx.helper.printable_graph(model.graph))

    inputs = [inp.name for inp in model.graph.input]
    outputs = [out.name for out in model.graph.output]
    log.info("Inputs:  %s", inputs)
    log.info("Outputs: %s", outputs)
