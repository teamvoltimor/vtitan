"""ONNX graph inspection utilities."""
from __future__ import annotations

from src.common import HailoError, get_logger

log = get_logger(__name__)


def inspect(model_path: str) -> None:
    """Print the ONNX graph structure, input names, and output names.

    Args:
        model_path: Path to the ``.onnx`` file to inspect.

    Raises:
        HailoError: If ``onnx`` is not installed or the file cannot be loaded.
    """
    try:
        import onnx
    except ImportError as exc:
        raise HailoError("onnx is not installed.") from exc

    from pathlib import Path
    if not Path(model_path).exists():
        raise HailoError(f"ONNX file not found: {model_path}")

    log.info("Loading %s", model_path)
    model = onnx.load(model_path)

    print(onnx.helper.printable_graph(model.graph))

    inputs = [inp.name for inp in model.graph.input]
    outputs = [out.name for out in model.graph.output]
    log.info("Inputs:  %s", inputs)
    log.info("Outputs: %s", outputs)
