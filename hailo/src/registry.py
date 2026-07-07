"""Model registry: known YOLO variants and their file/export metadata."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, NewType

from src.enums import Task
from src.errors import HailoError

ModelName = NewType("ModelName", str)

DATA_DIR = "data"
SHARED_WITH_DOCKER = "shared_with_docker"


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
