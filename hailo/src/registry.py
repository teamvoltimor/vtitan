"""Model registry: known YOLO variants and their file/export metadata."""

from __future__ import annotations

from dataclasses import dataclass
from typing import NewType

from src.constants import GMR_CHECKPOINT_PATH, OPSET_YOLO11, OPSET_YOLO12
from src.enums import ExportExtra, Task
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
        classes: Number of detection classes. Set this for retrained
            checkpoints whose class count differs from the zoo model's COCO
            default, so ``hailomz compile`` regenerates the NMS config to
            match. ``None`` keeps the zoo model's own class count.
    """

    pt_file: str
    onnx_file: str
    task: Task
    opset: int
    export_extras: tuple[tuple[ExportExtra, bool], ...]
    zoo_name: str | None = None
    classes: int | None = None

    def extra_kwargs(self) -> dict[str, bool]:
        """Return the export flags as the keyword arguments ``YOLO.export()`` takes."""
        return {k.value: v for k, v in self.export_extras}


MODEL_REGISTRY: dict[str, ModelEntry] = {
    "yolo11n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11n.pt",
        onnx_file=f"{DATA_DIR}/yolo11n.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO11,
        export_extras=(),
        zoo_name="yolov11n",
    ),
    "yolo11s": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11s.pt",
        onnx_file=f"{DATA_DIR}/yolo11s.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO11,
        export_extras=(),
        zoo_name="yolov11s",
    ),
    "yolo12n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo12n.pt",
        onnx_file=f"{DATA_DIR}/yolo12n.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO12,
        export_extras=((ExportExtra.SIMPLIFY, True), (ExportExtra.NMS, False), (ExportExtra.OPTIMIZE, False)),
        zoo_name="yolov12n",
    ),
    "yolo26n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26n.pt",
        onnx_file=f"{DATA_DIR}/yolo26n.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO12,
        export_extras=((ExportExtra.SIMPLIFY, True), (ExportExtra.NMS, False), (ExportExtra.OPTIMIZE, False)),
        zoo_name=None,
    ),
    "yolo26l": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l.pt",
        onnx_file=f"{DATA_DIR}/yolo26l.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO12,
        export_extras=((ExportExtra.SIMPLIFY, True), (ExportExtra.NMS, False)),
        zoo_name=None,
    ),
    "yolo26l-seg": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l-seg.pt",
        onnx_file=f"{DATA_DIR}/yolo26l-seg.onnx",
        task=Task.SEGMENT,
        opset=OPSET_YOLO12,
        export_extras=((ExportExtra.SIMPLIFY, True), (ExportExtra.NMS, False)),
        zoo_name=None,
    ),
    # Retrained YOLO11n owned by the auto-annotator: 3 classes
    # (green / red / magenta rectangular prism). Same architecture as
    # `yolo11n`, so it reuses the zoo's yolov11n graph config; only the class
    # count differs, which `classes` feeds to `hailomz compile --classes`.
    "gmr": ModelEntry(
        pt_file=GMR_CHECKPOINT_PATH,
        onnx_file=f"{DATA_DIR}/gmr.onnx",
        task=Task.DETECT,
        opset=OPSET_YOLO11,
        export_extras=(),
        zoo_name="yolov11n",
        classes=3,
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
        raise HailoError(msg)
    return entry
