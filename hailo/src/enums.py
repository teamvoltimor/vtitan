"""Domain enums for the Hailo YOLO pipeline."""

from __future__ import annotations

from enum import StrEnum


class Backend(StrEnum):
    """Inference backend for the ``test`` command.

    ``ONNX`` runs a raw ``onnxruntime`` session and expects the model to embed
    NMS (post-NMS ``(N, 6)`` output). The registered models export with
    ``nms=False``, so use ``ULTRAONNX`` — which lets Ultralytics decode the raw
    detection tensor — to test them.
    """

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


class EvalTarget(StrEnum):
    """Evaluation target."""

    EMULATOR = "emulator"
    HAILO8 = "hailo8"


class ExportExtra(StrEnum):
    """Ultralytics ``YOLO.export()`` keyword flags stored in the model registry."""

    SIMPLIFY = "simplify"
    NMS = "nms"
    OPTIMIZE = "optimize"
