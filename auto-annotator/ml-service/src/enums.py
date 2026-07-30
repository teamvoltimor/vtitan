"""src.enums – Application-wide enumerations.

Centralises all enum types so they can be imported without pulling in the
heavier modules that depend on them.
"""

from enum import IntEnum, StrEnum


class Status(IntEnum):
    """Image annotation status codes stored as integers in SQLite."""

    PENDING = 0
    DONE = 1
    SKIPPED = 2


class ComputeDevice(StrEnum):
    """Compute device for inference (CUDA or CPU)."""

    CUDA = "cuda"
    """NVIDIA GPU acceleration via CUDA."""

    CPU = "cpu"
    """CPU-only inference."""


class ModelType(StrEnum):
    """Supported SAM and YOLO model architectures."""

    SAM1 = "sam1"
    """Segment Anything Model v1 (original SAM)."""

    SAM2 = "sam2"
    """Segment Anything Model v2 (improved SAM)."""

    SAM3 = "sam3"
    """Segment Anything Model v3 (latest SAM)."""

    YOLO11 = "yolo11"
    """YOLOv11 detection model."""

    YOLOE = "yoloe"
    """YOLOv8 efficient detection model."""


class ServerCommand(StrEnum):
    """gRPC server command types."""

    PING = "ping"
    """Health check / device status query."""

    LIST_MODELS = "list_models"
    """Get list of available models and their status."""

    SET_MODEL = "set_model"
    """Load a model into memory."""

    SET_IMAGE = "set_image"
    """Set the image for inference."""

    PREDICT = "predict"
    """Run point-prompted segmentation."""

    PREDICT_TEXT = "predict_text"
    """Run text-prompted segmentation."""

