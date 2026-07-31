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
    """Supported SAM and YOLO model architectures.

    ``GROUNDING_DINO`` has a ``config/models.toml`` entry but no loader in
    ``src/server/loader.py`` yet; it's listed so config parsing doesn't reject
    the whole file, but ``ModelRegistry`` reports it unavailable
    (``_is_available`` has no case for it) and ``load_model`` raises
    ``ModelLoadError`` if ever selected (``_MODEL_LOADERS`` has no entry for it).
    """

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

    GROUNDING_DINO = "grounding_dino"
    """Grounding DINO + SAM (config entry exists; not yet implemented)."""

