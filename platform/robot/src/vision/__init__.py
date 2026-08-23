"""Vision module exports and detector factory."""

from enum import StrEnum

from shared.domain.models import Detection, SignColor

from src.vision.detector import (
    DEFAULT_CLASS_TO_COLOR,
    BBoxFormat,
    DetectorBase,
    DetectorConfig,
    HailoDetector,
    LocalYoloDetector,
)


class VisionBackend(StrEnum):
    """Available vision detector backends."""

    YOLO = "yolo"
    """Local YOLOv8 inference."""

    HAILO = "hailo"
    """Hailo-8 NPU inference (Raspberry Pi 5 only)."""


__all__ = [
    "DEFAULT_CLASS_TO_COLOR",
    "BBoxFormat",
    "Detection",
    "DetectorBase",
    "DetectorConfig",
    "HailoDetector",
    "LocalYoloDetector",
    "SignColor",
    "VisionBackend",
    "create_detector",
]


def create_detector(
    backend: VisionBackend | str = VisionBackend.YOLO, config: DetectorConfig | None = None
) -> DetectorBase:
    """Create a detector instance with optional configuration injection.

    Args:
        backend: Detector backend (VisionBackend enum or string 'yolo'/'hailo' for backward compat).
        config: Optional DetectorConfig for custom model path and class mappings.

    Returns:
        DetectorBase: Instantiated detector. The 'hailo' backend returns a
        HailoDetector that must be used as a context manager so the inference
        pipeline stays open for the session lifetime.

    Raises:
        ValueError: If backend is not recognized.
    """
    backend = VisionBackend(backend) if isinstance(backend, str) else backend
    if backend == VisionBackend.YOLO:
        if config is None:
            config = DetectorConfig(class_to_color=DEFAULT_CLASS_TO_COLOR)
        return LocalYoloDetector(config)
    if backend == VisionBackend.HAILO:
        try:
            from src.hardware.hailo.base import Config as HailoConfig
            from src.hardware.hailo.hailo_8.driver import Driver
        except ImportError as e:
            msg = "hailo_platform not found. Are you running on the Raspberry Pi 5 with HailoRT installed?"
            raise ImportError(msg) from e
        if config is None:
            config = DetectorConfig(
                model_path=HailoConfig().model_path,
                class_to_color=DEFAULT_CLASS_TO_COLOR,
            )
        return HailoDetector(Driver(HailoConfig(model_path=config.model_path)), config)
    msg = f"Unknown detector backend: {backend}"
    raise ValueError(msg)
