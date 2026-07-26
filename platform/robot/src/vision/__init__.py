"""Vision module exports and detector factory."""

from shared.domain.models import Detection

from src.vision.detector import (
    DEFAULT_CLASS_TO_COLOR,
    BBoxFormat,
    DetectorBase,
    DetectorConfig,
    HailoDetector,
    LocalYoloDetector,
    SignDetection,
    TrafficSignColor,
)

__all__ = [
    "DEFAULT_CLASS_TO_COLOR",
    "BBoxFormat",
    "Detection",
    "DetectorBase",
    "DetectorConfig",
    "HailoDetector",
    "LocalYoloDetector",
    "SignDetection",
    "TrafficSignColor",
    "create_detector",
]


def create_detector(backend: str = "yolo", config: DetectorConfig | None = None) -> DetectorBase:
    """Create a detector instance with optional configuration injection.

    Args:
        backend: Detector backend ('yolo' or 'hailo').
        config: Optional DetectorConfig for custom model path and class mappings.

    Returns:
        DetectorBase: Instantiated detector. The 'hailo' backend returns a
        HailoDetector that must be used as a context manager so the inference
        pipeline stays open for the session lifetime.

    Raises:
        ValueError: If backend is not recognized.
    """
    if backend == "yolo":
        if config is None:
            config = DetectorConfig(model_path="yolov8n.pt", class_to_color=DEFAULT_CLASS_TO_COLOR)
        return LocalYoloDetector(config)
    if backend == "hailo":
        try:
            from src.hardware.hailo.base import Config as HailoConfig  # noqa: PLC0415
            from src.hardware.hailo.hailo_8.driver import Driver  # noqa: PLC0415
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
