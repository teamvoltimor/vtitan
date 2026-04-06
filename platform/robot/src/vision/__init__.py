"""Vision module exports and detector factory."""

from src.vision.detector import (
    DetectorBase,
    DetectorConfig,
    HailoDetector,
    LocalYoloDetector,
    SignDetection,
    TrafficSignColor,
)

__all__ = [
    "DetectorBase",
    "DetectorConfig",
    "LocalYoloDetector",
    "HailoDetector",
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
        DetectorBase: Instantiated detector.

    Raises:
        ValueError: If backend is not recognized.
    """
    if backend == "yolo":
        if config is None:
            config = DetectorConfig(
                model_path="yolov8n.pt",
                class_to_color={
                    0: TrafficSignColor.RED,
                    1: TrafficSignColor.GREEN,
                    2: TrafficSignColor.MAGENTA,
                },
            )
        return LocalYoloDetector(config)
    elif backend == "hailo":
        if config is None:
            config = DetectorConfig(
                model_path="models/traffic_signs.hef",
                class_to_color={
                    0: TrafficSignColor.RED,
                    1: TrafficSignColor.GREEN,
                    2: TrafficSignColor.MAGENTA,
                },
            )
        return HailoDetector(config.model_path, config)
    else:
        raise ValueError(f"Unknown detector backend: {backend}")
