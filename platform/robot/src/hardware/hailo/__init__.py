"""Hailo module exports.

``StreamingDriver`` is resolved lazily: it pulls in the camera stack and so
``picamera2``, which only exists on the Pi. Importing it eagerly made every
module in this package unimportable off-device, including the pure-Python NMS
decoding in :mod:`~src.hardware.hailo.inferences` that has no hardware
dependency at all and needs to be unit-testable anywhere.
"""

from typing import TYPE_CHECKING

from src.hardware.hailo.base import Driver, InferenceResult
from src.hardware.hailo.config import Config, StreamingConfig
from src.hardware.hailo.inferences import BoundingBox, YoloDetection, iter_nms_by_class

if TYPE_CHECKING:
    from src.hardware.hailo.streaming import StreamingDriver

__all__ = [
    "BoundingBox",
    "Config",
    "Driver",
    "InferenceResult",
    "StreamingConfig",
    "StreamingDriver",
    "YoloDetection",
    "iter_nms_by_class",
]


def __getattr__(name: str) -> object:
    """Import the camera-dependent exports only when they are asked for."""
    if name == "StreamingDriver":
        from src.hardware.hailo.streaming import StreamingDriver  # noqa: PLC0415

        return StreamingDriver
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
