from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Iterator

_BATCHED_NMS_TENSOR_NDIM = 4
"""Rank of a Hailo NMS output tensor that still carries its batch dimension."""

_BOX_FIELDS = 5
"""Values per NMS box: ``[y_min, x_min, y_max, x_max, confidence]``."""

_NMS_TENSOR_NDIM = 3
"""Rank of a by-class NMS tensor once any batch dimension is stripped."""

Detection = tuple[int, float, tuple[float, float, float, float]]


class NmsFormatError(ValueError):
    """Raised when an NMS output tensor has no recognised by-class layout."""


def _unpack(class_id: int, box: np.ndarray) -> Detection:
    """Return one box as ``(class_id, confidence, corners)``."""
    return class_id, float(box[4]), (float(box[0]), float(box[1]), float(box[2]), float(box[3]))


def _as_boxes_last(tensor: np.ndarray) -> np.ndarray:
    """Normalise a by-class tensor to ``(n_classes, n_boxes, 5)``.

    Raises:
        NmsFormatError: When no axis holds the five box fields.
    """
    if tensor.ndim == _BATCHED_NMS_TENSOR_NDIM and tensor.shape[0] == 1:
        tensor = tensor[0]

    if tensor.ndim != _NMS_TENSOR_NDIM:
        msg = f"Expected a {_NMS_TENSOR_NDIM}-D NMS-by-class tensor, got shape {tensor.shape}."
        raise NmsFormatError(msg)

    # Prefer the boxes-last reading; fall back to the emulator's transposed one.
    if tensor.shape[-1] == _BOX_FIELDS:
        return tensor
    if tensor.shape[1] == _BOX_FIELDS:
        return tensor.transpose(0, 2, 1)

    msg = f"NMS tensor {tensor.shape} has no axis of {_BOX_FIELDS} box fields."
    raise NmsFormatError(msg)


def _iter_per_class_sequence(sequence: list | tuple) -> Iterator[Detection]:
    """Yield boxes from one variable-length array per class."""
    for class_id, boxes in enumerate(sequence):
        array = np.asarray(boxes)
        if array.size == 0:
            continue
        for box in array.reshape(-1, array.shape[-1]):
            yield _unpack(class_id, box)


def iter_nms_by_class(raw_output: object) -> Iterator[Detection]:
    """Yield ``(class_id, confidence, (ymin, xmin, ymax, xmax))`` from NMS output.

    The GMR HEF's output is ``HAILO NMS BY CLASS``: class identity is
    *positional*, and each box carries only five values, so there is no class-id
    column to read. HailoRT reports this in several shapes depending on version
    and whether the batch dimension survives, and the SDK emulator packs it
    differently again, so every known layout is normalised here rather than
    assumed at each call site.

    Accepted:
        * a sequence of per-class arrays, each ``(n_boxes, 5)``
        * ``(n_classes, n_boxes, 5)``
        * ``(1, n_classes, n_boxes, 5)``
        * ``(n_classes, 5, n_boxes)`` -- the SDK emulator's packing

    Args:
        raw_output: Whatever the driver returned for the NMS output.

    Yields:
        One tuple per box, with normalised corner coordinates.

    Raises:
        NmsFormatError: When the layout matches none of the above -- better a
            loud failure than silently reading coordinates as confidences.
    """
    # A list of per-class arrays: the class is the index, boxes vary per class.
    if isinstance(raw_output, list | tuple):
        yield from _iter_per_class_sequence(raw_output)
        return

    for class_id, boxes in enumerate(_as_boxes_last(np.asarray(raw_output))):
        for box in boxes:
            yield _unpack(class_id, box)


class BoundingBox(NamedTuple):
    """Represents a bounding box for detected objects in an image."""

    x: int
    y: int
    width: int
    height: int

    @classmethod
    def parse_from_yolo_format(
        cls,
        ymin: float,
        xmin: float,
        ymax: float,
        xmax: float,
        img_width: int,
        img_height: int,
    ) -> BoundingBox:
        """
        Parse bounding box from YOLO format (normalized coordinates) to pixel coordinates.

        Args:
            ymin: Normalized top-left y-coordinate of the bounding box (0 to 1).
            xmin: Normalized top-left x-coordinate of the bounding box (0 to 1).
            ymax: Normalized bottom-right y-coordinate of the bounding box (0 to 1).
            xmax: Normalized bottom-right x-coordinate of the bounding box (0 to 1).
            img_width: Original image width in pixels for scaling.
            img_height: Original image height in pixels for scaling

        Returns:
            BoundingBox instance with pixel coordinates and dimensions.
        """
        x = int(xmin * img_width)
        y = int(ymin * img_height)
        width = int((xmax - xmin) * img_width)
        height = int((ymax - ymin) * img_height)
        return cls(x=x, y=y, width=width, height=height)


class YoloDetection(NamedTuple):
    """Represents a single detection from a YOLO model."""

    class_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox


class InferenceResult(NamedTuple):
    """Represents the result of an inference, including detections and latency."""

    detections: list[YoloDetection]
    latency_ms: float
    image: np.ndarray | None = None

    @classmethod
    def parse_yolo_nms_output(
        cls,
        raw_tensor: np.ndarray,
        img_width: int,
        img_height: int,
        class_map: dict[int, str],
        latency_ms: float,
        conf_threshold: float | None = None,
        image: np.ndarray | None = None,
    ) -> InferenceResult:
        """
        Parse Hailo NMS output tensor to typed detections.

        Args:
            raw_tensor: The raw output tensor from the Hailo NMS layer.
            img_width: Original image width for scaling bounding boxes.
            img_height: Original image height for scaling bounding boxes.
            class_map: Mapping of class IDs to human-readable class names.
            latency_ms: Inference latency in milliseconds to include in the result.
            conf_threshold: Minimum confidence threshold to filter detections.
                Defaults to the configured value (hailo.toml ``min_confidence``) when
                not passed -- the caller that owns a config always passes it
                explicitly, so this only fires for standalone/fallback callers.
            image: Optional original image for streaming/visualization.

        Returns:
            InferenceResult containing a list of YoloDetections and the latency.
        """
        if conf_threshold is None:
            from src.hardware.hailo.config import Config as HailoConfig

            conf_threshold = HailoConfig().min_confidence
        detections = []
        for class_id, confidence, (ymin, xmin, ymax, xmax) in iter_nms_by_class(raw_tensor):
            if confidence < conf_threshold:
                continue

            detections.append(
                YoloDetection(
                    class_id=class_id,
                    class_name=class_map.get(class_id, f"Unknown_{class_id}"),
                    confidence=confidence,
                    bbox=BoundingBox.parse_from_yolo_format(
                        ymin=ymin,
                        xmin=xmin,
                        ymax=ymax,
                        xmax=xmax,
                        img_width=img_width,
                        img_height=img_height,
                    ),
                ),
            )

        return InferenceResult(detections=detections, latency_ms=latency_ms, image=image)
