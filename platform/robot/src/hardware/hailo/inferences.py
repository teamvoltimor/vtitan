from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

if TYPE_CHECKING:
    import numpy as np

_BATCHED_NMS_TENSOR_NDIM = 4
"""Rank of a Hailo NMS output tensor that still carries its batch dimension."""


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
        raw_tensor: object,
        img_width: int,
        img_height: int,
        class_map: dict[int, str],
        latency_ms: float,
        conf_threshold: float = 0.5,
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
            image: Optional original image for streaming/visualization.

        Returns:
            InferenceResult containing a list of YoloDetections and the latency.
        """
        detections = []

        # The raw tensor may have an extra batch dimension (e.g., shape [1, num_classes, max_boxes, 5])
        if raw_tensor.ndim == _BATCHED_NMS_TENSOR_NDIM and raw_tensor.shape[0] == 1:
            raw_tensor = raw_tensor[0]

        # The tensor is expected to have shape [num_classes, max_boxes, 5] where the last dimension contains [ymin, xmin, ymax, xmax, confidence]
        num_classes = raw_tensor.shape[0]

        # Iterate over each class and its detected boxes
        for class_id in range(num_classes):
            class_boxes = raw_tensor[class_id]

            # Each box is expected to be in the format [ymin, xmin, ymax, xmax, confidence]
            for box in class_boxes:
                confidence = box[4]
                if confidence < conf_threshold:
                    continue

                bbox = BoundingBox.parse_from_yolo_format(
                    ymin=box[0],
                    xmin=box[1],
                    ymax=box[2],
                    xmax=box[3],
                    img_width=img_width,
                    img_height=img_height,
                )

                # Create a YoloDetection instance with the class ID, class name, confidence, and bounding box
                detection = YoloDetection(
                    class_id=class_id,
                    class_name=class_map.get(class_id, f"Unknown_{class_id}"),
                    confidence=confidence,
                    bbox=bbox,
                )

                detections.append(detection)

        return InferenceResult(detections=detections, latency_ms=latency_ms, image=image)
