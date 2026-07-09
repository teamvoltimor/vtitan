"""Segmentation inference service.

Wraps SAM inference pipeline and handles mask-to-shape conversion.
Complete abstraction that encapsulates all inference logic.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

from src.coordinates import NormalizedPoint, YOLOPoint, yolo_bbox_to_corners
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.inference import run_sam_inference
from src.models import ClassInfo, InferenceRequest, Point, Shape

if TYPE_CHECKING:
    from src.models import AppContext, ImageRepositoryProtocol


class SegmentationService:
    """SAM inference and mask-to-shape conversion.

    Complete abstraction: callers provide image ID and click points,
    receive segmentation shapes. All image loading, point conversion,
    inference execution, and error handling are internal.
    """

    def __init__(self, repository: ImageRepositoryProtocol | None):
        """Initialize with repository for image access.

        ``repository`` is only read by :meth:`segment`; the gRPC-facing
        :meth:`segment_path` resolves image paths itself and accepts
        ``None`` here (see ``src.grpc_server.servicers.SegmentationServicer``).
        """
        self.repository = repository

    def segment(
        self,
        image_id: int,
        points: list[dict],
        app_context: AppContext,
        classes: list[ClassInfo],
    ) -> Shape | None:
        """Run SAM inference for click points and return best segmentation.

        ASSUMES input is already validated (non-empty points, valid classes).

        Args:
            image_id: Database image ID.
            points: List of {"x": float, "y": float, "pointType": "positive"|"negative"}.
            app_context: Application context with model client and inference state.
            classes: List of classes (assumed to include all classes in points).

        Returns:
            Shape for the best mask, or None if inference fails or mask too small.

        """
        if self.repository is None:
            msg = "segment() requires a repository; use segment_path() when none is available"
            raise ValueError(msg)
        record = self.repository.images.get_by_id(image_id)
        return self.segment_path(record.path, image_id, points, app_context, classes)

    def segment_path(
        self,
        image_path: str,
        image_id: int,
        points: list[dict],
        app_context: AppContext,
        classes: list[ClassInfo],
    ) -> Shape | None:
        """Run SAM inference given an explicit image path (no repository lookup).

        Used by the gRPC SegmentationService, where the caller (the Go API, the
        single DB owner) resolves and supplies the image path.
        """
        # Load image from disk
        try:
            image = Image.open(image_path).convert("RGB")
            image_np = np.array(image)
        except Exception as e:
            msg = f"Failed to load image: {e}"
            raise ValueError(msg) from e

        width, height = image.width, image.height

        # Find class for this segmentation (first positive point determines class)
        class_map = {cls.name: cls for cls in classes}

        # Convert frontend points (normalized 0-1) to inference points (pixels)
        inference_points: list[Point] = []
        selected_class = None

        for click in points:
            cls_info = class_map.get(click["class_name"])
            # Validation ensures cls_info is not None, so this is safe
            if cls_info and click["point_type"] == "positive" and selected_class is None:
                selected_class = cls_info

            pixel = NormalizedPoint(x=click["x"], y=click["y"]).to_pixel(width, height)
            label = 1 if click["point_type"] == "positive" else 0
            inference_points.append(Point(x=pixel.x, y=pixel.y, label=label, class_id=cls_info.id))

        # Validation ensures selected_class is not None
        if selected_class is None:
            selected_class = class_map[points[0]["class_name"]]

        result = run_sam_inference(
            InferenceRequest(image=image_np, points=inference_points),
            app_context.client,
            app_context.inference,
        )
        if not result.ok or result.masks is None:
            msg = f"Inference failed: {result.error}"
            raise ValueError(msg)

        # Convert best mask to shape
        best_mask = result.masks[result.best_idx]
        return self._mask_to_shape(best_mask, selected_class.name, image_id, result.best_idx)

    def _mask_to_shape(
        self,
        mask: np.ndarray,
        class_name: str,
        image_id: int,
        mask_idx: int,
    ) -> Shape | None:
        """Convert SAM binary mask to YOLO polygon or bounding box.

        Attempts polygon conversion first (preserves detail), falls back
        to bounding box if polygon is too small or simplifies to <3 points.

        Args:
            mask: Boolean HxW binary mask from SAM.
            class_name: Annotation class name for this shape.
            image_id: Database image ID (for shape ID).
            mask_idx: Mask granularity index (0/1/2).

        Returns:
            Shape if conversion succeeds, None if mask too small.
        """
        polygon = mask_to_yolo_polygon(mask)
        if polygon:
            yolo_points = [YOLOPoint(x=polygon[i], y=polygon[i + 1]) for i in range(0, len(polygon), 2)]
            points = [p.to_normalized() for p in yolo_points]
            return Shape(
                id=f"mask-{image_id}-{mask_idx}",
                class_name=class_name,
                points=points,
            )

        bbox = mask_to_yolo_bbox(mask)
        if bbox:
            xc, yc, w, h = bbox
            yolo_points = yolo_bbox_to_corners(xc, yc, w, h)
            points = [p.to_normalized() for p in yolo_points]
            return Shape(
                id=f"bbox-{image_id}-{mask_idx}",
                class_name=class_name,
                points=points,
            )

        return None
