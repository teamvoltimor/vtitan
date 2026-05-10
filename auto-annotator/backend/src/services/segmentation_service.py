"""Segmentation inference service.

Wraps SAM inference pipeline and handles mask-to-shape conversion.
Complete abstraction that encapsulates all inference logic.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

from src.api.schemas import SegmentationShape
from src.coordinates import NormalizedPoint, YOLOPoint, yolo_bbox_to_corners
from src.db.repository import Repository
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.inference import run_sam_inference
from src.models import AppState, ClassInfo, Point

if TYPE_CHECKING:
    from src.models import AppContext


class SegmentationService:
    """SAM inference and mask-to-shape conversion.

    Complete abstraction: callers provide image ID and click points,
    receive segmentation shapes. All image loading, point conversion,
    inference execution, and error handling are internal.
    """

    def __init__(self, repository: Repository):
        """Initialize with repository for image access.

        Args:
            repository: Data access for image records.
        """
        self.repository = repository

    def segment(
        self,
        image_id: int,
        points: list[dict],
        app_context: AppContext,
        classes: list[ClassInfo],
    ) -> SegmentationShape | None:
        """Run SAM inference for click points and return best segmentation.

        ASSUMES input is already validated (non-empty points, valid classes).

        Args:
            image_id: Database image ID.
            points: List of {"x": float, "y": float, "pointType": "positive"|"negative"}.
            app_context: Application context with model client and inference state.
            classes: List of classes (assumed to include all classes in points).

        Returns:
            SegmentationShape for the best mask, or None if inference fails or mask too small.

        Raises:
            ValueError: If image not found.
        """
        record = self.repository.images.get_by_id(image_id)
        if record is None:
            raise ValueError(f"Image {image_id} not found")

        # Load image from disk
        try:
            image = Image.open(record.path).convert("RGB")
            image_np = np.array(image)
        except Exception as e:
            raise ValueError(f"Failed to load image: {e}") from e

        width, height = image.width, image.height

        # Find class for this segmentation (first positive point determines class)
        class_map = {cls.name: cls for cls in classes}

        # Convert frontend points (normalized 0-1) to inference points (pixels)
        inference_points: list[Point] = []
        selected_class = None

        for click in points:
            cls_info = class_map.get(click["className"])
            # Validation ensures cls_info is not None, so this is safe
            if cls_info and click["pointType"] == "positive" and selected_class is None:
                selected_class = cls_info

            # Normalize and convert to pixel coords
            nx = max(0.0, min(1.0, click["x"]))
            ny = max(0.0, min(1.0, click["y"]))
            px = int(nx * (width - 1))
            py = int(ny * (height - 1))

            label = 1 if click["pointType"] == "positive" else 0
            inference_points.append(Point(x=px, y=py, label=label, class_id=cls_info.id))

        # Validation ensures selected_class is not None
        if selected_class is None:
            selected_class = class_map[points[0]["className"]]

        # Run inference
        state = AppState(
            current_image=image_np,
            current_image_id=image_id,
            classes=classes,
            point_buffer=inference_points,
            pending_class_db_id=selected_class.id,
        )

        result = run_sam_inference(state, app_context.client, app_context.inference)
        if not result.ok or result.masks is None:
            raise ValueError(f"Inference failed: {result.error}")

        # Convert best mask to shape
        best_mask = result.masks[result.best_idx]
        return self._mask_to_shape(best_mask, selected_class.name, image_id, result.best_idx)

    def _mask_to_shape(
        self,
        mask: np.ndarray,
        class_name: str,
        image_id: int,
        mask_idx: int,
    ) -> SegmentationShape | None:
        """Convert SAM binary mask to YOLO polygon or bounding box.

        Attempts polygon conversion first (preserves detail), falls back
        to bounding box if polygon is too small or simplifies to <3 points.

        Args:
            mask: Boolean H×W binary mask from SAM.
            class_name: Annotation class name for this shape.
            image_id: Database image ID (for shape ID).
            mask_idx: Mask granularity index (0/1/2).

        Returns:
            SegmentationShape if conversion succeeds, None if mask too small.
        """
        polygon = mask_to_yolo_polygon(mask)
        if polygon:
            yolo_points = [YOLOPoint(x=polygon[i], y=polygon[i + 1]) for i in range(0, len(polygon), 2)]
            points = [p.to_normalized() for p in yolo_points]
            return SegmentationShape(
                id=f"mask-{image_id}-{mask_idx}",
                className=class_name,
                points=points,
            )

        bbox = mask_to_yolo_bbox(mask)
        if bbox:
            xc, yc, w, h = bbox
            yolo_points = yolo_bbox_to_corners(xc, yc, w, h)
            points = [p.to_normalized() for p in yolo_points]
            return SegmentationShape(
                id=f"bbox-{image_id}-{mask_idx}",
                className=class_name,
                points=points,
            )

        return None
