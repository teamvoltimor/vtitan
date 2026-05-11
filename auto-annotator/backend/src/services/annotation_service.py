"""Annotation loading and parsing service.

Eliminates duplicate annotation parsing logic that was scattered
across gallery, annotations, and grouped gallery endpoints.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from src.api.schemas import SegmentationShape
from src.coordinates import yolo_bbox_to_corners, yolo_coords_to_polygon
from src.enums import Status

if TYPE_CHECKING:
    from src.gallery_cache import AnnotationCache
    from src.models import ClassInfo, ImageRecord


class AnnotationService:
    """Load and convert saved annotations for images."""

    def __init__(self, cache: AnnotationCache):
        """Initialize with annotation cache.

        Args:
            cache: AnnotationCache for mtime-aware label file parsing.
        """
        self.cache = cache

    def load_annotations(
        self, image: ImageRecord, classes: list[ClassInfo],
    ) -> list[SegmentationShape]:
        """Load and parse saved annotations for an image.

        Returns empty list if image is not done or has no saved labels.

        Args:
            image: ImageRecord to load annotations for.
            classes: List of class objects for ID → name mapping.

        Returns:
            List of SegmentationShape objects (polygon or bbox).
        """
        if image.status != Status.DONE or not image.format:
            return []

        img_path = Path(image.path)
        class_dir = img_path.parent.name
        label_lines = self.cache.get(class_dir, img_path.stem)

        if not label_lines:
            return []

        idx_to_name: dict[int, str] = {idx: cls.name for idx, cls in enumerate(classes)}
        fmt = image.format

        shapes: list[SegmentationShape] = []
        for i, raw in enumerate(label_lines):
            parts = raw.strip().split()
            if not parts:
                continue

            try:
                class_id = int(parts[0])
                cls_name = idx_to_name.get(class_id, f"class_{class_id}")
                coords = [float(v) for v in parts[1:]]
            except (ValueError, IndexError):
                continue

            # Parse YOLO format (det or seg) to normalized points
            if fmt == "det" and len(coords) == 4:
                xc, yc, w, h = coords
                yolo_points = yolo_bbox_to_corners(xc, yc, w, h)
                points = [p.to_normalized() for p in yolo_points]
            elif len(coords) >= 4 and len(coords) % 2 == 0:
                yolo_points = yolo_coords_to_polygon(coords)
                points = [p.to_normalized() for p in yolo_points]
            else:
                continue

            shapes.append(
                SegmentationShape(
                    id=f"loaded-{image.id}-{i}",
                    className=cls_name,
                    points=points,
                ),
            )

        return shapes
