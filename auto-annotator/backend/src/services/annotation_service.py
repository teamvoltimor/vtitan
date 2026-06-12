"""Annotation loading and parsing service."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from src.coordinates import yolo_bbox_to_corners, yolo_coords_to_polygon
from src.enums import ExportFormat, Status
from src.models import Shape

if TYPE_CHECKING:
    from src.label_store import LabelStore
    from src.models import ClassInfo, ImageRecord


class AnnotationService:
    """Load and convert saved annotations for images."""

    def __init__(self, label_store: LabelStore) -> None:
        self.label_store = label_store

    def load_annotations(self, image: ImageRecord, classes: list[ClassInfo]) -> list[Shape]:
        """Load and parse saved annotations for an image.

        Returns empty list if image is not done or has no saved labels.
        """
        if image.status != Status.DONE or not image.format_used:
            return []

        img_path = Path(image.path)
        records = self.label_store.load(img_path.parent.name, img_path.stem)
        if not records:
            return []

        idx_to_name: dict[int, str] = {idx: cls.name for idx, cls in enumerate(classes)}
        fmt = image.format_used

        shapes: list[Shape] = []
        for i, record in enumerate(records):
            cls_name = idx_to_name.get(record.class_id, f"class_{record.class_id}")
            coords = record.coords

            if fmt == ExportFormat.DET and len(coords) == 4:
                xc, yc, w, h = coords
                points = [p.to_normalized() for p in yolo_bbox_to_corners(xc, yc, w, h)]
            elif len(coords) >= 4 and len(coords) % 2 == 0:
                points = [p.to_normalized() for p in yolo_coords_to_polygon(coords)]
            else:
                continue

            shapes.append(Shape(id=f"loaded-{image.id}-{i}", class_name=cls_name, points=points))

        return shapes
