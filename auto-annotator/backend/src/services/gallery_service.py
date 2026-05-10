"""Gallery and image listing service.

Handles gallery response building, grouping, and annotation loading.
Separates business logic from HTTP endpoints.
"""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path

from src.api.schemas import (
    GalleryItem,
    GalleryResponse,
    GalleryStats,
    GroupedGalleryItem,
    GroupedGalleryResponse,
    SegmentationShape,
)
from src.constants import API_PUBLIC_URL
from src.db.repository import Repository
from src.enums import Status
from src.models import BrowseRow, ClassInfo, GroupedRow
from src.services.annotation_service import AnnotationService


class GalleryService:
    """Build gallery responses from database and annotations."""

    def __init__(self, repository: Repository, annotation_service: AnnotationService):
        """Initialize with repository and annotation service.

        Args:
            repository: Data access abstraction for images, classes, stats.
            annotation_service: Service for loading annotations.
        """
        self.repository = repository
        self.annotation_service = annotation_service

    def build_gallery_response(self) -> GalleryResponse:
        """Build complete gallery response from database and cache.

        Returns:
            GalleryResponse with all images and statistics.
        """
        rows = self.repository.images.get_all()
        stats = self.repository.stats.get_stats()
        classes = self.repository.classes.get_all()

        items = [self._row_to_gallery_item(row, classes) for row in rows]

        return GalleryResponse(
            items=items,
            stats=GalleryStats(
                pending=stats.pending,
                done=stats.done,
                skipped=stats.skipped,
                total=stats.total,
                pct=stats.pct,
            ),
        )

    def build_grouped_gallery_response(self) -> GroupedGalleryResponse:
        """Build gallery grouped by class.

        Returns:
            GroupedGalleryResponse with groups organized by class.
        """
        rows = self.repository.images.get_grouped()
        stats = self.repository.stats.get_stats()
        classes = self.repository.classes.get_all()

        groups_dict: dict[str, list[BrowseRow]] = defaultdict(list)
        for row in rows:
            groups_dict[row.class_name].append(row)

        groups = [
            GroupedGalleryItem(
                className=class_name,
                count=len(items),
                images=[self._row_to_gallery_item(row, classes) for row in items],
            )
            for class_name, items in sorted(groups_dict.items())
        ]

        return GroupedGalleryResponse(
            groups=groups,
            stats=GalleryStats(
                pending=stats.pending,
                done=stats.done,
                skipped=stats.skipped,
                total=stats.total,
                pct=stats.pct,
            ),
        )

    def _row_to_gallery_item(self, row: BrowseRow, classes: list[ClassInfo]) -> GalleryItem:
        """Convert database row to gallery item with annotations.

        Args:
            row: Database image record.
            classes: List of classes for annotation parsing.

        Returns:
            GalleryItem with annotations loaded.
        """
        # Get image record for annotation loading
        image_record = db.get_by_id(row.id)
        if image_record is None:
            annotations: list[SegmentationShape] = []
        else:
            annotations = self.annotation_service.load_annotations(image_record, classes)

        return GalleryItem(
            id=row.id,
            label=row.filename,
            src=f"{API_PUBLIC_URL}/images/{row.id}",
            format=row.format or "",
            status=row.status,
            updated=row.updated_at or "",
            annotations=annotations,
        )
