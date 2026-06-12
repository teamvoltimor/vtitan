"""Repository abstraction layer for image and class data.

Provides a single interface for all data access, enabling:
- Dependency injection in services and routers
- Mocking in unit tests
- Future storage backend changes (SQLite → PostgreSQL, etc.)
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from src.db import core as db_core

if TYPE_CHECKING:
    from src.models import AugmentedImage, BrowseRow, ClassInfo, GroupedRow, ImageRecord, StatsResult


class ImageRepository(Protocol):
    """Interface for image data access."""

    def get_by_id(self, image_id: int) -> ImageRecord | None:
        """Retrieve image record by ID."""
        ...

    def get_all(self) -> list[BrowseRow]:
        """Retrieve all images with basic info."""
        ...

    def get_grouped(self) -> list[GroupedRow]:
        """Retrieve all images grouped (with augmentation count)."""
        ...

    def add_from_paths(self, paths: list[str]) -> None:
        """Import images from file paths."""
        ...

    def delete(self, image_id: int) -> None:
        """Delete image record by ID."""
        ...

    def mark_done(self, image_id: int, export_format: str) -> None:
        """Mark image as annotated with given export format."""
        ...

    def mark_skipped(self, image_id: int) -> None:
        """Mark image as skipped."""
        ...

    def register_augmented(self, image: AugmentedImage) -> int:
        """Register an augmented image as a new record.

        Args:
            image: Domain object describing the new augmented image.

        Returns:
            Integer primary-key id of the newly inserted record.
        """
        ...


class ClassRepository(Protocol):
    """Interface for class data access."""

    def get_all(self) -> list[ClassInfo]:
        """Retrieve all annotation classes."""
        ...

    def upsert(self, name: str, color: str) -> None:
        """Insert or update a class by name."""
        ...


class StatsRepository(Protocol):
    """Interface for aggregate statistics."""

    def get_stats(self) -> StatsResult:
        """Get image status counts and percentage complete."""
        ...


class Repository(Protocol):
    """Unified repository for all data access."""

    images: ImageRepository
    classes: ClassRepository
    stats: StatsRepository


class DefaultRepository:
    """Default repository implementation backed by SQLite."""

    def __init__(self, db_path: Path | None = None) -> None:
        """Initialize with an explicit db_path, or fall back to the module default."""
        self._db_path = db_path

    @property
    def images(self) -> ImageRepository:
        """Images repository."""
        return _DefaultImageRepository(self._db_path)

    @property
    def classes(self) -> ClassRepository:
        """Classes repository."""
        return _DefaultClassRepository(self._db_path)

    @property
    def stats(self) -> StatsRepository:
        """Stats repository."""
        return _DefaultStatsRepository(self._db_path)


class _DefaultImageRepository:
    """ImageRepository backed by db.core, bound to a specific db_path."""

    def __init__(self, db_path: Path | None) -> None:
        self._db_path = db_path

    def get_by_id(self, image_id: int) -> ImageRecord | None:
        return db_core.get_by_id(image_id, self._db_path)

    def get_all(self) -> list[BrowseRow]:
        return db_core.get_all_images(self._db_path)

    def get_grouped(self) -> list[GroupedRow]:
        return db_core.get_grouped_images(self._db_path)

    def add_from_paths(self, paths: list[str]) -> None:
        db_core.add_images_from_paths(paths, self._db_path)

    def delete(self, image_id: int) -> None:
        db_core.delete_image(image_id, self._db_path)

    def mark_done(self, image_id: int, export_format: str) -> None:
        db_core.mark_done(image_id, export_format, self._db_path)

    def mark_skipped(self, image_id: int) -> None:
        db_core.mark_skipped(image_id, self._db_path)

    def register_augmented(self, image: AugmentedImage) -> int:
        return db_core.register_augmented_image(image.path, image.format_used, image.parent_id, self._db_path)


class _DefaultClassRepository:
    """ClassRepository backed by db.core, bound to a specific db_path."""

    def __init__(self, db_path: Path | None) -> None:
        self._db_path = db_path

    def get_all(self) -> list[ClassInfo]:
        return db_core.get_classes(self._db_path)

    def upsert(self, name: str, color: str) -> None:
        db_core.upsert_class(name, color, self._db_path)


class _DefaultStatsRepository:
    """StatsRepository backed by db.core, bound to a specific db_path."""

    def __init__(self, db_path: Path | None) -> None:
        self._db_path = db_path

    def get_stats(self) -> StatsResult:
        return db_core.get_stats(self._db_path)
