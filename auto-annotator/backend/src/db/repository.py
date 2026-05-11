"""Repository abstraction layer for image and class data.

Provides a single interface for all data access, enabling:
- Dependency injection in services and routers
- Mocking in unit tests
- Future storage backend changes (SQLite → PostgreSQL, etc.)
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

from src.db import core as db_core

if TYPE_CHECKING:
    from src.models import BrowseRow, ClassInfo, GroupedRow, ImageRecord, StatsResult


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

    def register_augmented(self, path: str, format_used: str, parent_id: int) -> int:
        """Register an augmented image as a new record.

        Args:
            path: Absolute path to the augmented image file.
            format_used: Export format (``"seg"`` or ``"det"``).
            parent_id: DB id of the original image.

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

    def __init__(self) -> None:
        """Initialize with default database connections."""
        self._db = db_core

    @property
    def images(self) -> ImageRepository:
        """Images repository."""
        return _DefaultImageRepository(self._db)

    @property
    def classes(self) -> ClassRepository:
        """Classes repository."""
        return _DefaultClassRepository(self._db)

    @property
    def stats(self) -> StatsRepository:
        """Stats repository."""
        return _DefaultStatsRepository(self._db)


class _DefaultImageRepository:
    """ImageRepository backed by db.core."""

    def __init__(self, db: object) -> None:
        self._db = db

    def get_by_id(self, image_id: int) -> ImageRecord | None:
        return db_core.get_by_id(image_id)

    def get_all(self) -> list[BrowseRow]:
        return db_core.get_all_images()

    def get_grouped(self) -> list[GroupedRow]:
        return db_core.get_grouped_images()

    def add_from_paths(self, paths: list[str]) -> None:
        return db_core.add_images_from_paths(paths)

    def delete(self, image_id: int) -> None:
        return db_core.delete_image(image_id)

    def mark_done(self, image_id: int, export_format: str) -> None:
        return db_core.mark_done(image_id, export_format)

    def mark_skipped(self, image_id: int) -> None:
        return db_core.mark_skipped(image_id)

    def register_augmented(self, path: str, format_used: str, parent_id: int) -> int:
        return db_core.register_augmented_image(path, format_used, parent_id)


class _DefaultClassRepository:
    """ClassRepository backed by db.core."""

    def __init__(self, db: object) -> None:
        self._db = db

    def get_all(self) -> list[ClassInfo]:
        return db_core.get_classes()

    def upsert(self, name: str, color: str) -> None:
        return db_core.upsert_class(name, color)


class _DefaultStatsRepository:
    """StatsRepository backed by db.core."""

    def __init__(self, db: object) -> None:
        self._db = db

    def get_stats(self) -> StatsResult:
        return db_core.get_stats()
