"""src.gallery_cache – In-memory cache of parsed annotation files.

Eliminates duplicated label-file parsing on every gallery request.
Watches the label directory for changes and invalidates on updates.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.utils import get_logger

if TYPE_CHECKING:
    from pathlib import Path

logger = get_logger(__name__)


class AnnotationCache:
    """In-memory cache of parsed annotations from label files.

    Parses label files once on first access, invalidates on file changes.

    Attributes:
        _labels_dir: Root directory containing per-class label subdirectories.
        _cache: Dict mapping (class_dir, image_stem) → list of label lines.
        _mtime: Dict mapping file path → modification time for change detection.
    """

    def __init__(self, labels_dir: Path | None = None):
        """Initialize empty cache.

        Args:
            labels_dir: Root labels directory. Falls back to AppConfig when omitted.
        """
        if labels_dir is None:
            from src.core.config import AppConfig  # noqa: PLC0415
            labels_dir = AppConfig.load().paths.labels_dir
        self._labels_dir: Path = labels_dir
        self._cache: dict[tuple[str, str], list[str]] = {}
        self._mtime: dict[Path, float] = {}

    def get(self, class_dir: str, image_stem: str) -> list[str]:
        """Get parsed annotation lines for an image.

        Returns list of label lines (space-separated class_id and coordinates).
        Returns empty list if file doesn't exist or is unparseable.

        Args:
            class_dir: Directory name of the image class.
            image_stem: Filename stem (without extension).

        Returns:
            List of label lines (each is "class_id coord1 coord2 ...").
        """
        cache_key = (class_dir, image_stem)
        label_path = self._labels_dir / class_dir / (image_stem + ".txt")

        if not label_path.exists():
            self._cache.pop(cache_key, None)
            return []

        try:
            current_mtime = label_path.stat().st_mtime
            cached_mtime = self._mtime.get(label_path)

            if cached_mtime == current_mtime and cache_key in self._cache:
                return self._cache[cache_key]

            content = label_path.read_text(encoding="utf-8")
            lines = [line.strip() for line in content.splitlines() if line.strip()]
            self._cache[cache_key] = lines
            self._mtime[label_path] = current_mtime
        except (OSError, UnicodeDecodeError) as e:
            logger.warning(
                "Failed to read label file",
                extra={"_extra": {"path": str(label_path), "error": str(e)}},
            )
            self._cache.pop(cache_key, None)
            return []

        return lines

    def invalidate(self, class_dir: str | None = None, image_stem: str | None = None) -> None:
        """Clear cached annotations for a directory or entire cache.

        Args:
            class_dir: Optional directory to invalidate. If None, clears all.
            image_stem: Optional specific image. If None, clears entire class_dir.
        """
        if class_dir is None:
            self._cache.clear()
            self._mtime.clear()
        elif image_stem is None:
            keys_to_remove = [k for k in self._cache if k[0] == class_dir]
            for k in keys_to_remove:
                self._cache.pop(k, None)
            paths_to_remove = [p for p in self._mtime if p.parent.name == class_dir]
            for p in paths_to_remove:
                self._mtime.pop(p, None)
        else:
            cache_key = (class_dir, image_stem)
            self._cache.pop(cache_key, None)
            label_path = self._labels_dir / class_dir / (image_stem + ".txt")
            self._mtime.pop(label_path, None)

    def __repr__(self) -> str:
        return f"AnnotationCache({len(self._cache)} cached, {len(self._mtime)} watched)"
