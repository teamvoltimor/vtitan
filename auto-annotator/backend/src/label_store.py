"""src.label_store – Single owner of YOLO label file reads and writes.

All code that touches .txt label files goes through LabelStore.
Format details (space-separated class_id + coords, one annotation per line)
live here; callers work in terms of LabelRecord objects.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from src.utils import get_logger

if TYPE_CHECKING:
    from src.gallery_cache import AnnotationCache

logger = get_logger(__name__)


@dataclass(frozen=True)
class LabelRecord:
    """One annotation line from a YOLO label file.

    Attributes:
        class_id: Integer class index.
        coords:   Flat list of normalised floats — either [xc, yc, w, h] for
                  detection or [x1, y1, x2, y2, ...] for segmentation.
    """

    class_id: int
    coords: list[float]


class LabelStore:
    """Read and write YOLO label files.

    ``load`` uses the mtime-aware AnnotationCache to avoid re-parsing unchanged
    files on every request.  ``load_raw`` reads directly from disk and is used
    by augmentation where the cache is bypassed.  ``save`` owns the wire format.
    """

    def __init__(self, cache: AnnotationCache) -> None:
        self._cache = cache

    def load(self, class_dir: str, stem: str) -> list[LabelRecord]:
        """Return cached annotation records for *class_dir*/*stem*."""
        return [r for r in (_parse_line(line) for line in self._cache.get(class_dir, stem)) if r is not None]

    def load_raw(self, label_path: Path) -> list[LabelRecord]:
        """Parse *label_path* directly from disk, bypassing the cache."""
        if not label_path.exists():
            return []
        records: list[LabelRecord] = []
        try:
            for line in label_path.read_text(encoding="utf-8").splitlines():
                record = _parse_line(line)
                if record is not None:
                    records.append(record)
        except (OSError, UnicodeDecodeError) as e:
            logger.warning("label_read_failed", extra={"_extra": {"path": str(label_path), "error": str(e)}})
        return records

    def save(self, path: Path, records: list[LabelRecord]) -> None:
        """Serialise *records* to *path* in YOLO label format."""
        lines = [f"{r.class_id} " + " ".join(f"{v:.6f}" for v in r.coords) for r in records]
        path.write_text("\n".join(lines), encoding="utf-8")


def _parse_line(line: str) -> LabelRecord | None:
    parts = line.strip().split()
    if not parts:
        return None
    try:
        return LabelRecord(class_id=int(parts[0]), coords=[float(v) for v in parts[1:]])
    except (ValueError, IndexError):
        return None
