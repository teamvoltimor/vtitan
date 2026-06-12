"""src.db.core – SQLite persistence layer for the SAM2 batch annotator.

All SQL is kept in src.db.queries; all schema DDL in src.db.schema.
Column-name strings come from src.db.constants; all functions return typed
dataclasses instead of raw sqlite3.Row objects or plain dicts.
"""

from __future__ import annotations

import sqlite3
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Generator

from src.constants import DB_PATH, IMAGES_DIR, LABELS_DIR, PENDING_DIR, VALID_EXTS
from src.db.constants import (
    COL_COLOR,
    COL_FORMAT_USED,
    COL_ID,
    COL_NAME,
    COL_PARENT_ID,
    COL_PATH,
    COL_STATUS,
    COL_UPDATED_AT,
    SQL_ALIAS_COUNT,
    STATUS_NAMES,
)
from src.db.queries import (
    QUERY_COUNT_CHILDREN_BY_PARENT,
    QUERY_COUNT_CLASSES,
    QUERY_DELETE_IMAGE,
    QUERY_INSERT_AUGMENTED_IMAGE,
    QUERY_INSERT_CLASS_DEFAULT_IGNORE,
    QUERY_INSERT_IMAGE_IGNORE,
    QUERY_PRAGMA_FOREIGN_KEYS,
    QUERY_PRAGMA_WAL,
    QUERY_SELECT_ALL_CLASSES,
    QUERY_SELECT_ALL_IMAGE_PATHS,
    QUERY_SELECT_ALL_IMAGES_FOR_BROWSE,
    QUERY_SELECT_ALL_IMAGES_GROUPED,
    QUERY_SELECT_CHILDREN_BY_PARENT,
    QUERY_SELECT_CLASS_ID_BY_NAME,
    QUERY_SELECT_DONE_ORIGINALS,
    QUERY_SELECT_FIRST_PENDING,
    QUERY_SELECT_IMAGE_BY_ID,
    QUERY_SELECT_NEXT_PENDING_AFTER_ID,
    QUERY_SELECT_PREV_IMAGE,
    QUERY_SELECT_STATUS_COUNTS,
    QUERY_UPDATE_IMAGE_DONE,
    QUERY_UPDATE_IMAGE_SKIPPED,
    QUERY_UPSERT_CLASS,
)
from src.db.schema import DEFAULT_CLASSES
from src.enums import Status
from src.models import BrowseRow, ClassInfo, GroupedRow, ImageRecord, StatsResult

# Module-level default used when callers don't pass explicit paths.
# Set at startup via init_db(); allows DefaultRepository to bind a specific db_path.
_db_path: Path = DB_PATH


def _connect(db_path: Path | None = None) -> sqlite3.Connection:
    """Open and configure a SQLite connection with WAL mode and foreign-key support."""
    conn = sqlite3.connect(db_path or _db_path)
    conn.row_factory = sqlite3.Row
    conn.execute(QUERY_PRAGMA_WAL)
    conn.execute(QUERY_PRAGMA_FOREIGN_KEYS)
    return conn


@contextmanager
def transaction(db_path: Path | None = None) -> Generator[sqlite3.Connection, None, None]:
    """Context manager that wraps operations in a single SQLite transaction.

    Commits on success, rolls back on any exception.  Use when multiple
    operations must succeed or fail together.
    """
    conn = _connect(db_path)
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def init_db(
    db_path: Path | None = None,
    pending_dir: Path | None = None,
    labels_dir: Path | None = None,
    images_dir: Path | None = None,
) -> None:
    """Create database tables if missing, scan pending/ for new images, and seed defaults.

    Uses Alembic to run database migrations to head before seeding data.
    Safe to call multiple times.
    """
    global _db_path
    if db_path is not None:
        _db_path = db_path

    _pending = pending_dir or PENDING_DIR
    _labels = labels_dir or LABELS_DIR
    _images = images_dir or IMAGES_DIR

    _pending.mkdir(parents=True, exist_ok=True)
    _labels.mkdir(parents=True, exist_ok=True)
    _images.mkdir(parents=True, exist_ok=True)

    # Run Alembic migrations
    from alembic import command
    from alembic.config import Config
    import importlib.resources
    
    # Locate alembic.ini relative to the backend package
    ini_path = importlib.resources.files("src").parent.joinpath("alembic.ini")
    alembic_cfg = Config(str(ini_path))
    # Override URL with resolved path to ensure tests use temp db
    alembic_cfg.set_main_option("sqlalchemy.url", f"sqlite:///{_db_path.resolve()}")
    command.upgrade(alembic_cfg, "head")

    with _connect() as conn:
        count = conn.execute(QUERY_COUNT_CLASSES).fetchone()[0]
        if count == 0:
            conn.executemany(QUERY_INSERT_CLASS_DEFAULT_IGNORE, DEFAULT_CLASSES)
            conn.commit()

        existing: set[str] = {row[COL_PATH] for row in conn.execute(QUERY_SELECT_ALL_IMAGE_PATHS).fetchall()}
        new_rows: list[tuple[str]] = [
            (str(f),) for f in sorted(_pending.iterdir()) if f.suffix.lower() in VALID_EXTS and str(f) not in existing
        ]
        if new_rows:
            conn.executemany(QUERY_INSERT_IMAGE_IGNORE, new_rows)
            conn.commit()


_FMT_ALIASES: dict[str, str] = {"segmentation": "seg", "detection": "det"}


def _row_to_image_record(row: sqlite3.Row) -> ImageRecord:
    """Convert a sqlite3.Row from the images table to an :class:`~src.models.ImageRecord`."""
    fmt_raw: str | None = row[COL_FORMAT_USED]
    format_used = _FMT_ALIASES.get(fmt_raw, fmt_raw) if fmt_raw else None
    return ImageRecord(
        id=row[COL_ID],
        path=row[COL_PATH],
        status=Status(row[COL_STATUS]),
        format_used=format_used,
        updated_at=row[COL_UPDATED_AT],
        parent_id=row.get(COL_PARENT_ID, None),
    )


def get_next(after_id: int | None = None, db_path: Path | None = None) -> ImageRecord | None:
    """Return the next pending image, optionally starting after *after_id*.

    If no pending image exists after *after_id*, wraps around to the first
    pending image in the entire table.  Returns ``None`` when no pending images
    exist at all.

    Args:
        after_id: DB id of the current image, or ``None`` to start from the beginning.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when no pending images exist.
    """
    with _connect(db_path) as conn:
        if after_id is not None:
            row = conn.execute(
                QUERY_SELECT_NEXT_PENDING_AFTER_ID,
                (Status.PENDING, after_id),
            ).fetchone()
            if row:
                return _row_to_image_record(row)
        row = conn.execute(QUERY_SELECT_FIRST_PENDING, (Status.PENDING,)).fetchone()
        return _row_to_image_record(row) if row else None


def get_prev(before_id: int, db_path: Path | None = None) -> ImageRecord | None:
    """Return the image with the largest ``id`` strictly less than *before_id*.

    Searches across all statuses so the user can navigate backwards through
    previously annotated images.

    Args:
        before_id: DB id of the current image.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when no earlier image exists.
    """
    with _connect(db_path) as conn:
        row = conn.execute(QUERY_SELECT_PREV_IMAGE, (before_id,)).fetchone()
        return _row_to_image_record(row) if row else None


def get_by_id(image_id: int, db_path: Path | None = None) -> ImageRecord | None:
    """Return a single image row by primary key, or ``None`` if not found.

    Args:
        image_id: Primary key of the image to fetch.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when the id does not exist.
    """
    with _connect(db_path) as conn:
        row = conn.execute(QUERY_SELECT_IMAGE_BY_ID, (image_id,)).fetchone()
        return _row_to_image_record(row) if row else None


def mark_done(image_id: int, fmt: str, db_path: Path | None = None) -> None:
    """Mark *image_id* as done and record the export format used.

    Args:
        image_id: Primary key of the image to update.
        fmt:      Export format string, e.g. ``"seg"`` or ``"det"``.
    """
    now = datetime.now(UTC).isoformat()
    with _connect(db_path) as conn:
        conn.execute(QUERY_UPDATE_IMAGE_DONE, (Status.DONE, fmt, now, image_id))
        conn.commit()


def mark_skipped(image_id: int, db_path: Path | None = None) -> None:
    """Mark *image_id* as skipped and record the current UTC timestamp.

    Args:
        image_id: Primary key of the image to update.
    """
    now = datetime.now(UTC).isoformat()
    with _connect(db_path) as conn:
        conn.execute(QUERY_UPDATE_IMAGE_SKIPPED, (Status.SKIPPED, now, image_id))
        conn.commit()


def get_stats(db_path: Path | None = None) -> StatsResult:
    """Return aggregate image counts by status as a typed dataclass.

    Returns:
        A :class:`~src.models.StatsResult` with pending, done, skipped, total,
        and pct (percentage done, rounded to one decimal place).
    """
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_STATUS_COUNTS).fetchall()

    counts: dict[int, int] = {Status.PENDING: 0, Status.DONE: 0, Status.SKIPPED: 0}
    for row in rows:
        counts[row[COL_STATUS]] = row[SQL_ALIAS_COUNT]

    total = sum(counts.values())
    done = counts[Status.DONE]
    pct = round(done / total * 100, 1) if total else 0.0

    return StatsResult(
        pending=counts[Status.PENDING],
        done=done,
        skipped=counts[Status.SKIPPED],
        total=total,
        pct=pct,
    )


def get_all_images(db_path: Path | None = None) -> list[BrowseRow]:
    """Return all image rows formatted for the browse view, ordered by id.

    Returns:
        A list of :class:`~src.models.BrowseRow` instances, one per image.
    """
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_ALL_IMAGES_FOR_BROWSE).fetchall()

        return [
            BrowseRow(
                id=row[COL_ID],
                filename=Path(row[COL_PATH]).name,
                status=STATUS_NAMES.get(row[COL_STATUS], str(row[COL_STATUS])),
                format=row[COL_FORMAT_USED] or "",
                updated_at=row[COL_UPDATED_AT] or "",
                path=str(row[COL_PATH]),
            )
            for row in rows
        ]


def upsert_class(name: str, color: str, db_path: Path | None = None) -> int:
    """Insert or update a class by name and return its database id.

    If a class with *name* already exists its colour is updated; otherwise a
    new row is inserted.

    Args:
        name:  Unique class name, e.g. ``"red_prism"``.
        color: Hex colour string, e.g. ``"#ee2737"``.

    Returns:
        The integer primary-key id of the inserted or updated row.
    """
    with _connect(db_path) as conn:
        conn.execute(QUERY_UPSERT_CLASS, (name, color))
        conn.commit()
        row = conn.execute(QUERY_SELECT_CLASS_ID_BY_NAME, (name,)).fetchone()
        return int(row[COL_ID])


def get_classes(db_path: Path | None = None) -> list[ClassInfo]:
    """Return all annotation classes ordered by database id ascending.

    Returns:
        A list of :class:`~src.models.ClassInfo` instances, one per class.
    """
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_ALL_CLASSES).fetchall()
    return [ClassInfo(id=row[COL_ID], name=row[COL_NAME], color=row[COL_COLOR]) for row in rows]


def classes_to_yolo_map() -> dict[int, int]:
    """Return a mapping of ``{db_id: yolo_class_index}`` (0-based, stable, ordered by id).

    The mapping is derived from the current class order and is used when writing
    YOLO ``.txt`` label files.

    Returns:
        Dict mapping each class DB id to its 0-based YOLO index.
    """
    classes = get_classes()
    return {cls.id: idx for idx, cls in enumerate(classes)}


def add_images_from_paths(paths: list[str], db_path: Path | None = None) -> int:
    """Register image file paths in the DB and return the count of new rows inserted.

    Paths that are already registered are silently ignored.

    Args:
        paths: Absolute path strings to image files.

    Returns:
        Number of newly inserted rows (0 if all paths already existed).
    """
    with _connect(db_path) as conn:
        existing: set[str] = {row[COL_PATH] for row in conn.execute(QUERY_SELECT_ALL_IMAGE_PATHS).fetchall()}
        new_rows = [(p,) for p in paths if p not in existing]
        if new_rows:
            conn.executemany(QUERY_INSERT_IMAGE_IGNORE, new_rows)
            conn.commit()
    return len(new_rows)


def delete_image(image_id: int, db_path: Path | None = None) -> None:
    """Delete an image row by setting the deleted_at timestamp (soft delete).

    Args:
        image_id: Primary key of the image to delete.
        db_path: Optional explicit database path.
    """
    now = datetime.now(UTC).isoformat()
    with _connect(db_path) as conn:
        conn.execute(QUERY_DELETE_IMAGE, (now, image_id))
        conn.commit()


def register_augmented_image(path: str, format_used: str, parent_id: int, db_path: Path | None = None) -> int:
    """Insert an augmented image row and return its id.

    Args:
        path:        Absolute path to the augmented image file.
        format_used: Export format of the parent (``"seg"`` or ``"det"``).
        parent_id:   DB id of the original image.

    Returns:
        Integer primary-key id of the inserted row.
    """
    now = datetime.now(UTC).isoformat()
    with _connect(db_path) as conn:
        cursor = conn.execute(QUERY_INSERT_AUGMENTED_IMAGE, (path, Status.DONE, format_used, parent_id, now))
        conn.commit()
        return cursor.lastrowid or 0


def get_done_originals(db_path: Path | None = None) -> list[ImageRecord]:
    """Return all done images that are original (no parent)."""
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_DONE_ORIGINALS).fetchall()
    return [_row_to_image_record(row) for row in rows]


def get_children(parent_id: int, db_path: Path | None = None) -> list[ImageRecord]:
    """Return all augmented copies of a parent image."""
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_CHILDREN_BY_PARENT, (parent_id,)).fetchall()
    return [_row_to_image_record(row) for row in rows]


def get_aug_count(parent_id: int, db_path: Path | None = None) -> int:
    """Return augmentation count for a parent image."""
    with _connect(db_path) as conn:
        return int(conn.execute(QUERY_COUNT_CHILDREN_BY_PARENT, (parent_id,)).fetchone()[0])


def get_grouped_images(db_path: Path | None = None) -> list[GroupedRow]:
    """Return parent images with their augmentation counts."""
    with _connect(db_path) as conn:
        rows = conn.execute(QUERY_SELECT_ALL_IMAGES_GROUPED).fetchall()
    return [
        GroupedRow(
            id=row[COL_ID],
            filename=Path(row[COL_PATH]).name,
            status=STATUS_NAMES.get(row[COL_STATUS], str(row[COL_STATUS])),
            format=row[COL_FORMAT_USED] or "",
            updated_at=row[COL_UPDATED_AT] or "",
            path=str(row[COL_PATH]),
            aug_count=row["aug_count"],
        )
        for row in rows
    ]
