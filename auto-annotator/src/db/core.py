"""src.db.core – SQLite persistence layer for the SAM2 batch annotator.

All SQL is kept in src.db.queries; all schema DDL in src.db.schema.
Column-name strings come from src.db.constants; all functions return typed
dataclasses instead of raw sqlite3.Row objects or plain dicts.
"""

from __future__ import annotations

import sqlite3
from datetime import UTC, datetime
from pathlib import Path

from src.constants import DB_PATH, LABELS_DIR, PENDING_DIR, VALID_EXTS
from src.db.constants import (
    COL_COLOR,
    COL_FORMAT_USED,
    COL_ID,
    COL_NAME,
    COL_PATH,
    COL_STATUS,
    COL_UPDATED_AT,
    SQL_ALIAS_COUNT,
    STATUS_NAMES,
)
from src.db.queries import (
    QUERY_COUNT_CLASSES,
    QUERY_INSERT_CLASS_DEFAULT_IGNORE,
    QUERY_INSERT_IMAGE_IGNORE,
    QUERY_PRAGMA_FOREIGN_KEYS,
    QUERY_PRAGMA_WAL,
    QUERY_SELECT_ALL_CLASSES,
    QUERY_SELECT_ALL_IMAGE_PATHS,
    QUERY_SELECT_ALL_IMAGES_FOR_BROWSE,
    QUERY_SELECT_CLASS_ID_BY_NAME,
    QUERY_SELECT_FIRST_PENDING,
    QUERY_SELECT_IMAGE_BY_ID,
    QUERY_SELECT_NEXT_PENDING_AFTER_ID,
    QUERY_SELECT_PREV_IMAGE,
    QUERY_SELECT_STATUS_COUNTS,
    QUERY_UPDATE_IMAGE_DONE,
    QUERY_UPDATE_IMAGE_SKIPPED,
    QUERY_UPSERT_CLASS,
)
from src.db.schema import DDL, DEFAULT_CLASSES
from src.enums import Status
from src.models import BrowseRow, ClassInfo, ImageRecord, StatsResult


def _connect() -> sqlite3.Connection:
    """Open and configure a SQLite connection with WAL mode and foreign-key support."""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute(QUERY_PRAGMA_WAL)
    conn.execute(QUERY_PRAGMA_FOREIGN_KEYS)
    return conn


def init_db() -> None:
    """Create database tables if missing, scan pending/ for new images, and seed defaults.

    Safe to call multiple times; all DDL uses ``IF NOT EXISTS`` and inserts use
    ``OR IGNORE`` so repeated calls are idempotent.
    """
    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    LABELS_DIR.mkdir(parents=True, exist_ok=True)

    with _connect() as conn:
        conn.executescript(DDL)
        conn.commit()

        count = conn.execute(QUERY_COUNT_CLASSES).fetchone()[0]
        if count == 0:
            conn.executemany(QUERY_INSERT_CLASS_DEFAULT_IGNORE, DEFAULT_CLASSES)
            conn.commit()

        existing: set[str] = {
            row[COL_PATH]
            for row in conn.execute(QUERY_SELECT_ALL_IMAGE_PATHS).fetchall()
        }
        new_rows: list[tuple[str]] = [
            (str(f),)
            for f in sorted(PENDING_DIR.iterdir())
            if f.suffix.lower() in VALID_EXTS and str(f) not in existing
        ]
        if new_rows:
            conn.executemany(QUERY_INSERT_IMAGE_IGNORE, new_rows)
            conn.commit()


def _row_to_image_record(row: sqlite3.Row) -> ImageRecord:
    """Convert a sqlite3.Row from the images table to an :class:`~src.models.ImageRecord`."""
    return ImageRecord(
        id=row[COL_ID],
        path=row[COL_PATH],
        status=row[COL_STATUS],
        format_used=row[COL_FORMAT_USED],
        updated_at=row[COL_UPDATED_AT],
    )


def get_next(after_id: int | None = None) -> ImageRecord | None:
    """Return the next pending image, optionally starting after *after_id*.

    If no pending image exists after *after_id*, wraps around to the first
    pending image in the entire table.  Returns ``None`` when no pending images
    exist at all.

    Args:
        after_id: DB id of the current image, or ``None`` to start from the beginning.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when no pending images exist.
    """
    with _connect() as conn:
        if after_id is not None:
            row = conn.execute(
                QUERY_SELECT_NEXT_PENDING_AFTER_ID, (Status.PENDING, after_id),
            ).fetchone()
            if row:
                return _row_to_image_record(row)
        row = conn.execute(QUERY_SELECT_FIRST_PENDING, (Status.PENDING,)).fetchone()
        return _row_to_image_record(row) if row else None


def get_prev(before_id: int) -> ImageRecord | None:
    """Return the image with the largest ``id`` strictly less than *before_id*.

    Searches across all statuses so the user can navigate backwards through
    previously annotated images.

    Args:
        before_id: DB id of the current image.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when no earlier image exists.
    """
    with _connect() as conn:
        row = conn.execute(QUERY_SELECT_PREV_IMAGE, (before_id,)).fetchone()
        return _row_to_image_record(row) if row else None


def get_by_id(image_id: int) -> ImageRecord | None:
    """Return a single image row by primary key, or ``None`` if not found.

    Args:
        image_id: Primary key of the image to fetch.

    Returns:
        An :class:`~src.models.ImageRecord`, or ``None`` when the id does not exist.
    """
    with _connect() as conn:
        row = conn.execute(QUERY_SELECT_IMAGE_BY_ID, (image_id,)).fetchone()
        return _row_to_image_record(row) if row else None


def mark_done(image_id: int, fmt: str) -> None:
    """Mark *image_id* as done and record the export format used.

    Args:
        image_id: Primary key of the image to update.
        fmt:      Export format string, e.g. ``"seg"`` or ``"det"``.
    """
    now = datetime.now(UTC).isoformat()
    with _connect() as conn:
        conn.execute(QUERY_UPDATE_IMAGE_DONE, (Status.DONE, fmt, now, image_id))
        conn.commit()


def mark_skipped(image_id: int) -> None:
    """Mark *image_id* as skipped and record the current UTC timestamp.

    Args:
        image_id: Primary key of the image to update.
    """
    now = datetime.now(UTC).isoformat()
    with _connect() as conn:
        conn.execute(QUERY_UPDATE_IMAGE_SKIPPED, (Status.SKIPPED, now, image_id))
        conn.commit()


def get_stats() -> StatsResult:
    """Return aggregate image counts by status as a typed dataclass.

    Returns:
        A :class:`~src.models.StatsResult` with pending, done, skipped, total,
        and pct (percentage done, rounded to one decimal place).
    """
    with _connect() as conn:
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


def get_all_images() -> list[BrowseRow]:
    """Return all image rows formatted for the browse view, ordered by id.

    Returns:
        A list of :class:`~src.models.BrowseRow` instances, one per image.
    """
    with _connect() as conn:
        rows = conn.execute(QUERY_SELECT_ALL_IMAGES_FOR_BROWSE).fetchall()

    return [
        BrowseRow(
            id=row[COL_ID],
            filename=Path(row[COL_PATH]).name,
            status=STATUS_NAMES.get(row[COL_STATUS], str(row[COL_STATUS])),
            format=row[COL_FORMAT_USED] or "",
            updated_at=row[COL_UPDATED_AT] or "",
        )
        for row in rows
    ]


def upsert_class(name: str, color: str) -> int:
    """Insert or update a class by name and return its database id.

    If a class with *name* already exists its colour is updated; otherwise a
    new row is inserted.

    Args:
        name:  Unique class name, e.g. ``"red_prism"``.
        color: Hex colour string, e.g. ``"#ee2737"``.

    Returns:
        The integer primary-key id of the inserted or updated row.
    """
    with _connect() as conn:
        conn.execute(QUERY_UPSERT_CLASS, (name, color))
        conn.commit()
        row = conn.execute(QUERY_SELECT_CLASS_ID_BY_NAME, (name,)).fetchone()
        return int(row[COL_ID])


def get_classes() -> list[ClassInfo]:
    """Return all annotation classes ordered by database id ascending.

    Returns:
        A list of :class:`~src.models.ClassInfo` instances, one per class.
    """
    with _connect() as conn:
        rows = conn.execute(QUERY_SELECT_ALL_CLASSES).fetchall()
    return [
        ClassInfo(id=row[COL_ID], name=row[COL_NAME], color=row[COL_COLOR])
        for row in rows
    ]


def classes_to_yolo_map() -> dict[int, int]:
    """Return a mapping of ``{db_id: yolo_class_index}`` (0-based, stable, ordered by id).

    The mapping is derived from the current class order and is used when writing
    YOLO ``.txt`` label files.

    Returns:
        Dict mapping each class DB id to its 0-based YOLO index.
    """
    classes = get_classes()
    return {cls.id: idx for idx, cls in enumerate(classes)}


def add_images_from_paths(paths: list[str]) -> int:
    """Register image file paths in the DB and return the count of new rows inserted.

    Paths that are already registered are silently ignored.

    Args:
        paths: Absolute path strings to image files.

    Returns:
        Number of newly inserted rows (0 if all paths already existed).
    """
    with _connect() as conn:
        existing: set[str] = {
            row[COL_PATH]
            for row in conn.execute(QUERY_SELECT_ALL_IMAGE_PATHS).fetchall()
        }
        new_rows = [(p,) for p in paths if p not in existing]
        if new_rows:
            conn.executemany(QUERY_INSERT_IMAGE_IGNORE, new_rows)
            conn.commit()
    return len(new_rows)
