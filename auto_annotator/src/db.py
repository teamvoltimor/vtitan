"""src.db – SQLite persistence layer for the SAM2 batch annotator."""

import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from src.constants import DB_PATH, LABELS_DIR, PENDING_DIR, VALID_EXTS
from src.enums import Status
from src.schema import DDL, DEFAULT_CLASSES


# ── Connection helper ──────────────────────────────────────────────────────────


def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


# ── Public API ────────────────────────────────────────────────────────────────


def init_db() -> None:
    """Create tables if needed, scan data/pending/ for new images, seed defaults."""
    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    LABELS_DIR.mkdir(parents=True, exist_ok=True)

    with _connect() as conn:
        conn.executescript(DDL)
        conn.commit()

        # Seed default classes if the table is empty
        count = conn.execute("SELECT COUNT(*) FROM classes").fetchone()[0]
        if count == 0:
            conn.executemany(
                "INSERT OR IGNORE INTO classes (name, color) VALUES (?, ?)",
                DEFAULT_CLASSES,
            )
            conn.commit()

        # Scan pending dir and insert new image paths
        existing = {
            row["path"]
            for row in conn.execute("SELECT path FROM images").fetchall()
        }

        new_rows: list[tuple[str]] = []
        for f in sorted(PENDING_DIR.iterdir()):
            if f.suffix.lower() in VALID_EXTS:
                path_str = str(f)
                if path_str not in existing:
                    new_rows.append((path_str,))

        if new_rows:
            conn.executemany("INSERT OR IGNORE INTO images (path) VALUES (?)", new_rows)
            conn.commit()


def get_next(after_id: int | None = None) -> sqlite3.Row | None:
    """Return the next pending image, wrapping around if needed."""
    with _connect() as conn:
        if after_id is not None:
            row = conn.execute(
                "SELECT * FROM images WHERE status=? AND id>? ORDER BY id ASC LIMIT 1",
                (Status.PENDING, after_id),
            ).fetchone()
            if row:
                return row
        return conn.execute(
            "SELECT * FROM images WHERE status=? ORDER BY id ASC LIMIT 1",
            (Status.PENDING,),
        ).fetchone()


def get_prev(before_id: int) -> sqlite3.Row | None:
    """Return the image with the largest id strictly less than before_id (any status)."""
    with _connect() as conn:
        return conn.execute(
            "SELECT * FROM images WHERE id<? ORDER BY id DESC LIMIT 1",
            (before_id,),
        ).fetchone()


def get_by_id(image_id: int) -> sqlite3.Row | None:
    with _connect() as conn:
        return conn.execute(
            "SELECT * FROM images WHERE id=?", (image_id,)
        ).fetchone()


def mark_done(image_id: int, fmt: str) -> None:
    now = datetime.now(timezone.utc).isoformat()
    with _connect() as conn:
        conn.execute(
            "UPDATE images SET status=?, format_used=?, updated_at=? WHERE id=?",
            (Status.DONE, fmt, now, image_id),
        )
        conn.commit()


def mark_skipped(image_id: int) -> None:
    now = datetime.now(timezone.utc).isoformat()
    with _connect() as conn:
        conn.execute(
            "UPDATE images SET status=?, updated_at=? WHERE id=?",
            (Status.SKIPPED, now, image_id),
        )
        conn.commit()


def get_stats() -> dict:
    with _connect() as conn:
        rows = conn.execute(
            "SELECT status, COUNT(*) AS cnt FROM images GROUP BY status"
        ).fetchall()

    counts = {Status.PENDING: 0, Status.DONE: 0, Status.SKIPPED: 0}
    for row in rows:
        counts[row["status"]] = row["cnt"]

    total = sum(counts.values())
    done = counts[Status.DONE]
    pct = round(done / total * 100, 1) if total else 0.0

    return {
        "pending": counts[Status.PENDING],
        "done": done,
        "skipped": counts[Status.SKIPPED],
        "total": total,
        "pct": pct,
    }


def get_all_images() -> list[dict]:
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, path, status, format_used, updated_at FROM images ORDER BY id ASC"
        ).fetchall()

    status_names = {
        Status.PENDING: "pending",
        Status.DONE: "done",
        Status.SKIPPED: "skipped",
    }
    return [
        {
            "id": row["id"],
            "filename": Path(row["path"]).name,
            "status": status_names.get(row["status"], str(row["status"])),
            "format": row["format_used"] or "",
            "updated_at": row["updated_at"] or "",
        }
        for row in rows
    ]


def upsert_class(name: str, color: str) -> int:
    """Insert or update a class by name.  Returns the class DB id."""
    with _connect() as conn:
        conn.execute(
            """
            INSERT INTO classes (name, color) VALUES (?, ?)
            ON CONFLICT(name) DO UPDATE SET color=excluded.color
            """,
            (name, color),
        )
        conn.commit()
        row = conn.execute("SELECT id FROM classes WHERE name=?", (name,)).fetchone()
        return row["id"]


def get_classes() -> list[dict]:
    """Return all classes ordered by DB id ascending."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, name, color FROM classes ORDER BY id ASC"
        ).fetchall()
    return [dict(row) for row in rows]


def classes_to_yolo_map() -> dict[int, int]:
    """Return {db_id: yolo_index} (0-based, stable, ordered by db id)."""
    classes = get_classes()
    return {cls["id"]: idx for idx, cls in enumerate(classes)}


def add_images_from_paths(paths: list[str]) -> int:
    """Register image paths in the DB.  Returns count of newly inserted rows."""
    with _connect() as conn:
        existing = {
            row["path"]
            for row in conn.execute("SELECT path FROM images").fetchall()
        }
        new_rows = [(p,) for p in paths if p not in existing]
        if new_rows:
            conn.executemany("INSERT OR IGNORE INTO images (path) VALUES (?)", new_rows)
            conn.commit()
    return len(new_rows)
