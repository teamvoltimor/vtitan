"""
db.py – SQLite persistence layer for the SAM2 batch annotator.

Tables
------
classes : per-project class registry (name → colour)
images  : image queue with status tracking
"""

import sqlite3
import os
from pathlib import Path
from datetime import datetime, timezone

# ── Paths ─────────────────────────────────────────────────────────────────────

BASE_DIR     = Path(__file__).parent
PENDING_DIR  = BASE_DIR / "data" / "pending"
LABELS_DIR   = BASE_DIR / "data" / "labels"
DB_PATH      = Path(os.environ.get("DB_PATH", BASE_DIR / "manifest.db"))

VALID_EXTS   = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# Status constants
STATUS_PENDING  = 0
STATUS_DONE     = 1
STATUS_SKIPPED  = 2


# ── Connection helper ──────────────────────────────────────────────────────────

def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


# ── Schema ────────────────────────────────────────────────────────────────────

_DDL = """
CREATE TABLE IF NOT EXISTS classes (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT UNIQUE NOT NULL,
    color      TEXT DEFAULT '#dc322f',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS images (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    path        TEXT UNIQUE NOT NULL,
    status      INTEGER DEFAULT 0,
    format_used TEXT,
    created_at  TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at  TIMESTAMP
);
"""


# ── Public API ────────────────────────────────────────────────────────────────

def init_db() -> None:
    """
    Create tables if needed, scan data/pending/ for new images, create dirs.
    """
    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    LABELS_DIR.mkdir(parents=True, exist_ok=True)

    with _connect() as conn:
        conn.executescript(_DDL)
        conn.commit()

        # Scan pending dir and insert new image paths
        existing = {
            row["path"]
            for row in conn.execute("SELECT path FROM images").fetchall()
        }

        new_rows = []
        for f in sorted(PENDING_DIR.iterdir()):
            if f.suffix.lower() in VALID_EXTS:
                path_str = str(f)
                if path_str not in existing:
                    new_rows.append((path_str,))

        if new_rows:
            conn.executemany("INSERT OR IGNORE INTO images (path) VALUES (?)", new_rows)
            conn.commit()


def get_next(after_id: int | None = None) -> sqlite3.Row | None:
    """
    Return the next pending image.
    - If after_id is given: first pending with id > after_id.
    - If nothing found after that id, wraps to the very first pending image.
    - Returns None if no pending images exist at all.
    """
    with _connect() as conn:
        if after_id is not None:
            row = conn.execute(
                "SELECT * FROM images WHERE status=? AND id>? ORDER BY id ASC LIMIT 1",
                (STATUS_PENDING, after_id),
            ).fetchone()
            if row:
                return row
        # wrap-around / initial load
        return conn.execute(
            "SELECT * FROM images WHERE status=? ORDER BY id ASC LIMIT 1",
            (STATUS_PENDING,),
        ).fetchone()


def get_prev(before_id: int) -> sqlite3.Row | None:
    """
    Return the image with the largest id strictly less than before_id (any status).
    """
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
            (STATUS_DONE, fmt, now, image_id),
        )
        conn.commit()


def mark_skipped(image_id: int) -> None:
    now = datetime.now(timezone.utc).isoformat()
    with _connect() as conn:
        conn.execute(
            "UPDATE images SET status=?, updated_at=? WHERE id=?",
            (STATUS_SKIPPED, now, image_id),
        )
        conn.commit()


def get_stats() -> dict:
    with _connect() as conn:
        rows = conn.execute(
            "SELECT status, COUNT(*) AS cnt FROM images GROUP BY status"
        ).fetchall()

    counts = {STATUS_PENDING: 0, STATUS_DONE: 0, STATUS_SKIPPED: 0}
    for row in rows:
        counts[row["status"]] = row["cnt"]

    total   = sum(counts.values())
    done    = counts[STATUS_DONE]
    pct     = round(done / total * 100, 1) if total else 0.0

    return {
        "pending": counts[STATUS_PENDING],
        "done":    done,
        "skipped": counts[STATUS_SKIPPED],
        "total":   total,
        "pct":     pct,
    }


def get_all_images() -> list[dict]:
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, path, status, format_used, updated_at FROM images ORDER BY id ASC"
        ).fetchall()

    status_names = {STATUS_PENDING: "pending", STATUS_DONE: "done", STATUS_SKIPPED: "skipped"}
    result = []
    for row in rows:
        result.append({
            "id":         row["id"],
            "filename":   Path(row["path"]).name,
            "status":     status_names.get(row["status"], str(row["status"])),
            "format":     row["format_used"] or "",
            "updated_at": row["updated_at"] or "",
        })
    return result


def upsert_class(name: str, color: str) -> int:
    """
    Insert or update a class by name. Returns the class db id.
    """
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
    """Return all classes ordered by db id ascending."""
    with _connect() as conn:
        rows = conn.execute(
            "SELECT id, name, color FROM classes ORDER BY id ASC"
        ).fetchall()
    return [dict(row) for row in rows]


def classes_to_yolo_map() -> dict[int, int]:
    """
    Return {db_id: yolo_index} where yolo_index is 0-based, stable,
    ordered by db id.
    """
    classes = get_classes()
    return {cls["id"]: idx for idx, cls in enumerate(classes)}


def add_images_from_folder(folder_path: str) -> tuple[int, int]:
    """
    Register all valid images found in folder_path (non-recursive).
    Returns (inserted, skipped_existing).
    """
    folder = Path(folder_path)
    if not folder.is_dir():
        return 0, 0

    with _connect() as conn:
        existing = {
            row["path"]
            for row in conn.execute("SELECT path FROM images").fetchall()
        }
        new_rows = []
        for f in sorted(folder.iterdir()):
            if f.suffix.lower() in VALID_EXTS:
                path_str = str(f.resolve())
                if path_str not in existing:
                    new_rows.append((path_str,))

        if new_rows:
            conn.executemany("INSERT OR IGNORE INTO images (path) VALUES (?)", new_rows)
            conn.commit()

    return len(new_rows), 0
