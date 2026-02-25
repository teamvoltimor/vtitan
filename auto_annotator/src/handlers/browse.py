"""src.handlers.browse – Browse tab event handlers."""

from __future__ import annotations

import shutil
from pathlib import Path

from src import db
from src.constants import PENDING_DIR, VALID_EXTS
from src.html import stats_html
from src.utils import get_logger

logger = get_logger("browse")


def refresh_browse():
    """Rescan data/pending/ and return updated dataframe rows + stats."""
    db.init_db()  # registers any new files in data/pending/
    rows = db.get_all_images()
    data = [
        [r["id"], r["filename"], r["status"], r["format"], r["updated_at"]]
        for r in rows
    ]
    return data, stats_html()


def import_images(files):
    """
    Copy uploaded files to data/pending/ and register them in the DB.
    *files* is a list of NamedString / temp-file-path objects from gr.UploadButton.
    Returns (dataframe_data, stats_html, status_message).
    """
    if not files:
        return *refresh_browse(), "No files selected."

    PENDING_DIR.mkdir(parents=True, exist_ok=True)
    copied = 0
    skipped = 0

    for f in files:
        src = Path(f if isinstance(f, str) else f.name)
        if src.suffix.lower() not in VALID_EXTS:
            skipped += 1
            continue
        dst = PENDING_DIR / src.name
        if not dst.exists():
            shutil.copy2(src, dst)
            copied += 1
        else:
            skipped += 1

    db.init_db()  # pick up newly copied files
    rows = db.get_all_images()
    data = [
        [r["id"], r["filename"], r["status"], r["format"], r["updated_at"]]
        for r in rows
    ]
    msg = f"Imported {copied} image(s) ({skipped} skipped)."
    return data, stats_html(), msg
