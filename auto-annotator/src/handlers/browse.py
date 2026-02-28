"""src.handlers.browse – Browse tab event handlers."""

from __future__ import annotations

import shutil
from pathlib import Path

from src import db
from src.constants import PENDING_DIR, VALID_EXTS
from src.handlers.responses import BrowseResponse, ImportResponse
from src.html import stats_html
from src.utils import get_logger

logger = get_logger("browse")


def _build_df_data(rows: list) -> list[list]:
    return [[r.id, r.filename, r.status, r.format, r.updated_at] for r in rows]


def _gallery_items(rows: list) -> list[str]:
    return [r.path for r in rows]


def refresh_browse() -> BrowseResponse:
    """Rescan data/pending/ and return updated dataframe rows + stats."""
    db.init_db()
    rows = db.get_all_images()
    return BrowseResponse(
        df_data=_build_df_data(rows),
        stats_html=stats_html(),
        gallery_items=_gallery_items(rows),
    )


def import_images(files: list) -> ImportResponse:
    """Copy uploaded files to data/pending/ and register them in the DB.

    *files* is a list of NamedString / temp-file-path objects from gr.UploadButton.
    """
    if not files:
        browse = refresh_browse()
        return ImportResponse(
            df_data=browse.df_data,
            stats_html=browse.stats_html,
            status_msg="No files selected.",
            gallery_items=browse.gallery_items,
        )

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

    db.init_db()
    rows = db.get_all_images()
    return ImportResponse(
        df_data=_build_df_data(rows),
        stats_html=stats_html(),
        status_msg=f"Imported {copied} image(s) ({skipped} skipped).",
        gallery_items=_gallery_items(rows),
    )
