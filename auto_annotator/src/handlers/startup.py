"""src.handlers.startup – Application startup handler."""

from __future__ import annotations

from pathlib import Path

from src import db
from src.handlers.navigation import _load_image
from src.html import stats_html
from src.models import AppState, ClassInfo
from src.render import render_state_image
from src.sam_client import ModelServerClient


def load_first_image(state: AppState, client: ModelServerClient | None, labels_dir: Path):
    """
    Called by demo.load() on page load.

    Returns:
      (display_img, state, status_box, stats_box, image_label,
       ann_box, browse_df, browse_stats)
    """
    db.init_db()
    raw = db.get_classes()
    state.classes = [ClassInfo.from_dict(r) for r in raw]

    # Try to populate model info from the server
    if client is not None:
        try:
            models = client.list_models()
            active = next((m for m in models if m.get("active")), None)
            if active:
                state.active_model_id = active["id"]
                state.model_supports_text = active.get("supports_text", False)
        except Exception:  # noqa: BLE001
            pass

    # Browse dataframe (always populated on load)
    rows = db.get_all_images()
    browse_data = [
        [r["id"], r["filename"], r["status"], r["format"], r["updated_at"]]
        for r in rows
    ]

    record = db.get_next()
    if record is None:
        return (
            render_state_image(state),
            state,
            "No pending images — import some via the Browse tab.",
            stats_html(),
            "",
            "(none)",
            browse_data,
            stats_html(),
        )

    rendered, label, stats = _load_image(record, state, client, labels_dir)
    return (
        rendered,
        state,
        f"Loaded: {label}",
        stats,
        label,
        "(none)",
        browse_data,
        stats_html(),
    )
