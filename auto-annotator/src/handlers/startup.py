"""src.handlers.startup – Application startup handler."""

from __future__ import annotations

from typing import TYPE_CHECKING

from src import db
from src.handlers.navigation import _ann_summary, _load_image
from src.handlers.responses import StartupResponse
from src.html import stats_html
from src.render import render_state_image

if TYPE_CHECKING:
    from pathlib import Path

    from src.models import AppContext, AppState


def load_first_image(
    state: AppState, app_ctx: AppContext, labels_dir: Path,
) -> StartupResponse:
    """Called by demo.load() on page load.

    Initialises the DB, loads class definitions, queries model server for the
    active model, and loads the first pending image.
    """
    db.init_db()
    state.classes = db.get_classes()

    if app_ctx.client is not None:
        try:
            models = app_ctx.client.list_models()
            active = next((m for m in models if m.get("active")), None)
            if active:
                state.active_model_id = active["id"]
                state.model_supports_text = active.get("supports_text", False)
        except Exception:  # noqa: BLE001, S110
            pass

    rows = db.get_all_images()
    browse_data = [[r.id, r.filename, r.status, r.format, r.updated_at] for r in rows]

    record = db.get_next()
    if record is None:
        return StartupResponse(
            display_img=render_state_image(state),
            state=state,
            status_msg="No pending images — import some via the Browse tab.",
            stats_html=stats_html(),
            image_label="",
            ann_summary="(none)",
            browse_df=browse_data,
            browse_stats_html=stats_html(),
        )

    rendered, label, html = _load_image(record, state, app_ctx, labels_dir)
    return StartupResponse(
        display_img=rendered,
        state=state,
        status_msg=f"Loaded: {label}",
        stats_html=html,
        image_label=label,
        ann_summary=_ann_summary(state.annotations),
        browse_df=browse_data,
        browse_stats_html=stats_html(),
    )
