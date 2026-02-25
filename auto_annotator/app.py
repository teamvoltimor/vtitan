"""
Auto-Annotator – Gradio entrypoint
───────────────────────────────────
Thin entrypoint that builds the UI, connects to model_server.py if running,
and wires all event handlers.

Usage:
    uv run python app.py
    # or inside Docker via docker-compose up
"""

import time as _time

_APP_START = _time.monotonic()

import os
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

import gradio as gr

# ── Paths ──────────────────────────────────────────────────────────────────────

os.environ.setdefault(
    "HF_HUB_CACHE",
    str(Path(os.environ.get("MODELS_DIR", Path(__file__).parent / "models"))),
)

from src.constants import LABELS_DIR, SERVER_PORT
from src.inference import initialize_inference
from src.models import AppState, ClassInfo
from src import db
from src.sam_client import ModelServerClient
from src.handlers.startup import load_first_image
from src.ui import annotate_tab, browse_tab, settings_tab

# ── Model server connection ────────────────────────────────────────────────────

_client_ref: list = []   # one-element list; [0] is ModelServerClient | None

_probe = ModelServerClient()
if _probe.ping():
    _client_ref.append(_probe)
    print(  # noqa: T201
        f"[app] Connected to model server on port {SERVER_PORT} — "
        "model stays loaded across restarts."
    )
else:
    _client_ref.append(None)
    print(f"[app] No model server on port {SERVER_PORT} — loading model directly.")  # noqa: T201

initialize_inference(_client_ref[0])


# ── Gradio app ────────────────────────────────────────────────────────────────

def _build_app() -> gr.Blocks:
    db.init_db()

    with gr.Blocks(title="Auto-Annotator", theme=gr.themes.Soft()) as demo:
        state = gr.State(value=AppState())

        ann_c = annotate_tab.build()
        browse_c = browse_tab.build()
        settings_c = settings_tab.build()

        # ── Wire events ───────────────────────────────────────────────────────
        annotate_tab.wire_events(ann_c, state, _client_ref, LABELS_DIR)
        browse_tab.wire_events(browse_c)
        settings_tab.wire_events(settings_c, state, _client_ref, ann_c)

        # ── On page load ──────────────────────────────────────────────────────
        def _on_load(st: AppState):
            return load_first_image(st, _client_ref[0], LABELS_DIR)

        # Populate model dropdown from server
        def _init_model_dd(st: AppState):
            client = _client_ref[0]
            if client is None:
                return gr.update()
            try:
                models = client.list_models()
                choices = [m["label"] for m in models]
                ids = [m["id"] for m in models]
                active_label = next(
                    (m["label"] for m in models if m.get("active")), None
                )
                # Map label→id so the dropdown value is an id
                label_to_id = dict(zip(choices, ids))
                return gr.update(
                    choices=list(label_to_id.keys()),
                    value=active_label,
                )
            except Exception:  # noqa: BLE001
                return gr.update()

        # Populate class dropdowns on load
        def _init_class_dds(st: AppState):
            choices = [c.name for c in st.classes]
            return gr.update(choices=choices), gr.update(choices=choices)

        demo.load(
            fn=_on_load,
            inputs=[state],
            outputs=[
                ann_c["display_img"],
                state,
                ann_c["status_box"],
                ann_c["stats_box"],
                ann_c["image_label"],
                ann_c["ann_box"],
                browse_c["browse_df"],
                browse_c["browse_stats"],
            ],
        )

        demo.load(
            fn=_init_model_dd,
            inputs=[state],
            outputs=[settings_c["model_dropdown"]],
        )

        demo.load(
            fn=_init_class_dds,
            inputs=[state],
            outputs=[ann_c["class_dropdown"], settings_c["edit_class_dd"]],
        )

    return demo


demo = _build_app()

if __name__ == "__main__":
    elapsed = _time.monotonic() - _APP_START
    print(f"[app] Built in {elapsed:.2f}s")  # noqa: T201
    demo.launch(server_name="0.0.0.0", server_port=7860)
