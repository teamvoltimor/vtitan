"""src.app – Gradio application builder and runner.

Builds the UI, connects to the model server if running, initialises local
inference as a fallback, and wires all event handlers.  Called from the root
``main.py`` when invoked in ``app`` mode.
"""

from __future__ import annotations

import os
from pathlib import Path

import gradio as gr

from src import db
from src.constants import LABELS_DIR, MODELS_DIR, SERVER_PORT
from src.handlers.startup import load_first_image
from src.inference import initialize_inference
from src.models import AppContext, AppState
from src.sam_client import ModelServerClient
from src.ui import annotate_tab, browse_tab, settings_tab

# ── Custom CSS (Catppuccin Mocha + Terminal Precision aesthetic) ──────────────
_CSS = """
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;700&family=Outfit:wght@300;400;600;700&display=swap');

:root {
  --ctp-base:     #1e1e2e; --ctp-mantle:   #181825; --ctp-crust:    #11111b;
  --ctp-surface0: #313244; --ctp-surface1: #45475a; --ctp-surface2: #585b70;
  --ctp-overlay0: #6c7086; --ctp-subtext1: #bac2de; --ctp-text:     #cdd6f4;
  --ctp-blue:     #89b4fa; --ctp-green:    #a6e3a1; --ctp-red:      #f38ba8;
  --ctp-peach:    #fab387; --ctp-purple:   #cba6f7; --ctp-teal:     #94e2d5;
  --ctp-lavender: #b4befe;
}

/* ── Base ── */
body, .gradio-container { background: var(--ctp-base) !important; }
.gradio-container       { font-family: 'Outfit', sans-serif !important; color: var(--ctp-text) !important; }

/* ── App header ── */
.aa-header {
  display: flex; align-items: center; gap: 12px;
  padding: 14px 20px;
  background: var(--ctp-mantle);
  border-bottom: 1px solid var(--ctp-surface0);
  margin-bottom: 4px;
}
.aa-header-logo {
  font-family: 'JetBrains Mono', monospace; font-size: 15px; font-weight: 700;
  letter-spacing: .12em; color: var(--ctp-blue);
}
.aa-header-dot {
  width: 8px; height: 8px; border-radius: 50%;
  background: var(--ctp-green);
  box-shadow: 0 0 6px var(--ctp-green);
  flex-shrink: 0;
}
.aa-header-badge {
  font-family: 'JetBrains Mono', monospace; font-size: 10px; font-weight: 400;
  color: var(--ctp-overlay0); letter-spacing: .1em; text-transform: uppercase;
  border: 1px solid var(--ctp-surface1); padding: 2px 7px; border-radius: 3px;
}

/* ── Tabs ── */
.tabs > .tab-nav               { background: var(--ctp-mantle) !important; border-bottom: 1px solid var(--ctp-surface0) !important; }
.tabs > .tab-nav > button      { font-family: 'Outfit', sans-serif !important; font-size: 13px !important; font-weight: 600 !important; letter-spacing: .04em !important; color: var(--ctp-overlay0) !important; border-bottom: 2px solid transparent !important; padding: 10px 18px !important; background: transparent !important; transition: color .15s, border-color .15s !important; }
.tabs > .tab-nav > button.selected    { color: var(--ctp-blue) !important; border-bottom-color: var(--ctp-blue) !important; }
.tabs > .tab-nav > button:hover:not(.selected) { color: var(--ctp-subtext1) !important; }

/* ── Panels & blocks ── */
.block, .panel, .form        { background: var(--ctp-mantle) !important; border: 1px solid var(--ctp-surface0) !important; border-radius: 6px !important; }
.gap { gap: 8px !important; }

/* ── Labels ── */
label > span, .label-wrap > span {
  font-family: 'Outfit', sans-serif !important; font-size: 11px !important;
  font-weight: 600 !important; letter-spacing: .08em !important;
  text-transform: uppercase !important; color: var(--ctp-overlay0) !important;
}

/* ── Textareas & text inputs ── */
textarea, input[type="text"] {
  background: var(--ctp-base) !important; color: var(--ctp-text) !important;
  border: 1px solid var(--ctp-surface1) !important;
  font-family: 'JetBrains Mono', monospace !important; font-size: 12px !important;
  border-radius: 4px !important; line-height: 1.6 !important;
}
textarea:focus, input[type="text"]:focus {
  border-color: var(--ctp-blue) !important;
  box-shadow: 0 0 0 1px var(--ctp-blue) !important; outline: none !important;
}

/* ── Buttons ── */
button.primary {
  background: var(--ctp-blue) !important; color: var(--ctp-crust) !important;
  border: none !important; font-family: 'Outfit', sans-serif !important;
  font-weight: 700 !important; font-size: 13px !important; letter-spacing: .03em !important;
  border-radius: 4px !important; transition: background .15s, transform .1s !important;
}
button.primary:hover  { background: var(--ctp-lavender) !important; transform: translateY(-1px) !important; }
button.primary:active { transform: translateY(0) !important; }
button.secondary {
  background: transparent !important; color: var(--ctp-subtext1) !important;
  border: 1px solid var(--ctp-surface1) !important; font-family: 'Outfit', sans-serif !important;
  font-weight: 500 !important; font-size: 13px !important;
  border-radius: 4px !important; transition: border-color .15s, color .15s !important;
}
button.secondary:hover { border-color: var(--ctp-blue) !important; color: var(--ctp-blue) !important; }

/* ── Dropdowns ── */
.wrap { background: var(--ctp-base) !important; border: 1px solid var(--ctp-surface1) !important; border-radius: 4px !important; }

/* ── Accordion ── */
details > summary {
  font-family: 'Outfit', sans-serif !important; font-weight: 600 !important;
  font-size: 12px !important; letter-spacing: .06em !important; text-transform: uppercase !important;
  color: var(--ctp-subtext1) !important; background: var(--ctp-surface0) !important;
  padding: 8px 12px !important; border-radius: 4px !important;
}

/* ── Dataframe ── */
table { font-family: 'JetBrains Mono', monospace !important; font-size: 12px !important; }
thead tr th { background: var(--ctp-surface0) !important; color: var(--ctp-blue) !important; font-weight: 600 !important; letter-spacing: .06em !important; }
tbody tr:nth-child(even) { background: rgba(49,50,68,.4) !important; }
tbody tr:hover { background: var(--ctp-surface1) !important; }

/* ── Scrollbars ── */
::-webkit-scrollbar       { width: 5px; height: 5px; }
::-webkit-scrollbar-track { background: var(--ctp-mantle); }
::-webkit-scrollbar-thumb { background: var(--ctp-surface1); border-radius: 3px; }
::-webkit-scrollbar-thumb:hover { background: var(--ctp-surface2); }
"""

_HEADER_HTML = """
<div class="aa-header">
  <span class="aa-header-logo">AUTO&#x2011;ANNOTATOR</span>
  <div class="aa-header-dot" title="Ready"></div>
  <span class="aa-header-badge">SAM&#x00B7;2</span>
  <span class="aa-header-badge">YOLO&#x00B7;SEG</span>
</div>
"""


def _connect_to_server() -> ModelServerClient | None:
    """Return a connected ModelServerClient, or None if the server is unreachable."""
    probe = ModelServerClient()
    if probe.ping():
        print(  # noqa: T201
            f"[app] Connected to model server on port {SERVER_PORT} — "
            "model stays loaded across restarts.",
        )
        return probe
    print(f"[app] No model server on port {SERVER_PORT} — loading model directly.")  # noqa: T201
    return None


def build_demo(app_ctx: AppContext) -> gr.Blocks:
    """Construct and return the Gradio Blocks application."""
    db.init_db()

    with gr.Blocks(title="Auto-Annotator", theme=gr.themes.Base(), css=_CSS) as demo:
        gr.HTML(_HEADER_HTML)
        state = gr.State(value=AppState())

        ann_c = annotate_tab.build()
        browse_c = browse_tab.build()
        settings_c = settings_tab.build()

        annotate_tab.wire_events(ann_c, state, app_ctx, LABELS_DIR)
        browse_tab.wire_events(browse_c)
        settings_tab.wire_events(settings_c, state, app_ctx, ann_c)

        def _on_load(st: AppState) -> tuple:
            return load_first_image(st, app_ctx, LABELS_DIR).to_gradio()

        def _init_model_dd(_st: AppState) -> gr.update:
            if app_ctx.client is None:
                return gr.update()
            try:
                models = app_ctx.client.list_models()
                choices = [m["label"] for m in models]
                ids = [m["id"] for m in models]
                active_label = next((m["label"] for m in models if m.get("active")), None)
                label_to_id = dict(zip(choices, ids, strict=True))
                return gr.update(choices=list(label_to_id.keys()), value=active_label)
            except Exception:  # noqa: BLE001
                return gr.update()

        def _init_class_dds(st: AppState) -> tuple:
            choices = [c.name for c in st.classes]
            return gr.update(choices=choices), gr.update(choices=choices)

        demo.load(
            fn=_on_load,
            inputs=[state],
            outputs=[
                ann_c.display_img,
                state,
                ann_c.status_box,
                ann_c.stats_box,
                ann_c.image_label,
                ann_c.ann_box,
                browse_c.browse_df,
                browse_c.browse_stats,
            ],
        )

        demo.load(
            fn=_init_model_dd,
            inputs=[state],
            outputs=[settings_c.model_dropdown],
        )

        demo.load(
            fn=_init_class_dds,
            inputs=[state],
            outputs=[ann_c.class_dropdown, settings_c.edit_class_dd],
        )

    return demo


def run_app() -> None:
    """Connect to the model server (or fall back to local inference) and launch Gradio."""
    os.environ.setdefault(
        "HF_HUB_CACHE",
        str(Path(os.environ.get("MODELS_DIR", str(MODELS_DIR)))),
    )

    app_ctx = AppContext(client=_connect_to_server())
    initialize_inference(app_ctx.client, app_ctx.inference)

    demo = build_demo(app_ctx)
    demo.launch(server_name="0.0.0.0", server_port=7860)  # noqa: S104
