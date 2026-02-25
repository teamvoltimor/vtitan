"""src.ui.settings_tab – Settings tab UI builder and event wiring."""

from __future__ import annotations

from typing import TYPE_CHECKING

import gradio as gr

from src.constants import OUTLINE_MODES
from src.utils import PALETTE_HEX

if TYPE_CHECKING:
    from src.models import AppState
    from src.sam_client import ModelServerClient


def build() -> dict:
    """Build the Settings tab.  Returns a dict of component references."""
    with gr.Tab("Settings"):

        # ── Model ──────────────────────────────────────────────────────────────
        with gr.Accordion("Model", open=True):
            model_dropdown = gr.Dropdown(
                label="SAM model",
                choices=[],
                interactive=True,
            )
            load_model_btn = gr.Button("Load model", variant="primary")
            model_status_box = gr.Textbox(
                label="Model status",
                interactive=False,
                placeholder="No model loaded.",
            )
            auto_annotate_btn = gr.Button(
                "Auto-annotate (text)",
                visible=False,
                variant="secondary",
            )

        # ── Classes ────────────────────────────────────────────────────────────
        with gr.Accordion("Classes", open=True):
            with gr.Row():
                class_input = gr.Textbox(label="New class name", placeholder="e.g. red_prism")
                color_picker = gr.ColorPicker(
                    label="Colour",
                    value=PALETTE_HEX[0],
                )
            add_class_btn = gr.Button("Add / update class", variant="primary")
            class_swatch = gr.HTML()

            gr.Markdown("### Edit class colour")
            with gr.Row():
                edit_class_dd = gr.Dropdown(label="Class to edit", choices=[], interactive=True)
                edit_color_picker = gr.ColorPicker(label="New colour")
            update_color_btn = gr.Button("Update colour")
            edit_status = gr.Textbox(label="Edit status", interactive=False)

        # ── Display ────────────────────────────────────────────────────────────
        with gr.Accordion("Display", open=False):
            outline_color_dd = gr.Dropdown(
                choices=OUTLINE_MODES,
                value="Class color",
                label="Outline colour mode",
                interactive=True,
            )

    return dict(
        model_dropdown=model_dropdown,
        load_model_btn=load_model_btn,
        model_status_box=model_status_box,
        auto_annotate_btn=auto_annotate_btn,
        class_input=class_input,
        color_picker=color_picker,
        add_class_btn=add_class_btn,
        class_swatch=class_swatch,
        edit_class_dd=edit_class_dd,
        edit_color_picker=edit_color_picker,
        update_color_btn=update_color_btn,
        edit_status=edit_status,
        outline_color_dd=outline_color_dd,
    )


def wire_events(c: dict, state: gr.State, client_ref: list, ann_c: dict) -> None:
    """Wire all Gradio events for the Settings tab.
    *ann_c* is the dict of Annotate tab components so we can push updates there.
    """
    from src.handlers import classes as cls_h  # noqa: PLC0415
    from src.handlers import model_ctrl as mc_h  # noqa: PLC0415
    from src.render import render_state_image  # noqa: PLC0415

    def _client():
        return client_ref[0]

    # ── Add class ──────────────────────────────────────────────────────────────
    c["add_class_btn"].click(
        fn=cls_h.add_class,
        inputs=[c["class_input"], c["color_picker"], state],
        outputs=[
            ann_c["class_dropdown"],
            state,
            c["color_picker"],
            c["edit_class_dd"],
            c["class_swatch"],
        ],
    )

    # ── Prefill edit color ─────────────────────────────────────────────────────
    c["edit_class_dd"].change(
        fn=cls_h.prefill_edit_color,
        inputs=[c["edit_class_dd"], state],
        outputs=[c["edit_color_picker"]],
    )

    # ── Update class color ─────────────────────────────────────────────────────
    c["update_color_btn"].click(
        fn=cls_h.update_class_color,
        inputs=[c["edit_class_dd"], c["edit_color_picker"], state],
        outputs=[state, ann_c["display_img"], c["edit_status"]],
    )

    # ── Outline color dropdown ─────────────────────────────────────────────────
    def _update_outline(mode, st):
        st.outline_color = mode
        return render_state_image(st), st

    c["outline_color_dd"].change(
        fn=_update_outline,
        inputs=[c["outline_color_dd"], state],
        outputs=[ann_c["display_img"], state],
    )

    # ── Load model ─────────────────────────────────────────────────────────────
    c["load_model_btn"].click(
        fn=lambda mid, st: mc_h.switch_model(mid, st, _client()),
        inputs=[c["model_dropdown"], state],
        outputs=[state, c["model_status_box"], c["auto_annotate_btn"]],
    )

    # ── Auto-annotate ──────────────────────────────────────────────────────────
    c["auto_annotate_btn"].click(
        fn=lambda st: mc_h.auto_annotate(st, _client()),
        inputs=[state],
        outputs=[ann_c["display_img"], state, c["model_status_box"], ann_c["ann_box"]],
    )


def populate_models(c: dict, client_ref: list) -> None:
    """Called after demo.load() to fill the model dropdown with server data."""
    client = client_ref[0]
    if client is None:
        return
    try:
        models = client.list_models()
        choices = [m["label"] for m in models]
        ids = [m["id"] for m in models]
        active = next((m["label"] for m in models if m.get("active")), None)
        # Gradio doesn't support updating choices from outside an event easily,
        # so we store choices+ids for use in the load event.
        client_ref.append({"model_choices": choices, "model_ids": ids, "active": active})
    except Exception:  # noqa: BLE001
        pass
