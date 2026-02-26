"""src.ui.settings_tab – Settings tab UI builder and event wiring."""

from __future__ import annotations

from typing import TYPE_CHECKING

import gradio as gr

from src.constants import OUTLINE_MODES
from src.handlers import (
    classes as cls_h,
    model_ctrl as mc_h,
)
from src.render import render_state_image
from src.ui.components import AnnotateTabComponents, SettingsTabComponents

if TYPE_CHECKING:
    from src.models import AppContext, AppState
from src.ui.constants import (
    ACCORDION_CLASSES,
    ACCORDION_DISPLAY,
    ACCORDION_MODEL,
    BTN_ADD_CLASS,
    BTN_AUTO_ANNOTATE,
    BTN_LOAD_MODEL,
    BTN_UPDATE_COLOR,
    DEFAULT_OUTLINE_MODE,
    HEADING_EDIT_CLASS_COLOR,
    LABEL_CLASS_TO_EDIT,
    LABEL_COLOR,
    LABEL_EDIT_STATUS,
    LABEL_MODEL_STATUS,
    LABEL_NEW_CLASS,
    LABEL_NEW_COLOR,
    LABEL_OUTLINE_MODE,
    LABEL_SAM_MODEL,
    PLACEHOLDER_CLASS_NAME,
    PLACEHOLDER_MODEL_STATUS,
    TAB_SETTINGS,
)
from src.utils import PALETTE_HEX


def build() -> SettingsTabComponents:
    """Build the Settings tab and return a typed component dataclass."""
    with gr.Tab(TAB_SETTINGS):

        with gr.Accordion(ACCORDION_MODEL, open=True):
            model_dropdown = gr.Dropdown(
                label=LABEL_SAM_MODEL,
                choices=[],
                interactive=True,
            )
            load_model_btn = gr.Button(BTN_LOAD_MODEL, variant="primary")
            model_status_box = gr.Textbox(
                label=LABEL_MODEL_STATUS,
                interactive=False,
                placeholder=PLACEHOLDER_MODEL_STATUS,
            )
            auto_annotate_btn = gr.Button(
                BTN_AUTO_ANNOTATE,
                visible=False,
                variant="secondary",
            )

        with gr.Accordion(ACCORDION_CLASSES, open=True):
            with gr.Row():
                class_input = gr.Textbox(label=LABEL_NEW_CLASS, placeholder=PLACEHOLDER_CLASS_NAME)
                color_picker = gr.ColorPicker(label=LABEL_COLOR, value=PALETTE_HEX[0])
            add_class_btn = gr.Button(BTN_ADD_CLASS, variant="primary")
            class_swatch = gr.HTML()

            gr.Markdown(HEADING_EDIT_CLASS_COLOR)
            with gr.Row():
                edit_class_dd = gr.Dropdown(
                    label=LABEL_CLASS_TO_EDIT, choices=[], interactive=True,
                )
                edit_color_picker = gr.ColorPicker(label=LABEL_NEW_COLOR)
            update_color_btn = gr.Button(BTN_UPDATE_COLOR)
            edit_status = gr.Textbox(label=LABEL_EDIT_STATUS, interactive=False)

        with gr.Accordion(ACCORDION_DISPLAY, open=False):
            outline_color_dd = gr.Dropdown(
                choices=OUTLINE_MODES,
                value=DEFAULT_OUTLINE_MODE,
                label=LABEL_OUTLINE_MODE,
                interactive=True,
            )

    return SettingsTabComponents(
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


def wire_events(
    c: SettingsTabComponents,
    state: gr.State,
    app_ctx: AppContext,
    ann_c: AnnotateTabComponents,
) -> None:
    """Wire all Gradio events for the Settings tab.

    *ann_c* is the Annotate tab component dataclass so updates can be pushed
    to the canvas and class dropdowns from here.
    """
    c.add_class_btn.click(
        fn=lambda name, color, st: cls_h.add_class(name, color, st).to_gradio(),
        inputs=[c.class_input, c.color_picker, state],
        outputs=[ann_c.class_dropdown, state, c.color_picker, c.edit_class_dd, c.class_swatch],
    )

    c.edit_class_dd.change(
        fn=cls_h.prefill_edit_color,
        inputs=[c.edit_class_dd, state],
        outputs=[c.edit_color_picker],
    )

    c.update_color_btn.click(
        fn=lambda cn, nc, st: cls_h.update_class_color(cn, nc, st).to_gradio(),
        inputs=[c.edit_class_dd, c.edit_color_picker, state],
        outputs=[state, ann_c.display_img, c.edit_status],
    )

    def _update_outline(mode: str, st: AppState) -> tuple:
        st.outline_color = mode
        return render_state_image(st), st

    c.outline_color_dd.change(
        fn=_update_outline,
        inputs=[c.outline_color_dd, state],
        outputs=[ann_c.display_img, state],
    )

    c.load_model_btn.click(
        fn=lambda mid, st: mc_h.switch_model(mid, st, app_ctx).to_gradio(),
        inputs=[c.model_dropdown, state],
        outputs=[state, c.model_status_box, c.auto_annotate_btn],
    )

    c.auto_annotate_btn.click(
        fn=lambda st: mc_h.auto_annotate(st, app_ctx).to_gradio(),
        inputs=[state],
        outputs=[ann_c.display_img, state, c.model_status_box, ann_c.ann_box],
    )
