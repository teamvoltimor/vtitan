"""src.ui.annotate_tab – Annotate tab UI builder and event wiring."""

from __future__ import annotations

from typing import TYPE_CHECKING

import gradio as gr

from src.constants import MASK_LABELS
from src.handlers import (
    annotation as ann_h,
    navigation as nav_h,
)
from src.render import render_state_image
from src.ui.components import AnnotateTabComponents
from src.ui.constants import (
    ANN_BOX_LINES,
    ANN_NONE,
    BTN_ACCEPT_MASK,
    BTN_CLEAR_POINTS,
    BTN_PREV,
    BTN_RESET_ZOOM,
    BTN_RUN_SAM,
    BTN_SAVE_NEXT,
    BTN_SKIP,
    BTN_UNDO,
    DEFAULT_EXPORT_FMT,
    DEFAULT_MASK_LEVEL,
    DEFAULT_POINT_TYPE,
    DEFAULT_ZOOM,
    ELEM_ID_ANNOTATE_CANVAS,
    LABEL_ACTIVE_CLASS,
    LABEL_ANNOTATIONS,
    LABEL_CURRENT_IMAGE,
    LABEL_EXPORT_FORMAT,
    LABEL_IMAGE,
    LABEL_LOG,
    LABEL_MASK_GRANULARITY,
    LABEL_POINT_TYPE,
    LABEL_ZOOM,
    LOG_LINES,
    PLACEHOLDER_LOG,
    RADIO_EXPORT_FMTS,
    RADIO_POINT_TYPES,
    TAB_ANNOTATE,
    ZOOM_MAX,
    ZOOM_MIN,
    ZOOM_STEP,
)

if TYPE_CHECKING:
    from pathlib import Path

    from src.models import AppContext, AppState


def build() -> AnnotateTabComponents:
    """Build the Annotate tab and return a typed component dataclass."""
    with gr.Tab(TAB_ANNOTATE), gr.Row():
        with gr.Column(scale=3, elem_classes="aa-canvas-column"), gr.Column(elem_classes="aa-canvas-wrapper"):
            display_img = gr.Image(
                label=LABEL_IMAGE,
                type="numpy",
                interactive=True,
                elem_id=ELEM_ID_ANNOTATE_CANVAS,
            )
            mask_level_dd = gr.Dropdown(
                choices=MASK_LABELS,
                value=DEFAULT_MASK_LEVEL,
                label=LABEL_MASK_GRANULARITY,
                visible=False,
                interactive=True,
                elem_classes="aa-input-select",
            )

        with gr.Column(scale=1, elem_classes="aa-annotate-controls"):
            image_label = gr.Textbox(label=LABEL_CURRENT_IMAGE, interactive=False)
            point_type = gr.Dropdown(
                label=LABEL_POINT_TYPE,
                choices=RADIO_POINT_TYPES,
                value=DEFAULT_POINT_TYPE,
                interactive=True,
                elem_classes="aa-input-select",
            )
            class_dropdown = gr.Dropdown(
                label=LABEL_ACTIVE_CLASS,
                choices=[],
                interactive=True,
                elem_classes="aa-input-select",
            )
            zoom_slider = gr.Slider(
                minimum=ZOOM_MIN,
                maximum=ZOOM_MAX,
                step=ZOOM_STEP,
                value=DEFAULT_ZOOM,
                label=LABEL_ZOOM,
                elem_classes="aa-input-select",
            )
            reset_zoom_btn = gr.Button(BTN_RESET_ZOOM, variant="secondary", elem_classes="aa-reset-zoom")
            with gr.Row(elem_classes="aa-action-row"):
                accept_btn = gr.Button(BTN_ACCEPT_MASK, variant="primary")
                undo_btn = gr.Button(BTN_UNDO)
            with gr.Row(elem_classes="aa-action-row"):
                run_btn = gr.Button(BTN_RUN_SAM)
                clear_btn = gr.Button(BTN_CLEAR_POINTS)
            with gr.Row(elem_classes="aa-action-row aa-nav-row"):
                save_btn = gr.Button(BTN_SAVE_NEXT, variant="primary")
                skip_btn = gr.Button(BTN_SKIP)
                prev_btn = gr.Button(BTN_PREV)
            export_fmt = gr.Dropdown(
                label=LABEL_EXPORT_FORMAT,
                choices=RADIO_EXPORT_FMTS,
                value=DEFAULT_EXPORT_FMT,
                interactive=True,
                elem_classes="aa-input-select",
            )
            status_box = gr.Textbox(
                label=LABEL_LOG,
                lines=LOG_LINES,
                interactive=False,
                placeholder=PLACEHOLDER_LOG,
            )
            ann_box = gr.Textbox(
                label=LABEL_ANNOTATIONS,
                lines=ANN_BOX_LINES,
                interactive=False,
                value=ANN_NONE,
            )
            stats_box = gr.HTML(label="Stats")

    return AnnotateTabComponents(
        display_img=display_img,
        mask_level_dd=mask_level_dd,
        image_label=image_label,
        point_type=point_type,
        class_dropdown=class_dropdown,
        accept_btn=accept_btn,
        undo_btn=undo_btn,
        run_btn=run_btn,
        clear_btn=clear_btn,
        save_btn=save_btn,
        skip_btn=skip_btn,
        prev_btn=prev_btn,
        export_fmt=export_fmt,
        status_box=status_box,
        ann_box=ann_box,
        stats_box=stats_box,
        zoom_slider=zoom_slider,
        reset_zoom_btn=reset_zoom_btn,
    )


def wire_events(
    c: AnnotateTabComponents,
    state: gr.State,
    app_ctx: AppContext,
    labels_dir: Path,
) -> None:
    """Wire all Gradio events for the Annotate tab."""

    def _handle_canvas_click(
        evt: gr.SelectData,
        point_type: str,
        active_class: str | None,
        st: AppState,
    ) -> tuple:
        return ann_h.handle_click(evt, st, point_type, active_class, app_ctx).to_gradio()

    def _set_zoom(zoom: float, st: AppState) -> tuple:
        st.zoom = zoom
        return render_state_image(st), st

    def _reset_zoom(st: AppState) -> tuple:
        st.zoom = DEFAULT_ZOOM
        return (
            gr.update(value=DEFAULT_ZOOM),
            render_state_image(st),
            st,
        )

    c.display_img.select(
        fn=_handle_canvas_click,
        inputs=[c.point_type, c.class_dropdown, state],
        outputs=[c.display_img, state, c.status_box, c.accept_btn, c.mask_level_dd],
    )

    c.accept_btn.click(
        fn=lambda st: ann_h.accept_mask(st).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.ann_box, c.mask_level_dd],
    )

    c.undo_btn.click(
        fn=lambda st: ann_h.undo_last(st, app_ctx).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.ann_box, c.mask_level_dd],
    )

    c.run_btn.click(
        fn=lambda st: ann_h.run_sam(st, app_ctx).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.accept_btn, c.mask_level_dd],
    )

    c.clear_btn.click(
        fn=lambda st: ann_h.clear_points(st).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.mask_level_dd],
    )

    c.mask_level_dd.change(
        fn=lambda lvl, st: ann_h.select_mask_level(lvl, st).to_gradio(),
        inputs=[c.mask_level_dd, state],
        outputs=[c.display_img, state],
    )

    c.zoom_slider.change(
        fn=_set_zoom,
        inputs=[c.zoom_slider, state],
        outputs=[c.display_img, state],
    )

    c.reset_zoom_btn.click(
        fn=_reset_zoom,
        inputs=[state],
        outputs=[c.zoom_slider, c.display_img, state],
    )

    c.save_btn.click(
        fn=lambda st, fmt: nav_h.save_and_next(st, fmt, app_ctx, labels_dir).to_gradio(),
        inputs=[state, c.export_fmt],
        outputs=[c.display_img, state, c.status_box, c.stats_box, c.image_label, c.ann_box],
    )

    c.skip_btn.click(
        fn=lambda st: nav_h.skip_image(st, app_ctx, labels_dir).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.stats_box, c.image_label, c.ann_box],
    )

    c.prev_btn.click(
        fn=lambda st: nav_h.go_prev(st, app_ctx, labels_dir).to_gradio(),
        inputs=[state],
        outputs=[c.display_img, state, c.status_box, c.stats_box, c.image_label, c.ann_box],
    )
