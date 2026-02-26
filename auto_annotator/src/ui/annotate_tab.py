"""src.ui.annotate_tab – Annotate tab UI builder and event wiring."""

from __future__ import annotations

from typing import TYPE_CHECKING

import gradio as gr

from src.constants import MASK_LABELS
from src.handlers import (
    annotation as ann_h,
    navigation as nav_h,
)

if TYPE_CHECKING:
    from pathlib import Path

    from src.models import AppContext
from src.ui.components import AnnotateTabComponents
from src.ui.constants import (
    ANN_BOX_LINES,
    ANN_NONE,
    BTN_ACCEPT_MASK,
    BTN_CLEAR_POINTS,
    BTN_PREV,
    BTN_RUN_SAM,
    BTN_SAVE_NEXT,
    BTN_SKIP,
    BTN_UNDO,
    DEFAULT_EXPORT_FMT,
    DEFAULT_MASK_LEVEL,
    DEFAULT_POINT_TYPE,
    ELEM_ID_ANNOTATE_CANVAS,
    LABEL_ACTIVE_CLASS,
    LABEL_ANNOTATIONS,
    LABEL_CURRENT_IMAGE,
    LABEL_EXPORT_FORMAT,
    LABEL_IMAGE,
    LABEL_LOG,
    LABEL_MASK_GRANULARITY,
    LABEL_POINT_TYPE,
    LOG_LINES,
    PLACEHOLDER_LOG,
    RADIO_EXPORT_FMTS,
    RADIO_POINT_TYPES,
    TAB_ANNOTATE,
)


def build() -> AnnotateTabComponents:
    """Build the Annotate tab and return a typed component dataclass."""
    with gr.Tab(TAB_ANNOTATE), gr.Row():
        with gr.Column(scale=3):
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
            )

        with gr.Column(scale=1):
            image_label = gr.Textbox(label=LABEL_CURRENT_IMAGE, interactive=False)
            point_type = gr.Radio(
                RADIO_POINT_TYPES,
                value=DEFAULT_POINT_TYPE,
                label=LABEL_POINT_TYPE,
            )
            class_dropdown = gr.Dropdown(
                label=LABEL_ACTIVE_CLASS,
                choices=[],
                interactive=True,
            )
            with gr.Row():
                accept_btn = gr.Button(BTN_ACCEPT_MASK, variant="primary")
                undo_btn = gr.Button(BTN_UNDO)
            with gr.Row():
                run_btn = gr.Button(BTN_RUN_SAM)
                clear_btn = gr.Button(BTN_CLEAR_POINTS)
            with gr.Row():
                save_btn = gr.Button(BTN_SAVE_NEXT, variant="primary")
                skip_btn = gr.Button(BTN_SKIP)
                prev_btn = gr.Button(BTN_PREV)
            export_fmt = gr.Radio(
                RADIO_EXPORT_FMTS,
                value=DEFAULT_EXPORT_FMT,
                label=LABEL_EXPORT_FORMAT,
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
    )


def wire_events(
    c: AnnotateTabComponents,
    state: gr.State,
    app_ctx: AppContext,
    labels_dir: Path,
) -> None:
    """Wire all Gradio events for the Annotate tab."""
    c.display_img.select(
        fn=lambda evt, st, pt, ac: ann_h.handle_click(evt, st, pt, ac, app_ctx).to_gradio(),
        inputs=[state, c.point_type, c.class_dropdown],
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
