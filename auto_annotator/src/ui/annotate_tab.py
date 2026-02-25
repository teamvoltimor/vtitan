"""src.ui.annotate_tab – Annotate tab UI builder and event wiring."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import gradio as gr

from src.constants import MASK_LABELS, OUTLINE_MODES
from src.handlers import annotation as ann_h

if TYPE_CHECKING:
    from src.models import AppState
    from src.sam_client import ModelServerClient


def build() -> dict:
    """Build the Annotate tab.  Returns a dict of component references."""
    with gr.Tab("Annotate"):
        with gr.Row():
            with gr.Column(scale=3):
                display_img = gr.Image(
                    label="Image",
                    type="numpy",
                    interactive=True,
                    elem_id="annotate-canvas",
                )
                mask_level_dd = gr.Dropdown(
                    choices=MASK_LABELS,
                    value="Object (1)",
                    label="Mask granularity",
                    visible=False,
                    interactive=True,
                )

            with gr.Column(scale=1):
                image_label = gr.Textbox(label="Current image", interactive=False)
                point_type = gr.Radio(
                    ["Positive", "Negative"],
                    value="Positive",
                    label="Point type",
                )
                class_dropdown = gr.Dropdown(
                    label="Active class",
                    choices=[],
                    interactive=True,
                )
                with gr.Row():
                    accept_btn = gr.Button("Accept mask ✓", variant="primary")
                    undo_btn = gr.Button("Undo ↩")
                with gr.Row():
                    run_btn = gr.Button("Re-run SAM")
                    clear_btn = gr.Button("Clear points ✕")
                with gr.Row():
                    save_btn = gr.Button("Save & Next →", variant="primary")
                    skip_btn = gr.Button("Skip ⏭")
                    prev_btn = gr.Button("← Prev")
                export_fmt = gr.Radio(
                    ["Segmentation", "Detection"],
                    value="Segmentation",
                    label="Export format",
                )

                status_box = gr.Textbox(
                    label="Log",
                    lines=4,
                    interactive=False,
                    placeholder="Activity log…",
                )
                ann_box = gr.Textbox(
                    label="Annotations",
                    lines=6,
                    interactive=False,
                    value="(none)",
                )
                stats_box = gr.HTML(label="Stats")

    return dict(
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


def wire_events(c: dict, state: gr.State, client_ref: list, labels_dir: Path) -> None:
    """Wire all Gradio events for the Annotate tab.
    *client_ref* is a one-element list so we can pass the client by reference.
    """
    from src.handlers import navigation as nav_h  # noqa: PLC0415

    def _client():
        return client_ref[0]

    # Click on canvas → add point + run SAM
    c["display_img"].select(
        fn=lambda evt, st, pt, ac: ann_h.handle_click(evt, st, pt, ac, _client()),
        inputs=[state, c["point_type"], c["class_dropdown"]],
        outputs=[c["display_img"], state, c["status_box"], c["accept_btn"], c["mask_level_dd"]],
    )

    # Accept mask
    c["accept_btn"].click(
        fn=ann_h.accept_mask,
        inputs=[state],
        outputs=[c["display_img"], state, c["status_box"], c["ann_box"], c["mask_level_dd"]],
    )

    # Undo
    c["undo_btn"].click(
        fn=lambda st: ann_h.undo_last(st, _client()),
        inputs=[state],
        outputs=[c["display_img"], state, c["status_box"], c["ann_box"], c["mask_level_dd"]],
    )

    # Re-run SAM
    c["run_btn"].click(
        fn=lambda st: ann_h.run_sam(st, _client()),
        inputs=[state],
        outputs=[c["display_img"], state, c["status_box"], c["accept_btn"], c["mask_level_dd"]],
    )

    # Clear points
    c["clear_btn"].click(
        fn=ann_h.clear_points,
        inputs=[state],
        outputs=[c["display_img"], state, c["status_box"], c["mask_level_dd"]],
    )

    # Mask level dropdown
    c["mask_level_dd"].change(
        fn=ann_h.select_mask_level,
        inputs=[c["mask_level_dd"], state],
        outputs=[c["display_img"], state],
    )

    # Save & Next
    c["save_btn"].click(
        fn=lambda st, fmt: nav_h.save_and_next(st, fmt, _client(), labels_dir),
        inputs=[state, c["export_fmt"]],
        outputs=[
            c["display_img"], state, c["status_box"],
            c["stats_box"], c["image_label"], c["ann_box"],
        ],
    )

    # Skip
    c["skip_btn"].click(
        fn=lambda st: nav_h.skip_image(st, _client(), labels_dir),
        inputs=[state],
        outputs=[
            c["display_img"], state, c["status_box"],
            c["stats_box"], c["image_label"], c["ann_box"],
        ],
    )

    # Prev
    c["prev_btn"].click(
        fn=lambda st: nav_h.go_prev(st, _client(), labels_dir),
        inputs=[state],
        outputs=[
            c["display_img"], state, c["status_box"],
            c["stats_box"], c["image_label"], c["ann_box"],
        ],
    )
