"""src.ui.components – Typed component dataclasses returned by each tab's build().

Using dataclasses instead of dicts gives IDE autocomplete and static type
checking on all component references in the event-wiring layer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import gradio as gr


@dataclass
class AnnotateTabComponents:
    """All Gradio components created by :func:`src.ui.annotate_tab.build`."""

    display_img: gr.Image
    mask_level_dd: gr.Dropdown
    image_label: gr.Textbox
    point_type: gr.Radio
    class_dropdown: gr.Dropdown
    accept_btn: gr.Button
    undo_btn: gr.Button
    run_btn: gr.Button
    clear_btn: gr.Button
    save_btn: gr.Button
    skip_btn: gr.Button
    prev_btn: gr.Button
    export_fmt: gr.Radio
    status_box: gr.Textbox
    ann_box: gr.Textbox
    stats_box: gr.HTML
    zoom_slider: gr.Slider
    reset_zoom_btn: gr.Button


@dataclass
class BrowseTabComponents:
    """All Gradio components created by :func:`src.ui.browse_tab.build`."""

    refresh_btn: gr.Button
    import_btn: gr.UploadButton
    view_toggle: gr.Radio
    import_status: gr.Textbox
    browse_stats: gr.HTML
    browse_df: gr.Dataframe
    browse_gallery: gr.Gallery
    modal_row: gr.Row
    modal_img: gr.Image
    modal_info: gr.Markdown
    modal_prev_btn: gr.Button
    modal_next_btn: gr.Button
    modal_close_btn: gr.Button
    modal_idx_state: gr.State


@dataclass
class SettingsTabComponents:
    """All Gradio components created by :func:`src.ui.settings_tab.build`."""

    model_dropdown: gr.Dropdown
    load_model_btn: gr.Button
    model_status_box: gr.Textbox
    auto_annotate_btn: gr.Button
    class_input: gr.Textbox
    color_picker: gr.ColorPicker
    add_class_btn: gr.Button
    class_swatch: gr.HTML
    edit_class_dd: gr.Dropdown
    edit_color_picker: gr.ColorPicker
    update_color_btn: gr.Button
    edit_status: gr.Textbox
    outline_color_dd: gr.Dropdown
