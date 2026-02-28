"""src.handlers.classes – Class management event handlers."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import gradio as gr

from src import db
from src.handlers.responses import AddClassResponse, UpdateColorResponse
from src.html import class_swatches_html
from src.render import render_state_image
from src.utils import PALETTE_HEX

if TYPE_CHECKING:
    from src.models import AppState
else:
    AppState = Any


def add_class(name: str, color: str, state: AppState) -> AddClassResponse:
    """Add or update a class, then refresh all class-related UI components."""
    name = name.strip()
    if not name:
        gr.Warning("Class name cannot be empty.")
        choices = [c.name for c in state.classes]
        return AddClassResponse(
            class_dd_update=gr.update(choices=choices),
            state=state,
            color_picker_value=color,
            edit_dd_update=gr.update(choices=choices),
            swatch_html="",
        )

    db.upsert_class(name, color)
    state.classes = db.get_classes()

    choices = [c.name for c in state.classes]
    next_color = PALETTE_HEX[len(state.classes) % len(PALETTE_HEX)]
    return AddClassResponse(
        class_dd_update=gr.update(choices=choices, value=name),
        state=state,
        color_picker_value=next_color,
        edit_dd_update=gr.update(choices=choices),
        swatch_html=class_swatches_html(state.classes),
    )


def prefill_edit_color(class_name: str, state: AppState) -> gr.update:
    """Pre-fill the edit color picker with the class's current color."""
    cls = next((c for c in state.classes if c.name == class_name), None)
    if cls:
        return gr.update(value=cls.color)
    return gr.update()


def update_class_color(class_name: str, new_color: str, state: AppState) -> UpdateColorResponse:
    """Update an existing class's color in the DB and re-render the image."""
    if not class_name:
        return UpdateColorResponse(
            state=state,
            display_img=render_state_image(state),
            status_msg="Select a class to edit.",
        )
    db.upsert_class(class_name, new_color)
    state.classes = db.get_classes()
    for ann in state.annotations:
        if ann.class_name == class_name:
            ann.class_color = new_color
    return UpdateColorResponse(
        state=state,
        display_img=render_state_image(state),
        status_msg=f"Color updated for '{class_name}'.",
    )
