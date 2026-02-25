"""src.handlers.classes – Class management event handlers."""

from __future__ import annotations

import gradio as gr

from src import db
from src.html import _swatch_html
from src.models import AppState, ClassInfo
from src.render import render_state_image
from src.utils import PALETTE_HEX


def add_class(name: str, color: str, state: AppState):
    """Add or update a class, then refresh all class-related UI components."""
    name = name.strip()
    if not name:
        gr.Warning("Class name cannot be empty.")
        choices = [c.name for c in state.classes]
        return gr.update(choices=choices), state, color, gr.update(choices=choices), ""

    db.upsert_class(name, color)
    raw = db.get_classes()
    state.classes = [ClassInfo.from_dict(r) for r in raw]

    choices = [c.name for c in state.classes]
    next_color = PALETTE_HEX[len(state.classes) % len(PALETTE_HEX)]
    return (
        gr.update(choices=choices, value=name),
        state,
        next_color,
        gr.update(choices=choices),
        _swatch_html(color, name),
    )


def class_color_swatch(class_name: str | None, state: AppState) -> str:
    """Return an HTML colour swatch for the selected class."""
    if not class_name:
        return ""
    cls = next((c for c in state.classes if c.name == class_name), None)
    return _swatch_html(cls.color, class_name) if cls else ""


def prefill_edit_color(class_name: str, state: AppState):
    """Pre-fill the edit color picker with the class's current color."""
    cls = next((c for c in state.classes if c.name == class_name), None)
    if cls:
        return gr.update(value=cls.color)
    return gr.update()


def update_class_color(class_name: str, new_color: str, state: AppState):
    """Update an existing class's color in the DB and re-render the image."""
    if not class_name:
        return state, render_state_image(state), "Select a class to edit."
    db.upsert_class(class_name, new_color)
    raw = db.get_classes()
    state.classes = [ClassInfo.from_dict(r) for r in raw]
    for ann in state.annotations:
        if ann.class_name == class_name:
            ann.class_color = new_color
    rendered = render_state_image(state)
    return state, rendered, f"Color updated for '{class_name}'."
