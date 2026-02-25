"""src.handlers.annotation – Annotation event handlers (click, accept, undo, clear).

Bug fix (class-assignment):
  Each Point stores the class_id at the time it was clicked.
  accept_mask uses state.pending_class_db_id (set on every click) so each
  accepted annotation correctly records the class that was active when the
  mask was created — not the currently selected class.

Bug fix (undo_last priority order):
  1. If point_buffer non-empty: pop last point; if still has points re-run SAM,
     else clear pending mask entirely.
  2. Elif pending_mask is not None: clear pending mask.
  3. Elif annotations non-empty: pop last annotation.
  4. Else: "Nothing to undo".
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import gradio as gr

from src.constants import LABELS_DIR, MASK_LABELS
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.inference import run_sam_inference
from src.models import Annotation, AppState, Point
from src.render import render_state_image
from src.sam_client import ModelServerClient


# ── Internal helpers ───────────────────────────────────────────────────────────


def _ann_summary(annotations: list[Annotation]) -> str:
    if not annotations:
        return "(none)"
    lines = [
        f"{i}. [{ann.yolo_class_id}] {ann.class_name}  ({len(ann.polygon) // 2} pts)"
        for i, ann in enumerate(annotations, 1)
    ]
    return "\n".join(lines)


def _log(state: AppState, msg: str) -> None:
    ts = datetime.now(timezone.utc).strftime("%H:%M:%S")
    state.log_entries.insert(0, f"[{ts}] {msg}")
    state.log_entries = state.log_entries[:50]


def _apply_sam_result(state: AppState, all_masks: list, best_idx: int, scores_str: str):
    """Store inference result in state and return the 5-tuple Gradio output."""
    state.pending_masks = all_masks
    state.pending_mask = all_masks[best_idx]
    state.pending_mask_idx = best_idx

    best_label = MASK_LABELS[best_idx] if best_idx < len(MASK_LABELS) else MASK_LABELS[0]
    show_picker = len(all_masks) > 1
    n_pos = sum(1 for p in state.point_buffer if p.label == 1)
    n_neg = sum(1 for p in state.point_buffer if p.label == 0)
    status = f"{n_pos}+ {n_neg}−  |  {best_label}  {scores_str}"

    rendered = render_state_image(state)
    return (
        rendered,
        state,
        status,
        gr.update(interactive=True),
        gr.update(visible=show_picker, value=best_label),
    )


def _err(state: AppState, msg: str):
    return (
        render_state_image(state),
        state,
        msg,
        gr.update(interactive=False),
        gr.update(visible=False),
    )


# ── Public handlers ────────────────────────────────────────────────────────────


def handle_click(
    evt: gr.SelectData,
    state: AppState,
    point_type: str,
    active_class: str | None,
    client: ModelServerClient | None,
):
    """Add a point then immediately run SAM with the full buffer."""
    if state.current_image is None:
        return _err(state, "No image loaded.")
    if not state.classes:
        return _err(state, "Add at least one class first.")
    if active_class is None:
        return _err(state, "Select an active class.")

    cls_info = next((c for c in state.classes if c.name == active_class), None)
    if cls_info is None:
        return _err(state, f"Class '{active_class}' not found in DB.")

    x, y = int(evt.index[0]), int(evt.index[1])
    label = 1 if point_type == "Positive" else 0

    # Each point records the class that was active when it was clicked.
    # This means mixed-class click sessions are fully tracked per-point,
    # and the mask accepted after these clicks uses the class from the
    # LAST click (state.pending_class_db_id), which is the expected behaviour.
    state.point_buffer.append(Point(x=x, y=y, label=label, class_id=cls_info.id))
    state.pending_class_db_id = cls_info.id

    all_masks, best_idx, scores_str, err = run_sam_inference(state, client)
    if err:
        n_pos = sum(1 for p in state.point_buffer if p.label == 1)
        n_neg = sum(1 for p in state.point_buffer if p.label == 0)
        rendered = render_state_image(state)
        return rendered, state, f"{n_pos}+ {n_neg}−  |  {err}", gr.update(interactive=False), gr.update(visible=False)

    return _apply_sam_result(state, all_masks, best_idx, scores_str)


def run_sam(state: AppState, client: ModelServerClient | None):
    """Manual re-run — useful after toggling point type."""
    if not state.point_buffer:
        return _err(state, "Add at least one point first.")

    all_masks, best_idx, scores_str, err = run_sam_inference(state, client)
    if err:
        return _err(state, err)
    return _apply_sam_result(state, all_masks, best_idx, scores_str)


def select_mask_level(level_str: str, state: AppState):
    """Switch the displayed pending mask without re-running SAM."""
    level_map = {lbl: i for i, lbl in enumerate(MASK_LABELS)}
    level = level_map.get(level_str, 1)
    masks = state.pending_masks
    if not masks or level >= len(masks):
        return render_state_image(state), state
    state.pending_mask = masks[level]
    state.pending_mask_idx = level
    return render_state_image(state), state


def accept_mask(state: AppState):
    """
    Accept the current pending mask.

    Uses state.pending_class_db_id (set when the mask was created) rather
    than re-reading the active_class dropdown.  This fixes the bug where
    switching classes between clicks caused all annotations to adopt the
    last selected class.
    """
    pending = state.pending_mask
    if pending is None:
        return (
            render_state_image(state),
            state,
            "No pending mask.",
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    cls_id = state.pending_class_db_id
    cls_info = next((c for c in state.classes if c.id == cls_id), None)
    if cls_info is None:
        return (
            render_state_image(state),
            state,
            "No class selected.",
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    from src import db as _db  # noqa: PLC0415

    yolo_map = _db.classes_to_yolo_map()
    yolo_cls_id = yolo_map.get(cls_id, 0)
    polygon = mask_to_yolo_polygon(pending)
    bbox = mask_to_yolo_bbox(pending)

    if not polygon:
        return (
            render_state_image(state),
            state,
            "Mask too small to polygonise.",
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    state.annotations.append(
        Annotation(
            class_db_id=cls_id,
            class_name=cls_info.name,
            class_color=cls_info.color,
            yolo_class_id=yolo_cls_id,
            polygon=polygon,
            bbox=bbox,
            mask=pending,
        )
    )
    _log(state, f"Accepted mask #{len(state.annotations)} – {cls_info.name}")

    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.pending_class_db_id = None
    state.point_buffer = []

    n = len(state.annotations)
    return (
        render_state_image(state),
        state,
        f"{n} annotation(s) accepted.",
        _ann_summary(state.annotations),
        gr.update(visible=False),
    )


def undo_last(state: AppState, client: ModelServerClient | None):
    """
    Undo the last action with the following priority:
    1. Pop last point from buffer → re-run SAM if points remain, else clear mask.
    2. Clear pending mask (if present and buffer is empty).
    3. Pop last accepted annotation.
    4. Nothing to undo.
    """
    if state.point_buffer:
        state.point_buffer.pop()

        if state.point_buffer:
            # Still have points → re-run SAM
            all_masks, best_idx, scores_str, err = run_sam_inference(state, client)
            if err:
                return _err(state, err)
            return _apply_sam_result(state, all_masks, best_idx, scores_str)

        # No points left → clear pending mask entirely
        state.pending_mask = None
        state.pending_masks = []
        state.pending_logits = None
        state.pending_mask_idx = 0
        state.pending_class_db_id = None
        _log(state, "Undo: cleared point buffer and pending mask")
        return (
            render_state_image(state),
            state,
            "Point removed; pending mask cleared.",
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    if state.pending_mask is not None:
        state.pending_mask = None
        state.pending_masks = []
        state.pending_logits = None
        state.pending_mask_idx = 0
        state.pending_class_db_id = None
        _log(state, "Undo: cleared pending mask")
        return (
            render_state_image(state),
            state,
            "Pending mask cleared.",
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    if state.annotations:
        removed = state.annotations.pop()
        _log(state, f"Undo: removed annotation {removed.class_name}")
        n = len(state.annotations)
        msg = f"{n} annotation(s) remaining." if n else "All annotations removed."
        return (
            render_state_image(state),
            state,
            msg,
            _ann_summary(state.annotations),
            gr.update(visible=False),
        )

    return (
        render_state_image(state),
        state,
        "Nothing to undo.",
        _ann_summary(state.annotations),
        gr.update(visible=False),
    )


def clear_points(state: AppState):
    state.point_buffer = []
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.pending_class_db_id = None
    return render_state_image(state), state, "Points cleared.", gr.update(visible=False)
