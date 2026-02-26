"""src.handlers.annotation – Annotation event handlers.

Class-assignment fix
--------------------
Each Point stores the class_id at click time.  accept_mask uses
state.pending_class_db_id (updated on every click) rather than re-reading the
active-class dropdown.  Switching classes mid-session no longer overwrites the
class of an already-started mask.

undo_last priority order
------------------------
1. point_buffer non-empty  → pop last point → re-run SAM if points remain,
                             else clear pending mask entirely.
2. pending_mask not None   → clear pending mask.
3. annotations non-empty  → pop last accepted annotation.
4. nothing                 → "Nothing to undo".
"""

from __future__ import annotations

from datetime import UTC, datetime

import gradio as gr

from src import db as _db
from src.constants import INFERENCE_LOG_MAX_ENTRIES, MASK_LABELS
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.handlers.responses import (
    AcceptUndoResponse,
    ClearPointsResponse,
    ClickRunResponse,
    SelectMaskResponse,
)
from src.handlers.utils import format_annotations_summary
from src.inference import run_sam_inference
from src.models import Annotation, AppContext, AppState, Point
from src.render import render_state_image


def _log(state: AppState, msg: str) -> None:
    ts = datetime.now(UTC).strftime("%H:%M:%S")
    state.log_entries.insert(0, f"[{ts}] {msg}")
    state.log_entries = state.log_entries[:INFERENCE_LOG_MAX_ENTRIES]


def _log_str(state: AppState) -> str:
    return "\n".join(state.log_entries)


def _sam_response(state: AppState, all_masks: list, best_idx: int, scores_str: str) -> ClickRunResponse:
    state.pending_masks = all_masks
    state.pending_mask = all_masks[best_idx]
    state.pending_mask_idx = best_idx

    best_label = MASK_LABELS[best_idx] if best_idx < len(MASK_LABELS) else MASK_LABELS[0]
    n_pos = sum(1 for p in state.point_buffer if p.label == 1)
    n_neg = sum(1 for p in state.point_buffer if p.label == 0)
    _log(state, f"{n_pos}+ {n_neg}\u2212  {best_label}  {scores_str}")

    return ClickRunResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        accept_btn_update=gr.update(interactive=True),
        mask_level_update=gr.update(visible=len(all_masks) > 1, value=best_label),
    )


def _err_click(state: AppState, msg: str) -> ClickRunResponse:
    _log(state, msg)
    return ClickRunResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        accept_btn_update=gr.update(interactive=False),
        mask_level_update=gr.update(visible=False),
    )


def _err_accept_undo(state: AppState, msg: str) -> AcceptUndoResponse:
    _log(state, msg)
    return AcceptUndoResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        ann_summary=format_annotations_summary(state.annotations),
        mask_level_update=gr.update(visible=False),
    )


def _clear_pending(state: AppState) -> None:
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.pending_class_db_id = None


def handle_click(
    evt: gr.SelectData,
    state: AppState,
    point_type: str,
    active_class: str | None,
    app_ctx: AppContext,
) -> ClickRunResponse:
    """Add a point then immediately run SAM with the full buffer."""
    if state.current_image is None:
        return _err_click(state, "No image loaded.")
    if not state.classes:
        return _err_click(state, "Add at least one class first.")
    if active_class is None:
        return _err_click(state, "Select an active class.")

    cls_info = next((c for c in state.classes if c.name == active_class), None)
    if cls_info is None:
        return _err_click(state, f"Class '{active_class}' not found in DB.")

    x, y = int(evt.index[0]), int(evt.index[1])
    label = 1 if point_type == "Positive" else 0

    state.point_buffer.append(Point(x=x, y=y, label=label, class_id=cls_info.id))
    state.pending_class_db_id = cls_info.id

    result = run_sam_inference(state, app_ctx.client, app_ctx.inference)
    if not result.ok:
        return _err_click(state, result.error)
    return _sam_response(state, result.masks, result.best_idx, result.scores_str)


def run_sam(state: AppState, app_ctx: AppContext) -> ClickRunResponse:
    """Manual re-run SAM on the current point buffer."""
    if not state.point_buffer:
        return _err_click(state, "Add at least one point first.")
    result = run_sam_inference(state, app_ctx.client, app_ctx.inference)
    if not result.ok:
        return _err_click(state, result.error)
    return _sam_response(state, result.masks, result.best_idx, result.scores_str)


def select_mask_level(level_str: str, state: AppState) -> SelectMaskResponse:
    """Switch the displayed pending mask without re-running SAM."""
    level_map = {lbl: i for i, lbl in enumerate(MASK_LABELS)}
    level = level_map.get(level_str, 1)
    masks = state.pending_masks
    if not masks or level >= len(masks):
        return SelectMaskResponse(display_img=render_state_image(state), state=state)
    state.pending_mask = masks[level]
    state.pending_mask_idx = level
    return SelectMaskResponse(display_img=render_state_image(state), state=state)


def accept_mask(state: AppState) -> AcceptUndoResponse:
    """Accept the pending mask using the class that was active when the mask was created."""
    pending = state.pending_mask
    if pending is None:
        return _err_accept_undo(state, "No pending mask.")

    cls_id = state.pending_class_db_id
    cls_info = next((c for c in state.classes if c.id == cls_id), None)
    if cls_info is None:
        return _err_accept_undo(state, "No class selected.")

    yolo_map = _db.classes_to_yolo_map()
    yolo_cls_id = yolo_map.get(cls_id, 0)
    polygon = mask_to_yolo_polygon(pending)
    bbox = mask_to_yolo_bbox(pending)

    if not polygon:
        _log(state, "Mask too small to polygonise.")
        return AcceptUndoResponse(
            display_img=render_state_image(state),
            state=state,
            log_str=_log_str(state),
            ann_summary=format_annotations_summary(state.annotations),
            mask_level_update=gr.update(visible=False),
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
        ),
    )
    _log(state, f"Accepted mask #{len(state.annotations)} \u2013 {cls_info.name}")

    _clear_pending(state)
    state.point_buffer = []

    return AcceptUndoResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        ann_summary=format_annotations_summary(state.annotations),
        mask_level_update=gr.update(visible=False),
    )


def undo_last(state: AppState, app_ctx: AppContext) -> AcceptUndoResponse:
    """Undo with priority: point → clear mask → annotation."""
    if state.point_buffer:
        state.point_buffer.pop()

        if state.point_buffer:
            result = run_sam_inference(state, app_ctx.client, app_ctx.inference)
            if not result.ok:
                return _err_accept_undo(state, result.error)
            best_label = MASK_LABELS[result.best_idx] if result.best_idx < len(MASK_LABELS) else MASK_LABELS[0]
            state.pending_masks = result.masks
            state.pending_mask = result.masks[result.best_idx]
            state.pending_mask_idx = result.best_idx
            _log(state, f"Undo: re-ran SAM  {result.scores_str}")
            return AcceptUndoResponse(
                display_img=render_state_image(state),
                state=state,
                log_str=_log_str(state),
                ann_summary=gr.update(),
                mask_level_update=gr.update(visible=len(result.masks) > 1, value=best_label),
            )

        _clear_pending(state)
        _log(state, "Undo: cleared points and pending mask")
        return AcceptUndoResponse(
            display_img=render_state_image(state),
            state=state,
            log_str=_log_str(state),
            ann_summary=format_annotations_summary(state.annotations),
            mask_level_update=gr.update(visible=False),
        )

    if state.pending_mask is not None:
        _clear_pending(state)
        _log(state, "Undo: cleared pending mask")
        return AcceptUndoResponse(
            display_img=render_state_image(state),
            state=state,
            log_str=_log_str(state),
            ann_summary=format_annotations_summary(state.annotations),
            mask_level_update=gr.update(visible=False),
        )

    if state.annotations:
        removed = state.annotations.pop()
        _log(state, f"Undo: removed {removed.class_name}")
        return AcceptUndoResponse(
            display_img=render_state_image(state),
            state=state,
            log_str=_log_str(state),
            ann_summary=format_annotations_summary(state.annotations),
            mask_level_update=gr.update(visible=False),
        )

    _log(state, "Nothing to undo")
    return AcceptUndoResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        ann_summary=format_annotations_summary(state.annotations),
        mask_level_update=gr.update(visible=False),
    )


def clear_points(state: AppState) -> ClearPointsResponse:
    """Clear all points and the pending mask."""
    state.point_buffer = []
    _clear_pending(state)
    _log(state, "Points cleared")
    return ClearPointsResponse(
        display_img=render_state_image(state),
        state=state,
        log_str=_log_str(state),
        mask_level_update=gr.update(visible=False),
    )
