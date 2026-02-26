"""src.handlers.model_ctrl – Model switching and auto-annotate handlers."""

from __future__ import annotations

import gradio as gr

from src import db
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.handlers.responses import AutoAnnotateResponse, SwitchModelResponse
from src.handlers.utils import format_annotations_summary
from src.models import Annotation, AppContext, AppState, ClassInfo
from src.render import render_state_image


def switch_model(model_id: str, state: AppState, app_ctx: AppContext) -> SwitchModelResponse:
    """Load a different SAM model on the server and reset image state."""
    # Guard: server connection and a selected model id are both required.
    if app_ctx.client is None:
        return SwitchModelResponse(
            state=state,
            status_msg="Model server not connected.",
            auto_btn_update=gr.update(visible=False),
        )
    if not model_id:
        return SwitchModelResponse(
            state=state,
            status_msg="Select a model first.",
            auto_btn_update=gr.update(visible=False),
        )

    resp = app_ctx.client.set_model(model_id)
    if "error" in resp:
        return SwitchModelResponse(
            state=state,
            status_msg=f"Error: {resp['error']}",
            auto_btn_update=gr.update(visible=False),
        )

    # Reset per-image state so the new model starts clean.
    state.image_set = False
    state.point_buffer = []
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.active_model_id = model_id

    # Query the server for updated model capabilities (e.g. text-seg support).
    models = app_ctx.client.list_models()
    active_cfg = next((m for m in models if m["id"] == model_id), {})
    state.model_supports_text = active_cfg.get("supports_text", False)

    return SwitchModelResponse(
        state=state,
        status_msg=f"Model loaded: {active_cfg.get('label', model_id)}",
        auto_btn_update=gr.update(visible=state.model_supports_text),
    )


def _append_mask_annotation(
    mask: object,
    class_info: ClassInfo,
    yolo_map: dict[int, int],
    state: AppState,
) -> bool:
    """Validate *mask* and append an Annotation to *state* when valid.

    Args:
        mask:       Boolean numpy mask array from the server result.
        class_info: ClassInfo for the detected class.
        yolo_map:   Mapping from DB class id to YOLO class index.
        state:      Mutable session state to append the annotation to.

    Returns:
        ``True`` when the annotation was appended; ``False`` when the mask was
        empty or too small to produce a valid polygon.
    """
    import numpy as np

    mask_arr = np.asarray(mask)
    if not mask_arr.any():
        return False

    polygon = mask_to_yolo_polygon(mask_arr)
    bbox = mask_to_yolo_bbox(mask_arr)
    if not polygon:
        return False

    state.annotations.append(
        Annotation(
            class_db_id=class_info.id,
            class_name=class_info.name,
            class_color=class_info.color,
            yolo_class_id=yolo_map.get(class_info.id, 0),
            polygon=polygon,
            bbox=bbox,
            mask=mask_arr,
        ),
    )
    return True


def auto_annotate(state: AppState, app_ctx: AppContext) -> AutoAnnotateResponse:
    """Run SAM3 text-prompted segmentation for every defined class."""
    # Guard: server, loaded image, and at least one class are all required.
    if app_ctx.client is None:
        return AutoAnnotateResponse(
            display_img=render_state_image(state),
            state=state,
            status_msg="Model server not connected.",
            ann_box_update=gr.update(),
        )

    if state.current_image is None:
        return AutoAnnotateResponse(
            display_img=render_state_image(state),
            state=state,
            status_msg="No image loaded.",
            ann_box_update=gr.update(),
        )

    if not state.classes:
        return AutoAnnotateResponse(
            display_img=render_state_image(state),
            state=state,
            status_msg="Add at least one class first.",
            ann_box_update=gr.update(),
        )

    # Send all class names to the server for text-prompted segmentation.
    class_names = [c.name for c in state.classes]
    try:
        results = app_ctx.client.predict_text(state.current_image, class_names)
    except Exception as e:  # noqa: BLE001
        return AutoAnnotateResponse(
            display_img=render_state_image(state),
            state=state,
            status_msg=f"Error: {e}",
            ann_box_update=gr.update(),
        )

    # Convert valid server masks to local Annotation objects.
    yolo_map = db.classes_to_yolo_map()
    classes_by_name = {c.name: c for c in state.classes}
    added_count = 0

    for result in results:
        if result.get("error"):
            continue
        class_name = result["class_name"]
        class_info = classes_by_name.get(class_name)
        if class_info is None:
            continue
        for mask in result.get("masks", []):
            if _append_mask_annotation(mask, class_info, yolo_map, state):
                added_count += 1

    ann_text = format_annotations_summary(state.annotations)
    return AutoAnnotateResponse(
        display_img=render_state_image(state),
        state=state,
        status_msg=f"Auto-annotated: {added_count} mask(s) added.",
        ann_box_update=gr.update(value=ann_text),
    )
