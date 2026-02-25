"""src.handlers.model_ctrl – Model switching and auto-annotate handlers."""

from __future__ import annotations

import gradio as gr

from src import db
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.models import Annotation, AppState
from src.render import render_state_image
from src.sam_client import ModelServerClient


def switch_model(model_id: str, state: AppState, client: ModelServerClient | None):
    """Load a different SAM model on the server and reset image state."""
    if client is None:
        return state, "Model server not connected.", gr.update(visible=False)
    if not model_id:
        return state, "Select a model first.", gr.update(visible=False)

    resp = client.set_model(model_id)
    if "error" in resp:
        return state, f"Error: {resp['error']}", gr.update(visible=False)

    state.image_set = False
    state.point_buffer = []
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.active_model_id = model_id

    models = client.list_models()
    active_cfg = next((m for m in models if m["id"] == model_id), {})
    state.model_supports_text = active_cfg.get("supports_text", False)

    return (
        state,
        f"Model loaded: {active_cfg.get('label', model_id)}",
        gr.update(visible=state.model_supports_text),
    )


def auto_annotate(state: AppState, client: ModelServerClient | None):
    """Run SAM3 text-prompted segmentation for every defined class."""
    if client is None:
        return render_state_image(state), state, "Model server not connected.", gr.update()

    img = state.current_image
    if img is None:
        return render_state_image(state), state, "No image loaded.", gr.update()

    if not state.classes:
        return render_state_image(state), state, "Add at least one class first.", gr.update()

    class_names = [c.name for c in state.classes]
    try:
        results = client.predict_text(img, class_names)
    except Exception as e:  # noqa: BLE001
        return render_state_image(state), state, f"Error: {e}", gr.update()

    yolo_map = db.classes_to_yolo_map()
    cls_by_name = {c.name: c for c in state.classes}
    added = 0

    for result in results:
        if result.get("error"):
            continue
        cname = result["class_name"]
        cls_info = cls_by_name.get(cname)
        if cls_info is None:
            continue

        for mask in result.get("masks", []):
            if not mask.any():
                continue
            polygon = mask_to_yolo_polygon(mask)
            bbox = mask_to_yolo_bbox(mask)
            if not polygon:
                continue
            state.annotations.append(
                Annotation(
                    class_db_id=cls_info.id,
                    class_name=cname,
                    class_color=cls_info.color,
                    yolo_class_id=yolo_map.get(cls_info.id, 0),
                    polygon=polygon,
                    bbox=bbox,
                    mask=mask,
                )
            )
            added += 1

    rendered = render_state_image(state)
    ann_txt = "\n".join(
        f"{i}. [{a.yolo_class_id}] {a.class_name}  ({len(a.polygon) // 2} pts)"
        for i, a in enumerate(state.annotations, 1)
    )
    return rendered, state, f"Auto-annotated: {added} mask(s) added.", gr.update(value=ann_txt)
