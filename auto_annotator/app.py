"""
SAM 2.1 Interactive Annotator
──────────────────────────────
Click objects on an uploaded image to segment and label them.
Exports YOLO segmentation .txt format (class_id x1 y1 x2 y2 ...).

Hardware: optimised for RTX 4050 (6 GB VRAM) using sam2.1_s.pt.

Usage:
    pip install sam2 gradio opencv-python torch torchvision
    python app.py
"""

import contextlib
import os
from pathlib import Path

import cv2
import numpy as np
import torch
import gradio as gr

# ── Colour palette (20 visually distinct RGB tuples) ─────────────────────────
CLASS_COLORS: list[tuple[int, int, int]] = [
    (220,  50,  47),   # red
    ( 42, 161,  53),   # green
    ( 38, 139, 210),   # blue
    (181, 137,   0),   # yellow
    (211,  54, 130),   # magenta
    ( 42, 161, 152),   # cyan
    (203,  75,  22),   # orange
    (108, 113, 196),   # violet
    (133, 153,   0),   # olive
    (  0, 128, 128),   # teal
    (255, 165,   0),   # amber
    (  0, 255, 127),   # spring green
    (255,  20, 147),   # deep pink
    ( 64, 224, 208),   # turquoise
    (255, 215,   0),   # gold
    (138,  43, 226),   # blue-violet
    (255, 127,  80),   # coral
    (  0, 191, 255),   # deep sky blue
    (154, 205,  50),   # yellow-green
    (255,  99,  71),   # tomato
]

# ── Device + autocast helper ──────────────────────────────────────────────────
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
LOCAL_CKPT = Path(__file__).parent / "sam2.1_s.pt"


def _autocast_ctx():
    """Return an autocast context: bfloat16 on CUDA, no-op on CPU."""
    if DEVICE == "cuda":
        return torch.autocast("cuda", dtype=torch.bfloat16)
    return contextlib.nullcontext()


def _empty_cache():
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


# ── SAM 2.1 initialisation (native Meta API preferred) ───────────────────────
predictor = None
USE_NATIVE_SAM2 = False

try:
    from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore

    if LOCAL_CKPT.exists():
        # Local checkpoint: build_sam2 needs the config YAML resolved from the
        # sam2 package's bundled configs directory.
        from sam2.build_sam import build_sam2  # type: ignore
        import sam2 as _sam2_pkg

        _cfg_dir = Path(_sam2_pkg.__file__).parent / "configs" / "sam2.1"
        _cfg = str(_cfg_dir / "sam2.1_hiera_s.yaml")
        _model = build_sam2(_cfg, str(LOCAL_CKPT), device=DEVICE)
        predictor = SAM2ImagePredictor(_model)
        print(f"[SAM2] Loaded local checkpoint: {LOCAL_CKPT}")
    else:
        # Download from HuggingFace on first run
        predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2.1-hiera-small")
        print("[SAM2] Loaded from HuggingFace: facebook/sam2.1-hiera-small")

    USE_NATIVE_SAM2 = True

except Exception as _sam2_err:
    print(f"[SAM2] Native sam2 package unavailable ({_sam2_err}), falling back to Ultralytics.")
    try:
        from ultralytics import SAM as _UltSAM  # type: ignore

        _ult_model = _UltSAM("sam2.1_s.pt")
        predictor = _ult_model
        USE_NATIVE_SAM2 = False
        print("[SAM2] Loaded via Ultralytics SAM API.")
    except Exception as _ult_err:
        print(f"[SAM2] FATAL: could not load any SAM2 backend. {_ult_err}")


# ── Pure helpers ──────────────────────────────────────────────────────────────

def mask_to_yolo_polygon(mask: np.ndarray, epsilon_factor: float = 0.002) -> list[float]:
    """
    Convert a boolean H×W mask to a flat list of normalized YOLO polygon coords
    [x1, y1, x2, y2, ..., xn, yn].  Returns [] on failure.
    """
    mask_u8 = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 10:
        return []
    epsilon = epsilon_factor * cv2.arcLength(largest, closed=True)
    approx = cv2.approxPolyDP(largest, epsilon, closed=True)
    pts = approx.reshape(-1, 2).astype(np.float32)
    if len(pts) < 3:
        return []
    H, W = mask.shape
    pts[:, 0] /= W
    pts[:, 1] /= H
    pts = pts.clip(0.0, 1.0)
    return pts.flatten().tolist()


def render_overlay(
    original: np.ndarray,
    annotations: list[dict],
    alpha: float = 0.45,
) -> np.ndarray:
    """
    Composite all annotation masks (colored, semi-transparent) + contour outlines
    + a class legend over the original RGB image.
    """
    result = original.astype(np.float32)

    # Blended fill per annotation
    for ann in annotations:
        color = CLASS_COLORS[ann["class_id"] % len(CLASS_COLORS)]
        mask = ann["mask"]  # bool H×W
        colored = np.zeros_like(result)
        colored[mask] = color
        result = np.where(mask[:, :, None], (1.0 - alpha) * result + alpha * colored, result)

    result = result.clip(0, 255).astype(np.uint8)

    # Crisp outlines
    for ann in annotations:
        color = CLASS_COLORS[ann["class_id"] % len(CLASS_COLORS)]
        contours, _ = cv2.findContours(
            ann["mask"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(result, contours, -1, color, thickness=2)

    # Legend (top-left)
    seen: dict[int, str] = {}
    for ann in annotations:
        cid = ann["class_id"]
        if cid not in seen:
            seen[cid] = ann["class_name"]

    y = 10
    for cid, name in sorted(seen.items()):
        color = CLASS_COLORS[cid % len(CLASS_COLORS)]
        cv2.rectangle(result, (8, y), (26, y + 16), color, -1)
        cv2.putText(
            result, f"{cid}: {name}", (30, y + 13),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )
        y += 22

    return result


# ── Session state schema ──────────────────────────────────────────────────────

def initial_state() -> dict:
    return {
        "classes": [],        # list[str]  — ordered class names
        "annotations": [],    # list[dict] — {class_id, class_name, polygon, mask}
        "original_image": None,  # np.ndarray H×W×3 uint8 RGB
        "image_set": False,   # whether predictor.set_image() has been called
    }


# ── Gradio event handlers ─────────────────────────────────────────────────────

def add_class(class_name: str, state: dict):
    """Append a new class to the session list and update the dropdown."""
    name = class_name.strip()
    if not name:
        gr.Warning("Class name cannot be empty.")
        return gr.update(), state
    if name in state["classes"]:
        gr.Warning(f"Class '{name}' already exists.")
        return gr.update(), state
    state["classes"].append(name)
    choices = list(state["classes"])
    return gr.update(choices=choices, value=choices[-1]), state


def load_image(image: np.ndarray | None, state: dict):
    """
    Called when the user uploads a new image.
    Resets annotations and pre-computes the SAM image embedding.
    """
    if image is None:
        return None, state

    state["original_image"] = image
    state["annotations"] = []
    state["image_set"] = False

    if USE_NATIVE_SAM2 and predictor is not None:
        try:
            with torch.inference_mode(), _autocast_ctx():
                predictor.set_image(image)
            state["image_set"] = True
        except Exception as e:
            gr.Warning(f"SAM image embedding failed: {e}")

    _empty_cache()
    return image, state


def handle_click(evt: gr.SelectData, state: dict, active_class: str | None):
    """
    Fired when the user clicks the display image.
    Runs SAM inference at the clicked point and records the annotation.
    """
    img = state["original_image"]

    # Guard clauses
    if img is None:
        gr.Warning("Upload an image first.")
        return None, state, "No image loaded."
    if not state["classes"]:
        gr.Warning("Add at least one class first.")
        return img, state, "No classes defined."
    if active_class is None or active_class not in state["classes"]:
        gr.Warning("Select a valid class from the dropdown.")
        return img, state, "No class selected."
    if predictor is None:
        gr.Warning("SAM model not loaded.")
        return img, state, "Model unavailable."

    class_id = state["classes"].index(active_class)
    x, y = int(evt.index[0]), int(evt.index[1])

    try:
        if USE_NATIVE_SAM2:
            if not state["image_set"]:
                # Lazy fallback: set_image was not pre-computed (e.g. model loaded after upload)
                with torch.inference_mode(), _autocast_ctx():
                    predictor.set_image(img)
                state["image_set"] = True

            point_coords = np.array([[x, y]], dtype=np.float32)
            point_labels = np.array([1], dtype=np.int32)

            with torch.inference_mode(), _autocast_ctx():
                masks, scores, _ = predictor.predict(
                    point_coords=point_coords,
                    point_labels=point_labels,
                    multimask_output=True,
                )

            best_mask: np.ndarray = masks[int(np.argmax(scores))].astype(bool)

        else:
            # Ultralytics SAM fallback (re-encodes image every call — slower)
            results = predictor(img, points=[[x, y]], labels=[1])
            if not results or results[0].masks is None:
                gr.Warning("SAM returned no mask. Try a different point.")
                overlay = render_overlay(img, state["annotations"])
                return overlay, state, "No mask returned."
            best_mask = results[0].masks.data[0].cpu().numpy().astype(bool)

    except torch.cuda.OutOfMemoryError:
        _empty_cache()
        gr.Warning("CUDA out of memory — try a smaller image or restart.")
        return img, state, "CUDA OOM."
    except Exception as e:
        gr.Warning(f"Inference error: {e}")
        return img, state, f"Error: {e}"
    finally:
        _empty_cache()

    polygon = mask_to_yolo_polygon(best_mask)
    if not polygon:
        gr.Warning("SAM returned an empty or tiny mask. Try clicking elsewhere.")
        overlay = render_overlay(img, state["annotations"])
        return overlay, state, "Empty mask."

    state["annotations"].append({
        "class_id": class_id,
        "class_name": active_class,
        "polygon": polygon,
        "mask": best_mask,
    })

    overlay = render_overlay(img, state["annotations"])
    n = len(state["annotations"])
    return overlay, state, f"{n} annotation(s)  |  last: {active_class} @ ({x}, {y})"


def export_annotations(state: dict, filename: str):
    """Write a YOLO segmentation .txt and return the path for download."""
    if not state["annotations"]:
        gr.Warning("No annotations to export.")
        return None, ""

    filename = filename.strip() or "annotations.txt"
    if not filename.endswith(".txt"):
        filename += ".txt"

    lines = []
    for ann in state["annotations"]:
        if not ann["polygon"]:
            continue
        coords = " ".join(f"{v:.6f}" for v in ann["polygon"])
        lines.append(f"{ann['class_id']} {coords}")

    output_path = Path(filename)
    output_path.write_text("\n".join(lines) + "\n")

    preview = "\n".join(lines[:10])
    if len(lines) > 10:
        preview += f"\n… ({len(lines) - 10} more lines)"

    return str(output_path), preview


def clear_annotations(state: dict):
    state["annotations"] = []
    img = state["original_image"]
    return (img, state, "All annotations cleared.")


def undo_last(state: dict):
    if not state["annotations"]:
        gr.Warning("Nothing to undo.")
        return state["original_image"], state, "Nothing to undo."
    state["annotations"].pop()
    img = state["original_image"]
    if not state["annotations"]:
        return img, state, "All annotations removed."
    overlay = render_overlay(img, state["annotations"])
    n = len(state["annotations"])
    return overlay, state, f"{n} annotation(s) remaining."


# ── Gradio UI ─────────────────────────────────────────────────────────────────

with gr.Blocks(title="SAM2 Annotator", theme=gr.themes.Base()) as demo:
    state = gr.State(initial_state())

    gr.Markdown(
        "# SAM 2.1 Interactive Annotator\n"
        "Define your classes → upload an image → select a class → click an object to segment it."
    )

    with gr.Row():
        # ── Left control panel ────────────────────────────────────────────────
        with gr.Column(scale=1, min_width=280):
            gr.Markdown("### 1 · Define classes")
            with gr.Row():
                class_input = gr.Textbox(
                    label="Class name",
                    placeholder="e.g. red_prism",
                    scale=3,
                    container=False,
                )
                add_btn = gr.Button("Add", scale=1, variant="secondary")
            class_dropdown = gr.Dropdown(
                label="Active class",
                choices=[],
                interactive=True,
            )

            gr.Markdown("### 2 · Upload image")
            upload_img = gr.Image(
                label="Upload",
                type="numpy",
                sources=["upload"],
            )

            gr.Markdown("### 3 · Export")
            filename_input = gr.Textbox(label="Output filename", value="annotations.txt")
            export_btn = gr.Button("Export YOLO .txt", variant="primary")
            download_file = gr.File(label="Download .txt")
            export_preview = gr.Textbox(
                label="File preview",
                lines=6,
                interactive=False,
                max_lines=10,
            )

            with gr.Row():
                undo_btn = gr.Button("↩ Undo last")
                clear_btn = gr.Button("🗑 Clear all", variant="stop")

        # ── Right display panel ───────────────────────────────────────────────
        with gr.Column(scale=2):
            gr.Markdown("### Annotated image  *(click to segment)*")
            display_img = gr.Image(
                label="Click to annotate",
                type="numpy",
                interactive=False,
                height=640,
            )
            status_box = gr.Textbox(label="Status", interactive=False)

    # ── Class info legend below ───────────────────────────────────────────────
    with gr.Row():
        gr.Markdown(
            "_Tip: press **Enter** in the class name box to quickly add a class. "
            "Each class gets a unique colour automatically._"
        )

    # ── Event wiring ──────────────────────────────────────────────────────────

    # Add class via button or Enter key
    add_btn.click(add_class, [class_input, state], [class_dropdown, state])
    class_input.submit(add_class, [class_input, state], [class_dropdown, state])

    # Image upload → pre-compute SAM embedding
    upload_img.change(load_image, [upload_img, state], [display_img, state])

    # Click on display image → SAM inference → overlay update
    display_img.select(handle_click, [state, class_dropdown], [display_img, state, status_box])

    # Export
    export_btn.click(export_annotations, [state, filename_input], [download_file, export_preview])

    # Undo / clear
    undo_btn.click(undo_last, [state], [display_img, state, status_box])
    clear_btn.click(clear_annotations, [state], [display_img, state, status_box])


if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", share=False)
