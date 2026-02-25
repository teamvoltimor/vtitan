"""
SAM 2.1 Interactive Annotator – V2
────────────────────────────────────
Batch-labelling server with SQLite image queue, multi-point SAM inference,
negative points, dual YOLO export (seg + bbox), per-class colour picker,
image navigation with annotation restoration, and a stats dashboard.

Usage:
    uv run python app.py
    # or inside Docker via docker-compose up
"""

import contextlib
import os
from pathlib import Path

import cv2
import numpy as np
import torch
import gradio as gr

import db

# ── Paths & env ───────────────────────────────────────────────────────────────

MODELS_DIR = Path(os.environ.get("MODELS_DIR", Path(__file__).parent / "models"))
LOCAL_CKPT = MODELS_DIR / "sam2.1_l.pt"
LABELS_DIR = db.LABELS_DIR

# ── Colour palette (20 visually distinct hex strings) ─────────────────────────

_PALETTE_HEX: list[str] = [
    "#dc322f",
    "#2aa12a",
    "#268bd2",
    "#b58900",
    "#d33682",
    "#2aa198",
    "#cb4b16",
    "#6c71c4",
    "#859900",
    "#008080",
    "#ffa500",
    "#00ff7f",
    "#ff1493",
    "#40e0d0",
    "#ffd700",
    "#8a2be2",
    "#ff7f50",
    "#00bfff",
    "#9acd32",
    "#ff6347",
]


def _hex_to_bgr(h: str) -> tuple[int, int, int]:
    if h.startswith("rgb"):
        # simple parser for rgb(r, g, b) or rgba(r, g, b, a)
        parts = h.split("(")[1].split(")")[0].split(",")
        r, g, b = int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))
        return (b, g, r)
    h = h.lstrip("#")
    try:
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return (b, g, r)
    except ValueError:
        # fallback for malformed hex
        return (0, 0, 255)


def _hex_to_rgb(h: str) -> tuple[int, int, int]:
    if h.startswith("rgb"):
        parts = h.split("(")[1].split(")")[0].split(",")
        return int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))
    h = h.lstrip("#")
    try:
        return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    except ValueError:
        # fallback for malformed hex
        return (255, 0, 0)


# ── Device + autocast ─────────────────────────────────────────────────────────

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def _autocast_ctx():
    if DEVICE == "cuda":
        return torch.autocast("cuda", dtype=torch.bfloat16)
    return contextlib.nullcontext()


def _empty_cache():
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


# ── SAM 2.1 initialisation ────────────────────────────────────────────────────

predictor = None
USE_NATIVE = False

try:
    from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore

    if LOCAL_CKPT.exists():
        from sam2.build_sam import build_sam2  # type: ignore
        import sam2 as _sam2_pkg

        _cfg_dir = Path(_sam2_pkg.__file__).parent / "configs" / "sam2.1"
        _cfg = str(_cfg_dir / "sam2.1_hiera_l.yaml")
        _model = build_sam2(_cfg, str(LOCAL_CKPT), device=DEVICE)
        predictor = SAM2ImagePredictor(_model)
        print(f"[SAM2] Loaded local checkpoint: {LOCAL_CKPT}")
    else:
        predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2.1-hiera-large")
        print("[SAM2] Loaded from HuggingFace: facebook/sam2.1-hiera-large")

    USE_NATIVE = True

except Exception as _sam2_err:
    print(f"[SAM2] Native sam2 unavailable ({_sam2_err}), trying Ultralytics.")
    try:
        from ultralytics import SAM as _UltSAM  # type: ignore

        predictor = _UltSAM("sam2.1_l.pt")
        USE_NATIVE = False
        print("[SAM2] Loaded via Ultralytics SAM API.")
    except Exception as _ult_err:
        print(f"[SAM2] FATAL: no SAM2 backend available. {_ult_err}")


# ── Pure geometry helpers ─────────────────────────────────────────────────────


def mask_to_yolo_polygon(
    mask: np.ndarray, epsilon_factor: float = 0.002
) -> list[float]:
    """Bool H×W → flat normalised YOLO polygon coords.  Returns [] on failure."""
    mask_u8 = mask.astype(np.uint8) * 255
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
    return pts.clip(0.0, 1.0).flatten().tolist()


def mask_to_yolo_bbox(mask: np.ndarray) -> list[float]:
    """Bool H×W → [xc, yc, w, h] normalised.  Returns [] if mask is empty."""
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return []
    H, W = mask.shape
    x1, x2 = xs.min(), xs.max()
    y1, y2 = ys.min(), ys.max()
    xc = (x1 + x2) / 2 / W
    yc = (y1 + y2) / 2 / H
    w = (x2 - x1) / W
    h = (y2 - y1) / H
    return [xc, yc, w, h]


# ── Render ────────────────────────────────────────────────────────────────────


def render_state_image(state: dict) -> np.ndarray:
    """
    Composite all layers onto the current image and return an RGB uint8 array.

    Layers (bottom → top):
      1. Accepted annotation fills  (class colour, alpha=0.45)
      2. Accepted annotation contour outlines (class colour, 2 px)
      3. Pending mask fill (class colour, alpha=0.30) + white dashed outline
      4. Point buffer circles: green solid (pos) / red solid + × (neg)
      5. Top-left legend
    """
    img = state.get("current_image")
    if img is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)

    result = img.astype(np.float32)

    # ── 1: Accepted annotation fills (bottom layer) ────────────────────────────
    for ann in state["annotations"]:
        color_rgb = _hex_to_rgb(ann["class_color"])
        mask = ann["mask"]
        colored = np.zeros_like(result)
        colored[mask] = color_rgb
        result = np.where(mask[:, :, None], 0.55 * result + 0.45 * colored, result)

    # ── 2: Pending mask fill (above accepted fills, below all contours) ────────
    pending_mask = state.get("pending_mask")
    pending_class = state.get("pending_class_db_id")
    if pending_mask is not None and pending_class is not None:
        cls_map = {c["id"]: c["color"] for c in state["classes"]}
        p_color_hex = cls_map.get(pending_class, "#ffffff")
        p_color_rgb = _hex_to_rgb(p_color_hex)
        colored = np.zeros_like(result)
        colored[pending_mask] = p_color_rgb
        result = np.where(pending_mask[:, :, None], 0.55 * result + 0.45 * colored, result)

    result_u8 = result.clip(0, 255).astype(np.uint8)

    # ── 3: Accepted annotation contour outlines ────────────────────────────────
    for ann in state["annotations"]:
        color_rgb = _hex_to_rgb(ann["class_color"])
        contours, _ = cv2.findContours(
            ann["mask"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(result_u8, contours, -1, color_rgb, thickness=2)

    # ── 4: Pending mask dashed white outline ───────────────────────────────────
    if pending_mask is not None and pending_class is not None:
        pmask_u8 = pending_mask.astype(np.uint8) * 255
        contours_p, _ = cv2.findContours(
            pmask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        dash_on = 10
        period  = 18
        for cnt in contours_p:
            pts = cnt.reshape(-1, 2)
            n   = len(pts)
            for j in range(0, n, period):
                for k in range(j, min(j + dash_on, n)):
                    p1 = (int(pts[k][0]),           int(pts[k][1]))
                    p2 = (int(pts[(k + 1) % n][0]), int(pts[(k + 1) % n][1]))
                    cv2.line(result_u8, p1, p2, (255, 255, 255), 2, cv2.LINE_AA)

    # ── 4: Point buffer ───────────────────────────────────────────────────────

    # Create a quick lookup for class colors
    class_colors_map = {c["id"]: c["color"] for c in state.get("classes", [])}

    # Fallback/default logic for compatibility with old points that lack "class_id"
    default_pending_color = (0, 255, 0)
    pending_class_id = state.get("pending_class_db_id")
    if pending_class_id is not None:
        p_hex = class_colors_map.get(pending_class_id, "#00ff00")
        default_pending_color = _hex_to_rgb(p_hex)

    for pt in state.get("point_buffer", []):
        px, py = int(pt["x"]), int(pt["y"])

        # Determine color for this specific point
        pt_color = default_pending_color
        if "class_id" in pt and pt["class_id"] in class_colors_map:
            pt_hex = class_colors_map[pt["class_id"]]
            pt_color = _hex_to_rgb(pt_hex)

        if pt["label"] == 1:
            cv2.circle(result_u8, (px, py), 6, pt_color, -1)
            cv2.circle(result_u8, (px, py), 6, (255, 255, 255), 1)
        else:
            cv2.circle(result_u8, (px, py), 6, (0, 0, 255), -1)
            cv2.circle(result_u8, (px, py), 6, (255, 255, 255), 1)
            sz = 4
            cv2.line(
                result_u8, (px - sz, py - sz), (px + sz, py + sz), (255, 255, 255), 2
            )
            cv2.line(
                result_u8, (px + sz, py - sz), (px - sz, py + sz), (255, 255, 255), 2
            )

    # ── 5: Legend ─────────────────────────────────────────────────────────────
    seen: dict[int, tuple[str, str]] = {}
    for ann in state["annotations"]:
        cid = ann["class_db_id"]
        if cid not in seen:
            seen[cid] = (ann["class_name"], ann["class_color"])

    y = 10
    for cid, (cname, chex) in sorted(seen.items()):
        color_rgb = _hex_to_rgb(chex)
        cv2.rectangle(result_u8, (8, y), (26, y + 16), color_rgb, -1)
        cv2.putText(
            result_u8,
            f"{cid}: {cname}",
            (30, y + 13),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 22

    return result_u8


# ── Annotation restoration ────────────────────────────────────────────────────


def _restore_annotations(
    image_id: int, img: np.ndarray, classes: list[dict]
) -> list[dict]:
    """
    If a .txt label file exists for this image and format_used='seg', parse it
    and reconstruct annotation dicts (including pixel masks).
    """
    record = db.get_by_id(image_id)
    if record is None or record["format_used"] != "seg":
        return []

    path = Path(record["path"])
    txt = LABELS_DIR / (path.stem + ".txt")
    if not txt.exists():
        return []

    H, W = img.shape[:2]
    yolo_map = db.classes_to_yolo_map()  # {db_id: yolo_index}
    rev_map = {v: k for k, v in yolo_map.items()}  # {yolo_index: db_id}
    cls_by_id = {c["id"]: c for c in classes}

    annotations = []
    for line in txt.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 7:  # class + at least 3 xy pairs
            continue
        try:
            yolo_cls_id = int(parts[0])
            coords = [float(v) for v in parts[1:]]
        except ValueError:
            continue

        db_id = rev_map.get(yolo_cls_id)
        if db_id is None or db_id not in cls_by_id:
            continue

        cls_info = cls_by_id[db_id]

        # Denormalise polygon → pixel coords
        xs_norm = coords[0::2]
        ys_norm = coords[1::2]
        pts_px = np.array(
            [[int(x * W), int(y * H)] for x, y in zip(xs_norm, ys_norm)],
            dtype=np.int32,
        )

        # Rasterise polygon → bool mask
        canvas = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(canvas, [pts_px], 255)
        mask = canvas.astype(bool)

        annotations.append(
            {
                "class_db_id": db_id,
                "class_name": cls_info["name"],
                "class_color": cls_info["color"],
                "yolo_class_id": yolo_cls_id,
                "polygon": coords,
                "bbox": mask_to_yolo_bbox(mask),
                "mask": mask,
            }
        )

    return annotations


# ── Stats HTML ────────────────────────────────────────────────────────────────


def stats_html() -> str:
    s = db.get_stats()
    pct = s["pct"]
    bar_fill = int(pct)

    return (
        f'<div style="font-family:monospace;background:#1e1e2e;color:#cdd6f4;'
        f'padding:8px 12px;border-radius:6px;font-size:13px;line-height:1.6">'
        f'Pending: <b style="color:#f38ba8">{s["pending"]}</b> &nbsp;'
        f'Done: <b style="color:#a6e3a1">{s["done"]}</b> &nbsp;'
        f'Skipped: <b style="color:#fab387">{s["skipped"]}</b> &nbsp;'
        f'Total: <b style="color:#89b4fa">{s["total"]}</b> &nbsp;|&nbsp; '
        f'<b style="color:#cba6f7">{pct}%</b>'
        f'<div style="background:#313244;border-radius:4px;height:6px;margin-top:4px">'
        f'<div style="background:#a6e3a1;width:{bar_fill}%;height:100%;border-radius:4px"></div>'
        f"</div></div>"
    )


# ── Annotation summary text ───────────────────────────────────────────────────


def _ann_summary(annotations: list[dict]) -> str:
    if not annotations:
        return "(none)"
    lines = []
    for i, ann in enumerate(annotations, 1):
        n_pts = len(ann["polygon"]) // 2
        lines.append(
            f"{i}. [{ann['yolo_class_id']}] {ann['class_name']}  ({n_pts} pts)"
        )
    return "\n".join(lines)


# ── Shared image-load helper ──────────────────────────────────────────────────


def _load_image(record, state: dict) -> tuple[np.ndarray, str, str]:
    """
    Load image from DB record into state.  Returns (rendered, label, stats).
    """
    path = Path(record["path"])
    bgr = cv2.imread(str(path))
    if bgr is None:
        return (
            render_state_image(state),
            f"ERROR: cannot read {path.name}",
            stats_html(),
        )

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    state["current_image_id"] = record["id"]
    state["current_image"] = rgb
    state["point_buffer"] = []
    state["pending_mask"] = None
    state["pending_class_db_id"] = None
    state["image_set"] = False

    # Pre-compute SAM embedding
    if USE_NATIVE and predictor is not None:
        try:
            with torch.inference_mode(), _autocast_ctx():
                predictor.set_image(rgb)
            state["image_set"] = True
        except Exception as e:
            print(f"[SAM2] set_image failed: {e}")
    _empty_cache()

    # Restore prior annotations
    state["annotations"] = _restore_annotations(record["id"], rgb, state["classes"])

    rendered = render_state_image(state)
    label = path.name
    return rendered, label, stats_html()


# ── Session state ─────────────────────────────────────────────────────────────


def initial_state() -> dict:
    db.init_db()
    classes = db.get_classes()
    return {
        "classes": classes,
        "current_image_id": None,
        "current_image": None,
        "image_set": False,
        "point_buffer": [],
        "pending_mask": None,
        "pending_masks": [],   # all 3 SAM candidate masks
        "pending_class_db_id": None,
        "annotations": [],
    }


# ── Event handlers ────────────────────────────────────────────────────────────


def _swatch_html(color: str, name: str = "") -> str:
    """Small HTML badge: coloured square + hex + optional name."""
    label = f"{name}  {color}" if name else color
    return (
        f'<div style="display:flex;align-items:center;gap:6px;'
        f'padding:3px 8px;border-radius:4px;background:#313244;'
        f'font-family:monospace;font-size:12px">'
        f'<div style="width:14px;height:14px;border-radius:3px;'
        f'background:{color};border:1px solid #45475a;flex-shrink:0"></div>'
        f'<span style="color:#cdd6f4">{label}</span>'
        f'</div>'
    )


def class_color_swatch(class_name: str | None, state: dict) -> str:
    if not class_name:
        return ""
    cls = next((c for c in state["classes"] if c["name"] == class_name), None)
    return _swatch_html(cls["color"], class_name) if cls else ""


def add_class(name: str, color: str, state: dict):
    name = name.strip()
    if not name:
        gr.Warning("Class name cannot be empty.")
        choices = [c["name"] for c in state["classes"]]
        return gr.update(choices=choices), state, color, gr.update(choices=choices), ""

    db.upsert_class(name, color)
    state["classes"] = db.get_classes()

    choices = [c["name"] for c in state["classes"]]
    next_color = _PALETTE_HEX[len(state["classes"]) % len(_PALETTE_HEX)]
    return (
        gr.update(choices=choices, value=name),
        state,
        next_color,
        gr.update(choices=choices),
        _swatch_html(color, name),
    )


def handle_click(
    evt: gr.SelectData, state: dict, point_type: str, active_class: str | None
):
    img = state.get("current_image")
    if img is None:
        return (
            render_state_image(state),
            state,
            "No image loaded.",
            gr.update(interactive=False),
        )
    if not state["classes"]:
        return (
            render_state_image(state),
            state,
            "Add at least one class first.",
            gr.update(interactive=False),
        )
    if active_class is None:
        return (
            render_state_image(state),
            state,
            "Select an active class.",
            gr.update(interactive=False),
        )

    cls_info = next((c for c in state["classes"] if c["name"] == active_class), None)
    if cls_info is None:
        return (
            render_state_image(state),
            state,
            f"Class '{active_class}' not found in DB.",
            gr.update(interactive=False),
        )

    x, y = int(evt.index[0]), int(evt.index[1])
    label = 1 if point_type == "Positive" else 0
    state["point_buffer"].append(
        {"x": x, "y": y, "label": label, "class_id": cls_info["id"]}
    )
    state["pending_class_db_id"] = cls_info["id"]

    n_pos = sum(1 for p in state["point_buffer"] if p["label"] == 1)
    n_neg = sum(1 for p in state["point_buffer"] if p["label"] == 0)
    status = (
        f"Point buffer: {n_pos} positive, {n_neg} negative — click Run SAM to segment."
    )

    rendered = render_state_image(state)
    return rendered, state, status, gr.update(interactive=False)


_MASK_LABELS = ["Precise (0)", "Object (1)", "Broad (2)"]

def run_sam(state: dict, active_class: str | None):
    _err = lambda msg: (
        render_state_image(state), state, msg,
        gr.update(interactive=False), gr.update(visible=False),
    )

    img = state.get("current_image")
    if img is None:
        return _err("No image loaded.")
    if not state["point_buffer"]:
        return _err("Add points first.")
    if predictor is None:
        return _err("SAM model not loaded.")

    pts    = state["point_buffer"]
    coords = np.array([[p["x"], p["y"]] for p in pts], dtype=np.float32)
    labels = np.array([p["label"] for p in pts],       dtype=np.int32)

    try:
        if USE_NATIVE:
            if not state["image_set"]:
                with torch.inference_mode(), _autocast_ctx():
                    predictor.set_image(img)
                state["image_set"] = True

            with torch.inference_mode(), _autocast_ctx():
                masks, scores, _ = predictor.predict(
                    point_coords=coords,
                    point_labels=labels,
                    multimask_output=True,
                )

            # SAM2 returns 3 masks (fine → coarse): [sub-part, object, context]
            scores_flat = scores.flatten()
            best_idx    = int(np.argmax(scores_flat))

            all_masks = [masks[i].astype(bool) for i in range(len(masks))]

        else:
            pos_pts = [[p["x"], p["y"]] for p in pts if p["label"] == 1]
            pos_lbl = [1] * len(pos_pts)
            results = predictor(img, points=[pos_pts], labels=[pos_lbl])
            if not results or results[0].masks is None:
                return _err("SAM returned no mask.")
            all_masks = [results[0].masks.data[0].cpu().numpy().astype(bool)]
            best_idx  = 0

    except torch.cuda.OutOfMemoryError:
        _empty_cache()
        return _err("CUDA OOM — try a smaller image.")
    except Exception as e:
        _empty_cache()
        return _err(f"Inference error: {e}")
    finally:
        _empty_cache()

    state["pending_masks"] = all_masks
    state["pending_mask"]  = all_masks[best_idx]

    best_label  = _MASK_LABELS[best_idx] if best_idx < len(_MASK_LABELS) else _MASK_LABELS[0]
    show_picker = len(all_masks) > 1

    rendered = render_state_image(state)
    scores_str = "  ".join(
        f"{_MASK_LABELS[i]}: {scores_flat[i]:.3f}" for i in range(len(all_masks))
    ) if USE_NATIVE else ""
    status = f"Mask ready ({best_label} selected by score). {scores_str}"

    return (
        rendered, state, status,
        gr.update(interactive=True),
        gr.update(visible=show_picker, value=best_label),
    )


def select_mask_level(level_str: str, state: dict):
    """Switch the displayed pending mask without re-running SAM."""
    level_map = {lbl: i for i, lbl in enumerate(_MASK_LABELS)}
    level     = level_map.get(level_str, 1)
    masks     = state.get("pending_masks", [])
    if not masks or level >= len(masks):
        return render_state_image(state), state
    state["pending_mask"] = masks[level]
    return render_state_image(state), state


def accept_mask(state: dict, active_class: str | None):
    pending = state.get("pending_mask")
    if pending is None:
        return (
            render_state_image(state),
            state,
            "No pending mask.",
            _ann_summary(state["annotations"]),
        )

    cls_id = state.get("pending_class_db_id")
    cls_info = next((c for c in state["classes"] if c["id"] == cls_id), None)
    if cls_info is None:
        return (
            render_state_image(state),
            state,
            "No class selected.",
            _ann_summary(state["annotations"]),
        )

    yolo_map = db.classes_to_yolo_map()
    yolo_cls_id = yolo_map.get(cls_id, 0)
    polygon = mask_to_yolo_polygon(pending)
    bbox = mask_to_yolo_bbox(pending)

    if not polygon:
        return (
            render_state_image(state),
            state,
            "Mask too small to polygonise.",
            _ann_summary(state["annotations"]),
        )

    state["annotations"].append(
        {
            "class_db_id": cls_id,
            "class_name": cls_info["name"],
            "class_color": cls_info["color"],
            "yolo_class_id": yolo_cls_id,
            "polygon": polygon,
            "bbox": bbox,
            "mask": pending,
        }
    )

    state["pending_mask"]          = None
    state["pending_masks"]         = []
    state["pending_class_db_id"]   = None
    state["point_buffer"]          = []

    rendered = render_state_image(state)
    n = len(state["annotations"])
    return (
        rendered, state,
        f"{n} annotation(s) accepted.",
        _ann_summary(state["annotations"]),
        gr.update(visible=False),
    )


def clear_points(state: dict):
    state["point_buffer"]        = []
    state["pending_mask"]        = None
    state["pending_masks"]       = []
    state["pending_class_db_id"] = None
    rendered = render_state_image(state)
    return rendered, state, "Points cleared.", gr.update(visible=False)


def undo_last(state: dict):
    if not state["annotations"]:
        return (
            render_state_image(state),
            state,
            "Nothing to undo.",
            _ann_summary(state["annotations"]),
        )
    state["annotations"].pop()
    rendered = render_state_image(state)
    n = len(state["annotations"])
    msg = f"{n} annotation(s) remaining." if n else "All annotations removed."
    return rendered, state, msg, _ann_summary(state["annotations"])


def _write_labels(state: dict, export_fmt: str) -> bool:
    """Write YOLO .txt and return True on success."""
    img_id = state.get("current_image_id")
    if img_id is None:
        return False
    record = db.get_by_id(img_id)
    if record is None:
        return False

    stem = Path(record["path"]).stem
    txt = LABELS_DIR / (stem + ".txt")

    lines = []
    for ann in state["annotations"]:
        cid = ann["yolo_class_id"]
        if export_fmt == "seg":
            if not ann["polygon"]:
                continue
            coords = " ".join(f"{v:.6f}" for v in ann["polygon"])
            lines.append(f"{cid} {coords}")
        else:  # bbox / detection
            if not ann["bbox"]:
                continue
            xc, yc, w, h = ann["bbox"]
            lines.append(f"{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    txt.write_text("\n".join(lines) + ("\n" if lines else ""))
    return True


def save_and_next(state: dict, export_fmt: str):
    img_id = state.get("current_image_id")
    fmt = "seg" if export_fmt == "Segmentation" else "det"

    if img_id is not None and _write_labels(state, fmt):
        db.mark_done(img_id, fmt)

    record = db.get_next(after_id=img_id)
    if record is None:
        state["current_image"] = None
        return (
            render_state_image(state),
            state,
            "All done — no more pending images!",
            stats_html(),
            "",
            _ann_summary([]),
        )

    rendered, label, stats = _load_image(record, state)
    return (
        rendered,
        state,
        f"Saved. Loaded: {label}",
        stats,
        label,
        _ann_summary(state["annotations"]),
    )


def skip_image(state: dict):
    img_id = state.get("current_image_id")
    if img_id is not None:
        db.mark_skipped(img_id)

    record = db.get_next(after_id=img_id)
    if record is None:
        state["current_image"] = None
        return (
            render_state_image(state),
            state,
            "No more pending images.",
            stats_html(),
            "",
            _ann_summary([]),
        )

    rendered, label, stats = _load_image(record, state)
    return (
        rendered,
        state,
        f"Skipped. Loaded: {label}",
        stats,
        label,
        _ann_summary(state["annotations"]),
    )


def go_prev(state: dict):
    img_id = state.get("current_image_id")
    if img_id is None:
        record = db.get_next()
    else:
        record = db.get_prev(img_id)

    if record is None:
        return (
            render_state_image(state),
            state,
            "No previous image.",
            stats_html(),
            "",
            _ann_summary(state["annotations"]),
        )

    rendered, label, stats = _load_image(record, state)
    return (
        rendered,
        state,
        f"Loaded: {label}",
        stats,
        label,
        _ann_summary(state["annotations"]),
    )


def go_next_pending(state: dict):
    img_id = state.get("current_image_id")
    record = db.get_next(after_id=img_id)
    if record is None:
        return (
            render_state_image(state),
            state,
            "No more pending images.",
            stats_html(),
            "",
            _ann_summary(state["annotations"]),
        )

    rendered, label, stats = _load_image(record, state)
    return (
        rendered,
        state,
        f"Loaded: {label}",
        stats,
        label,
        _ann_summary(state["annotations"]),
    )


def refresh_browse(state: dict):
    rows = db.get_all_images()
    data = [
        [r["id"], r["filename"], r["status"], r["format"], r["updated_at"]]
        for r in rows
    ]
    return data, stats_html()


def prefill_edit_color(class_name: str, state: dict):
    """Pre-fill the edit color picker with the class's current color."""
    cls = next((c for c in state["classes"] if c["name"] == class_name), None)
    if cls:
        return gr.update(value=cls["color"])
    return gr.update()


def update_class_color(class_name: str, new_color: str, state: dict):
    """Update an existing class's color in the DB and re-render the image."""
    if not class_name:
        return state, render_state_image(state), "Select a class to edit."
    db.upsert_class(class_name, new_color)
    state["classes"] = db.get_classes()
    for ann in state["annotations"]:
        if ann["class_name"] == class_name:
            ann["class_color"] = new_color
    rendered = render_state_image(state)
    return state, rendered, f"Color updated for '{class_name}'."


def scan_folder(state: dict):
    """Rescan data/pending/ for new images and refresh the browse table."""
    db.init_db()

    # Refresh browse data
    rows = db.get_all_images()
    data = [
        [r["id"], r["filename"], r["status"], r["format"], r["updated_at"]]
        for r in rows
    ]

    # Check if we need to auto-load an image (if none is currently loaded)
    img_id = state.get("current_image_id")
    current_img = state.get("current_image")

    # Default returns (no change to image state)
    rendered = gr.update()
    status_msg = gr.update()
    label_txt = gr.update()
    ann_txt = gr.update()

    if img_id is None and current_img is None:
        # Try to load next pending
        record = db.get_next()
        if record:
            rendered_img, label, stats = _load_image(record, state)
            rendered = rendered_img
            status_msg = f"Loaded: {label}"
            label_txt = label
            ann_txt = _ann_summary(state["annotations"])

    return data, stats_html(), rendered, state, status_msg, label_txt, ann_txt


def import_folder(folder_path: str, state: dict):
    """Register all images from an arbitrary folder path and refresh browse."""
    folder_path = folder_path.strip()
    if not folder_path:
        return gr.update(), stats_html(), "Enter a folder path first."
    inserted, _ = db.add_images_from_folder(folder_path)
    rows = db.get_all_images()
    data = [[r["id"], r["filename"], r["status"], r["format"], r["updated_at"]] for r in rows]
    msg = f"Added {inserted} new image(s) from {folder_path}" if inserted else "No new images found (all already registered)."
    return data, stats_html(), msg


def load_first_image(state: dict):
    """Called on app startup to auto-load the first pending image."""
    db.init_db()
    state["classes"] = db.get_classes()

    choices = [c["name"] for c in state["classes"]]
    dd_update = gr.update(choices=choices, value=choices[0] if choices else None)

    record = db.get_next()
    if record is None:
        return (
            render_state_image(state),
            state,
            "No pending images. Drop files into data/pending/ and restart.",
            stats_html(),
            "",
            _ann_summary([]),
            dd_update,
            gr.update(choices=choices),
        )

    rendered, label, stats = _load_image(record, state)
    return (
        rendered,
        state,
        f"Loaded: {label}",
        stats,
        label,
        _ann_summary(state["annotations"]),
        dd_update,
        gr.update(choices=choices),
    )


# ── Gradio UI ─────────────────────────────────────────────────────────────────

with gr.Blocks(title="SAM2 Annotator V2") as demo:
    state = gr.State(initial_state())

    stats_bar = gr.HTML(value=stats_html())

    with gr.Tabs():
        # ── Annotate tab ──────────────────────────────────────────────────────
        with gr.Tab("Annotate"):
            with gr.Row():
                # ── Left: image + navigation ──────────────────────────────────
                with gr.Column(scale=3):
                    img_label = gr.Textbox(
                        label="Current image", value="", interactive=False,
                    )
                    display_img = gr.Image(
                        label="Click to add points",
                        type="numpy", interactive=False, height=640,
                    )
                    with gr.Row():
                        prev_btn      = gr.Button("← Prev",  size="sm")
                        next_btn      = gr.Button("→ Next",  size="sm")
                        skip_btn      = gr.Button("Skip",    size="sm")
                        save_next_btn = gr.Button("✓ Save", variant="primary", size="sm")
                    with gr.Row():
                        export_fmt = gr.Radio(
                            ["Segmentation", "Detection"], value="Segmentation",
                            label="Export format", interactive=True,
                        )
                        status_box = gr.Textbox(
                            label="Status", interactive=False, scale=2,
                        )

                # ── Right: controls ───────────────────────────────────────────
                with gr.Column(scale=1, min_width=260):

                    with gr.Accordion("Classes", open=True):
                        with gr.Row():
                            class_input = gr.Textbox(
                                label="Name", placeholder="e.g. red_prism",
                                scale=3, container=False,
                            )
                            color_picker = gr.ColorPicker(
                                value=_PALETTE_HEX[0], label="Color",
                                scale=1, container=False,
                            )
                        add_btn = gr.Button("Add class", variant="secondary", size="sm")
                        class_dropdown = gr.Dropdown(
                            label="Active class", choices=[], value=None,
                            interactive=True,
                        )
                        # Colour swatch for the currently selected class
                        class_swatch = gr.HTML(value="")

                    with gr.Accordion("Edit Class Color", open=False):
                        edit_class_dd = gr.Dropdown(
                            label="Class", choices=[], value=None, interactive=True,
                        )
                        edit_color_picker = gr.ColorPicker(
                            value=_PALETTE_HEX[0], label="New color",
                        )
                        update_color_btn = gr.Button(
                            "Update Color", variant="secondary", size="sm",
                        )

                    with gr.Accordion("Points & SAM", open=True):
                        point_type = gr.Radio(
                            ["Positive", "Negative"], value="Positive",
                            label="Point type", interactive=True,
                        )
                        with gr.Row():
                            run_sam_btn = gr.Button("Run SAM", variant="primary", size="sm")
                            accept_btn  = gr.Button(
                                "Accept Mask", variant="secondary",
                                size="sm", interactive=False,
                            )
                        # Shown only after Run SAM — lets you pick among the 3
                        # granularity levels SAM2 always returns:
                        #   Precise (0) = sub-part   Object (1) = whole object   Broad (2) = context
                        mask_level_radio = gr.Radio(
                            _MASK_LABELS, value="Object (1)",
                            label="Mask granularity",
                            interactive=True, visible=False,
                        )
                        with gr.Row():
                            clear_pts_btn = gr.Button("Clear Points", size="sm")
                            undo_btn      = gr.Button("Undo Last",    size="sm")

                    with gr.Accordion("Annotations", open=True):
                        ann_box = gr.Textbox(
                            value="(none)", lines=5,
                            interactive=False, show_label=False,
                        )

        # ── Browse tab ────────────────────────────────────────────────────────
        with gr.Tab("Browse"):
            with gr.Row():
                refresh_btn     = gr.Button("Refresh")
                scan_folder_btn = gr.Button("Scan Pending Folder", variant="secondary")
            with gr.Row():
                folder_input = gr.Textbox(
                    label="Import folder path",
                    placeholder="/path/to/my/images",
                    scale=4,
                )
                import_folder_btn = gr.Button("Import Folder", variant="primary", scale=1)
            browse_status = gr.Textbox(label="", interactive=False, show_label=False)
            browse_df = gr.Dataframe(
                headers=["id", "filename", "status", "format", "updated_at"],
                interactive=False,
            )

    # ── Event wiring ──────────────────────────────────────────────────────────

    _add_class_outs = [class_dropdown, state, color_picker, edit_class_dd, class_swatch]
    add_btn.click(add_class, [class_input, color_picker, state], _add_class_outs)
    class_input.submit(add_class, [class_input, color_picker, state], _add_class_outs)

    class_dropdown.change(
        class_color_swatch,
        inputs=[class_dropdown, state],
        outputs=[class_swatch],
    )

    edit_class_dd.change(
        prefill_edit_color,
        inputs=[edit_class_dd, state],
        outputs=[edit_color_picker],
    )
    update_color_btn.click(
        update_class_color,
        inputs=[edit_class_dd, edit_color_picker, state],
        outputs=[state, display_img, status_box],
    )

    display_img.select(
        handle_click,
        inputs=[state, point_type, class_dropdown],
        outputs=[display_img, state, status_box, accept_btn],
    )
    run_sam_btn.click(
        run_sam,
        inputs=[state, class_dropdown],
        outputs=[display_img, state, status_box, accept_btn, mask_level_radio],
    )
    mask_level_radio.change(
        select_mask_level,
        inputs=[mask_level_radio, state],
        outputs=[display_img, state],
    )
    accept_btn.click(
        accept_mask,
        inputs=[state, class_dropdown],
        outputs=[display_img, state, status_box, ann_box, mask_level_radio],
    )
    clear_pts_btn.click(
        clear_points,
        inputs=[state],
        outputs=[display_img, state, status_box, mask_level_radio],
    )
    undo_btn.click(
        undo_last,
        inputs=[state],
        outputs=[display_img, state, status_box, ann_box],
    )

    _nav_outputs = [display_img, state, status_box, stats_bar, img_label, ann_box]
    save_next_btn.click(save_and_next, [state, export_fmt], _nav_outputs)
    skip_btn.click(skip_image,      [state], _nav_outputs)
    prev_btn.click(go_prev,         [state], _nav_outputs)
    next_btn.click(go_next_pending, [state], _nav_outputs)

    refresh_btn.click(refresh_browse, [state], [browse_df, stats_bar])
    scan_folder_btn.click(
        scan_folder,
        inputs=[state],
        outputs=[browse_df, stats_bar, display_img, state, status_box, img_label, ann_box],
    )
    import_folder_btn.click(
        import_folder,
        inputs=[folder_input, state],
        outputs=[browse_df, stats_bar, browse_status],
    )

    # Auto-load first pending image on startup
    demo.load(
        load_first_image,
        inputs=[state],
        outputs=[
            display_img, state, status_box, stats_bar,
            img_label, ann_box, class_dropdown, edit_class_dd,
        ],
    )


if __name__ == "__main__":
    demo.launch(
        server_name="0.0.0.0",
        server_port=7860,
        share=False,
        theme=gr.themes.Base(),
    )
