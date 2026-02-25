"""src.handlers.navigation – Image navigation event handlers."""

from __future__ import annotations

from pathlib import Path

import cv2

from src import db
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.html import stats_html
from src.models import Annotation, AppState, ClassInfo
from src.render import render_state_image
from src.sam_client import ModelServerClient


def _ann_summary(annotations: list[Annotation]) -> str:
    if not annotations:
        return "(none)"
    lines = [
        f"{i}. [{ann.yolo_class_id}] {ann.class_name}  ({len(ann.polygon) // 2} pts)"
        for i, ann in enumerate(annotations, 1)
    ]
    return "\n".join(lines)


def _restore_annotations(
    image_id: int, img, classes: list[ClassInfo], labels_dir: Path
) -> list[Annotation]:
    """Restore annotations from YOLO .txt if the image was previously saved as 'seg'."""
    record = db.get_by_id(image_id)
    if record is None or record["format_used"] != "seg":
        return []

    path = Path(record["path"])
    txt = labels_dir / (path.stem + ".txt")
    if not txt.exists():
        return []

    H, W = img.shape[:2]
    yolo_map = db.classes_to_yolo_map()
    rev_map = {v: k for k, v in yolo_map.items()}
    cls_by_id = {c.id: c for c in classes}

    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415

    annotations = []
    for line in txt.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 7:
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
        xs_norm = coords[0::2]
        ys_norm = coords[1::2]
        pts_px = np.array(
            [[int(x * W), int(y * H)] for x, y in zip(xs_norm, ys_norm)],
            dtype=np.int32,
        )

        canvas = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(canvas, [pts_px], 255)
        mask = canvas.astype(bool)

        annotations.append(
            Annotation(
                class_db_id=db_id,
                class_name=cls_info.name,
                class_color=cls_info.color,
                yolo_class_id=yolo_cls_id,
                polygon=coords,
                bbox=mask_to_yolo_bbox(mask),
                mask=mask,
            )
        )

    return annotations


def _load_image(record, state: AppState, client: ModelServerClient | None, labels_dir: Path):
    """Load image from DB record into state.  Returns (rendered, label, stats_html)."""
    path = Path(record["path"])
    bgr = cv2.imread(str(path))
    if bgr is None:
        return render_state_image(state), f"ERROR: cannot read {path.name}", stats_html()

    import numpy as np  # noqa: PLC0415

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    state.current_image_id = record["id"]
    state.current_image = rgb
    state.point_buffer = []
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.pending_class_db_id = None
    state.image_set = False

    if client is not None:
        try:
            client.set_image(rgb)
            state.image_set = True
        except Exception:  # noqa: BLE001
            pass

    state.annotations = _restore_annotations(record["id"], rgb, state.classes, labels_dir)

    rendered = render_state_image(state)
    return rendered, path.name, stats_html()


def _write_labels(state: AppState, export_fmt: str, labels_dir: Path) -> bool:
    img_id = state.current_image_id
    if img_id is None:
        return False
    record = db.get_by_id(img_id)
    if record is None:
        return False

    stem = Path(record["path"]).stem
    txt = labels_dir / (stem + ".txt")

    lines = []
    for ann in state.annotations:
        cid = ann.yolo_class_id
        if export_fmt == "seg":
            if not ann.polygon:
                continue
            coords = " ".join(f"{v:.6f}" for v in ann.polygon)
            lines.append(f"{cid} {coords}")
        else:
            if not ann.bbox:
                continue
            xc, yc, w, h = ann.bbox
            lines.append(f"{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    txt.write_text("\n".join(lines) + ("\n" if lines else ""))
    return True


def save_and_next(state: AppState, export_fmt: str, client, labels_dir: Path):
    img_id = state.current_image_id
    fmt = "seg" if export_fmt == "Segmentation" else "det"

    if img_id is not None and _write_labels(state, fmt, labels_dir):
        db.mark_done(img_id, fmt)

    record = db.get_next(after_id=img_id)
    if record is None:
        state.current_image = None
        return (
            render_state_image(state),
            state,
            "All done — no more pending images!",
            stats_html(),
            "",
            _ann_summary([]),
        )

    rendered, label, stats = _load_image(record, state, client, labels_dir)
    return rendered, state, f"Saved. Loaded: {label}", stats, label, _ann_summary(state.annotations)


def skip_image(state: AppState, client, labels_dir: Path):
    img_id = state.current_image_id
    if img_id is not None:
        db.mark_skipped(img_id)

    record = db.get_next(after_id=img_id)
    if record is None:
        state.current_image = None
        return (
            render_state_image(state),
            state,
            "No more pending images.",
            stats_html(),
            "",
            _ann_summary([]),
        )

    rendered, label, stats = _load_image(record, state, client, labels_dir)
    return rendered, state, f"Skipped. Loaded: {label}", stats, label, _ann_summary(state.annotations)


def go_prev(state: AppState, client, labels_dir: Path):
    img_id = state.current_image_id
    record = db.get_next() if img_id is None else db.get_prev(img_id)

    if record is None:
        return (
            render_state_image(state),
            state,
            "No previous image.",
            stats_html(),
            "",
            _ann_summary(state.annotations),
        )

    rendered, label, stats = _load_image(record, state, client, labels_dir)
    return rendered, state, f"Loaded: {label}", stats, label, _ann_summary(state.annotations)


def go_next_pending(state: AppState, client, labels_dir: Path):
    img_id = state.current_image_id
    record = db.get_next(after_id=img_id)

    if record is None:
        return (
            render_state_image(state),
            state,
            "No more pending images.",
            stats_html(),
            "",
            _ann_summary(state.annotations),
        )

    rendered, label, stats = _load_image(record, state, client, labels_dir)
    return rendered, state, f"Loaded: {label}", stats, label, _ann_summary(state.annotations)
