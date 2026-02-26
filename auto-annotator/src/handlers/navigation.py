"""src.handlers.navigation – Image navigation and label-writing event handlers."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src import db as _db
from src.geometry import mask_to_yolo_bbox
from src.handlers.responses import NavigationResponse
from src.handlers.utils import format_annotations_summary
from src.html import stats_html
from src.models import Annotation, AppContext, AppState, ClassInfo, ImageRecord
from src.render import render_state_image

# Minimum parts on a YOLO segmentation line: 1 class-id + 3 x/y pairs = 7 tokens.
# Derived from GEOMETRY_MINIMUM_POLYGON_POINTS * 2 + 1.
_YOLO_SEG_MIN_TOKENS: int = 7


def _restore_annotations(
    image_id: int, image: np.ndarray, classes: list[ClassInfo], labels_dir: Path,
) -> list[Annotation]:
    """Restore annotations from a YOLO .txt file when the image was saved as 'seg'."""
    record = _db.get_by_id(image_id)
    if record is None or record.format_used != "seg":
        return []

    label_file = labels_dir / (Path(record.path).stem + ".txt")
    if not label_file.exists():
        return []

    # Build class maps for translating YOLO indices ↔ DB class ids.
    image_h, image_w = image.shape[:2]
    yolo_map = _db.classes_to_yolo_map()
    reverse_yolo_map = {v: k for k, v in yolo_map.items()}
    classes_by_id = {c.id: c for c in classes}

    # Parse each line into an Annotation; skip malformed or unrecognised entries.
    annotations: list[Annotation] = []
    for line in label_file.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < _YOLO_SEG_MIN_TOKENS:
            continue
        annotation = _parse_yolo_seg_line(
            parts, reverse_yolo_map, classes_by_id, image_h, image_w,
        )
        if annotation is not None:
            annotations.append(annotation)

    return annotations


def _parse_yolo_seg_line(
    parts: list[str],
    reverse_yolo_map: dict[int, int],
    classes_by_id: dict[int, ClassInfo],
    image_h: int,
    image_w: int,
) -> Annotation | None:
    """Parse one YOLO segmentation line into an Annotation, or return None on error.

    Args:
        parts:            Whitespace-split tokens from a single .txt line.
        reverse_yolo_map: Mapping from YOLO class index to DB class id.
        classes_by_id:    Mapping from DB class id to ClassInfo.
        image_h:          Image height in pixels.
        image_w:          Image width in pixels.

    Returns:
        A fully populated :class:`Annotation`, or ``None`` when the line is
        malformed or the class is not recognised.
    """
    # Parse class index and flat normalised coordinate list.
    try:
        yolo_class_id = int(parts[0])
        coords = [float(v) for v in parts[1:]]
    except ValueError:
        return None

    # Map YOLO class index → DB class id → ClassInfo.
    db_class_id = reverse_yolo_map.get(yolo_class_id)
    if db_class_id is None or db_class_id not in classes_by_id:
        return None

    class_info = classes_by_id[db_class_id]

    # Denormalise coordinates from [0, 1] to pixel space and rasterise the polygon.
    xs_norm = coords[0::2]
    ys_norm = coords[1::2]
    polygon_pixels = np.array(
        [[int(x * image_w), int(y * image_h)] for x, y in zip(xs_norm, ys_norm, strict=True)],
        dtype=np.int32,
    )

    canvas = np.zeros((image_h, image_w), dtype=np.uint8)
    cv2.fillPoly(canvas, [polygon_pixels], 255)
    mask = canvas.astype(bool)

    return Annotation(
        class_db_id=db_class_id,
        class_name=class_info.name,
        class_color=class_info.color,
        yolo_class_id=yolo_class_id,
        polygon=coords,
        bbox=mask_to_yolo_bbox(mask),
        mask=mask,
    )


def _load_image(
    record: ImageRecord,
    state: AppState,
    app_ctx: AppContext,
    labels_dir: Path,
) -> tuple[np.ndarray, str, str]:
    """Load image from an ImageRecord into state.  Returns (rendered, label, stats_html)."""
    # Read file and convert from OpenCV BGR to RGB.
    image_path = Path(record.path)
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        return render_state_image(state), f"ERROR: cannot read {image_path.name}", stats_html()

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # Reset all per-image state fields before populating.
    state.current_image_id = record.id
    state.current_image = rgb
    state.point_buffer = []
    state.pending_mask = None
    state.pending_masks = []
    state.pending_logits = None
    state.pending_mask_idx = 0
    state.pending_class_db_id = None
    state.image_set = False

    # Pre-load the image on the model server to avoid the round-trip on first click.
    if app_ctx.client is not None:
        try:
            app_ctx.client.set_image(rgb)
            state.image_set = True
        except Exception:  # noqa: BLE001, S110
            pass

    # Restore previously saved annotations from the YOLO label file.
    state.annotations = _restore_annotations(record.id, rgb, state.classes, labels_dir)

    return render_state_image(state), image_path.name, stats_html()


def _write_labels(state: AppState, export_format: str, labels_dir: Path) -> bool:
    image_id = state.current_image_id
    if image_id is None:
        return False
    record = _db.get_by_id(image_id)
    if record is None:
        return False

    # Build one YOLO line per annotation, skipping entries with missing geometry.
    label_file = labels_dir / (Path(record.path).stem + ".txt")
    lines: list[str] = []

    for ann in state.annotations:
        yolo_class_id = ann.yolo_class_id
        if export_format == "seg":
            if not ann.polygon:
                continue
            coords = " ".join(f"{v:.6f}" for v in ann.polygon)
            lines.append(f"{yolo_class_id} {coords}")
        else:
            if not ann.bbox:
                continue
            xc, yc, w, h = ann.bbox
            lines.append(f"{yolo_class_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    label_file.write_text("\n".join(lines) + ("\n" if lines else ""))
    return True


def _build_empty_response(state: AppState, status_msg: str) -> NavigationResponse:
    """Build a NavigationResponse for when no image record is found.

    Args:
        state:      Current session state.
        status_msg: Message to display in the status box.

    Returns:
        A NavigationResponse with an empty label and annotation summary.
    """
    state.current_image = None
    return NavigationResponse(
        display_img=render_state_image(state),
        state=state,
        status_msg=status_msg,
        stats_html=stats_html(),
        image_label="",
        ann_summary=format_annotations_summary([]),
    )


def _build_loaded_response(
    record: ImageRecord,
    state: AppState,
    app_ctx: AppContext,
    labels_dir: Path,
    status_prefix: str,
) -> NavigationResponse:
    """Load *record* and build a NavigationResponse.

    Args:
        record:        The image record to display.
        state:         Current session state (mutated in-place).
        app_ctx:       Application context holding the model client.
        labels_dir:    Directory for YOLO label files.
        status_prefix: Text prepended to the image name in the status message.

    Returns:
        A NavigationResponse populated with the newly loaded image.
    """
    rendered, label, html = _load_image(record, state, app_ctx, labels_dir)
    return NavigationResponse(
        display_img=rendered,
        state=state,
        status_msg=f"{status_prefix}{label}",
        stats_html=html,
        image_label=label,
        ann_summary=format_annotations_summary(state.annotations),
    )


def save_and_next(
    state: AppState, export_fmt: str, app_ctx: AppContext, labels_dir: Path,
) -> NavigationResponse:
    """Write labels, mark image done, and load the next pending image."""
    image_id = state.current_image_id
    normalized_format = "seg" if export_fmt == "Segmentation" else "det"

    if image_id is not None and _write_labels(state, normalized_format, labels_dir):
        _db.mark_done(image_id, normalized_format)

    record = _db.get_next(after_id=image_id)
    if record is None:
        return _build_empty_response(state, "All done — no more pending images!")

    return _build_loaded_response(record, state, app_ctx, labels_dir, "Saved. Loaded: ")


def skip_image(state: AppState, app_ctx: AppContext, labels_dir: Path) -> NavigationResponse:
    """Mark the current image skipped and load the next pending image."""
    image_id = state.current_image_id
    if image_id is not None:
        _db.mark_skipped(image_id)

    record = _db.get_next(after_id=image_id)
    if record is None:
        return _build_empty_response(state, "No more pending images.")

    return _build_loaded_response(record, state, app_ctx, labels_dir, "Skipped. Loaded: ")


def go_prev(state: AppState, app_ctx: AppContext, labels_dir: Path) -> NavigationResponse:
    """Load the previous image (by DB id) without saving."""
    image_id = state.current_image_id
    record = _db.get_next() if image_id is None else _db.get_prev(image_id)

    if record is None:
        return _build_empty_response(state, "No previous image.")

    return _build_loaded_response(record, state, app_ctx, labels_dir, "Loaded: ")


def go_next_pending(state: AppState, app_ctx: AppContext, labels_dir: Path) -> NavigationResponse:
    """Load the next pending image without saving."""
    image_id = state.current_image_id
    record = _db.get_next(after_id=image_id)

    if record is None:
        return _build_empty_response(state, "No more pending images.")

    return _build_loaded_response(record, state, app_ctx, labels_dir, "Loaded: ")
