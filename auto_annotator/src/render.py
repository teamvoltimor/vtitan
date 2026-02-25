"""src.render – Composite image renderer for the annotation canvas."""

from __future__ import annotations

import cv2
import numpy as np

from src.constants import MASK_LABELS
from src.models import AppState
from src.utils import _hex_to_rgb


def _resolve_outline_color(
    class_color: str, mask: np.ndarray, img_u8: np.ndarray, mode: str
) -> tuple[int, int, int]:
    """Return the RGB outline colour for one annotation given the current mode."""
    if mode == "Class color":
        return _hex_to_rgb(class_color)
    if mode == "Black":
        return (0, 0, 0)
    if mode == "White":
        return (255, 255, 255)
    # "High contrast": sample the image along the mask border
    border = cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) - mask.astype(np.uint8)
    pixels = img_u8[border.astype(bool)]
    if len(pixels) == 0:
        return (255, 255, 255)
    luminance = float(np.dot(pixels.mean(axis=0), [0.299, 0.587, 0.114]))
    return (0, 0, 0) if luminance > 128 else (255, 255, 255)


def render_state_image(state: AppState) -> np.ndarray:
    """
    Composite all layers onto the current image and return an RGB uint8 array.

    Layers (bottom → top):
      1. Accepted annotation fills  (class colour, alpha=0.45)
      2. Accepted annotation contour outlines (class colour, 2 px)
      3. Pending mask fill (class colour, alpha=0.45) + white solid outline
      4. Point buffer circles: class-coloured solid (pos) / red + × (neg)
      5. Top-left legend
    """
    img = state.current_image
    if img is None:
        return np.zeros((480, 640, 3), dtype=np.uint8)

    result = img.astype(np.float32)

    # ── 1: Accepted annotation fills ───────────────────────────────────────────
    for ann in state.annotations:
        color_rgb = _hex_to_rgb(ann.class_color)
        mask = ann.mask
        colored = np.zeros_like(result)
        colored[mask] = color_rgb
        result = np.where(mask[:, :, None], 0.55 * result + 0.45 * colored, result)

    # ── 2: Pending mask fill ───────────────────────────────────────────────────
    pending_mask = state.pending_mask
    pending_class = state.pending_class_db_id
    if pending_mask is not None and pending_class is not None:
        cls_map = {c.id: c.color for c in state.classes}
        p_color_hex = cls_map.get(pending_class, "#ffffff")
        p_color_rgb = _hex_to_rgb(p_color_hex)
        colored = np.zeros_like(result)
        colored[pending_mask] = p_color_rgb
        result = np.where(pending_mask[:, :, None], 0.55 * result + 0.45 * colored, result)

    result_u8 = result.clip(0, 255).astype(np.uint8)
    outline_mode = state.outline_color

    # ── 3: Accepted annotation contour outlines ────────────────────────────────
    for ann in state.annotations:
        outline_rgb = _resolve_outline_color(ann.class_color, ann.mask, result_u8, outline_mode)
        contours, _ = cv2.findContours(
            ann.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(result_u8, contours, -1, (0, 0, 0), thickness=4)
        cv2.drawContours(result_u8, contours, -1, outline_rgb, thickness=2)

    # ── 4: Pending mask outline ────────────────────────────────────────────────
    if pending_mask is not None:
        pmask_u8 = pending_mask.astype(np.uint8) * 255
        contours_p, _ = cv2.findContours(pmask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(result_u8, contours_p, -1, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.drawContours(result_u8, contours_p, -1, (255, 255, 255), 2, cv2.LINE_AA)

    # ── 5: Point buffer ────────────────────────────────────────────────────────
    class_colors_map = {c.id: c.color for c in state.classes}
    default_pending_color = (0, 255, 0)
    if state.pending_class_db_id is not None:
        p_hex = class_colors_map.get(state.pending_class_db_id, "#00ff00")
        default_pending_color = _hex_to_rgb(p_hex)

    for pt in state.point_buffer:
        px, py = pt.x, pt.y
        pt_color = default_pending_color
        if pt.class_id in class_colors_map:
            pt_color = _hex_to_rgb(class_colors_map[pt.class_id])

        if pt.label == 1:
            cv2.circle(result_u8, (px, py), 6, pt_color, -1)
            cv2.circle(result_u8, (px, py), 6, (255, 255, 255), 1)
        else:
            cv2.circle(result_u8, (px, py), 6, (0, 0, 255), -1)
            cv2.circle(result_u8, (px, py), 6, (255, 255, 255), 1)
            sz = 4
            cv2.line(result_u8, (px - sz, py - sz), (px + sz, py + sz), (255, 255, 255), 2)
            cv2.line(result_u8, (px + sz, py - sz), (px - sz, py + sz), (255, 255, 255), 2)

    # ── 6: Legend ──────────────────────────────────────────────────────────────
    seen: dict[int, tuple[str, str]] = {}
    for ann in state.annotations:
        cid = ann.class_db_id
        if cid not in seen:
            seen[cid] = (ann.class_name, ann.class_color)

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
