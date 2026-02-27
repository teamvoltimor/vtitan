"""src.render – Composite image renderer for the annotation canvas.

Layers drawn bottom to top:
  1. Accepted annotation fills        (class colour, alpha = CANVAS_ANNOTATION_FILL_ALPHA)
  2. Accepted annotation outlines     (class/high-contrast/black/white, shadow + colour)
  3. Pending mask fill                (class colour, same alpha)
  4. Pending mask outline             (white + black shadow)
  5. Point buffer circles             (class-colour solid pos / blue + × neg)
  6. Top-left class legend            (colour swatch + class name)
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.constants import (
    CANVAS_ANNOTATION_BASE_ALPHA,
    CANVAS_ANNOTATION_FILL_ALPHA,
    CANVAS_CONTOUR_OUTLINE_THICKNESS,
    CANVAS_CONTOUR_SHADOW_THICKNESS,
    CANVAS_LEGEND_FONT_SCALE,
    CANVAS_LEGEND_ROW_STEP_Y,
    CANVAS_LEGEND_START_Y,
    CANVAS_LEGEND_SWATCH_HEIGHT,
    CANVAS_LEGEND_SWATCH_X1,
    CANVAS_LEGEND_SWATCH_X2,
    CANVAS_LEGEND_TEXT_OFFSET_Y,
    CANVAS_LEGEND_TEXT_X,
    CANVAS_LUMINANCE_THRESHOLD,
    CANVAS_NEGATIVE_CROSS_SIZE,
    CANVAS_PLACEHOLDER_HEIGHT,
    CANVAS_PLACEHOLDER_WIDTH,
    CANVAS_POINT_BORDER_THICKNESS,
    CANVAS_POINT_RADIUS,
    COLOR_BLACK_RGB,
    COLOR_BLUE_RGB,
    COLOR_GREEN_HEX,
    COLOR_LUMINANCE_WEIGHTS,
    COLOR_WHITE_HEX,
    COLOR_WHITE_RGB,
    DILATION_KERNEL,
)
from src.enums import OutlineMode

if TYPE_CHECKING:
    from src.models import AppState

from src.utils import hex_to_rgb


def _resolve_outline_color(
    class_color: str,
    mask: np.ndarray,
    img_u8: np.ndarray,
    mode: str,
) -> tuple[int, int, int]:
    """Choose an annotation outline colour based on the current outline mode.

    Args:
        class_color: Hex colour string of the annotation class.
        mask:        Boolean H×W mask array used when sampling border pixels.
        img_u8:      Uint8 RGB image used for luminance sampling in high-contrast mode.
        mode:        One of the :class:`~src.enums.OutlineMode` string values.

    Returns:
        ``(R, G, B)`` int tuple for the outline colour.
    """
    if mode == OutlineMode.CLASS_COLOR:
        return hex_to_rgb(class_color)
    if mode == OutlineMode.BLACK:
        return COLOR_BLACK_RGB
    if mode == OutlineMode.WHITE:
        return COLOR_WHITE_RGB

    # High-contrast mode: sample the pixels just outside the mask border and
    # pick whichever of black or white maximises contrast against the background.
    border = cv2.dilate(mask.astype(np.uint8), DILATION_KERNEL) - mask.astype(np.uint8)
    pixels = img_u8[border.astype(bool)]
    if len(pixels) == 0:
        return COLOR_WHITE_RGB
    luminance = float(np.dot(pixels.mean(axis=0), COLOR_LUMINANCE_WEIGHTS))
    return COLOR_BLACK_RGB if luminance > CANVAS_LUMINANCE_THRESHOLD else COLOR_WHITE_RGB


def render_state_image(state: AppState) -> np.ndarray:
    """Composite all annotation layers onto the current image and return RGB uint8.

    Returns a black ``CANVAS_PLACEHOLDER_HEIGHT × CANVAS_PLACEHOLDER_WIDTH``
    placeholder when no image is loaded.

    Args:
        state: Current :class:`AppState` holding the image and all annotation data.

    Returns:
        Rendered RGB uint8 numpy array of the same dimensions as the source image.
    """
    img = state.current_image
    if img is None:
        return np.zeros((CANVAS_PLACEHOLDER_HEIGHT, CANVAS_PLACEHOLDER_WIDTH, 3), dtype=np.uint8)

    result = img.astype(np.float32)

    # Layer 1: accepted annotation fills.
    for ann in state.annotations:
        color_rgb = hex_to_rgb(ann.class_color)
        mask = ann.mask
        colored = np.zeros_like(result)
        colored[mask] = color_rgb
        result = np.where(
            mask[:, :, None],
            CANVAS_ANNOTATION_BASE_ALPHA * result + CANVAS_ANNOTATION_FILL_ALPHA * colored,
            result,
        )

    # Layer 2: pending mask fill (same alpha blending as accepted annotations).
    pending_mask = state.pending_mask
    pending_class = state.pending_class_db_id
    if pending_mask is not None and pending_class is not None:
        cls_map = {c.id: c.color for c in state.classes}
        p_color_rgb = hex_to_rgb(cls_map.get(pending_class, COLOR_WHITE_HEX))
        colored = np.zeros_like(result)
        colored[pending_mask] = p_color_rgb
        result = np.where(
            pending_mask[:, :, None],
            CANVAS_ANNOTATION_BASE_ALPHA * result + CANVAS_ANNOTATION_FILL_ALPHA * colored,
            result,
        )

    result_u8 = result.clip(0, 255).astype(np.uint8)
    outline_mode = state.outline_color

    # Layer 3: accepted annotation contour outlines.
    for ann in state.annotations:
        outline_rgb = _resolve_outline_color(ann.class_color, ann.mask, result_u8, outline_mode)
        contours, _ = cv2.findContours(
            ann.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )
        # Black shadow drawn first; coloured outline drawn on top.
        cv2.drawContours(result_u8, contours, -1, COLOR_BLACK_RGB, CANVAS_CONTOUR_SHADOW_THICKNESS)
        cv2.drawContours(result_u8, contours, -1, outline_rgb, CANVAS_CONTOUR_OUTLINE_THICKNESS)

    # Layer 4: pending mask outline (white with black shadow).
    if pending_mask is not None:
        pmask_u8 = pending_mask.astype(np.uint8) * 255
        contours_p, _ = cv2.findContours(pmask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(
            result_u8, contours_p, -1, COLOR_BLACK_RGB, CANVAS_CONTOUR_SHADOW_THICKNESS, cv2.LINE_AA,
        )
        cv2.drawContours(
            result_u8, contours_p, -1, COLOR_WHITE_RGB, CANVAS_CONTOUR_OUTLINE_THICKNESS, cv2.LINE_AA,
        )

    # Layer 5: click-point circles.
    class_colors_map = {c.id: c.color for c in state.classes}
    pending_class_id = state.pending_class_db_id
    default_point_color: tuple[int, int, int] = hex_to_rgb(
        class_colors_map[pending_class_id]
        if pending_class_id is not None and pending_class_id in class_colors_map
        else COLOR_GREEN_HEX,
    )

    for pt in state.point_buffer:
        px, py = pt.x, pt.y
        pt_color = (
            hex_to_rgb(class_colors_map[pt.class_id])
            if pt.class_id in class_colors_map
            else default_point_color
        )
        if pt.label == 1:
            # Positive point: filled circle in class colour with white border.
            cv2.circle(result_u8, (px, py), CANVAS_POINT_RADIUS, pt_color, -1)
            cv2.circle(result_u8, (px, py), CANVAS_POINT_RADIUS, COLOR_WHITE_RGB, CANVAS_POINT_BORDER_THICKNESS)
        else:
            # Negative point: blue circle with a white × mark.
            cv2.circle(result_u8, (px, py), CANVAS_POINT_RADIUS, COLOR_BLUE_RGB, -1)
            cv2.circle(result_u8, (px, py), CANVAS_POINT_RADIUS, COLOR_WHITE_RGB, CANVAS_POINT_BORDER_THICKNESS)
            sz = CANVAS_NEGATIVE_CROSS_SIZE
            cv2.line(result_u8, (px - sz, py - sz), (px + sz, py + sz), COLOR_WHITE_RGB, CANVAS_CONTOUR_OUTLINE_THICKNESS)
            cv2.line(result_u8, (px + sz, py - sz), (px - sz, py + sz), COLOR_WHITE_RGB, CANVAS_CONTOUR_OUTLINE_THICKNESS)

    # Layer 6: top-left class legend.
    # Collect unique classes from accepted annotations to show in the legend.
    seen: dict[int, tuple[str, str]] = {}
    for ann in state.annotations:
        if ann.class_db_id not in seen:
            seen[ann.class_db_id] = (ann.class_name, ann.class_color)

    y = CANVAS_LEGEND_START_Y
    for _cid, (cname, chex) in sorted(seen.items()):
        color_rgb = hex_to_rgb(chex)
        cv2.rectangle(
            result_u8,
            (CANVAS_LEGEND_SWATCH_X1, y),
            (CANVAS_LEGEND_SWATCH_X2, y + CANVAS_LEGEND_SWATCH_HEIGHT),
            color_rgb,
            -1,
        )
        cv2.putText(
            result_u8,
            f"{_cid}: {cname}",
            (CANVAS_LEGEND_TEXT_X, y + CANVAS_LEGEND_TEXT_OFFSET_Y),
            cv2.FONT_HERSHEY_SIMPLEX,
            CANVAS_LEGEND_FONT_SCALE,
            COLOR_WHITE_RGB,
            1,
            cv2.LINE_AA,
        )
        y += CANVAS_LEGEND_ROW_STEP_Y

    return result_u8
