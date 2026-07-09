"""src.geometry – Pure geometry helpers for YOLO export.

Converts boolean mask arrays into normalised YOLO polygon and bounding-box
coordinates.  No side effects; no dependency on application state.
"""

from __future__ import annotations

import cv2
import numpy as np

from src.constants import (
    GEOMETRY_DEFAULT_EPSILON_FACTOR,
    GEOMETRY_MINIMUM_CONTOUR_AREA,
    GEOMETRY_MINIMUM_POLYGON_POINTS,
)


def mask_to_yolo_polygon(
    mask: np.ndarray,
    epsilon_factor: float = GEOMETRY_DEFAULT_EPSILON_FACTOR,
) -> list[float]:
    """Convert a boolean H×W mask to a flat list of normalised YOLO polygon coordinates.

    Uses the largest external contour and Douglas-Peucker simplification.
    Returns an empty list when the mask is empty, has no valid contours, or
    produces fewer than :data:`~src.constants.GEOMETRY_MINIMUM_POLYGON_POINTS` vertices.

    Args:
        mask:           Boolean H×W numpy array.
        epsilon_factor: Douglas-Peucker epsilon as a fraction of arc-length.
                        Smaller values preserve more detail; larger simplify more.

    Returns:
        Flat list ``[x0, y0, x1, y1, ...]`` of normalised (0-1) coordinates,
        or ``[]`` on failure.
    """
    mask_u8 = mask.astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < GEOMETRY_MINIMUM_CONTOUR_AREA:
        return []

    epsilon = epsilon_factor * cv2.arcLength(largest, closed=True)
    approx = cv2.approxPolyDP(largest, epsilon, closed=True)
    pts = approx.reshape(-1, 2).astype(np.float32)

    if len(pts) < GEOMETRY_MINIMUM_POLYGON_POINTS:
        return []

    h, w = mask.shape
    pts[:, 0] /= w
    pts[:, 1] /= h
    return pts.clip(0.0, 1.0).flatten().tolist()


def polygon_to_yolo_bbox(points: list[tuple[float, float]]) -> list[float]:
    """Convert normalised polygon points to a YOLO bounding box.

    Args:
        points: List of ``(x, y)`` tuples with coordinates normalised to [0, 1].

    Returns:
        ``[xc, yc, w, h]`` normalised bounding box, or ``[]`` if *points* is empty.
    """
    if not points:
        return []
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return [(x_min + x_max) / 2, (y_min + y_max) / 2, x_max - x_min, y_max - y_min]


def mask_to_yolo_bbox(mask: np.ndarray) -> list[float]:
    """Convert a boolean H×W mask to a normalised YOLO bounding-box list.

    Args:
        mask: Boolean H×W numpy array.

    Returns:
        ``[xc, yc, w, h]`` normalised to the image dimensions, or ``[]`` if the
        mask contains no ``True`` pixels.
    """
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return []

    mask_h, mask_w = mask.shape
    x1, x2 = int(xs.min()), int(xs.max())
    y1, y2 = int(ys.min()), int(ys.max())
    xc = (x1 + x2) / 2 / mask_w
    yc = (y1 + y2) / 2 / mask_h
    w = (x2 - x1) / mask_w
    h = (y2 - y1) / mask_h
    return [xc, yc, w, h]
