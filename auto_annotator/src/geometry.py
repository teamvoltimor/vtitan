"""src.geometry – Pure geometry helpers for YOLO export."""

import cv2
import numpy as np


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
