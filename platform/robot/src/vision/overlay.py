"""Draw detections onto a frame, for debugging what the model actually sees.

Kept out of the detector so the inference path carries no drawing cost when the
debug video is off -- which is every race run.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np
from shared.domain.models import SignColor

from src.vision.hud import _DEFAULT_HUD_CONFIG

if TYPE_CHECKING:
    from shared.domain.models import Detection

# Box colours in RGB, chosen to read against the prism they outline rather than
# to match it: an exactly-matching outline is invisible on the object.
_BOX_RGB: dict[SignColor, tuple[int, int, int]] = {
    SignColor.RED: (255, 80, 80),
    SignColor.GREEN: (80, 255, 80),
    SignColor.MAGENTA: (255, 80, 255),
}
_FALLBACK_RGB = (255, 255, 0)
_THICKNESS = 2


def annotate(rgb: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Return a copy of *rgb* with each detection boxed and labelled.

    Args:
        rgb: Frame in RGB order, as the detector consumes it.
        detections: Detections whose bboxes are in that frame's pixel space.

    Returns:
        A new array; the input is left untouched so the caller can still
        publish or store the clean frame.
    """
    canvas = np.ascontiguousarray(rgb).copy()
    height, width = canvas.shape[:2]

    for detection in detections:
        x1, y1, x2, y2 = (round(v) for v in detection.as_bbox())
        # Detections are clipped to the frame: a box running off the edge makes
        # cv2 draw nothing at all rather than the visible part.
        x1, x2 = max(0, min(x1, width - 1)), max(0, min(x2, width - 1))
        y1, y2 = max(0, min(y1, height - 1)), max(0, min(y2, height - 1))
        if x2 <= x1 or y2 <= y1:
            continue

        colour = _BOX_RGB.get(detection.class_name, _FALLBACK_RGB)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), colour, _THICKNESS)

        label = f"{detection.class_name} {detection.confidence:.2f}"
        (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, _DEFAULT_HUD_CONFIG.font_scale, 1)
        # Put the label inside the box when there is no room above it, so it
        # never lands off-frame for a detection touching the top edge.
        text_y = y1 - baseline if y1 - text_h - baseline >= 0 else y1 + text_h + baseline
        cv2.rectangle(
            canvas,
            (x1, text_y - text_h - baseline),
            (x1 + text_w, text_y + baseline),
            colour,
            cv2.FILLED,
        )
        cv2.putText(
            canvas,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            _DEFAULT_HUD_CONFIG.font_scale,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    return canvas
