"""Tests for the debug overlay drawing.

The overlay is only ever seen by a human, so nothing downstream catches it
being wrong. These pin the parts that would silently produce a blank or
misleading picture.
"""

from __future__ import annotations

import numpy as np

from src.vision.detector import SignDetection, TrafficSignColor
from src.vision.overlay import annotate


def _blank(width: int = 200, height: int = 120) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def _detection(bbox: tuple[float, float, float, float], colour: TrafficSignColor) -> SignDetection:
    return SignDetection(color=colour, bbox=bbox, confidence=0.9)


def test_draws_something_for_a_detection() -> None:
    frame = _blank()
    out = annotate(frame, [_detection((20.0, 20.0, 80.0, 90.0), TrafficSignColor.RED)])
    assert out.any(), "expected the box to mark the frame"


def test_leaves_the_input_untouched() -> None:
    # The caller may still want the clean frame, e.g. to record both.
    frame = _blank()
    annotate(frame, [_detection((10.0, 10.0, 50.0, 50.0), TrafficSignColor.GREEN)])
    assert not frame.any()


def test_no_detections_returns_an_unmarked_copy() -> None:
    frame = _blank()
    out = annotate(frame, [])
    assert not out.any()


def test_box_off_the_right_edge_is_clipped_not_dropped() -> None:
    # cv2.rectangle draws nothing at all when both corners sit outside the
    # frame, so a detection running off the edge would vanish entirely.
    frame = _blank(width=100, height=100)
    out = annotate(frame, [_detection((60.0, 40.0, 400.0, 80.0), TrafficSignColor.MAGENTA)])
    assert out.any()


def test_degenerate_box_is_skipped_without_raising() -> None:
    frame = _blank()
    out = annotate(frame, [_detection((50.0, 50.0, 50.0, 50.0), TrafficSignColor.RED)])
    assert out.shape == frame.shape


def test_detection_at_the_top_edge_still_gets_its_label() -> None:
    # The label goes above the box by default; at y=0 that is off-frame, so it
    # has to flip inside or the text is lost.
    frame = _blank()
    out = annotate(frame, [_detection((10.0, 0.0, 60.0, 40.0), TrafficSignColor.GREEN)])
    # The filled label background is solid colour: expect more than the outline.
    assert np.count_nonzero(out.any(axis=2)) > 200
