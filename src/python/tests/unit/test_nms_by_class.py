"""Tests for NMS-by-class decoding.

The GMR HEF emits ``HAILO NMS BY CLASS``: five values per box with the class
carried positionally rather than in a column. Misreading that layout does not
raise -- it reads a coordinate as a confidence and a confidence as a class id,
so every case here is guarding against a silent wrong answer rather than a
crash.
"""

from __future__ import annotations

import numpy as np
import pytest

from src.hardware.hailo.inferences import InferenceResult, NmsFormatError, iter_nms_by_class

# One box for class 0 and one for class 2, as [ymin, xmin, ymax, xmax, score].
GREEN_BOX = [0.10, 0.20, 0.30, 0.40, 0.90]
RED_BOX = [0.50, 0.60, 0.70, 0.80, 0.75]


def _boxes_last() -> np.ndarray:
    """Layout ``(n_classes, n_boxes, 5)``."""
    tensor = np.zeros((3, 2, 5), dtype=np.float32)
    tensor[0, 0] = GREEN_BOX
    tensor[2, 0] = RED_BOX
    return tensor


def test_boxes_last_layout_maps_class_by_position() -> None:
    rows = list(iter_nms_by_class(_boxes_last()))
    confident = [(cid, conf, box) for cid, conf, box in rows if conf > 0]
    assert [cid for cid, _, _ in confident] == [0, 2]
    assert confident[0][1] == pytest.approx(0.90)
    assert confident[0][2] == pytest.approx((0.10, 0.20, 0.30, 0.40))


def test_batch_dimension_is_stripped() -> None:
    batched = _boxes_last()[np.newaxis, ...]
    assert list(iter_nms_by_class(batched)) == list(iter_nms_by_class(_boxes_last()))


def test_emulator_transposed_layout_agrees_with_boxes_last() -> None:
    # The SDK emulator packs (n_classes, 5, n_boxes); both must decode alike.
    transposed = _boxes_last().transpose(0, 2, 1)
    assert list(iter_nms_by_class(transposed)) == list(iter_nms_by_class(_boxes_last()))


def test_per_class_sequence_layout() -> None:
    # HailoRT may hand back one variable-length array per class.
    rows = list(iter_nms_by_class([np.array([GREEN_BOX]), np.empty((0, 5)), np.array([RED_BOX])]))
    assert [cid for cid, _, _ in rows] == [0, 2]
    assert rows[1][1] == pytest.approx(0.75)


def test_unrecognised_layout_raises_rather_than_misreading() -> None:
    # A 6-column row is the NMS-by-score layout; decoding it as by-class would
    # silently treat a coordinate as a confidence.
    with pytest.raises(NmsFormatError):
        list(iter_nms_by_class(np.zeros((3, 4, 6), dtype=np.float32)))


def test_flat_tensor_raises() -> None:
    with pytest.raises(NmsFormatError):
        list(iter_nms_by_class(np.zeros((10, 5), dtype=np.float32)))


def test_parse_yolo_nms_output_applies_the_class_map_and_threshold() -> None:
    result = InferenceResult.parse_yolo_nms_output(
        raw_tensor=_boxes_last(),
        img_width=640,
        img_height=480,
        class_map={0: "green", 1: "magenta", 2: "red"},
        latency_ms=1.0,
        conf_threshold=0.8,
    )
    # Only the 0.90 box clears the threshold; the 0.75 red box is filtered.
    assert [d.class_name for d in result.detections] == ["green"]
    assert result.detections[0].bbox.x == int(0.20 * 640)
    assert result.detections[0].bbox.y == int(0.10 * 480)
