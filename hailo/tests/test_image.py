"""Unit tests for the pure coordinate/geometry math in src.image."""

from __future__ import annotations

import numpy as np
import pytest

from src.errors import HailoError
from src.image import apply_boxes, letterbox, scale_coords, unletterbox_mask


def test_letterbox_preserves_aspect_ratio_for_wide_image() -> None:
    img = np.zeros((100, 200, 3), dtype=np.uint8)
    result = letterbox(img, new_shape=(640, 640))

    assert result.image.shape[:2] == (640, 640)
    # Width is the limiting dimension for a wide image, so height gets padded.
    assert result.ratio == pytest.approx(640 / 200)
    assert result.pad_width == pytest.approx(0.0)
    assert result.pad_height > 0


def test_letterbox_preserves_aspect_ratio_for_tall_image() -> None:
    img = np.zeros((200, 100, 3), dtype=np.uint8)
    result = letterbox(img, new_shape=(640, 640))

    assert result.image.shape[:2] == (640, 640)
    assert result.ratio == pytest.approx(640 / 200)
    assert result.pad_height == pytest.approx(0.0)
    assert result.pad_width > 0


def test_letterbox_square_image_has_no_padding() -> None:
    img = np.zeros((320, 320, 3), dtype=np.uint8)
    result = letterbox(img, new_shape=(640, 640))

    assert result.image.shape[:2] == (640, 640)
    assert result.ratio == pytest.approx(2.0)
    assert result.pad_width == pytest.approx(0.0)
    assert result.pad_height == pytest.approx(0.0)


def test_scale_coords_round_trips_through_letterbox() -> None:
    # A 100x200 (HxW) image letterboxed into 640x640: ratio=3.2, dw=0, dh=240.
    img = np.zeros((100, 200, 3), dtype=np.uint8)
    result = letterbox(img, new_shape=(640, 640))

    # A box drawn directly on the padded 640x640 canvas at its full extent
    # (accounting for the vertical letterbox bars) should map back to the
    # original image's full extent.
    boxes = np.array([[0.0, result.pad_height, 640.0, 640.0 - result.pad_height]])
    scaled = scale_coords(boxes, result.ratio, result.pad_width, result.pad_height, img.shape)

    assert scaled[0] == pytest.approx([0.0, 0.0, 200.0, 100.0])


def test_scale_coords_clips_to_image_bounds() -> None:
    img_shape = (100, 200, 3)
    # Way out-of-range box; ratio=1, no padding, so scaling is a no-op and
    # only the clip should apply.
    boxes = np.array([[-50.0, -50.0, 500.0, 500.0]])
    scaled = scale_coords(boxes, ratio=1.0, dw=0.0, dh=0.0, img_shape=img_shape)

    assert scaled[0] == pytest.approx([0.0, 0.0, 200.0, 100.0])


def test_scale_coords_mutates_in_place() -> None:
    boxes = np.array([[10.0, 10.0, 20.0, 20.0]])
    result = scale_coords(boxes, ratio=1.0, dw=0.0, dh=0.0, img_shape=(100, 100, 3))
    assert result is boxes


def test_unletterbox_mask_crops_and_resizes_to_original_shape() -> None:
    # Mask in letterboxed (640x640) space, with 240px vertical bars (dh=240).
    mask = np.zeros((640, 640), dtype=np.uint8)
    mask[240:400, :] = 255  # the "real" content band

    out = unletterbox_mask(mask, orig_shape=(100, 200, 3), _ratio=3.2, dw=0.0, dh=240.0)

    assert out.shape == (100, 200)


def test_unletterbox_mask_falls_back_when_crop_is_empty() -> None:
    # Padding that would crop the mask down to zero size must not raise --
    # it should fall back to resizing the uncropped mask instead.
    mask = np.full((10, 10), 255, dtype=np.uint8)
    out = unletterbox_mask(mask, orig_shape=(50, 50, 3), _ratio=1.0, dw=20.0, dh=20.0)
    assert out.shape == (50, 50)


def test_apply_boxes_returns_early_for_none() -> None:
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    # Must not raise or modify the image.
    apply_boxes(None, conf=0.5, ratio=1.0, dw=0.0, dh=0.0, image=image)
    assert image.sum() == 0


def test_apply_boxes_returns_early_for_empty() -> None:
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    empty = np.zeros((0, 6))
    apply_boxes(empty, conf=0.5, ratio=1.0, dw=0.0, dh=0.0, image=image)
    assert image.sum() == 0


def test_apply_boxes_rejects_wrong_column_count() -> None:
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    # Raw (channels, anchors)-shaped tensor from an nms=False export -- not
    # post-NMS (N, 6) -- must raise a clear HailoError, not a numpy error.
    raw = np.zeros((84, 8400))
    with pytest.raises(HailoError, match="expected post-NMS"):
        apply_boxes(raw, conf=0.5, ratio=1.0, dw=0.0, dh=0.0, image=image)


def test_apply_boxes_draws_boxes_above_confidence_threshold() -> None:
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    # One box above threshold, one below -- only the first should be drawn.
    raw = np.array(
        [
            [10.0, 10.0, 50.0, 50.0, 0.9, 0.0],
            [5.0, 5.0, 15.0, 15.0, 0.1, 0.0],
        ],
    )
    apply_boxes(raw, conf=0.5, ratio=1.0, dw=0.0, dh=0.0, image=image)
    assert image.sum() > 0
