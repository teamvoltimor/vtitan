"""Unit tests for src.constants' invariants."""

from __future__ import annotations

from src import constants


def test_letterbox_pad_color_is_a_valid_bgr_triplet() -> None:
    assert len(constants.LETTERBOX_PAD_COLOR) == 3
    assert all(0 <= c <= 255 for c in constants.LETTERBOX_PAD_COLOR)


def test_transpose_hwc_to_chw_is_a_permutation_of_hwc_axes() -> None:
    assert sorted(constants.TRANSPOSE_HWC_TO_CHW) == [0, 1, 2]


def test_normalize_factor_matches_uint8_range() -> None:
    assert constants.NORMALIZE_FACTOR == 255.0


def test_detection_output_cols_account_for_box_score_and_class() -> None:
    assert constants.DETECTION_OUTPUT_COLS == constants.DETECTION_BOX_COLS + 2
    assert constants.DETECTION_SCORE_COL == constants.DETECTION_BOX_COLS
    assert constants.DETECTION_CLASS_COL == constants.DETECTION_SCORE_COL + 1


def test_image_extensions_are_lowercase_dotted() -> None:
    for ext in constants.IMAGE_EXTENSIONS:
        assert ext.startswith(".")
        assert ext == ext.lower()


def test_label_extensions_are_lowercase_dotted() -> None:
    for ext in constants.LABEL_EXTENSIONS:
        assert ext.startswith(".")
        assert ext == ext.lower()


def test_opset_versions_are_positive_ints() -> None:
    assert constants.OPSET_YOLO11 > 0
    assert constants.OPSET_YOLO12 > 0


def test_default_calib_name_and_labels_name_are_distinct() -> None:
    assert constants.DEFAULT_CALIB_NAME != constants.DEFAULT_LABELS_NAME
