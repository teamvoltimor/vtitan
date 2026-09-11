"""Unit tests for eval.metrics' detection scoring maths."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

from eval.metrics import (
    IMAGE_SIZE,
    average_precision,
    confusion,
    iou,
    letterbox,
    load_ground_truth,
    mean_average_precision,
    print_report,
    sample,
    summarise,
)

if TYPE_CHECKING:
    import pytest


def test_letterbox_pads_a_wide_image_on_the_vertical_axis() -> None:
    img = Image.new("RGB", (200, 100))
    result = letterbox(img)

    assert result.canvas.shape == (IMAGE_SIZE, IMAGE_SIZE, 3)
    assert result.scale == IMAGE_SIZE / 200
    assert result.pad_x == 0
    assert result.pad_y > 0


def test_letterbox_pads_a_tall_image_on_the_horizontal_axis() -> None:
    img = Image.new("RGB", (100, 200))
    result = letterbox(img)

    assert result.scale == IMAGE_SIZE / 200
    assert result.pad_y == 0
    assert result.pad_x > 0


def test_load_ground_truth_returns_empty_for_missing_label_file(tmp_path: Path) -> None:
    assert load_ground_truth(tmp_path, "missing", (100, 100), (1.0, 0, 0)) == []


def test_load_ground_truth_remaps_class_and_applies_scale_and_padding(tmp_path: Path) -> None:
    (tmp_path / "sample.txt").write_text("1 0.5 0.5 0.5 0.5\n")

    boxes = load_ground_truth(tmp_path, "sample", (100, 100), (2.0, 10, 20))

    assert len(boxes) == 1
    class_id, box = boxes[0]
    assert class_id == 0  # LABEL_TO_MODEL[1] == 0
    np.testing.assert_allclose(box, [10 + 50, 20 + 50, 10 + 150, 20 + 150])


def test_load_ground_truth_skips_malformed_lines(tmp_path: Path) -> None:
    (tmp_path / "sample.txt").write_text("0 0.1 0.1\n")
    assert load_ground_truth(tmp_path, "sample", (100, 100), (1.0, 0, 0)) == []


def test_iou_of_identical_boxes_is_one() -> None:
    box = np.array([0.0, 0.0, 10.0, 10.0])
    assert iou(box, box) == 1.0


def test_iou_of_disjoint_boxes_is_zero() -> None:
    a = np.array([0.0, 0.0, 5.0, 5.0])
    b = np.array([10.0, 10.0, 15.0, 15.0])
    assert iou(a, b) == 0.0


def test_iou_of_partially_overlapping_boxes() -> None:
    a = np.array([0.0, 0.0, 10.0, 10.0])
    b = np.array([5.0, 0.0, 15.0, 10.0])
    assert iou(a, b) == 50.0 / 150.0


def test_average_precision_is_nan_with_no_ground_truth() -> None:
    assert math.isnan(average_precision([(0.9, True)], 0))


def test_average_precision_is_zero_with_no_predictions() -> None:
    assert average_precision([], 5) == 0.0


def test_average_precision_is_one_for_a_perfect_ranking() -> None:
    scored = [(0.9, True), (0.8, True), (0.7, True)]
    assert average_precision(scored, 3) == 1.0


def test_average_precision_penalises_false_positives_ranked_first() -> None:
    scored = [(0.9, False), (0.8, True)]
    assert average_precision(scored, 1) < 1.0


def test_mean_average_precision_for_a_perfect_single_class_match() -> None:
    box = np.array([0.0, 0.0, 10.0, 10.0])
    predictions = [[(0, 0.9, box)]]
    truth = [[(0, box)]]

    map_value, per_class = mean_average_precision(predictions, truth, threshold=0.5)

    assert map_value == 1.0
    assert per_class == {0: 1.0}


def test_confusion_counts_true_positive_false_positive_and_missed() -> None:
    box = np.array([0.0, 0.0, 10.0, 10.0])
    far_box = np.array([100.0, 100.0, 110.0, 110.0])
    predictions = [[(0, 0.9, box), (1, 0.9, far_box)]]
    truth = [[(0, box), (2, far_box + 200)]]

    result = confusion(predictions, truth)

    assert result[(0, 0)] == 1  # correct match
    assert result[(-1, 1)] == 1  # spurious false positive, no ground truth overlap
    assert result[(2, -1)] == 1  # missed ground truth


def test_sample_returns_all_paths_when_limit_exceeds_count() -> None:
    paths = [Path(f"img{i}") for i in range(5)]
    assert sample(paths, limit=10) == paths


def test_sample_returns_all_paths_when_limit_is_non_positive() -> None:
    paths = [Path(f"img{i}") for i in range(5)]
    assert sample(paths, limit=0) == paths


def test_sample_evenly_spreads_a_subset() -> None:
    paths = [Path(str(i)) for i in range(100)]
    subset = sample(paths, limit=10)
    assert len(subset) <= 10
    assert subset == sorted(subset)


def test_summarise_reports_perfect_scores_for_a_perfect_match() -> None:
    box = np.array([0.0, 0.0, 10.0, 10.0])
    predictions = [[(0, 0.9, box)]]
    truth = [[(0, box)]]

    result = summarise(predictions, truth)

    assert result.mAP50 == 1.0
    assert result.mAP75 == 1.0


def test_print_report_prints_summary_tables(capsys: pytest.CaptureFixture[str]) -> None:
    box = np.array([0.0, 0.0, 10.0, 10.0])
    result = summarise([[(0, 0.9, box)]], [[(0, box)]])

    print_report({"model": result})

    out = capsys.readouterr().out
    assert "SUMMARY" in out
    assert "model" in out
