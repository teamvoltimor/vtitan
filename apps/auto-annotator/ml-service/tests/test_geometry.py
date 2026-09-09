"""Tests for src.geometry — pure mask/polygon/bbox conversion helpers."""

import numpy as np
import pytest

from src.geometry.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon, polygon_to_yolo_bbox


def _square_mask(size: int = 20, x0: int = 5, y0: int = 5, side: int = 10) -> np.ndarray:
    mask = np.zeros((size, size), dtype=bool)
    mask[y0 : y0 + side, x0 : x0 + side] = True
    return mask


class TestMaskToYoloPolygon:
    def test_empty_mask_returns_empty_list(self):
        mask = np.zeros((20, 20), dtype=bool)
        assert mask_to_yolo_polygon(mask) == []

    def test_square_mask_returns_normalized_polygon(self):
        mask = _square_mask()
        result = mask_to_yolo_polygon(mask)

        assert result != []
        # Flat [x0, y0, x1, y1, ...] list with at least 3 points (6 floats).
        assert len(result) >= 6
        assert len(result) % 2 == 0
        # All coordinates normalized to [0, 1].
        assert all(0.0 <= v <= 1.0 for v in result)

    def test_tiny_noise_below_minimum_area_returns_empty(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[0, 0] = True  # single-pixel contour, area ~0 < GEOMETRY_MINIMUM_CONTOUR_AREA
        assert mask_to_yolo_polygon(mask) == []

    def test_epsilon_factor_affects_simplification(self):
        mask = _square_mask(size=100, x0=10, y0=10, side=60)
        coarse = mask_to_yolo_polygon(mask, epsilon_factor=0.05)
        fine = mask_to_yolo_polygon(mask, epsilon_factor=0.0001)

        # A larger epsilon simplifies more aggressively -> fewer or equal points.
        assert len(coarse) <= len(fine)


class TestPolygonToYoloBbox:
    def test_empty_points_returns_empty_list(self):
        assert polygon_to_yolo_bbox([]) == []

    def test_unit_square_bbox(self):
        points = [(0.2, 0.2), (0.8, 0.2), (0.8, 0.8), (0.2, 0.8)]
        xc, yc, w, h = polygon_to_yolo_bbox(points)

        assert xc == 0.5
        assert yc == 0.5
        assert w == pytest.approx(0.6)
        assert h == pytest.approx(0.6)

    def test_single_point_has_zero_size(self):
        result = polygon_to_yolo_bbox([(0.5, 0.5)])
        assert result == [0.5, 0.5, 0.0, 0.0]


class TestMaskToYoloBbox:
    def test_empty_mask_returns_empty_list(self):
        mask = np.zeros((20, 20), dtype=bool)
        assert mask_to_yolo_bbox(mask) == []

    def test_square_mask_bbox_matches_expected_geometry(self):
        # 10x10 square at (5,5) in a 20x20 image -> x in [5,14], y in [5,14].
        mask = _square_mask(size=20, x0=5, y0=5, side=10)
        xc, yc, w, h = mask_to_yolo_bbox(mask)

        expected_xc = (5 + 14) / 2 / 20
        expected_yc = (5 + 14) / 2 / 20
        expected_w = (14 - 5) / 20
        expected_h = (14 - 5) / 20

        assert xc == pytest.approx(expected_xc)
        assert yc == pytest.approx(expected_yc)
        assert w == pytest.approx(expected_w)
        assert h == pytest.approx(expected_h)

    def test_single_pixel_mask_has_zero_size_bbox(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[10, 10] = True
        xc, yc, w, h = mask_to_yolo_bbox(mask)

        assert w == 0.0
        assert h == 0.0
        assert xc == 10 / 20
        assert yc == 10 / 20
