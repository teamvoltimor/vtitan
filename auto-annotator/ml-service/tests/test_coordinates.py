"""Tests for src.coordinates — pure coordinate-space conversion helpers."""

import pytest

from src.coordinates import (
    NormalizedPoint,
    PixelPoint,
    YOLOPoint,
    yolo_bbox_to_corners,
    yolo_coords_to_polygon,
)


class TestNormalizedPoint:
    def test_to_pixel_scales_by_dimensions(self):
        p = NormalizedPoint(x=0.5, y=0.25)
        pixel = p.to_pixel(width=101, height=101)

        assert pixel == PixelPoint(x=50, y=25)

    def test_to_pixel_clamps_out_of_range_values(self):
        p = NormalizedPoint(x=1.5, y=-0.5)
        pixel = p.to_pixel(width=101, height=101)

        assert pixel == PixelPoint(x=100, y=0)

    def test_to_yolo_is_identity(self):
        p = NormalizedPoint(x=0.3, y=0.7)
        assert p.to_yolo() == (0.3, 0.7)

    def test_is_frozen(self):
        p = NormalizedPoint(x=0.1, y=0.1)
        with pytest.raises(AttributeError):
            p.x = 0.9  # type: ignore[misc]


class TestPixelPoint:
    def test_to_normalized_round_trips(self):
        pixel = PixelPoint(x=50, y=25)
        normalized = pixel.to_normalized(width=101, height=101)

        assert normalized.x == pytest.approx(0.5)
        assert normalized.y == pytest.approx(0.25)


class TestYOLOPoint:
    def test_to_normalized_clamps(self):
        p = YOLOPoint(x=1.2, y=-0.2)
        normalized = p.to_normalized()

        assert normalized == NormalizedPoint(x=1.0, y=0.0)

    def test_to_pixel_composes_normalized_and_pixel_conversion(self):
        p = YOLOPoint(x=0.5, y=0.5)
        pixel = p.to_pixel(width=101, height=101)

        assert pixel == PixelPoint(x=50, y=50)


class TestYoloCoordsToPolygon:
    def test_empty_list_returns_empty(self):
        assert yolo_coords_to_polygon([]) == []

    def test_pairs_are_parsed_in_order(self):
        coords = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        result = yolo_coords_to_polygon(coords)

        assert result == [
            YOLOPoint(x=0.1, y=0.2),
            YOLOPoint(x=0.3, y=0.4),
            YOLOPoint(x=0.5, y=0.6),
        ]


class TestYoloBboxToCorners:
    def test_returns_four_corners_in_tl_tr_br_bl_order(self):
        corners = yolo_bbox_to_corners(xc=0.5, yc=0.5, w=0.4, h=0.2)

        assert corners == [
            YOLOPoint(x=0.3, y=0.4),  # top-left
            YOLOPoint(x=0.7, y=0.4),  # top-right
            YOLOPoint(x=0.7, y=0.6),  # bottom-right
            YOLOPoint(x=0.3, y=0.6),  # bottom-left
        ]

    def test_zero_size_box_collapses_to_a_single_point(self):
        corners = yolo_bbox_to_corners(xc=0.5, yc=0.5, w=0.0, h=0.0)
        assert all(c == YOLOPoint(x=0.5, y=0.5) for c in corners)
