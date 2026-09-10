"""Unit tests for src.coco's class labels and colour palette."""

from __future__ import annotations

from src.coco import COCO_CLASSES, COLOR_CHANNELS, COLOR_MAX, COLOR_MIN, COLORS


def test_coco_classes_has_80_entries() -> None:
    assert len(COCO_CLASSES) == 80
    assert COCO_CLASSES[0] == "person"
    assert COCO_CLASSES[-1] == "toothbrush"


def test_coco_classes_entries_are_unique() -> None:
    assert len(set(COCO_CLASSES)) == len(COCO_CLASSES)


def test_colors_has_one_entry_per_class() -> None:
    assert len(COLORS) == len(COCO_CLASSES)


def test_colors_are_rgb_tuples_in_range() -> None:
    for color in COLORS:
        assert len(color) == COLOR_CHANNELS
        for channel in color:
            assert isinstance(channel, int)
            assert COLOR_MIN <= channel < COLOR_MAX


def test_colors_are_deterministic_across_import() -> None:
    import importlib  # noqa: PLC0415

    import src.coco as coco_module  # noqa: PLC0415

    reloaded = importlib.reload(coco_module)
    assert reloaded.COLORS == COLORS
