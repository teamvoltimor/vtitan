"""Unit tests for src.registry's model registry."""

from __future__ import annotations

import dataclasses

import pytest

from src.enums import Task
from src.errors import HailoError
from src.registry import MODEL_REGISTRY, ModelEntry, get_entry


def test_get_entry_returns_the_registered_entry() -> None:
    entry = get_entry("yolo11n")
    assert entry is MODEL_REGISTRY["yolo11n"]
    assert entry.zoo_name == "yolov11n"
    assert entry.task == Task.DETECT


def test_get_entry_raises_on_unknown_key() -> None:
    with pytest.raises(HailoError, match="Unknown model 'nonexistent'"):
        get_entry("nonexistent")


def test_get_entry_error_lists_valid_options() -> None:
    with pytest.raises(HailoError, match="yolo11n"):
        get_entry("nonexistent")


def test_gmr_entry_has_three_classes() -> None:
    entry = get_entry("gmr")
    assert entry.classes == 3
    assert entry.zoo_name == "yolov11n"


def test_seg_model_has_segment_task() -> None:
    entry = get_entry("yolo26l-seg")
    assert entry.task == Task.SEGMENT


def test_extra_kwargs_converts_export_extras_to_dict() -> None:
    entry = get_entry("yolo12n")
    kwargs = entry.extra_kwargs()
    assert kwargs == {"simplify": True, "nms": False, "optimize": False}


def test_extra_kwargs_is_empty_when_no_export_extras() -> None:
    entry = get_entry("yolo11n")
    assert entry.extra_kwargs() == {}


def test_model_entry_is_frozen() -> None:
    entry = ModelEntry(
        pt_file="data/x.pt",
        onnx_file="data/x.onnx",
        task=Task.DETECT,
        opset=13,
        export_extras=(),
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        entry.pt_file = "changed"  # type: ignore[misc]


def test_all_registry_entries_have_onnx_under_data_dir() -> None:
    for key, entry in MODEL_REGISTRY.items():
        assert entry.onnx_file.startswith("data/"), key
