"""Unit tests for src.hailomz's pure zoo-name resolution logic."""

from __future__ import annotations

import pytest

from src.config import CompileConfig
from src.enums import HWArch
from src.errors import HailoError
from src.hailomz import _resolve_zoo_name, compile_model


def test_resolve_zoo_name_prefers_explicit_override() -> None:
    # yolo11n's registry zoo_name is "yolov11n" -- the override must win.
    assert _resolve_zoo_name("yolo11n", "custom-override") == "custom-override"


def test_resolve_zoo_name_falls_back_to_registry() -> None:
    assert _resolve_zoo_name("yolo11n", None) == "yolov11n"


def test_resolve_zoo_name_raises_when_registry_entry_has_no_zoo_name() -> None:
    # yolo26n is registered but has zoo_name=None (not yet in the Model Zoo).
    with pytest.raises(HailoError, match="Supply --zoo-name"):
        _resolve_zoo_name("yolo26n", None)


def test_resolve_zoo_name_raises_for_unknown_model() -> None:
    with pytest.raises(HailoError, match="Supply --zoo-name"):
        _resolve_zoo_name("not-a-real-model", None)


def test_compile_rejects_model_script_together_with_performance() -> None:
    # hailomz puts these in one mutually exclusive group; catching it here beats
    # letting the container fail minutes into a run.
    config = CompileConfig(
        model="gmr",
        zoo_name=None,
        hw=HWArch.HAILO8,
        calib_path="/local/shared_with_docker/calib_data_gmr",
        docker=None,
        model_script="/local/shared_with_docker/custom.alls",
        performance=True,
    )
    with pytest.raises(HailoError, match="not both"):
        compile_model(config)
