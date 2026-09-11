"""Unit tests for src.config's CLI parameter dataclasses."""

from __future__ import annotations

import dataclasses

import pytest

from src.config import (
    CompileConfig,
    ConvertConfig,
    DownloadConfig,
    EvalConfig,
    ExportConfig,
    ProfileConfig,
    StageConfig,
    TestConfig,
)
from src.enums import EvalTarget, HWArch
from src.registry import SHARED_WITH_DOCKER


def test_stage_config_applies_defaults() -> None:
    config = StageConfig(model="yolov8n", calib="calib_data")

    assert config.shared_dir == SHARED_WITH_DOCKER
    assert config.calib_name == "calib_data"
    assert config.labels is None
    assert config.labels_name == "calib_labels"


def test_stage_config_overrides_defaults() -> None:
    config = StageConfig(
        model="yolov8n",
        calib="calib_data",
        shared_dir="custom_shared",
        calib_name="custom_calib",
        labels="labels_dir",
        labels_name="custom_labels",
    )

    assert config.shared_dir == "custom_shared"
    assert config.labels == "labels_dir"
    assert config.labels_name == "custom_labels"


def test_compile_config_defaults() -> None:
    config = CompileConfig(
        model="yolov8n",
        zoo_name=None,
        docker=None,
        hw=HWArch.HAILO8,
        calib_path="/local/shared_with_docker/calib_data",
    )

    assert config.classes is None
    assert config.model_script is None
    assert config.performance is False


def test_eval_config_requires_all_fields() -> None:
    config = EvalConfig(
        model="yolov8n",
        zoo_name="yolov8n",
        docker="container",
        har=None,
        target=EvalTarget.EMULATOR,
        data_count=512,
        visualize=False,
    )

    assert config.har is None
    assert config.data_count == 512


def test_profile_config_requires_all_fields() -> None:
    config = ProfileConfig(model="yolov8n", zoo_name=None, docker=None, hef=None)

    assert config.hef is None


@pytest.mark.parametrize(
    "cls",
    (ExportConfig, DownloadConfig, ConvertConfig, TestConfig, StageConfig, CompileConfig, EvalConfig, ProfileConfig),
)
def test_configs_are_frozen(cls: type) -> None:
    fields = dataclasses.fields(cls)
    kwargs = {f.name: f.default for f in fields if f.default is not dataclasses.MISSING}
    for f in fields:
        if f.name not in kwargs:
            kwargs[f.name] = None

    instance = cls(**kwargs)
    with pytest.raises(dataclasses.FrozenInstanceError):
        instance.__setattr__(fields[0].name, "changed")
