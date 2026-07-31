"""Unit tests for src.test's backend dispatch and inference-loop orchestration.

The real backend handlers (`_PTHandler`, `_ONNXDetectHandler`, ...) need a GPU
or a real model file to run inference, so they're out of scope here. What is
testable without hardware is the dispatch table lookup, the unsupported
backend/task error path, and the generic setup/iterate/infer/save loop -- all
exercised below with a fake `InferenceHandler`.
"""

from __future__ import annotations

import typing
from typing import TYPE_CHECKING

import numpy as np
import pytest

from src.config import TestConfig
from src.enums import Backend, Task
from src.errors import HailoError
from src.test import _DISPATCH, _PTHandler, run

if TYPE_CHECKING:
    from pathlib import Path


class _FakeHandler:
    setup_called = False
    infer_calls: typing.ClassVar[list[str]] = []

    def __init__(self, config: TestConfig) -> None:
        self.config = config

    def setup(self) -> None:
        _FakeHandler.setup_called = True

    def infer(self, img_path: str) -> np.ndarray:
        _FakeHandler.infer_calls.append(img_path)
        return np.zeros((4, 4, 3), dtype=np.uint8)


def _write_image(path: Path) -> None:
    import cv2  # noqa: PLC0415

    img = np.zeros((8, 8, 3), dtype=np.uint8)
    cv2.imwrite(str(path), img)


def test_run_raises_for_unsupported_backend_task_combo(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("src.test._DISPATCH", {})
    config = TestConfig(
        model="model.onnx",
        backend=Backend.ONNX,
        task=Task.DETECT,
        input=str(tmp_path),
        output=str(tmp_path / "out"),
        conf=0.3,
    )
    with pytest.raises(HailoError, match="No handler for"):
        run(config)


def test_run_dispatches_to_the_registered_handler_and_saves_output(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _FakeHandler.setup_called = False
    _FakeHandler.infer_calls = []
    monkeypatch.setattr("src.test._DISPATCH", {(Backend.ONNX, Task.DETECT): _FakeHandler})

    input_dir = tmp_path / "in"
    output_dir = tmp_path / "out"
    input_dir.mkdir()
    _write_image(input_dir / "a.jpg")

    config = TestConfig(
        model="model.onnx",
        backend=Backend.ONNX,
        task=Task.DETECT,
        input=str(input_dir),
        output=str(output_dir),
        conf=0.3,
    )
    run(config)

    assert _FakeHandler.setup_called
    assert _FakeHandler.infer_calls == [str(input_dir / "a.jpg")]
    assert (output_dir / "a.jpg").exists()


def test_dispatch_table_covers_every_backend_task_combination_it_declares() -> None:
    for (backend, task), handler_class in _DISPATCH.items():
        assert isinstance(backend, Backend)
        assert isinstance(task, Task)
        assert hasattr(handler_class, "setup")
        assert hasattr(handler_class, "infer")


def test_ultralytics_backend_setup_requires_ultralytics(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("src.test.YOLO", None)
    monkeypatch.setattr("src.test._yolo_import_err", None)
    config = TestConfig(model="model.pt", backend=Backend.PT, task=Task.DETECT, input=".", output=".", conf=0.3)
    handler = _PTHandler(config)

    with pytest.raises(HailoError, match="not installed"):
        handler.setup()
