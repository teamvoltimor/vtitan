"""Unit tests for src.deps' lazy optional-dependency sentinels."""

from __future__ import annotations

import builtins
import importlib
from typing import TYPE_CHECKING

from src.deps import YOLO, _onnx_import_err, _ort_import_err, _yolo_import_err, onnx, ort

if TYPE_CHECKING:
    import pytest


def test_sentinel_value_and_import_err_are_mutually_exclusive() -> None:
    assert (YOLO is None) == (_yolo_import_err is not None)
    assert (ort is None) == (_ort_import_err is not None)
    assert (onnx is None) == (_onnx_import_err is not None)


def test_deps_are_present_in_the_test_environment() -> None:
    assert YOLO is not None
    assert ort is not None
    assert onnx is not None
    assert _yolo_import_err is None
    assert _ort_import_err is None
    assert _onnx_import_err is None


def test_missing_ultralytics_sets_sentinel_none_and_captures_the_error(monkeypatch: pytest.MonkeyPatch) -> None:
    import src.deps as deps_module  # noqa: PLC0415

    real_import = builtins.__import__
    import_err_msg = "no module named 'ultralytics'"

    def fake_import(
        name: str,
        globals: dict[str, object] | None = None,  # noqa: A002
        locals: dict[str, object] | None = None,  # noqa: A002
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> object:
        if name == "ultralytics":
            raise ImportError(import_err_msg)
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    reloaded = importlib.reload(deps_module)

    try:
        assert reloaded.YOLO is None
        assert isinstance(reloaded._yolo_import_err, ImportError)  # noqa: SLF001
    finally:
        monkeypatch.undo()
        importlib.reload(deps_module)
