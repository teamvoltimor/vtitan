"""Unit tests for src.export's YOLO -> ONNX export orchestration.

Runs the real export path against a mocked ``YOLO`` rather than the actual
ultralytics export -- that call is slow and would overwrite the checked-in
``data/gmr.onnx`` fixture as a side effect. The mock verifies the logic
(checkpoint lookup, opset/extras resolution, move-on-mismatch) independent of
the real export.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from src.config import ExportConfig
from src.enums import ExportExtra, Task
from src.errors import ModelNotFoundError
from src.export import run
from src.registry import ModelEntry, ModelName

if TYPE_CHECKING:
    from pathlib import Path


class _FakeYOLO:
    """Records the export call and writes a fake ONNX file at *produced_path*."""

    last_export_kwargs: dict[str, object] | None = None

    def __init__(self, produced_path: Path) -> None:
        self._produced_path = produced_path

    def export(self, **kwargs: object) -> str:
        _FakeYOLO.last_export_kwargs = kwargs
        self._produced_path.parent.mkdir(parents=True, exist_ok=True)
        self._produced_path.write_bytes(b"fake-onnx")
        return str(self._produced_path)


def _entry(
    *,
    pt_file: str,
    onnx_file: str = "data/unused.onnx",
    opset: int = 13,
    export_extras: tuple[tuple[ExportExtra, bool], ...] = (),
) -> ModelEntry:
    return ModelEntry(
        pt_file=pt_file,
        onnx_file=onnx_file,
        task=Task.DETECT,
        opset=opset,
        export_extras=export_extras,
    )


def test_run_raises_when_checkpoint_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("src.export.YOLO", object())
    monkeypatch.setattr("src.export.get_entry", lambda _model: _entry(pt_file="does/not/exist.pt"))

    with pytest.raises(ModelNotFoundError, match="Checkpoint not found"):
        run(ExportConfig(model=ModelName("gmr"), imgsz=640, opset=None, no_simplify=False))


def test_run_exports_and_moves_the_produced_onnx(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    checkpoint = tmp_path / "best.pt"
    checkpoint.write_bytes(b"fake-checkpoint")
    produced = tmp_path / "best.onnx"
    destination = tmp_path / "out" / "gmr.onnx"

    entry = _entry(pt_file=str(checkpoint), onnx_file=str(destination), opset=11)
    monkeypatch.setattr("src.export.get_entry", lambda _model: entry)
    monkeypatch.setattr("src.export.YOLO", lambda _path: _FakeYOLO(produced))

    run(ExportConfig(model=ModelName("gmr"), imgsz=640, opset=None, no_simplify=False))

    assert destination.exists()
    assert not produced.exists()
    assert _FakeYOLO.last_export_kwargs is not None
    assert _FakeYOLO.last_export_kwargs["opset"] == 11
    assert _FakeYOLO.last_export_kwargs["imgsz"] == 640


def test_run_prefers_config_opset_over_registry_default(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    checkpoint = tmp_path / "best.pt"
    checkpoint.write_bytes(b"fake-checkpoint")
    produced = tmp_path / "best.onnx"
    destination = tmp_path / "gmr.onnx"

    entry = _entry(pt_file=str(checkpoint), onnx_file=str(destination), opset=11)
    monkeypatch.setattr("src.export.get_entry", lambda _model: entry)
    monkeypatch.setattr("src.export.YOLO", lambda _path: _FakeYOLO(produced))

    run(ExportConfig(model=ModelName("gmr"), imgsz=640, opset=17, no_simplify=False))

    assert _FakeYOLO.last_export_kwargs is not None
    assert _FakeYOLO.last_export_kwargs["opset"] == 17


def test_run_strips_simplify_when_no_simplify_is_set(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    checkpoint = tmp_path / "best.pt"
    checkpoint.write_bytes(b"fake-checkpoint")
    produced = tmp_path / "best.onnx"
    destination = tmp_path / "gmr.onnx"

    entry = _entry(
        pt_file=str(checkpoint),
        onnx_file=str(destination),
        opset=11,
        export_extras=((ExportExtra.SIMPLIFY, True), (ExportExtra.NMS, False)),
    )
    monkeypatch.setattr("src.export.get_entry", lambda _model: entry)
    monkeypatch.setattr("src.export.YOLO", lambda _path: _FakeYOLO(produced))

    run(ExportConfig(model=ModelName("gmr"), imgsz=640, opset=None, no_simplify=True))

    assert _FakeYOLO.last_export_kwargs is not None
    assert "simplify" not in _FakeYOLO.last_export_kwargs
    assert _FakeYOLO.last_export_kwargs["nms"] is False
