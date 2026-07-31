"""Unit tests for src.graph's ONNX inspection, using the real GMR export."""

from __future__ import annotations

from pathlib import Path

import pytest

from src.errors import HailoError
from src.graph import inspect

GMR_ONNX = Path(__file__).resolve().parent.parent / "data" / "gmr.onnx"

pytestmark = pytest.mark.skipif(not GMR_ONNX.exists(), reason="data/gmr.onnx fixture not present")


def test_inspect_loads_and_logs_the_real_gmr_graph(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level("INFO"):
        inspect(str(GMR_ONNX))

    joined = "\n".join(caplog.messages)
    assert "Inputs:" in joined
    assert "Outputs:" in joined


def test_inspect_raises_when_file_missing(tmp_path: Path) -> None:
    missing = tmp_path / "does-not-exist.onnx"
    with pytest.raises(HailoError, match="ONNX file not found"):
        inspect(str(missing))
