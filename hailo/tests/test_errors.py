"""Unit tests for src.errors' dependency guard."""

from __future__ import annotations

import pytest

from src.errors import HailoError, require_dep


def test_require_dep_passes_for_present_module() -> None:
    require_dep(object(), "some-package")  # must not raise


def test_require_dep_raises_for_missing_module() -> None:
    with pytest.raises(HailoError, match="not installed"):
        require_dep(None, "fiftyone")


def test_require_dep_chains_the_real_import_error() -> None:
    original = ImportError("no module named 'fiftyone'")
    with pytest.raises(HailoError) as exc_info:
        require_dep(None, "fiftyone", cause=original)
    assert exc_info.value.__cause__ is original
