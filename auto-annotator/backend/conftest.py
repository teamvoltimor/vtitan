"""Shared pytest fixtures and configuration for auto-annotator tests.

Provides isolated database fixtures, mock clients, and FastAPI test client.
"""

from __future__ import annotations

import io
import os
from typing import TYPE_CHECKING

import pytest
from fastapi.testclient import TestClient
from PIL import Image

from src.api.app import app
from src.db import core as db_core

if TYPE_CHECKING:
    from collections.abc import Generator
    from pathlib import Path

pytest_plugins = ("pytest_asyncio",)


@pytest.fixture()
def temp_db_path(tmp_path: Path) -> Generator[Path, None, None]:
    """Provide isolated test database path.

    Args:
        tmp_path: pytest temporary directory fixture.

    Yields:
        Path to test database file.
    """
    db_path = tmp_path / "test.db"
    old_db_path = os.environ.get("DB_PATH")
    os.environ["DB_PATH"] = str(db_path)
    yield db_path
    # Restore old environment
    if old_db_path:
        os.environ["DB_PATH"] = old_db_path
    else:
        os.environ.pop("DB_PATH", None)


@pytest.fixture()
def test_db(temp_db_path: Path) -> Path:
    """Initialize and provide test database.

    Args:
        temp_db_path: Isolated database path fixture.

    Returns:
        Path to initialized test database.
    """
    db_core.init_db()
    return temp_db_path


@pytest.fixture()
def api_client() -> Generator[TestClient, None, None]:
    """Provide FastAPI test client with initialized database.

    Yields:
        FastAPI TestClient ready for endpoint testing.
    """
    with TestClient(app) as client:
        yield client


@pytest.fixture()
def sample_image_bytes() -> bytes:
    """Provide sample image bytes for upload testing.

    Returns:
        PNG image data (1×1 white pixel).
    """
    img = Image.new("RGB", (1, 1), color="white")
    bio = io.BytesIO()
    img.save(bio, format="PNG")
    return bio.getvalue()
