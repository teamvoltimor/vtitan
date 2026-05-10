"""Shared pytest fixtures and configuration for auto-annotator tests.

Provides isolated database fixtures, mock clients, and FastAPI test client.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Generator

import pytest
from fastapi.testclient import TestClient

pytest_plugins = ("pytest_asyncio",)


@pytest.fixture
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


@pytest.fixture
def test_db(temp_db_path: Path) -> Generator[Path, None, None]:
    """Initialize and provide test database.

    Args:
        temp_db_path: Isolated database path fixture.

    Yields:
        Path to initialized test database.
    """
    from src.db import core

    core.init_db()
    yield temp_db_path


@pytest.fixture
def api_client(test_db: Path) -> Generator[TestClient, None, None]:
    """Provide FastAPI test client with initialized database.

    Args:
        test_db: Initialized test database fixture.

    Yields:
        FastAPI TestClient ready for endpoint testing.
    """
    from src.api.app import app

    with TestClient(app) as client:
        yield client


@pytest.fixture
def sample_image_bytes() -> bytes:
    """Provide sample image bytes for upload testing.

    Returns:
        PNG image data (1×1 white pixel).
    """
    import io

    from PIL import Image

    img = Image.new("RGB", (1, 1), color="white")
    bio = io.BytesIO()
    img.save(bio, format="PNG")
    return bio.getvalue()
