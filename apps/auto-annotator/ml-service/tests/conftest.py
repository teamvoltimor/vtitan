"""Shared pytest fixtures and configuration for ml-service tests."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.fixture
def sample_mask_empty() -> np.ndarray:
    """Empty mask (all False)."""
    return np.zeros((100, 100), dtype=bool)


@pytest.fixture
def sample_mask_circle() -> np.ndarray:
    """Small circular mask for contour detection."""
    mask = np.zeros((100, 100), dtype=bool)
    y, x = np.ogrid[:100, :100]
    circle = (x - 50) ** 2 + (y - 50) ** 2 <= 20**2
    mask[circle] = True
    return mask


@pytest.fixture
def sample_mask_rectangle() -> np.ndarray:
    """Rectangular mask."""
    mask = np.zeros((200, 300), dtype=bool)
    mask[50:150, 75:225] = True
    return mask


@pytest.fixture
def sample_mask_small() -> np.ndarray:
    """Mask smaller than GEOMETRY_MINIMUM_CONTOUR_AREA."""
    mask = np.zeros((100, 100), dtype=bool)
    mask[50:52, 50:52] = True  # 4 pixels
    return mask
