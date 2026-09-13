"""Vision-layer exceptions."""

from __future__ import annotations

from shared.domain.exceptions import PlatformError


class VisionError(PlatformError):
    """Base exception for vision detector and factory failures."""


class UnknownVisionBackendError(VisionError):
    """Raised when ``create_detector`` is given an unrecognised backend."""
