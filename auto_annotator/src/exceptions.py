"""src.exceptions – Domain exception hierarchy for the auto-annotator.

All application-specific exceptions inherit from :class:`AutoAnnotatorError`
so callers can catch the base class for broad handling or subclasses for
precise recovery strategies.

Hierarchy::

    AutoAnnotatorError
    ├── DBError          – SQLite or data-layer failures.
    ├── InferenceError   – SAM inference pipeline failures (OOM, bad output, etc.).
    ├── ModelServerError – TCP model server communication failures.
    └── NavigationError  – Image navigation or file I/O failures.
"""

from __future__ import annotations


class AutoAnnotatorError(Exception):
    """Base class for all auto-annotator domain exceptions."""


class DBError(AutoAnnotatorError):
    """Raised when a SQLite operation or data-layer invariant fails.

    Args:
        message: Human-readable description of the failure.
    """


class InferenceError(AutoAnnotatorError):
    """Raised when SAM inference produces an invalid or unrecoverable result.

    Args:
        message: Human-readable description of the failure.
    """


class ModelServerError(AutoAnnotatorError):
    """Raised when communication with the model-server TCP daemon fails.

    Args:
        message: Human-readable description of the failure.
    """


class NavigationError(AutoAnnotatorError):
    """Raised when image navigation, file reading, or label saving fails.

    Args:
        message: Human-readable description of the failure.
    """
