"""src.exceptions – Domain exception hierarchy for the auto-annotator.

All application-specific exceptions inherit from :class:`AutoAnnotatorError`
so callers can catch the base class for broad handling or subclasses for
precise recovery strategies.

Hierarchy::

    AutoAnnotatorError
    ├── InferenceBackendError – Backend-specific inference failure.
    ├── InferenceGPUMemory    – CUDA out-of-memory.
    ├── ModelNotFound         – Model ID not in registry.
    ├── ModelNotAvailable     – Model config does not meet availability rules.
    └── ModelLoadError        – Loading failed (checkpoint missing, etc.).
"""

from __future__ import annotations


class AutoAnnotatorError(Exception):
    """Base class for all auto-annotator domain exceptions."""


class InferenceBackendError(AutoAnnotatorError):
    """Raised when a backend-specific inference failure occurs."""


class InferenceGPUMemory(AutoAnnotatorError):
    """Raised when CUDA runs out of memory during inference."""


class ModelNotFound(AutoAnnotatorError):
    """Raised when a model ID is not found in the registry."""


class ModelNotAvailable(AutoAnnotatorError):
    """Raised when a model does not meet availability rules."""


class ModelLoadError(AutoAnnotatorError):
    """Raised when loading a model fails."""
