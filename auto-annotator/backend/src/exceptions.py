"""src.exceptions – Domain exception hierarchy for the auto-annotator.

All application-specific exceptions inherit from :class:`AutoAnnotatorError`
so callers can catch the base class for broad handling or subclasses for
precise recovery strategies.

Hierarchy::

    AutoAnnotatorError
    ├── DBError                  – SQLite or data-layer failures.
    │   ├── DBTransactionError   – Transaction-level violations or rollbacks.
    │   └── DBIntegrityError     – Constraint violations.
    ├── InferenceError           – SAM inference pipeline failures.
    │   ├── InferenceGPUMemory   – CUDA out-of-memory.
    │   ├── InferenceModelUnavailable – Model not loaded.
    │   ├── InferenceBadInput    – Invalid input (no points, bad image, etc.).
    │   └── InferenceBackendError – Backend-specific failure.
    ├── ModelError               – Model loading, registry, or availability failures.
    │   ├── ModelNotFound        – Model ID not in registry.
    │   ├── ModelNotAvailable    – Model config does not meet availability rules.
    │   └── ModelLoadError       – Loading failed (checkpoint missing, etc.).
    ├── ModelServerError         – TCP model server communication failures.
    │   └── ModelServerConnectError – Connection or socket failure.
    └── NavigationError          – Image navigation or file I/O failures.
        └── FileIOError         – Reading or writing files.
"""

from __future__ import annotations


class AutoAnnotatorError(Exception):
    """Base class for all auto-annotator domain exceptions."""


class DBError(AutoAnnotatorError):
    """Raised when a SQLite operation or data-layer invariant fails."""


class DBTransactionError(DBError):
    """Raised when a transaction is violated or rolled back."""


class DBIntegrityError(DBError):
    """Raised when a data constraint violation occurs."""


class InferenceError(AutoAnnotatorError):
    """Raised when SAM inference produces an invalid or unrecoverable result."""


class InferenceGPUMemory(InferenceError):
    """Raised when CUDA runs out of memory during inference."""


class InferenceModelUnavailable(InferenceError):
    """Raised when the required model is not loaded."""


class InferenceBadInput(InferenceError):
    """Raised when input is invalid (no points, bad image, etc.)."""


class InferenceBackendError(InferenceError):
    """Raised when a backend-specific inference failure occurs."""


class ModelError(AutoAnnotatorError):
    """Raised when model loading, registry, or availability checks fail."""


class ModelNotFound(ModelError):
    """Raised when a model ID is not found in the registry."""


class ModelNotAvailable(ModelError):
    """Raised when a model does not meet availability rules."""


class ModelLoadError(ModelError):
    """Raised when loading a model fails."""


class ModelServerError(AutoAnnotatorError):
    """Raised when communication with the model-server TCP daemon fails."""


class ModelServerConnectError(ModelServerError):
    """Raised when establishing a connection to the model server fails."""


class NavigationError(AutoAnnotatorError):
    """Raised when image navigation, file reading, or label saving fails."""


class FileIOError(NavigationError):
    """Raised when reading or writing files fails."""
