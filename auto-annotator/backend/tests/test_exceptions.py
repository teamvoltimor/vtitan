"""Test exception hierarchy and error handling.

Validates that exceptions are properly typed, chained, and used consistently.
"""

from __future__ import annotations

import pytest

from src.exceptions import (
    AutoAnnotatorError,
    DBError,
    DBTransactionError,
    InferenceError,
    InferenceGPUMemory,
    InferenceModelUnavailable,
    ModelError,
    ModelNotFound,
    ModelNotAvailable,
    ModelLoadError,
    ModelServerError,
    NavigationError,
)


class TestExceptionHierarchy:
    """Test exception inheritance structure."""

    def test_all_exceptions_inherit_from_base(self) -> None:
        """Verify all domain exceptions inherit from AutoAnnotatorError."""
        exceptions = [
            DBError,
            DBTransactionError,
            InferenceError,
            InferenceGPUMemory,
            InferenceModelUnavailable,
            ModelError,
            ModelNotFound,
            ModelLoadError,
            ModelServerError,
            NavigationError,
        ]
        for exc_class in exceptions:
            assert issubclass(exc_class, AutoAnnotatorError)

    def test_db_error_hierarchy(self) -> None:
        """Verify DBError subclasses."""
        assert issubclass(DBTransactionError, DBError)

    def test_inference_error_hierarchy(self) -> None:
        """Verify InferenceError subclasses."""
        assert issubclass(InferenceGPUMemory, InferenceError)
        assert issubclass(InferenceModelUnavailable, InferenceError)

    def test_model_error_hierarchy(self) -> None:
        """Verify ModelError subclasses."""
        assert issubclass(ModelNotFound, ModelError)
        assert issubclass(ModelNotAvailable, ModelError)
        assert issubclass(ModelLoadError, ModelError)


class TestExceptionRaising:
    """Test exception instantiation and raising."""

    def test_raise_model_not_found(self) -> None:
        """Verify ModelNotFound can be raised and caught."""
        with pytest.raises(ModelNotFound):
            raise ModelNotFound("Model 'sam3' not in registry")

    def test_raise_inference_gpu_memory(self) -> None:
        """Verify InferenceGPUMemory can be raised and caught."""
        with pytest.raises(InferenceGPUMemory):
            raise InferenceGPUMemory("CUDA out of memory")

    def test_exception_chaining(self) -> None:
        """Verify exception chaining preserves original error."""
        original_error = ValueError("Original error")
        try:
            try:
                raise original_error
            except ValueError as e:
                raise ModelLoadError(f"Failed to load model: {e}") from e
        except ModelLoadError as e:
            assert e.__cause__ is original_error

    def test_catch_base_exception(self) -> None:
        """Verify specific exceptions can be caught by base class."""
        with pytest.raises(AutoAnnotatorError):
            raise InferenceGPUMemory("CUDA OOM")

    def test_exception_message_preserved(self) -> None:
        """Verify exception message is preserved."""
        msg = "Model checkpoint not found at /path/to/model.pt"
        with pytest.raises(ModelNotAvailable, match="checkpoint"):
            raise ModelNotAvailable(msg)
