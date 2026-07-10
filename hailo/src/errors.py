"""Domain exceptions and dependency guards for the Hailo YOLO pipeline."""

from __future__ import annotations


class HailoError(Exception):
    """Base error for the Hailo pipeline."""


class ModelNotFoundError(HailoError):
    """Raised when a requested model file does not exist on disk."""


class CalibrationDataError(HailoError):
    """Raised when calibration data is missing or malformed."""


class CompileError(HailoError):
    """Raised when the Hailo DFC fails to compile a model."""


def require_dep(module: object | None, package_name: str, *, cause: BaseException | None = None) -> None:
    """Raise HailoError if an optional dependency is not installed.

    Args:
        module: The imported module object, or None if import failed.
        package_name: Name of the package for the error message.
        cause: The ``ImportError`` caught at import time, if any. Chained onto
            the raised ``HailoError`` so the traceback shows *why* the import
            failed (missing package vs. a broken transitive import) instead of
            losing that context.

    Raises:
        HailoError: If module is None.
    """
    if module is None:
        msg = f"{package_name} is not installed. Run: uv add {package_name}"
        raise HailoError(msg) from cause
