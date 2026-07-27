"""Shared lazy imports for optional Hailo dependencies.

Each dependency is wrapped in a ``try/except ImportError`` so that
consumers can use the sentinel variable with :func:`src.errors.require_dep`
to produce a clear error message when the package is missing, rather than
raising a cryptic ``NameError`` at the call site.
"""

from __future__ import annotations

_yolo_import_err: ImportError | None = None
try:
    from ultralytics import YOLO
except ImportError as _exc:
    YOLO = None  # type: ignore[assignment, misc]
    _yolo_import_err = _exc

_ort_import_err: ImportError | None = None
try:
    import onnxruntime as ort
except ImportError as _exc:
    ort = None
    _ort_import_err = _exc

_onnx_import_err: ImportError | None = None
try:
    import onnx
except ImportError as _exc:
    onnx = None  # type: ignore[assignment]
    _onnx_import_err = _exc
