"""Entry point that forwards the package path to the backend implementation."""

from __future__ import annotations

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
BACKEND_SRC = PACKAGE_ROOT.parent / "backend" / "src"

try:
    BACKEND_SRC_PATH = str(BACKEND_SRC)
    if BACKEND_SRC_PATH not in __path__:
        __path__.append(BACKEND_SRC_PATH)
except Exception:
    raise ImportError("Unable to locate backend/src path for the src package")
