"""src.enums – Application-wide enumerations.

Centralises all enum types so they can be imported without pulling in the
heavier modules that depend on them.
"""

from enum import IntEnum, StrEnum


class Status(IntEnum):
    """Image annotation status codes stored as integers in SQLite."""

    PENDING = 0
    DONE = 1
    SKIPPED = 2


class ExportFormat(StrEnum):
    """YOLO label export format identifiers."""

    SEG = "seg"
    """Segmentation format: one polygon per annotation."""

    DET = "det"
    """Detection format: one bounding box per annotation."""
