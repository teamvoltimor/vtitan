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


class OutlineMode(StrEnum):
    """Annotation contour outline colour modes shown in the Settings accordion.

    Because this is a StrEnum the values compare equal to plain strings,
    so ``"Class color" == OutlineMode.CLASS_COLOR`` is ``True``.
    """

    CLASS_COLOR = "Class color"
    """Draw the outline in the annotation class's own colour."""

    HIGH_CONTRAST = "High contrast"
    """Automatically choose black or white to maximise contrast with the background."""

    BLACK = "Black"
    """Always draw the outline in pure black."""

    WHITE = "White"
    """Always draw the outline in pure white."""
