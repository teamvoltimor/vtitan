"""src.enums – Application-wide enumerations."""

from enum import Enum


class Status(int, Enum):
    """Image annotation status codes (stored in SQLite as integers)."""

    PENDING = 0
    DONE = 1
    SKIPPED = 2


class ExportFormat(str, Enum):
    """YOLO export formats."""

    SEG = "seg"
    DET = "det"
