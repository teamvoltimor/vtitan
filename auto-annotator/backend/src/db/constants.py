"""src.db.constants – Database-specific string constants.

All column names, status labels, status icon glyphs, status lookup tables, and
query-result key aliases are centralised here.  Higher-level modules (handlers,
UI) should access these values through the typed dataclasses returned by
src.db.core rather than importing raw constants directly.
"""

from __future__ import annotations

from src.enums import Status

# Column name constants – match the SQLite schema exactly.
# Used when constructing queries or reading sqlite3.Row objects.

COL_ID: str = "id"
"""Primary key column: ``id``."""

COL_PATH: str = "path"
"""Image file-path column: ``path``."""

COL_STATUS: str = "status"
"""Image processing-status integer column: ``status``."""

COL_FORMAT_USED: str = "format_used"
"""Export format column: ``format_used``."""

COL_NAME: str = "name"
"""Class name column: ``name``."""

COL_COLOR: str = "color"
"""Class hex-colour column: ``color``."""

COL_CREATED_AT: str = "created_at"
"""Row creation timestamp column: ``created_at``."""

COL_UPDATED_AT: str = "updated_at"
"""Last-modified timestamp column: ``updated_at``."""

COL_PARENT_ID: str = "parent_id"
"""Parent image FK column: ``parent_id`` (NULL for original images)."""

# SQL aggregate alias used in QUERY_SELECT_STATUS_COUNTS.
# The query writes "COUNT(*) AS cnt" so rows are accessed with this key.

SQL_ALIAS_COUNT: str = "cnt"
"""Column alias for COUNT(*) aggregates in status-count queries."""

# Logical key names for computed fields in BrowseRow.
# get_all_images() derives these from raw DB columns; they are not DB columns.

BROWSE_KEY_FILENAME: str = "filename"
"""Logical key for the image basename derived from the path column."""

BROWSE_KEY_FORMAT: str = "format"
"""Logical key for the export format string (NULL coerced to empty string)."""

# Human-readable status label strings shown in the browse view.

STATUS_LABEL_PENDING: str = "pending"
"""Human-readable label for images with Status.PENDING."""

STATUS_LABEL_DONE: str = "done"
"""Human-readable label for images with Status.DONE."""

STATUS_LABEL_SKIPPED: str = "skipped"
"""Human-readable label for images with Status.SKIPPED."""

# Status icon glyphs displayed next to image entries in the browse modal.

STATUS_ICON_PENDING: str = "\u23f3"
"""Hourglass glyph (⏳) shown next to pending images."""

STATUS_ICON_DONE: str = "\u2713"
"""Check-mark glyph (✓) shown next to done images."""

STATUS_ICON_SKIPPED: str = "\u23ed"
"""Skip-forward glyph (⏭) shown next to skipped images."""

# Lookup tables mapping integer Status codes to display strings.
# Populated from the STATUS_LABEL_* and STATUS_ICON_* constants above.

STATUS_NAMES: dict[int, str] = {
    Status.PENDING: STATUS_LABEL_PENDING,
    Status.DONE: STATUS_LABEL_DONE,
    Status.SKIPPED: STATUS_LABEL_SKIPPED,
}
"""Map from integer Status code to its human-readable label string."""

STATUS_ICONS: dict[int, str] = {
    Status.PENDING: STATUS_ICON_PENDING,
    Status.DONE: STATUS_ICON_DONE,
    Status.SKIPPED: STATUS_ICON_SKIPPED,
}
"""Map from integer Status code to its display glyph character."""
