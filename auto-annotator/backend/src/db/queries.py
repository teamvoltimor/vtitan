"""src.db.queries – All SQL query strings used by the database layer.

Every SQL string is a named constant so that:
  * Magic strings are eliminated from the implementation module.
  * All queries can be reviewed, tested, and modified in one place.
  * IDEs can locate every usage via the constant name.
"""

# Connection configuration pragmas applied on every connection open.

QUERY_PRAGMA_WAL: str = "PRAGMA journal_mode=WAL"
"""Enable Write-Ahead Logging for better concurrent read performance."""

QUERY_PRAGMA_FOREIGN_KEYS: str = "PRAGMA foreign_keys=ON"
"""Enable SQLite foreign-key constraint enforcement."""

# Classes table queries.

QUERY_COUNT_CLASSES: str = "SELECT COUNT(*) FROM classes"
"""Count the total number of rows in the classes table."""

QUERY_INSERT_CLASS_DEFAULT_IGNORE: str = "INSERT OR IGNORE INTO classes (name, color) VALUES (?, ?)"
"""Batch-insert default classes on first run, ignoring name conflicts."""

QUERY_UPSERT_CLASS: str = """
INSERT INTO classes (name, color) VALUES (?, ?)
ON CONFLICT(name) DO UPDATE SET color=excluded.color
"""
"""Insert a new class or update its colour if the name already exists."""

QUERY_SELECT_CLASS_ID_BY_NAME: str = "SELECT id FROM classes WHERE name=?"
"""Look up the integer id of a class by its unique name (used after upsert)."""

QUERY_SELECT_ALL_CLASSES: str = "SELECT id, name, color FROM classes ORDER BY id ASC"
"""Retrieve all classes ordered by creation id, ascending."""

# Images table – insert and path queries.

QUERY_SELECT_ALL_IMAGE_PATHS: str = "SELECT path FROM images WHERE deleted_at IS NULL"
"""Retrieve all registered image paths (used for duplicate detection on scan)."""

QUERY_INSERT_IMAGE_IGNORE: str = "INSERT OR IGNORE INTO images (path) VALUES (?)"
"""Insert a new image path, silently ignoring the row if path already exists."""

# Images table - select queries.

QUERY_SELECT_NEXT_PENDING_AFTER_ID: str = "SELECT * FROM images WHERE status=? AND id>? AND deleted_at IS NULL ORDER BY id ASC LIMIT 1"
"""Find the first pending image with an id greater than the given value."""

QUERY_SELECT_FIRST_PENDING: str = "SELECT * FROM images WHERE status=? AND deleted_at IS NULL ORDER BY id ASC LIMIT 1"
"""Find the first pending image (used as wrap-around fallback)."""

QUERY_SELECT_PREV_IMAGE: str = "SELECT * FROM images WHERE id<? AND deleted_at IS NULL ORDER BY id DESC LIMIT 1"
"""Find the image with the highest id strictly less than the given value."""

QUERY_SELECT_IMAGE_BY_ID: str = "SELECT * FROM images WHERE id=? AND deleted_at IS NULL"
"""Fetch a single full image row by its primary key."""

QUERY_SELECT_STATUS_COUNTS: str = "SELECT status, COUNT(*) AS cnt FROM images WHERE deleted_at IS NULL GROUP BY status"
"""Aggregate image counts grouped by processing status."""

QUERY_SELECT_ALL_IMAGES_FOR_BROWSE: str = "SELECT id, path, status, format_used, updated_at FROM images WHERE deleted_at IS NULL ORDER BY id ASC"
"""Retrieve all images with the columns needed for the browse view, ordered by id."""

# Images table - update queries.

QUERY_UPDATE_IMAGE_DONE: str = "UPDATE images SET status=?, format_used=?, updated_at=? WHERE id=?"
"""Mark an image as done and record the export format and timestamp."""

QUERY_UPDATE_IMAGE_SKIPPED: str = "UPDATE images SET status=?, updated_at=? WHERE id=?"
"""Mark an image as skipped and record the timestamp."""

QUERY_DELETE_IMAGE: str = "UPDATE images SET deleted_at=? WHERE id=?"
"""Delete an image row by setting the deleted_at timestamp (soft delete)."""

QUERY_INSERT_AUGMENTED_IMAGE: str = (
    "INSERT OR IGNORE INTO images (path, status, format_used, parent_id, updated_at) VALUES (?, ?, ?, ?, ?)"
)
"""Insert an augmented image row with a parent reference."""

QUERY_SELECT_DONE_ORIGINALS: str = (
    "SELECT id, path, status, format_used, parent_id, updated_at FROM images "
    "WHERE status = 1 AND parent_id IS NULL AND deleted_at IS NULL ORDER BY id ASC"
)
"""All done images that are original (no parent)."""

QUERY_SELECT_CHILDREN_BY_PARENT: str = (
    "SELECT id, path, status, format_used, parent_id, updated_at FROM images WHERE parent_id = ? AND deleted_at IS NULL"
)
"""All augmented copies of a given parent image."""

QUERY_COUNT_CHILDREN_BY_PARENT: str = "SELECT COUNT(*) FROM images WHERE parent_id = ? AND deleted_at IS NULL"
"""Count augmented copies for a parent image."""

QUERY_SELECT_ALL_IMAGES_GROUPED: str = (
    "SELECT i.id, i.path, i.status, i.format_used, i.updated_at, "
    "COUNT(c.id) as aug_count "
    "FROM images i LEFT JOIN images c ON c.parent_id = i.id AND c.deleted_at IS NULL "
    "WHERE i.parent_id IS NULL AND i.deleted_at IS NULL "
    "GROUP BY i.id ORDER BY i.id ASC"
)
"""Parent images with augmentation count (for grouped gallery)."""
