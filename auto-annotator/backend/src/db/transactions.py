"""src.db.transactions – Transaction management for multi-step database operations.

Ensures file I/O (augmentation, training) operations either fully commit or fully
rollback, preventing database inconsistency when file writes fail or are interrupted.
"""

from __future__ import annotations

import sqlite3
from contextlib import contextmanager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Generator


@contextmanager
def transaction(conn: sqlite3.Connection) -> Generator[None, None, None]:
    """Context manager for explicit transaction handling.

    Commits if the block completes successfully; rolls back if any exception is raised.
    Useful for multi-step operations like: write file → register in database.

    Args:
        conn: SQLite connection object.

    Yields:
        None (connection is ready to execute queries).

    Example:
        with transaction(conn):
            # File I/O operation
            image_path.write_bytes(data)
            # Database operation
            cursor.execute(QUERY_INSERT_IMAGE, (str(image_path), ...))
            # If either fails, both are rolled back
    """
    try:
        yield
        conn.commit()
    except Exception:
        conn.rollback()
        raise


def get_transaction_connection(db_path: str) -> sqlite3.Connection:
    """Create a SQLite connection configured for transaction handling.

    Sets up WAL mode and foreign-key constraints.

    Args:
        db_path: Path to SQLite database file.

    Returns:
        Configured connection ready for transaction context.
    """
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn
