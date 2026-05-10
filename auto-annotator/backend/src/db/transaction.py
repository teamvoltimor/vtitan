"""src.db.transaction – Atomic transaction support with rollback on error.

Enables multi-step operations (image upload + label write + DB insert) to
succeed or fail as a unit, preventing partial state.
"""

from __future__ import annotations

import sqlite3
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING

from src.constants import DB_PATH
from src.exceptions import DBTransactionError
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Generator

logger = get_logger(__name__)


class Transaction:
    """Scoped database transaction with automatic rollback on error.

    Usage:
        with Transaction() as txn:
            txn.execute("INSERT INTO images ...")
            write_label_file(...)
            if error_occurs:
                raise SomeError()  # triggers rollback
    """

    def __init__(self):
        """Initialize transaction state (connection not opened yet)."""
        self.conn: sqlite3.Connection | None = None
        self._in_transaction = False

    def __enter__(self) -> Transaction:
        """Open connection and begin transaction."""
        try:
            self.conn = sqlite3.connect(DB_PATH)
            self.conn.row_factory = sqlite3.Row
            self.conn.execute("PRAGMA foreign_keys = ON")
            self.conn.execute("PRAGMA journal_mode = WAL")
            self.conn.execute("BEGIN IMMEDIATE")
            self._in_transaction = True
            return self
        except sqlite3.Error as e:
            raise DBTransactionError(f"Failed to begin transaction: {e}") from e

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Commit on success, rollback on error."""
        if not self.conn:
            return False

        try:
            if exc_type is not None:
                self.conn.execute("ROLLBACK")
                logger.info("Transaction rolled back", extra={"_extra": {"exc_type": exc_type.__name__}})
                return False

            self.conn.execute("COMMIT")
            logger.debug("Transaction committed")
            return False
        except sqlite3.Error as e:
            logger.error("Commit/rollback failed", extra={"_extra": {"error": str(e)}})
            raise DBTransactionError(f"Failed to commit: {e}") from e
        finally:
            self.conn.close()
            self._in_transaction = False

    def execute(self, query: str, params: tuple = ()) -> sqlite3.Cursor:
        """Execute a query within the transaction.

        Args:
            query: SQL query string.
            params: Optional parameters tuple.

        Returns:
            Cursor with results.

        Raises:
            DBTransactionError: If not in a transaction context.
        """
        if not self._in_transaction or self.conn is None:
            msg = "execute() called outside transaction context"
            raise DBTransactionError(msg)

        try:
            return self.conn.execute(query, params)
        except sqlite3.Error as e:
            raise DBTransactionError(f"Query execution failed: {e}") from e

    def executemany(self, query: str, params: list[tuple]) -> sqlite3.Cursor:
        """Execute multiple queries within the transaction."""
        if not self._in_transaction or self.conn is None:
            msg = "executemany() called outside transaction context"
            raise DBTransactionError(msg)

        try:
            return self.conn.executemany(query, params)
        except sqlite3.Error as e:
            raise DBTransactionError(f"Batch execution failed: {e}") from e


@contextmanager
def txn() -> Generator[Transaction, None, None]:
    """Context manager factory for transactions.

    Usage:
        with txn() as t:
            t.execute(...)
    """
    transaction = Transaction()
    with transaction as t:
        yield t
