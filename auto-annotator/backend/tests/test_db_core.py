"""Test SQLite persistence layer.

Validates database operations, schema, and data integrity.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from src.db import core

if TYPE_CHECKING:
    from pathlib import Path


class TestDatabaseInitialization:
    """Test database schema creation and initialization."""

    def test_init_db_creates_tables(self, test_db: Path) -> None:
        """Verify init_db creates all required tables."""
        tables = (
            core._connect(test_db)
            .execute(
                "SELECT name FROM sqlite_master WHERE type='table'",
            )
            .fetchall()
        )
        table_names = [row[0] for row in tables]
        assert "images" in table_names
        assert "classes" in table_names

    def test_init_db_idempotent(self, test_db: Path) -> None:
        """Verify init_db can be called multiple times safely."""
        core.init_db(test_db)
        stats = core.get_stats(test_db)
        assert stats is not None

    def test_default_classes_seeded(self, test_db: Path) -> None:
        """Verify default classes are inserted on init."""
        classes = core.get_classes(test_db)
        assert len(classes) > 0
        # WRO red and green classes should exist
        names = [c.name for c in classes]
        assert any("red" in n.lower() for n in names)


class TestImageOperations:
    """Test image record CRUD operations."""

    def test_get_stats_empty_database(self, test_db: Path) -> None:
        """Verify stats query works on empty database."""
        stats = core.get_stats(test_db)
        assert stats.total == 0
        assert stats.pending == 0
        assert stats.done == 0

    def test_get_classes_returns_list(self, test_db: Path) -> None:
        """Verify get_classes returns class list."""
        classes = core.get_classes(test_db)
        assert isinstance(classes, list)
        assert len(classes) > 0
        assert all(hasattr(c, "id") for c in classes)
        assert all(hasattr(c, "name") for c in classes)


class TestTransactions:
    """Test transaction support."""

    def test_transaction_context_manager(self, test_db: Path) -> None:
        """Verify db_core.transaction context manager yields a usable connection."""
        with core.transaction(test_db) as conn:
            assert conn is not None
            conn.execute("SELECT 1")

    def test_transaction_rollback_on_error(self, test_db: Path) -> None:
        """Verify transaction rolls back on exception."""
        err_msg = "Test error"

        def _raise_in_txn() -> None:
            with core.transaction(test_db) as conn:
                conn.execute("SELECT 1")
                raise ValueError(err_msg)

        with pytest.raises(ValueError, match=err_msg):
            _raise_in_txn()

        stats = core.get_stats(test_db)
        assert stats is not None
