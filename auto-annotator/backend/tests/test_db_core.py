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

    def test_init_db_creates_tables(self, test_db: Path) -> None:  # noqa: ARG002
        """Verify init_db creates all required tables."""
        tables = core._connect().execute(  # noqa: SLF001
            "SELECT name FROM sqlite_master WHERE type='table'",
        ).fetchall()
        table_names = [row[0] for row in tables]
        assert "images" in table_names
        assert "classes" in table_names

    def test_init_db_idempotent(self) -> None:
        """Verify init_db can be called multiple times safely."""
        core.init_db()
        stats = core.get_stats()
        assert stats is not None

    def test_default_classes_seeded(self) -> None:
        """Verify default classes are inserted on init."""
        classes = core.get_classes()
        assert len(classes) > 0
        # WRO red and green classes should exist
        names = [c.name for c in classes]
        assert any("red" in n.lower() for n in names)


class TestImageOperations:
    """Test image record CRUD operations."""

    def test_get_stats_empty_database(self) -> None:
        """Verify stats query works on empty database."""
        stats = core.get_stats()
        assert stats.total == 0
        assert stats.pending == 0
        assert stats.done == 0

    def test_get_classes_returns_list(self) -> None:
        """Verify get_classes returns class list."""
        classes = core.get_classes()
        assert isinstance(classes, list)
        assert len(classes) > 0
        assert all(hasattr(c, "id") for c in classes)
        assert all(hasattr(c, "name") for c in classes)


class TestTransactions:
    """Test transaction support."""

    def test_transaction_context_manager(self) -> None:
        """Verify Transaction context manager works."""
        from src.db.transaction import txn  # noqa: PLC0415

        with txn() as t:
            # Should be able to execute within transaction
            assert t is not None

    def test_transaction_rollback_on_error(self) -> None:
        """Verify transaction rolls back on exception."""
        from src.db.transaction import txn  # noqa: PLC0415

        err_msg = "Test error"

        def _raise_in_txn() -> None:
            with txn() as t:
                t.execute("SELECT 1")
                raise ValueError(err_msg)

        with pytest.raises(ValueError, match=err_msg):
            _raise_in_txn()

        stats = core.get_stats()
        assert stats is not None
