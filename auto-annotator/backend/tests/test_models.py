"""Test dataclasses and type safety.

Validates that model classes enforce type safety and immutability where required.
"""

from __future__ import annotations

import pytest

from src.enums import Status
from src.models import ClassInfo, Point, ImageRecord
from src.types import ClassId, ImageId


class TestDomainTypes:
    """Test NewType domain type safety."""

    def test_class_id_type_distinct(self) -> None:
        """Verify ClassId is distinct from int."""
        assert ClassId.__supertype__ is int

    def test_image_id_type_distinct(self) -> None:
        """Verify ImageId is distinct from int."""
        assert ImageId.__supertype__ is int


class TestImmutableModels:
    """Test frozen dataclass immutability."""

    def test_point_immutable(self) -> None:
        """Verify Point dataclass is frozen."""
        point = Point(x=1.0, y=2.0, label=1)
        with pytest.raises(Exception):  # FrozenInstanceError
            point.x = 3.0  # type: ignore[misc]

    def test_class_info_immutable(self) -> None:
        """Verify ClassInfo dataclass is frozen."""
        cls = ClassInfo(id=1, name="red", color="#ff0000")
        with pytest.raises(Exception):  # FrozenInstanceError
            cls.name = "blue"  # type: ignore[misc]

    def test_image_record_immutable(self) -> None:
        """Verify ImageRecord dataclass is frozen."""
        record = ImageRecord(
            id=1,
            path="/test/image.jpg",
            filename="image.jpg",
            status=Status.PENDING,
            updated_at="2026-01-01",
            format=None,
        )
        with pytest.raises(Exception):  # FrozenInstanceError
            record.status = Status.DONE  # type: ignore[misc]


class TestModelSerialization:
    """Test dataclass serialization and representation."""

    def test_point_repr(self) -> None:
        """Verify Point has meaningful repr."""
        point = Point(x=1.5, y=2.5, label=1)
        assert "1.5" in repr(point)
        assert "2.5" in repr(point)

    def test_class_info_equality(self) -> None:
        """Verify ClassInfo equality by value."""
        cls1 = ClassInfo(id=1, name="red", color="#ff0000")
        cls2 = ClassInfo(id=1, name="red", color="#ff0000")
        assert cls1 == cls2

    def test_different_class_info_not_equal(self) -> None:
        """Verify different ClassInfo are not equal."""
        cls1 = ClassInfo(id=1, name="red", color="#ff0000")
        cls2 = ClassInfo(id=2, name="blue", color="#0000ff")
        assert cls1 != cls2
