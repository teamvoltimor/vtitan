"""WRO 2026 simulation enumerations.

Defines strongly-typed domain enums for track sections and robot direction.
Using enums instead of bare strings eliminates typo-prone comparisons and
provides IDE autocomplete throughout the codebase.
"""

from __future__ import annotations

from enum import Enum


class Section(Enum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> Section:
        """Look up a Section by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "north", "SOUTH").

        Returns:
            Matching Section enum member.

        Raises:
            ValueError: If value does not match any Section.
        """
        try:
            return cls(value.lower())
        except ValueError:
            raise ValueError(f"Invalid section: {value!r}. Expected one of {[s.value for s in cls]}")

    @property
    def capitalized(self) -> str:
        """Return the section name with the first letter capitalised."""
        return self.value.capitalize()


class Direction(Enum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> Direction:
        """Look up a Direction by its string value (case-insensitive).

        Args:
            value: String representation (e.g. "clockwise").

        Returns:
            Matching Direction enum member.

        Raises:
            ValueError: If value does not match any Direction.
        """
        try:
            return cls(value.lower())
        except ValueError:
            raise ValueError(f"Invalid direction: {value!r}. Expected one of {[d.value for d in cls]}")