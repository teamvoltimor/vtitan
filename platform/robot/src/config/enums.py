"""WRO 2026 domain enumerations used by the robot navigation stack."""

from __future__ import annotations

from enum import Enum, StrEnum


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
        """Look up a Section by its string value (case-insensitive)."""
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(f"Invalid section: {value!r}. Expected one of {options}") from err

    @property
    def capitalized(self) -> str:
        return self.value.capitalize()


class Direction(Enum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> Direction:
        """Look up a Direction by its string value (case-insensitive)."""
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(d.value for d in cls)
            raise ValueError(f"Invalid direction: {value!r}. Expected one of {options}") from err


class ScenarioType(StrEnum):
    """WRO 2026 challenge type."""

    OPEN = "open"
    OBSTACLES = "obstacles"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_string(cls, value: str) -> ScenarioType:
        """Look up a ScenarioType by its string value (case-insensitive)."""
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(f"Invalid scenario type: {value!r}. Expected one of {options}") from err
