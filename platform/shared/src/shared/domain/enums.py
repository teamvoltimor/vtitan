"""WRO 2026 simulation enumerations.

Defines strongly-typed domain enums for track sections, robot direction,
and challenge type.
Using enums instead of bare strings eliminates typo-prone comparisons and
provides IDE autocomplete throughout the codebase.
"""

from __future__ import annotations

from enum import StrEnum


class Section(StrEnum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    @classmethod
    def from_string(cls, value: str) -> Section:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid section: {value!r}. Expected one of {options}"
            ) from err

    @property
    def capitalized(self) -> str:
        return self.value.capitalize()


class Direction(StrEnum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    @classmethod
    def from_string(cls, value: str) -> Direction:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(d.value for d in cls)
            raise ValueError(
                f"Invalid direction: {value!r}. Expected one of {options}"
            ) from err


class ScenarioType(StrEnum):
    """WRO 2026 challenge type."""

    OPEN = "open"
    OBSTACLES = "obstacles"

    @classmethod
    def from_string(cls, value: str) -> ScenarioType:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid scenario type: {value!r}. Expected one of {options}"
            ) from err


class RiskLevel(StrEnum):
    """Collision risk classification for navigation logic."""

    SAFE = "safe"
    CRITICAL = "critical"
    OBSTACLE = "obstacle"


class LightingScenario(StrEnum):
    """Realistic lighting scenarios for simulation."""

    DIRECT_SUNLIGHT = "direct_sunlight"
    CLOUDY = "cloudy"
    INDOOR_BRIGHT = "indoor_bright"
    INDOOR_DIM = "indoor_dim"
    EVENING = "evening"
    MIXED = "mixed"


class RobotState(StrEnum):
    """Robot state machine states."""

    BOOT_CHECK = "boot_check"
    READY = "ready"
    RACING = "racing"
    FINISHED = "finished"

    @classmethod
    def from_string(cls, value: str) -> RobotState:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid robot state: {value!r}. Expected one of {options}"
            ) from err


class NodeHealth(StrEnum):
    """Telemetry node health status."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"

    @classmethod
    def from_string(cls, value: str) -> NodeHealth:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(n.value for n in cls)
            raise ValueError(
                f"Invalid node health: {value!r}. Expected one of {options}"
            ) from err
