"""TypedDict definitions for inter-function data structures."""

from typing import TypedDict
from shared.config.enums import Direction, Section


class StartingPosition(TypedDict):
    """Robot starting position coordinates."""

    x: float
    y: float


class StartingConditions(TypedDict):
    """Metadata for the robot's starting configuration."""

    direction: Direction
    section: Section
    section_name: str
    position: tuple[float, float]
    yaw: float


class CorridorWidthTypeDict(TypedDict):
    """Inner dictionary for a corridor's width definition."""

    type: str
    width_mm: int


class CorridorWidthDict(TypedDict):
    """Corridor width metadata definitions."""

    north: CorridorWidthTypeDict
    south: CorridorWidthTypeDict
    east: CorridorWidthTypeDict
    west: CorridorWidthTypeDict


class ParkingConfig(TypedDict):
    """Metadata for the generated parking configuration."""

    direction: str
    side: str
    blocks: list[dict[str, float]]
    starting_zone: dict[str, float]
