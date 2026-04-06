"""TypedDict definitions for inter-function data structures."""

from typing import TypedDict

from shared.config.enums import Direction, LightingScenario, Section


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


class CorridorWidthConfig(TypedDict):
    """Configuration for a single corridor's width.

    Replaces the old ambiguous CorridorWidthTypeDict with clear semantics.
    """

    type: str  # WidthTypes.NARROW or WidthTypes.WIDE
    width: float  # Width in meters (0.6 or 1.0)


class CorridorWidths(TypedDict):
    """Mapping of corridor sections to their width configurations.

    Use as: dict[Section, CorridorWidthConfig]
    """

    pass  # This is just a marker; actual usage is dict[Section, CorridorWidthConfig]


class LightingConfig(TypedDict):
    """Configuration for lighting in a scenario.

    Represents all lighting parameters that affect the simulation
    environment appearance and camera behavior.
    """

    intensity: float  # Sun intensity [0.0, 1.0]
    ambient_intensity: float  # Ambient light [0.0, 1.0]
    direction: list[float]  # Sun direction [x, y, z] (3 elements)
    cast_shadows: bool  # Whether sun casts shadows
    scenario: str  # Lighting scenario name (LightingScenario value)


class ParkingBlock(TypedDict):
    """Configuration for a single parking block."""

    position: tuple[float, float]  # (x, y) world coordinates
    yaw: float  # Rotation angle in radians


class ParkingLotConfig(TypedDict):
    """Parking lot configuration for obstacles challenge.

    Contains positions and orientations for two parking blocks.
    """

    block1_pos: tuple[float, float]  # First block (x, y)
    block2_pos: tuple[float, float]  # Second block (x, y)
    block1_yaw: float  # First block rotation
    block2_yaw: float  # Second block rotation
    depth: float  # Depth of blocks from grid entry


class StartingZoneConfig(TypedDict):
    """Configuration for the starting zone placement."""

    length: float  # Length of starting zone
    x: float  # X position (world coordinates)
    y: float  # Y position (world coordinates)


class TrafficSignColor(TypedDict):
    """Traffic sign color specification."""

    name: str  # ColorNames.RED or ColorNames.GREEN
    rgb: list[float]  # Normalized RGB [0.0, 1.0]
