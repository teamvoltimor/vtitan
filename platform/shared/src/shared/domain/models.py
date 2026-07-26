"""Domain models using Python 3.10+ strict dataclasses.

Replaces dictionaries and raw tuples with type-safe domain objects.
"""

from __future__ import annotations

from dataclasses import dataclass

from pydantic import BaseModel

from shared.domain.enums import Direction, ScenarioType, Section


@dataclass(slots=True, frozen=True)
class Pose:
    """Robot position and orientation in world space."""

    x: float
    y: float
    yaw: float


@dataclass(slots=True, frozen=True)
class Velocity:
    """Linear and angular velocity."""

    linear: float  # m/s
    angular: float  # rad/s


@dataclass(slots=True, frozen=True)
class IMUReading:
    """IMU sensor reading."""

    yaw: float
    pitch: float
    roll: float


@dataclass(slots=True, frozen=True)
class Waypoint:
    """A point to navigate towards."""

    x: float
    y: float


@dataclass(slots=True, frozen=True)
class Detection:
    """Computer vision object detection."""

    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # x_min, y_min, x_max, y_max
    x: float
    y: float
    width: float
    height: float
    area: float


@dataclass(slots=True, frozen=True)
class Bounds:
    """A bounding box for collision detection."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float


# ── Scenario Metadata Pydantic Models ────────────────────────────────────────
# These replace ``dict[str, Any]`` metadata objects that were passed raw across
# the simulation/navigation boundary.  They match the schema produced by
# ``simgen`` and ``scenario_builder.py``.


class CorridorWidthEntry(BaseModel):
    """Width and type for one side of the track corridor."""

    type: str = "wide"
    width_mm: int = 500


class CorridorWidths(BaseModel):
    """Corridor widths for all four sides."""

    north: CorridorWidthEntry = CorridorWidthEntry()
    south: CorridorWidthEntry = CorridorWidthEntry()
    east: CorridorWidthEntry = CorridorWidthEntry()
    west: CorridorWidthEntry = CorridorWidthEntry()


class Position2D(BaseModel):
    """A 2D world position."""

    x: float = 0.0
    y: float = 0.0


class StartingConditions(BaseModel):
    """Robot starting pose and direction."""

    direction: str = Direction.COUNTERCLOCKWISE.value
    section: str = Section.SOUTH.value
    position: Position2D = Position2D()
    yaw: float = 0.0


class SignPosition(BaseModel):
    """A traffic sign placed on the track."""

    x: float
    y: float
    color: str = "red"


class BlockPosition(BaseModel):
    """A single parking block position in world coordinates."""

    x: float
    y: float


class ParkingLot(BaseModel):
    """Parking lot with two blocks (WRO obstacles challenge).

    Both blocks are required. ``BlockPosition`` has no defaults, so an empty
    one cannot be built -- and should not be: a lot defaulting to (0, 0) would
    put both markers at the track origin, and the park controller would aim at
    a bay that is not there rather than failing outright. A scenario with no
    lot is expressed as ``ScenarioMetadata.parking_lot = None``.
    """

    block1_position: BlockPosition
    block2_position: BlockPosition


class ScenarioMetadata(BaseModel):
    """Full scenario description for Open and Obstacles challenges.

    Matches the schema produced by ``simgen`` and ``scenario_builder.py``.
    """

    scenario_id: int = 0
    challenge_type: str = ScenarioType.OPEN.value
    seed: int | None = None
    num_signs: int = 0
    has_parking_lot: bool = False
    parking_lot: ParkingLot | None = None
    sign_positions: list[SignPosition] = []
    corridor_widths: CorridorWidths = CorridorWidths()
    starting_conditions: StartingConditions = StartingConditions()
