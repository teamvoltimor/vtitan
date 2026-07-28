"""Domain models using Python 3.10+ strict dataclasses.

Replaces dictionaries and raw tuples with type-safe domain objects.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, StrEnum

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


@dataclass(slots=True, frozen=True)
class InnerBlock:
    """Bounding box of the track's central obstacle block."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float


@dataclass(slots=True, frozen=True)
class LidarClearances:
    """Directional LIDAR clearance distances from obstacles (meters)."""

    front_m: float
    left_m: float
    right_m: float
    back_m: float = 0.0


@dataclass(slots=True, frozen=True)
class CorridorGeometry:
    """Complete track corridor width layout (meters)."""

    north_width_m: float
    south_width_m: float
    east_width_m: float
    west_width_m: float
    inner_block: InnerBlock

    @property
    def min_width_m(self) -> float:
        return min(self.north_width_m, self.south_width_m, self.east_width_m, self.west_width_m)

    @property
    def mean_width_m(self) -> float:
        return (self.north_width_m + self.south_width_m + self.east_width_m + self.west_width_m) / 4

    def to_widths_dict(self) -> dict[Section, float]:
        return {
            Section.NORTH: self.north_width_m,
            Section.SOUTH: self.south_width_m,
            Section.EAST: self.east_width_m,
            Section.WEST: self.west_width_m,
        }


@dataclass(slots=True, frozen=True)
class MotorStateSnapshot:
    """Motor command and feedback state for one control cycle."""

    steering_angle_deg: float
    drive_speed: float
    encoder_position: int
    timestamp: float
    is_valid: bool = True


# ── Phase 2: Robustness Dataclasses ─────────────────────────────────────────


@dataclass(slots=True, frozen=True)
class SectorRanges:
    """Aggregated LIDAR measurements over an angular sector."""

    bearing_rad: float
    half_fov_rad: float
    mean_range_m: float
    min_range_m: float
    max_range_m: float
    valid_count: int


@dataclass(slots=True, frozen=True)
class PathPlannability:
    """Result of checking whether a path fits within corridor constraints."""

    is_feasible: bool
    min_required_m: float
    min_available_m: float
    margin_m: float
    reason: str | None = None


@dataclass(slots=True, frozen=True)
class CorridorWidthMeasurement:
    """Result of measuring corridor width from LIDAR data."""

    width_m: float
    is_plausible: bool
    is_aligned: bool
    alignment_error_rad: float
    side_range_left_m: float
    side_range_right_m: float
    sample_count: int = 0

    @property
    def is_valid(self) -> bool:
        return self.is_plausible and self.is_aligned


class SignColor(StrEnum):
    """Traffic sign color (per WRO rules)."""

    RED = "red"
    GREEN = "green"


@dataclass(slots=True, frozen=True)
class TrafficSignObservation:
    """Single traffic sign detection with world pose and confidence."""

    world_x_m: float
    world_y_m: float
    color: SignColor
    confidence: float
    detected_at_timestamp: float
    in_robot_frame: bool = False
    bbox_ymin: int | None = None
    bbox_xmin: int | None = None
    bbox_ymax: int | None = None
    bbox_xmax: int | None = None


class ImageRotation(IntEnum):
    """Image rotation in degrees."""

    NONE = 0
    CW_90 = 90
    CW_180 = 180
    CW_270 = 270


@dataclass(slots=True, frozen=True)
class CameraSize:
    """Camera image resolution and orientation metadata."""

    width_px: int
    height_px: int
    rotation_deg: ImageRotation = ImageRotation.NONE
    hflip: bool = False
    vflip: bool = False

    @property
    def effective_width(self) -> int:
        if self.rotation_deg in (ImageRotation.CW_90, ImageRotation.CW_270):
            return self.height_px
        return self.width_px

    @property
    def effective_height(self) -> int:
        if self.rotation_deg in (ImageRotation.CW_90, ImageRotation.CW_270):
            return self.width_px
        return self.height_px


# ── Phase 3: Polish Dataclasses ────────────────────────────────────────────


@dataclass(slots=True, frozen=True)
class ParkingLotGeometry:
    """Computed parking lot bounding box and approach corridor."""

    lot: ParkingLot
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    approach_bearing_rad: float


@dataclass(slots=True, frozen=True)
class ThrottleCommand:
    """Normalized drive and steering command for motor hardware."""

    speed_normalized: float
    steering_normalized: float
    duration_ms: int | None = None


@dataclass(slots=True, frozen=True)
class LoopProgress:
    """Race progress: lap number, waypoint index, and distance markers."""

    lap_number: int
    waypoint_index: int
    distance_m: float
    total_distance_m: float

    @property
    def progress_percent(self) -> float:
        if self.total_distance_m == 0:
            return 0.0
        return (self.distance_m / self.total_distance_m) * 100


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
