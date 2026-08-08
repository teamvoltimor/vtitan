"""Domain models using Python 3.10+ strict dataclasses.

Replaces dictionaries and raw tuples with type-safe domain objects.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, StrEnum
from typing import ClassVar

from pydantic import BaseModel

from shared.domain.enums import CorridorWidthType, Direction, NavigatorPhase, ScenarioType, Section


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


# Robustness dataclasses


@dataclass(slots=True, frozen=True)
class SectorRanges:
    """Aggregated LIDAR measurements over an angular sector."""

    bearing_rad: float
    half_fov_rad: float
    mean_range_m: float
    min_range_m: float
    max_range_m: float
    valid_count: int
    wedge_masked: bool = False
    """True when valid_count is 0 because every ray in this sector fell inside
    a known LIDAR blind wedge (mount occlusion), not because nothing is out
    there. Distinguishes "this bearing can't be trusted" from "genuinely
    clear" for callers that care (e.g. telemetry); the numeric fields still
    fall back to the same no-data sentinel either way."""


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


# Polish dataclasses


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


# Scenario metadata Pydantic models
# These replace ``dict[str, Any]`` metadata objects that were passed raw across
# the simulation/navigation boundary.  They match the schema produced by
# ``simgen`` and ``scenario_builder.py``.


class CorridorWidthEntry(BaseModel):
    """Width and type for one side of the track corridor.

    The two fields have to agree: ``type`` names one of the two legal widths and
    ``width_mm`` states it. The default was ``type="wide"`` with
    ``width_mm=500``, which is both self-contradictory and not a legal width at
    all -- and it was not inert, since ``CorridorWidths`` and
    ``ScenarioMetadata`` both default through it, so metadata built without
    explicit widths silently described a track that cannot exist. It did not
    even fail planning: 0.5 m clears the width the chassis needs, so a path got
    planned on a fictional mat.

    ``WIDE_WIDTH_MM`` is stated here rather than read from
    ``CorridorDimensions``, because this layer does not import config. The two
    are pinned to each other by a test in the robot suite, which may import
    both.
    """

    WIDE_WIDTH_MM: ClassVar[int] = 1000

    type: CorridorWidthType = CorridorWidthType.WIDE
    width_mm: int = WIDE_WIDTH_MM


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


class NavigatorDebugSnapshot(BaseModel):
    """Complete per-tick internal state of ``CoreNavigator.step()``.

    Diagnosing 2026-08-03's real-hardware failures (full-lock steering
    oscillation, wrong-direction turns, wedged-against-a-wall thrashing) meant
    reconstructing crosstrack error, angle error, which lookahead/target were
    chosen, and risk state by hand from raw ``/motor/*`` and ``/imu/data``
    topics after the fact -- slow, and some internal values (e.g. crosstrack
    error, the chosen steer target) cannot be recovered from those topics at
    all. Published every control tick regardless of which branch ``step()``
    took, so this is always current, not just on request.

    ``phase`` identifies which branch of ``step()`` produced this snapshot --
    a field being ``None`` means "not computed on this tick's branch", not
    "unknown" or a bug. E.g. ``crosstrack_error_m`` is only set on the
    ``normal_drive`` phase; it is meaningless (and left ``None``) while an
    escape maneuver is latched.
    """

    phase: NavigatorPhase = NavigatorPhase.NOT_YET_STEPPED

    # Pose and race state -- available on every phase except "no_pose".
    pose_x: float | None = None
    pose_y: float | None = None
    pose_yaw: float | None = None
    # Typed rather than str: both are StrEnum, so the wire format is unchanged
    # and bags recorded before this still parse, but every consumer stopped
    # having to rebuild {s.value: s for s in Section} to get back to the enum
    # the producer already had.
    direction: Direction | None = None
    current_corridor: Section | None = None
    waypoint_index: int | None = None
    laps_completed: int = 0
    num_laps: int = 0

    # Stuck detection -- set whenever the detector runs (see StuckDetector.get_diagnostics).
    is_stuck: bool | None = None
    stuck_count: int | None = None
    recent_movement_m: float | None = None

    # Perception / risk -- set on the normal_drive phase.
    forward_clearance_m: float | None = None
    min_lidar_range_m: float | None = None
    risk: str | None = None
    escape_risk: str | None = None
    rear_clearance_m: float | None = None

    # Path tracking -- set on the normal_drive phase.
    crosstrack_error_m: float | None = None
    lookahead_distance_m: float | None = None
    # Heading change the planned path makes within the preview distance. Reads
    # ~0 on a straight and rises before a corner, so a bag shows whether the
    # short lookahead armed on entry or (as before 2026-08-06) a corner late.
    path_turn_ahead_rad: float | None = None
    steer_target_x: float | None = None
    steer_target_y: float | None = None
    angle_error_rad: float | None = None

    # Speed selection -- set on the normal_drive phase.
    clearance_speed_mps: float | None = None
    heading_speed_mps: float | None = None

    # Final command -- set on every phase that actually publishes a drive command.
    commanded_speed_mps: float | None = None
    commanded_steering_norm: float | None = None

    # Escape/stuck maneuver -- set whenever one is latched or begun.
    active_maneuver_type: str | None = None
    maneuver_steering: float | None = None
    maneuver_speed_mps: float | None = None
    maneuver_frames_left: int | None = None
    escape_count: int | None = None

    # Parking -- set once a ParkController exists.
    parking_engaged: bool | None = None
    park_phase: str | None = None

    # Obstacles Challenge sign routing -- set on the normal_drive phase
    # whenever a SignRouter is attached.
    active_sign_count: int | None = None
    sign_deform_magnitude_m: float | None = None

    # Blind creep / direction inference -- set only on the "blind_creep" phase,
    # before the travel direction has settled and CoreNavigator.step() has ever
    # run (see corridor_follower.follow_corridor and direction_estimator.py).
    direction_gate_verdict: str | None = None
    direction_left_range_m: float | None = None
    direction_right_range_m: float | None = None
    direction_votes_clockwise: int | None = None
    direction_votes_counterclockwise: int | None = None
    corridor_width_belief_m: float | None = None

    # Per-corridor width belief (CorridorWidthEstimator) -- set whenever the
    # robot is running blind (widths estimated from LIDAR, not told them),
    # regardless of phase: this is track_navigator_node's own state, not
    # CoreNavigator's, so it is overlaid after every phase rather than tied
    # to just one of them.
    belief_north_m: float | None = None
    belief_south_m: float | None = None
    belief_east_m: float | None = None
    belief_west_m: float | None = None

    # What LidarLocalizer was actually handed, as distinct from the fused pose
    # reported above. The localizer only ever solves for position and takes
    # yaw as given, so a heading that is wrong (notably by pi, when blind
    # direction inference overturns the assumed direction) produces a
    # confidently-tracked but wrong position with nothing in the pose itself
    # to show for it. Diagnosing the 2026-08-05 CCW run required replaying
    # recorded scans to recover these, because pose_yaw is the corrected
    # heading and cannot reveal a mismatch with what the localizer consumed.
    localizer_input_yaw_rad: float | None = None
    localizer_prior_x: float | None = None
    localizer_prior_y: float | None = None

    # Where the robot measured itself to be when direction inference settled,
    # against where it had assumed it was. The 2026-08-05 rounds were lost to a
    # start that was asserted rather than observed -- 0.69 m of track ahead
    # while planning for 1.5 m -- and nothing in the bag showed it, because a
    # pose the robot never doubted looks identical to a correct one.
    # ``start_measurement_ahead_m`` is the number that would have shown it.
    # All None on a scan the measurement refused, which is itself the signal
    # that the robot was obstructed or not on the track.
    start_measured_x: float | None = None
    start_measured_y: float | None = None
    start_measurement_ahead_m: float | None = None
    start_measured_corridor_width_m: float | None = None
