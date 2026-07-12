"""
WRO 2026 Future Engineers - Simulation Constants.

This file contains all official WRO specifications and simulation parameters.
All measurements are in meters unless otherwise specified.

Official Source: WRO Future Engineers Competition Rules 2026
Last Updated: 2026-02-08
"""

import math
from dataclasses import dataclass
from typing import Final

from shared.config import robot_constants_gen as _gen
from shared.domain.enums import LightingScenario, Section


class TrackDimensions:
    """Official WRO track dimensions (meters)."""

    # Mat and track sizes
    MAT_SIZE: Final[float] = 3.2  # 3200mm mat size
    TRACK_SIZE: Final[float] = 3.0  # 3000mm inner track size

    # Coordinate system (bottom-left origin)
    MIN_COORD: Final[float] = 0.0  # Minimum coordinate (bottom-left)
    MAX_COORD: Final[float] = 3.0  # Maximum coordinate (top-right)
    CENTER_COORD: Final[float] = 1.5  # Center of track

    # Corner section (obstacles challenge)
    CORNER_MIN: Final[float] = 1.0  # Corner section start
    CORNER_MAX: Final[float] = 2.0  # Corner section end
    CORNER_SIZE: Final[float] = 1.0  # 1000mm × 1000mm corner


class WallSpecs:
    """Wall dimensions and positioning."""

    HEIGHT: Final[float] = 0.1  # 100mm wall height
    THICKNESS: Final[float] = 0.1  # 100mm wall thickness (visual)
    # Collision extends 40mm per side past visual to prevent LIDAR pass-through.
    # LIDAR is 30mm ahead of chassis front, so 40mm buffer ensures the LIDAR
    # stays ≥10mm outside the visual wall face even at full contact.
    COLLISION_THICKNESS: Final[float] = 0.18  # 100mm visual + 40mm per side

    # Wall positioning offsets
    EXTERIOR_OFFSET: Final[float] = 0.05  # Half thickness for exterior walls
    INTERIOR_OFFSET: Final[float] = 0.05  # Half thickness for interior walls

    # Color (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = (0.0, 0.0, 0.0)  # Black


class CorridorDimensions:
    """Corridor width specifications for different challenges."""

    # Open challenge: Variable corridor widths
    NARROW: Final[float] = 0.6  # 600mm narrow corridor
    WIDE: Final[float] = 1.0  # 1000mm wide corridor

    # Validation bounds (wider range to allow test tolerance)
    MIN_WIDTH: Final[float] = 0.5  # Minimum valid corridor width (500mm)
    MAX_WIDTH: Final[float] = 1.5  # Maximum valid corridor width (1500mm)

    # Obstacles challenge: Fixed corridor width
    OBSTACLES_WIDTH: Final[float] = 1.0  # 1000mm fixed width

    # Corridor division grid
    DIVISION_OUTER: Final[float] = 0.4  # 400mm from outer wall
    DIVISION_INNER: Final[float] = 0.6  # 600mm from outer wall
    DIVISION_WIDTH: Final[float] = 0.2  # 200mm middle section width


class TrafficSignSpecs:
    """Official WRO traffic sign dimensions and colors."""

    WIDTH: Final[float] = 0.05  # 50mm
    DEPTH: Final[float] = 0.05  # 50mm
    HEIGHT: Final[float] = 0.10  # 100mm
    Z_POSITION: Final[float] = 0.05  # Half height (50mm)

    # Official colors (RGB normalized 0-1)
    # WRO Spec 13.21-13.22
    RED_COLOR: Final[tuple[float, float, float]] = (0.933, 0.153, 0.216)  # RGB(238, 39, 55)
    GREEN_COLOR: Final[tuple[float, float, float]] = (0.267, 0.839, 0.173)  # RGB(68, 214, 44)

    # Color randomization (standard deviation for Gaussian noise)
    RED_STD: Final[tuple[float, float, float]] = (0.05, 0.02, 0.02)
    GREEN_STD: Final[tuple[float, float, float]] = (0.02, 0.05, 0.02)

    # Grid positions (intersections of corridor divisions)
    GRID_DEPTH_NEAR: Final[float] = 1.0  # Entry position
    GRID_DEPTH_MIDDLE: Final[float] = 1.5  # Center position
    GRID_DEPTH_FAR: Final[float] = 2.0  # Exit position
    GRID_WIDTH_OUTER: Final[float] = 0.4  # Outer division line
    GRID_WIDTH_INNER: Final[float] = 0.6  # Inner division line

    # Number of signs
    MIN_SIGNS: Final[int] = 6  # Minimum per round
    MAX_SIGNS: Final[int] = 14  # Maximum per round (7 red + 7 green)


class ParkingLotSpecs:
    """Parking block dimensions and positioning (obstacles challenge only)."""

    # Block dimensions (meters)
    LENGTH: Final[float] = 0.20  # 200mm
    WIDTH: Final[float] = 0.02  # 20mm
    HEIGHT: Final[float] = 0.10  # 100mm
    Z_POSITION: Final[float] = 0.05  # Half height (50mm)

    # Color (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = (1.0, 0.0, 1.0)  # Magenta RGB(255, 0, 255)

    # Positioning
    WALL_OFFSET: Final[float] = 0.1  # Half of LENGTH (100mm from wall edge)
    BLOCK_SPACING_FACTOR: Final[float] = 1.5  # Spacing = 1.5 × robot_width


class CompetitionSpecs:
    """Official WRO Future Engineers match rules (round timing, lap counts)."""

    ROUND_TIME_LIMIT_S: Final[float] = 180.0  # Official round duration: 3 minutes
    OPEN_CHALLENGE_LAPS: Final[int] = 3  # Laps required per Open Challenge run


class StartingZoneSpecs:
    """Starting zone dimensions and visual appearance."""

    # Default dimensions (meters)
    DEFAULT_LENGTH: Final[float] = 0.5  # 500mm
    WIDTH: Final[float] = 0.2  # 200mm
    THICKNESS: Final[float] = 0.001  # 1mm visual marker

    # Obstacles challenge adjustment
    OBSTACLES_SIZE_FACTOR: Final[float] = 0.9  # Use 90% of available gap

    # Visual appearance (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = (0.5, 0.5, 0.5)  # Grey

    # Direction indicators
    CLOCKWISE_COLOR: Final[tuple[float, float, float]] = (0.2, 0.4, 1.0)  # Blue
    COUNTERCLOCKWISE_COLOR: Final[tuple[float, float, float]] = (0.2, 1.0, 0.4)  # Green
    INDICATOR_RADIUS: Final[float] = 0.035  # 35mm radius


class RobotSpecs:
    """WRO Future Engineers robot specs (LEGO Bugatti Bolide + Ackermann).

    Physical constants (chassis, Ackermann geometry, wheel, LIDAR/camera mount offsets) are
    generated from platform/shared/config/robot.toml — see robot_constants_gen.py — and must
    not be hand-edited here. Everything else in this class (LIDAR/IMU/camera sim parameters)
    is not duplicated in Go/xacro and stays hand-maintained.
    """

    # Chassis dimensions
    LENGTH: Final[float] = _gen.CHASSIS_LENGTH  # 300mm chassis length
    WIDTH: Final[float] = _gen.CHASSIS_WIDTH  # 200mm chassis width
    HEIGHT: Final[float] = _gen.CHASSIS_HEIGHT  # 100mm chassis height

    # Ackermann geometry (measured 2026-07-11)
    WHEELBASE: Final[float] = _gen.WHEELBASE  # 190mm axle-to-axle distance
    TRACK_WIDTH: Final[float] = _gen.TRACK_WIDTH  # 167.5mm wheel-to-wheel distance
    WHEEL_RADIUS: Final[float] = _gen.WHEEL_RADIUS  # 35mm (measured 70mm wheel diameter / 2)
    MAX_STEERING_ANGLE: Final[float] = _gen.MAX_STEERING_ANGLE  # ~30 degrees max front wheel angle

    # Wheel details (measured 2026-07-11)
    WHEEL_WIDTH: Final[float] = _gen.WHEEL_WIDTH  # 25mm
    WHEEL_MASS: Final[float] = _gen.WHEEL_MASS  # 50g per wheel
    CHASSIS_MASS: Final[float] = _gen.CHASSIS_MASS  # 800g total chassis

    # LIDAR (Slamtec C1) — mounted upside-down, centered left/right, at the front of the
    # chassis (measured 2026-07-11). See docs/robot-physical-constants.md.
    LIDAR_MIN_RANGE: Final[float] = 0.05  # 50mm minimum detection range (real sensor)
    LIDAR_SIM_MIN_RANGE: Final[float] = (
        0.01  # 10mm simulation min (detect near-wall, clamp to 50mm in callback)
    )
    LIDAR_MAX_RANGE: Final[float] = 12.0  # 12m maximum detection range
    LIDAR_SAMPLES: Final[int] = 500  # Slamtec C1 horizontal samples
    LIDAR_UPDATE_RATE: Final[float] = 10.0  # 10 Hz scan rate
    LIDAR_NOISE_STDDEV: Final[float] = 0.03  # 30mm noise
    # 55.6mm diameter x 41.3mm height: matches the lidar_link visual/collision mesh already
    # modeled in wro_robot.urdf.xacro (radius=0.0278, length=0.0413) — used here instead of
    # the C1's raw datasheet form factor so the mount-offset derivation below stays
    # self-consistent with the mesh actually rendered in sim.
    LIDAR_DIAMETER: Final[float] = 0.0556
    LIDAR_HEIGHT: Final[float] = 0.0413
    # = LENGTH/2 - LIDAR_DIAMETER/2 = 0.15 - 0.0278: the C1 mounted flush with the front
    # edge, offset back by its own puck radius (same derivation style as the camera mount
    # offset below). Cross-checked against the existing z-mount height (HEIGHT + LIDAR_HEIGHT/2
    # = 0.10 + 0.0207 ~= 0.1207, matching the long-standing z=0.12 lidar_link offset in
    # static_tfs.launch.py / the URDF within rounding).
    LIDAR_MOUNT_X_OFFSET: Final[float] = _gen.LIDAR_MOUNT_X_OFFSET
    # Rays that clip the chassis body itself (mount occlusion, cable clutter)
    # return as a self-reflection, not a real obstacle. Never applied to the
    # pure-forward bearing, where a genuine near-contact must still register.
    LIDAR_SELF_DETECTION_THRESHOLD: Final[float] = 0.08  # 80mm

    # IMU (Adafruit BNO085)
    IMU_UPDATE_RATE: Final[float] = 100.0  # 100 Hz update rate
    IMU_GYRO_NOISE: Final[float] = 0.054  # rad/s gyroscope noise stddev
    IMU_ACCEL_NOISE: Final[float] = 0.3  # m/s² accelerometer noise stddev
    IMU_MASS: Final[float] = 0.0025  # 2.5g board mass
    IMU_SIZE: Final[tuple[float, float, float]] = (0.0256, 0.0227, 0.0046)  # 25.6mm × 22.7mm × 4.6mm

    # Camera (RPi Camera 3 Wide) — mounted above the LIDAR, angled down (measured 2026-07-11,
    # approximate; see docs/robot-physical-constants.md).
    CAMERA_HFOV: Final[float] = 1.7802  # 102 degrees horizontal FOV (radians)
    CAMERA_WIDTH: Final[int] = 1536  # Horizontal resolution (pixels)
    CAMERA_HEIGHT: Final[int] = 864  # Vertical resolution (pixels)
    CAMERA_UPDATE_RATE: Final[float] = 30.0  # 30 FPS
    CAMERA_NEAR_CLIP: Final[float] = 0.05  # 50mm near clip
    CAMERA_FAR_CLIP: Final[float] = 10.0  # 10m far clip
    # Directly over the LIDAR (same x as LIDAR_MOUNT_X_OFFSET), mounted above its top edge
    # (LIDAR z=0.12 + LIDAR_HEIGHT/2 ~= 0.14) with a small mounting-bracket gap. Unlike
    # LIDAR_MOUNT_X_OFFSET, this z isn't derived from a datasheet — it's an estimate pending
    # a real measurement.
    CAMERA_MOUNT_X_OFFSET: Final[float] = _gen.CAMERA_MOUNT_X_OFFSET
    CAMERA_MOUNT_Z_OFFSET: Final[float] = _gen.CAMERA_MOUNT_Z_OFFSET
    CAMERA_MOUNT_PITCH_DEG: Final[float] = math.degrees(
        _gen.CAMERA_MOUNT_PITCH_RAD,
    )  # tilted down; angle is an estimate ("~30")


class TrackMarkings:
    """Corner lines and other track markings."""

    # Corner line colors (RGB normalized 0-1)
    ORANGE_COLOR = (1.0, 0.4, 0.0)  # RGB(255, 102, 0)
    BLUE_COLOR = (0.0, 0.2, 1.0)  # RGB(0, 51, 255)

    # Corner line angle
    ANGLE = 30  # 30° from corner


class LightingSpecs:
    """Simulation lighting parameters."""

    # Sun intensity range
    SUN_INTENSITY_MIN = 0.5
    SUN_INTENSITY_MAX = 1.0  # Clamped to 1.0 for valid SDF

    # Ambient light range
    AMBIENT_INTENSITY_MIN = 0.3
    AMBIENT_INTENSITY_MAX = 0.8

    # Direction variance (radians)
    DIRECTION_VARIANCE = 0.3


@dataclass(frozen=True, slots=True)
class LightingSpec:
    """Randomization ranges and shadow behavior for one lighting scenario.

    Attributes:
        intensity:     (min, max) sun intensity range [0.0, 1.0].
        ambient:       (min, max) ambient intensity range [0.0, 1.0].
        direction:     ((x_min, x_max), (y_min, y_max), z) sun direction, z fixed.
        cast_shadows:  Whether the sun casts shadows in this scenario.
    """

    intensity: tuple[float, float]
    ambient: tuple[float, float]
    direction: tuple[tuple[float, float], tuple[float, float], float]
    cast_shadows: bool


class LightingScenarios:
    """Table-driven lighting scenario specifications.

    Each scenario defines the ranges for intensity, ambient intensity,
    direction, and shadow casting behavior. This table-driven approach
    eliminates 50+ lines of duplicated branching code.
    """

    SPECS: Final[dict[LightingScenario, LightingSpec]] = {
        LightingScenario.DIRECT_SUNLIGHT: LightingSpec(
            intensity=(0.9, 1.0),
            ambient=(0.3, 0.4),
            direction=((-0.7, -0.3), (-0.7, -0.3), -1.0),
            cast_shadows=True,
        ),
        LightingScenario.CLOUDY: LightingSpec(
            intensity=(0.6, 0.75),
            ambient=(0.5, 0.6),
            direction=((-0.5, -0.5), (-0.5, -0.5), -1.0),
            cast_shadows=True,
        ),
        LightingScenario.INDOOR_BRIGHT: LightingSpec(
            intensity=(0.7, 0.85),
            ambient=(0.6, 0.7),
            direction=((0.0, 0.0), (0.0, 0.0), -1.0),
            cast_shadows=False,
        ),
        LightingScenario.INDOOR_DIM: LightingSpec(
            intensity=(0.5, 0.65),
            ambient=(0.4, 0.5),
            direction=((0.0, 0.0), (0.0, 0.0), -1.0),
            cast_shadows=False,
        ),
        LightingScenario.EVENING: LightingSpec(
            intensity=(0.6, 0.8),
            ambient=(0.3, 0.4),
            direction=((-0.9, -0.7), (-0.5, 0.5), -0.3),
            cast_shadows=True,
        ),
        LightingScenario.MIXED: LightingSpec(
            intensity=(0.7, 0.9),
            ambient=(0.5, 0.65),
            direction=((-0.6, -0.4), (-0.6, -0.4), -1.0),
            cast_shadows=True,
        ),
    }


class ZLayers:
    """Z-axis positioning for visual layering and collision.

    Centralizes all Z-position constants to prevent scattered magic
    numbers throughout the codebase. These values ensure proper layering
    and prevent rendering artifacts.
    """

    TRACK_FLOOR = 0.00001  # Lowest level: track surface
    GRID_LINES = 0.0001  # Grid lines on track
    STARTING_ZONE_BASE = 0.0002  # Starting zone visual marker
    DIRECTION_INDICATOR = 0.004  # Direction indicator on starting zone
    TRAFFIC_SIGN = 0.05  # Traffic signs (half their height)
    PARKING_BLOCK = 0.05  # Parking blocks (half their height)
    COLLISION_SURFACE = 0.05  # Collision detection surface
    ROBOT_BASE = None  # Computed from RobotSpecs.WHEEL_RADIUS (dynamic)


class GridSections:
    """Track section definitions."""

    # All sections as a tuple (immutable — never append to this at runtime)
    SECTIONS = (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST)

    # Sections as strings (for JSON serialization)
    SECTION_NAMES = tuple(s.value for s in SECTIONS)

    # Length section centers (for starting zone randomization)
    LENGTH_SECTION_LEFT = 1.25  # Center of [1.0-1.5] section
    LENGTH_SECTION_RIGHT = 1.75  # Center of [1.5-2.0] section


class FilePaths:
    """Default file paths and templates."""

    BASE_WORLD_TEMPLATE = "worlds/wro_track_2026.sdf"
    OUTPUT_DIR_DEFAULT = "training_data"
    SCENARIO_PREFIX = "scenario_"
    METADATA_SUFFIX = "_metadata.json"


class RandomizationRanges:
    """Physics and appearance randomization ranges."""

    # Friction
    FRICTION_MIN = 0.6
    FRICTION_MAX = 1.2

    # Mass variance
    MASS_VARIANCE = 0.1

    # Traffic sign quantity
    SIGNS_MIN = 6
    SIGNS_MAX = 14


class DictKeys:
    """Dictionary keys used throughout the codebase for type safety."""

    # Corridor width dictionary keys
    TYPE = "type"
    WIDTH = "width"
    WIDTH_MM = "width_mm"

    # Starting conditions keys
    DIRECTION = "direction"
    SECTION = "section"
    SECTION_NAME = "section_name"
    POSITION = "position"
    YAW = "yaw"

    # Position coordinate keys
    X = "x"
    Y = "y"
    Z = "z"

    # Parking lot configuration keys
    BLOCK1_POS = "block1_pos"
    BLOCK2_POS = "block2_pos"
    BLOCK1_YAW = "block1_yaw"
    BLOCK2_YAW = "block2_yaw"
    DEPTH = "depth"
    BLOCK1_POSITION = "block1_position"
    BLOCK2_POSITION = "block2_position"

    # Metadata keys
    SCENARIO_ID = "scenario_id"
    CHALLENGE_TYPE = "challenge_type"
    WORLD_FILE = "world_file"
    CORRIDOR_WIDTHS = "corridor_widths"
    STARTING_CONDITIONS = "starting_conditions"
    NUM_SIGNS = "num_signs"
    HAS_PARKING_LOT = "has_parking_lot"
    SIGN_POSITIONS = "sign_positions"
    PARKING_LOT = "parking_lot"
    COLOR = "color"

    # Randomization config keys
    LIGHTING = "lighting"
    COLORS = "colors"
    PHYSICS = "physics"
    MEAN = "mean"
    STD = "std"
    INTENSITY_RANGE = "intensity_range"
    DIRECTION_VARIANCE = "direction_variance"
    FRICTION_RANGE = "friction_range"
    MASS_VARIANCE = "mass_variance"
    INTENSITY = "intensity"
    AMBIENT_INTENSITY = "ambient_intensity"
    CAST_SHADOWS = "cast_shadows"
    SCENARIO = "scenario"

    # Track bounds keys
    MIN = "min"
    MAX = "max"
    CENTER = "center"
    Z_SIGN = "z_sign"


class WidthTypes:
    """Corridor width type identifiers."""

    NARROW = "narrow"
    WIDE = "wide"
    DEFAULT = "default"
    FIXED = "fixed"


class ColorNames:
    """Traffic sign color identifiers."""

    RED = "red"
    GREEN = "green"


class ModelNames:
    """Gazebo model name prefixes and identifiers."""

    # Wall models
    INTERIOR_WALL_NORTH = "interior_wall_north"
    INTERIOR_WALL_SOUTH = "interior_wall_south"
    INTERIOR_WALL_EAST = "interior_wall_east"
    INTERIOR_WALL_WEST = "interior_wall_west"

    # Traffic sign models
    RED_SIGN_PREFIX = "red_sign_"
    GREEN_SIGN_PREFIX = "green_sign_"

    # Parking lot models
    PARKING_LIMITATION_1 = "parking_limitation_1"
    PARKING_LIMITATION_2 = "parking_limitation_2"

    # Starting zone models
    STARTING_ZONE_PREFIX = "starting_zone_"

    # Light models
    SUN_LIGHT = "sun"
    AMBIENT_LIGHT = "ambient_light"


class FileExtensions:
    """File extensions for world and metadata files."""

    SDF = ".sdf"
    JSON = ".json"
    PNG = ".png"
    SVG = ".svg"
    MP4 = ".mp4"
    JPG = ".jpg"


class FolderNames:
    """Folder names for output organization."""

    SCENARIOS = "scenarios"
    FRAMES = "frames"
    OPEN = "open"
    OBSTACLES = "obstacles"
