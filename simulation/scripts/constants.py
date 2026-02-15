"""
WRO 2026 Future Engineers - Simulation Constants

This file contains all official WRO specifications and simulation parameters.
All measurements are in meters unless otherwise specified.

Official Source: WRO Future Engineers Competition Rules 2026
Last Updated: 2026-02-08
"""

from enums import Section


class TrackDimensions:
    """Official WRO track dimensions (meters)"""

    # Mat and track sizes
    MAT_SIZE = 3.2              # 3200mm mat size
    TRACK_SIZE = 3.0            # 3000mm inner track size

    # Coordinate system (bottom-left origin)
    MIN_COORD = 0.0             # Minimum coordinate (bottom-left)
    MAX_COORD = 3.0             # Maximum coordinate (top-right)
    CENTER_COORD = 1.5          # Center of track

    # Corner section (obstacles challenge)
    CORNER_MIN = 1.0            # Corner section start
    CORNER_MAX = 2.0            # Corner section end
    CORNER_SIZE = 1.0           # 1000mm × 1000mm corner

class WallSpecs:
    """Wall dimensions and positioning"""

    HEIGHT = 0.1                # 100mm wall height
    THICKNESS = 0.1             # 100mm wall thickness

    # Wall positioning offsets
    EXTERIOR_OFFSET = 0.05      # Half thickness for exterior walls
    INTERIOR_OFFSET = 0.05      # Half thickness for interior walls

    # Color (RGB normalized 0-1)
    COLOR = (0.0, 0.0, 0.0)     # Black

class CorridorDimensions:
    """Corridor width specifications for different challenges"""

    # Open challenge: Variable corridor widths
    NARROW = 0.6                # 600mm narrow corridor
    WIDE = 1.0                  # 1000mm wide corridor

    # Obstacles challenge: Fixed corridor width
    OBSTACLES_WIDTH = 1.0       # 1000mm fixed width

    # Corridor division grid
    DIVISION_OUTER = 0.4        # 400mm from outer wall
    DIVISION_INNER = 0.6        # 600mm from outer wall
    DIVISION_WIDTH = 0.2        # 200mm middle section width

class TrafficSignSpecs:
    """Official WRO traffic sign dimensions and colors"""

    # Dimensions (meters)
    WIDTH = 0.05                # 50mm
    DEPTH = 0.05                # 50mm
    HEIGHT = 0.10               # 100mm
    Z_POSITION = 0.05           # Half height (50mm)

    # Official colors (RGB normalized 0-1)
    # WRO Spec 13.21-13.22
    RED_COLOR = (0.933, 0.153, 0.216)     # RGB(238, 39, 55)
    GREEN_COLOR = (0.267, 0.839, 0.173)   # RGB(68, 214, 44)

    # Color randomization (standard deviation for Gaussian noise)
    RED_STD = (0.05, 0.02, 0.02)
    GREEN_STD = (0.02, 0.05, 0.02)

    # Grid positions (intersections of corridor divisions)
    GRID_DEPTH_NEAR = 1.0       # Entry position
    GRID_DEPTH_MIDDLE = 1.5     # Center position
    GRID_DEPTH_FAR = 2.0        # Exit position
    GRID_WIDTH_OUTER = 0.4      # Outer division line
    GRID_WIDTH_INNER = 0.6      # Inner division line

    # Number of signs
    MIN_SIGNS = 6               # Minimum per round
    MAX_SIGNS = 14              # Maximum per round (7 red + 7 green)

class ParkingLotSpecs:
    """Parking block dimensions and positioning (obstacles challenge only)"""

    # Block dimensions (meters)
    LENGTH = 0.20               # 200mm
    WIDTH = 0.02                # 20mm
    HEIGHT = 0.10               # 100mm
    Z_POSITION = 0.05           # Half height (50mm)

    # Color (RGB normalized 0-1)
    COLOR = (1.0, 0.0, 1.0)     # Magenta RGB(255, 0, 255)

    # Positioning
    WALL_OFFSET = 0.1           # Half of LENGTH (100mm from wall edge)
    BLOCK_SPACING_FACTOR = 1.5  # Spacing = 1.5 × robot_width

class StartingZoneSpecs:
    """Starting zone dimensions and visual appearance"""

    # Default dimensions (meters)
    DEFAULT_LENGTH = 0.5        # 500mm
    WIDTH = 0.2                 # 200mm
    THICKNESS = 0.001           # 1mm visual marker

    # Obstacles challenge adjustment
    OBSTACLES_SIZE_FACTOR = 0.9 # Use 90% of available gap

    # Visual appearance (RGB normalized 0-1)
    COLOR = (0.5, 0.5, 0.5)     # Grey

    # Direction indicators
    CLOCKWISE_COLOR = (0.2, 0.4, 1.0)      # Blue
    COUNTERCLOCKWISE_COLOR = (0.2, 1.0, 0.4)  # Green
    INDICATOR_RADIUS = 0.035    # 35mm radius

class RobotSpecs:
    """WRO Future Engineers robot specs (LEGO Bugatti Bolide + Ackermann)"""

    # Chassis dimensions
    LENGTH = 0.28               # 280mm chassis length
    WIDTH = 0.15                # 150mm chassis width
    HEIGHT = 0.10               # 100mm chassis height

    # Ackermann geometry
    WHEELBASE = 0.17            # 170mm axle-to-axle distance
    TRACK_WIDTH = 0.105         # 105mm wheel-to-wheel distance
    WHEEL_RADIUS = 0.0216       # 21.6mm LEGO Technic wheel radius
    MAX_STEERING_ANGLE = 0.5236 # ~30 degrees max front wheel angle

    # Wheel details
    WHEEL_WIDTH = 0.020         # 20mm LEGO Technic wheel width
    WHEEL_MASS = 0.05           # 50g per wheel
    CHASSIS_MASS = 0.8          # 800g total chassis

    # LIDAR (Slamtec C1)
    LIDAR_MIN_RANGE = 0.05      # 50mm minimum detection range
    LIDAR_MAX_RANGE = 12.0      # 12m maximum detection range
    LIDAR_SAMPLES = 500         # Slamtec C1 horizontal samples
    LIDAR_UPDATE_RATE = 10.0    # 10 Hz scan rate
    LIDAR_NOISE_STDDEV = 0.03   # 30mm noise

    # IMU (Adafruit BNO085)
    IMU_UPDATE_RATE = 100.0         # 100 Hz update rate
    IMU_GYRO_NOISE = 0.054          # rad/s gyroscope noise stddev
    IMU_ACCEL_NOISE = 0.3           # m/s² accelerometer noise stddev
    IMU_MASS = 0.0025               # 2.5g board mass
    IMU_SIZE = (0.0256, 0.0227, 0.0046)  # 25.6mm × 22.7mm × 4.6mm

    # Camera (RPi Camera 3 Wide)
    CAMERA_HFOV = 1.7802        # 102 degrees horizontal FOV (radians)
    CAMERA_WIDTH = 1536         # Horizontal resolution (pixels)
    CAMERA_HEIGHT = 864         # Vertical resolution (pixels)
    CAMERA_UPDATE_RATE = 30.0   # 30 FPS
    CAMERA_NEAR_CLIP = 0.05     # 50mm near clip
    CAMERA_FAR_CLIP = 10.0      # 10m far clip

class TrackMarkings:
    """Corner lines and other track markings"""

    # Corner line colors (RGB normalized 0-1)
    ORANGE_COLOR = (1.0, 0.4, 0.0)         # RGB(255, 102, 0)
    BLUE_COLOR = (0.0, 0.2, 1.0)           # RGB(0, 51, 255)

    # Corner line angle
    ANGLE = 30                  # 30° from corner

class LightingSpecs:
    """Simulation lighting parameters"""

    # Sun intensity range
    SUN_INTENSITY_MIN = 0.5
    SUN_INTENSITY_MAX = 1.0     # Clamped to 1.0 for valid SDF

    # Ambient light range
    AMBIENT_INTENSITY_MIN = 0.3
    AMBIENT_INTENSITY_MAX = 0.8

    # Direction variance (radians)
    DIRECTION_VARIANCE = 0.3

class ScenarioTypes:
    """Challenge types"""

    OPEN = "open"
    OBSTACLES = "obstacles"

class GridSections:
    """Track section definitions"""

    # All sections as list of enums
    SECTIONS = [Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST]

    # Sections as strings (for backward compatibility)
    SECTION_NAMES = [s.value for s in SECTIONS]

    # Length section centers (for starting zone randomization)
    LENGTH_SECTION_LEFT = 1.25  # Center of [1.0-1.5] section
    LENGTH_SECTION_RIGHT = 1.75 # Center of [1.5-2.0] section

class FilePaths:
    """Default file paths and templates"""

    BASE_WORLD_TEMPLATE = "worlds/wro_track_2026.sdf"
    OUTPUT_DIR_DEFAULT = "training_data"
    SCENARIO_PREFIX = "scenario_"
    METADATA_SUFFIX = "_metadata.json"

class RandomizationRanges:
    """Physics and appearance randomization ranges"""

    # Friction
    FRICTION_MIN = 0.6
    FRICTION_MAX = 1.2

    # Mass variance
    MASS_VARIANCE = 0.1

    # Traffic sign quantity
    SIGNS_MIN = 6
    SIGNS_MAX = 14

class DictKeys:
    """Dictionary keys used throughout the codebase for type safety"""

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

    # Track bounds keys
    MIN = "min"
    MAX = "max"
    CENTER = "center"
    Z_SIGN = "z_sign"

class WidthTypes:
    """Corridor width type identifiers"""

    NARROW = "narrow"
    WIDE = "wide"
    DEFAULT = "default"
    FIXED = "fixed"

class ColorNames:
    """Traffic sign color identifiers"""

    RED = "red"
    GREEN = "green"

class ModelNames:
    """Gazebo model name prefixes and identifiers"""

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
    """File extensions for world and metadata files"""

    SDF = ".sdf"
    JSON = ".json"
    PNG = ".png"
    SVG = ".svg"
    MP4 = ".mp4"
    JPG = ".jpg"

class FolderNames:
    """Folder names for output organization"""

    SCENARIOS = "scenarios"
    FRAMES = "frames"
    OPEN = "open"
    OBSTACLES = "obstacles"