"""String identifiers: dict keys, model name prefixes, file paths/extensions.

Traffic sign colors use ``shared.domain.models.SignColor`` directly rather
than a redundant string-alias class here.
"""

from __future__ import annotations


class FilePaths:
    """Default file paths and templates."""

    BASE_WORLD_TEMPLATE = "worlds/wro_track_2026.sdf"
    OUTPUT_DIR_DEFAULT = "training_data"
    SCENARIO_PREFIX = "scenario_"
    METADATA_SUFFIX = "_metadata.json"


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
