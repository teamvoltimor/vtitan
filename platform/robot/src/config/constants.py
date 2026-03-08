"""WRO 2026 constants used by the robot navigation stack.

All measurements are in metres unless otherwise specified.
"""

from __future__ import annotations


class TrackDimensions:
    """Official WRO track dimensions (metres)."""

    MIN_COORD = 0.0
    MAX_COORD = 3.0
    CENTER_COORD = 1.5


class RobotSpecs:
    """WRO Future Engineers robot specs (LEGO Bugatti Bolide + Ackermann)."""

    LENGTH = 0.28
    WIDTH = 0.15
    HEIGHT = 0.10
    WHEELBASE = 0.17
    TRACK_WIDTH = 0.105
    MAX_STEERING_ANGLE = 0.5236  # ~30 degrees

    LIDAR_MIN_RANGE = 0.05   # 50 mm real sensor floor
    LIDAR_SIM_MIN_RANGE = 0.01
    LIDAR_MAX_RANGE = 12.0
    LIDAR_SAMPLES = 500
    LIDAR_UPDATE_RATE = 10.0


class DictKeys:
    """Dictionary keys used in scenario metadata."""

    # Corridor widths
    TYPE = "type"
    WIDTH = "width"
    WIDTH_MM = "width_mm"

    # Starting conditions
    DIRECTION = "direction"
    SECTION = "section"
    POSITION = "position"
    YAW = "yaw"

    # Position coordinates
    X = "x"
    Y = "y"
    Z = "z"

    # Metadata
    SCENARIO_ID = "scenario_id"
    CHALLENGE_TYPE = "challenge_type"
    CORRIDOR_WIDTHS = "corridor_widths"
    STARTING_CONDITIONS = "starting_conditions"
    NUM_SIGNS = "num_signs"
    HAS_PARKING_LOT = "has_parking_lot"
    SIGN_POSITIONS = "sign_positions"
    PARKING_LOT = "parking_lot"
    COLOR = "color"
