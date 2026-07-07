"""Centralized test constants to reduce duplication across platform test suite."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

# LIDAR Geometry
NUM_RAYS = 360
ANGLES_FULL_ROTATION = np.linspace(-math.pi, math.pi, NUM_RAYS)

# Angle thresholds for sector detection (indices around forward/rear)
REAR_SECTOR_INDICES = 8
FORWARD_SECTOR_INDICES = 4
SIDE_SECTOR_INDICES = 4

# Threat Distance Thresholds
LIDAR_CLOSE_THREAT = 0.3  # m — immediate collision risk
LIDAR_DEFAULT_FAR = 10.0  # m — default "clear" distance for mock scans
LIDAR_NEAR_WALL = 0.05  # m — very close wall (testing edge cases)
LIDAR_WALL_DISTANCE = 1.5  # m — wall at moderate distance

# Clearance computation thresholds
MIN_FORWARD_CLEARANCE = 5.0  # m
MIN_REAR_CLEARANCE = 5.0  # m
MAX_THREAT_DISTANCE = 0.3  # m

# Corridor Geometry
CORRIDOR_DEPTH_MIDPOINT = 1.5  # m — center of 3m track depth
CORRIDOR_DEPTH_MIN = 1.0  # m
CORRIDOR_DEPTH_MAX = 2.0  # m
CORRIDOR_WIDTH_QUARTER_NORTH = 0.4  # m
CORRIDOR_WIDTH_QUARTER_SOUTH = 0.6  # m

# Track geometry positions (from TrackDimensions)
TRACK_CENTER_X = 1.5  # m
TRACK_CENTER_Y = 1.5  # m
TRACK_CORNER_SOUTH = 0.4
TRACK_CORNER_NORTH = 2.6
TRACK_CORNER_EAST = 2.6
TRACK_CORNER_WEST = 0.4

# Inner block geometry (WRO standard)
INNER_BLOCK_MIN = 1.0  # m
INNER_BLOCK_MAX = 2.0  # m

# Sign Router Test Config
SIGN_LATERAL_OFFSET = 0.15  # m
SIGN_ACTIVATION_DIST = 0.80  # m
SIGN_PASSED_DIST = 1.20  # m

# Sign position grid (WRO standard, 6 positions per corridor)
SIGN_GRID_POSITIONS: list[tuple[float, float]] = [
    (1.0, 0.4),
    (1.0, 0.6),
    (1.5, 0.4),
    (1.5, 0.6),
    (2.0, 0.4),
    (2.0, 0.6),
]

# Robot Specs
ROBOT_CHASSIS_WIDTH = 0.15  # m
ROBOT_FOOTPRINT_RADIUS = 0.075  # m

# Start Positions
START_POSITION_SOUTH = (1.5, 0.3)
START_POSITION_NORTH = (1.5, 2.7)
START_POSITION_EAST = (2.7, 1.5)
START_POSITION_WEST = (0.3, 1.5)

# Yaw angles (radians)
YAW_NORTH = math.pi / 2
YAW_SOUTH = -math.pi / 2
YAW_EAST = 0.0
YAW_WEST = math.pi

# Calibration & Steering
STEERING_LEFT_LIMIT = -45.0  # degrees
STEERING_RIGHT_LIMIT = 44.5  # degrees
STEERING_CENTER = 0.0  # degrees

# IMU offsets (m/s² for accel, rad/s for gyro)
IMU_ACCEL_OFFSET_X = 0.01
IMU_ACCEL_OFFSET_Y = -0.02
IMU_ACCEL_OFFSET_Z = 0.005
IMU_GYRO_OFFSET_X = 0.0
IMU_GYRO_OFFSET_Y = 0.0
IMU_GYRO_OFFSET_Z = 0.0

# Metadata Scenarios
SCENARIO_ID_OPEN = 0
SCENARIO_ID_OBSTACLES = 1
SCENARIO_ID_PARKING = 2

# Simulation Track Model Tests
TRACK_MODEL_CORRIDOR_WIDTH_WIDE = 1.0  # m — wide corridor in track model tests

# Collision detection distances
COLLISION_TEST_NEAR_WALL = 0.05  # m — distance that triggers collision
COLLISION_TEST_FOOTPRINT_CLEARANCE = 0.04  # m — clearance boundary
COLLISION_TEST_INNER_PENETRATION = 1.05  # m — point just inside inner block
COLLISION_TEST_RAYCAST_CLEARANCE = 0.5  # m — raycast distance result

# Parking test configurations (block positions per section)
PARKING_SOUTH_BLOCK1 = (1.00, 0.10)
PARKING_SOUTH_BLOCK2 = (1.30, 0.10)
PARKING_NORTH_BLOCK1 = (1.00, 2.90)
PARKING_NORTH_BLOCK2 = (1.30, 2.90)
PARKING_EAST_BLOCK1 = (2.90, 1.00)
PARKING_EAST_BLOCK2 = (2.90, 1.30)
PARKING_WEST_BLOCK1 = (0.10, 1.00)
PARKING_WEST_BLOCK2 = (0.10, 1.30)

# Parking zone detection thresholds
PARKING_ZONE_CENTER_Y = 0.08  # m
PARKING_ZONE_CENTER_X = 0.08  # m
PARKING_OUTSIDE_MARGIN = 0.80  # m — outside detection threshold

# Threat Direction Strings
THREAT_DIRECTION_FRONT = "front"
THREAT_DIRECTION_BACK = "back"
THREAT_DIRECTION_LEFT = "left"
THREAT_DIRECTION_RIGHT = "right"
THREAT_DIRECTION_NONE = "none"

THREAT_DIRECTIONS = [
    THREAT_DIRECTION_FRONT,
    THREAT_DIRECTION_BACK,
    THREAT_DIRECTION_LEFT,
    THREAT_DIRECTION_RIGHT,
    THREAT_DIRECTION_NONE,
]

# Sign Colors
SIGN_COLOR_RED = "red"
SIGN_COLOR_GREEN = "green"

# Corridor/Section Identifiers
CORRIDOR_SOUTH = "south"
CORRIDOR_NORTH = "north"
CORRIDOR_EAST = "east"
CORRIDOR_WEST = "west"

CORRIDORS = [CORRIDOR_SOUTH, CORRIDOR_NORTH, CORRIDOR_EAST, CORRIDOR_WEST]

# Challenge Types
CHALLENGE_TYPE_OPEN = "open"
CHALLENGE_TYPE_OBSTACLES = "obstacles"

# Direction Strings
DIRECTION_CLOCKWISE = "clockwise"
DIRECTION_COUNTER_CLOCKWISE = "counter_clockwise"

# Self-Detection & Sensor Filtering
SELF_DETECTION_RADIUS = 0.08  # m — chassis/cable reflection threshold
SELF_DETECTION_NEAR = 0.05  # m — inside self-detection zone
SELF_DETECTION_FAR = 0.15  # m — beyond self-detection (genuine wall)

# Risk Assessment Distances (for speed control)
CONTACT_DISTANCE = 0.10  # m — creep forward zone
SLOW_ZONE_DISTANCE = 0.25  # m — slow speed zone
SLOW_ZONE_TEST = 0.20  # m — test slow zone boundary
CONTACT_ZONE_TEST = 0.08  # m — test contact zone

# Approach & Proximity Distances
APPROACH_OFFSET = 0.3  # m — distance from sign when approaching
CLOSE_PROXIMITY_OFFSET = 0.30  # m — approach distance in tests
MEDIUM_OFFSET = 0.5  # m

# Metadata Field Names
META_SCENARIO_ID = "scenario_id"
META_CHALLENGE_TYPE = "challenge_type"
META_CORRIDOR_WIDTHS = "corridor_widths"
META_STARTING_CONDITIONS = "starting_conditions"
META_DIRECTION = "direction"
META_SECTION = "section"
META_POSITION = "position"
META_YAW = "yaw"
META_NUM_SIGNS = "num_signs"
META_SIGN_POSITIONS = "sign_positions"
META_PARKING_LOT = "parking_lot"
META_HAS_PARKING = "has_parking_lot"
META_SEED = "seed"

# Corridor Width Metadata
META_TYPE = "type"
META_WIDTH_MM = "width_mm"
CORRIDOR_TYPE_WIDE = "wide"
CORRIDOR_TYPE_NARROW = "narrow"

# Sign Metadata
META_SIGN_SECTION = "section"
META_SIGN_COLOR = "color"
META_SIGN_DEPTH = "depth"
META_SIGN_LANE = "lane"

# WRO Corridor Widths (mm)
CORRIDOR_WIDTH_WIDE_MM = 1000
CORRIDOR_WIDTH_NARROW_MM = 600

# Parking Metadata
META_PARKING_BLOCKS = "blocks"
META_PARKING_ZONE_START = "zone_start"
META_PARKING_ZONE_END = "zone_end"
META_BLOCK_X = "x"
META_BLOCK_Y = "y"
META_BLOCK_DEPTH = "depth"

# Parking Zone Boundaries
PARKING_ZONE_START = 0.3  # m
PARKING_ZONE_END = 0.7  # m
PARKING_BLOCK_X = 0.1  # m — typical x position for parking blocks
PARKING_BLOCK_Y_MID = 0.5  # m
PARKING_BLOCK_Y_OFFSET = 0.85  # m

# Common Test Yaw Values
YAW_ZERO = 0.0  # 0° (east)
YAW_QUARTER = math.pi / 2  # 90° (north)
YAW_THREE_QUARTER = -math.pi / 2  # -90° (south)
YAW_FULL = math.pi  # 180° (west)
YAW_PI_APPROX = 3.14  # Test approximation of π for metadata

# Sector Angle Indices (additional)
SECTOR_SIZE_6 = 6  # indices around a bearing
SECTOR_SIZE_4 = 4  # smaller sectors

# Fixtures Directory
FIXTURES_DIR = Path(__file__).parent / "fixtures"
