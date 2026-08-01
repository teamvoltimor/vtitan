"""Centralized test constants to reduce duplication across platform test suite."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from shared.config.constants import (
    CorridorDimensions,
    ParkingLotSpecs,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
)
from shared.config.navigation_tuning import NavigationTuning
from shared.config.starting_zone import STARTING_ZONE_LAYOUT

# Geometry and tuning below are read from the checked-in configuration rather
# than restated. A test constant that repeats a configured value is a second
# source of truth that drifts silently: it keeps passing while describing a
# robot or a mat that no longer exists. Values that are genuinely test
# *fixtures* -- chosen to make an assertion read cleanly, not to mirror
# reality -- stay literal and say so.
_TUNING = NavigationTuning.load_default()

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
CORRIDOR_DEPTH_MIDPOINT = TrackDimensions.CENTER_COORD  # m — center of track depth
CORRIDOR_DEPTH_MIN = TrackDimensions.CORNER_MIN  # m
CORRIDOR_DEPTH_MAX = TrackDimensions.CORNER_MAX  # m
# The two division lines painted across every corridor, out from the outer wall.
CORRIDOR_WIDTH_QUARTER_NORTH = CorridorDimensions.DIVISION_OUTER  # m
CORRIDOR_WIDTH_QUARTER_SOUTH = CorridorDimensions.DIVISION_INNER  # m

# Track geometry positions
TRACK_CENTER_X = TrackDimensions.CENTER_COORD  # m
TRACK_CENTER_Y = TrackDimensions.CENTER_COORD  # m
# One outer-division line in from each wall.
TRACK_CORNER_SOUTH = CorridorDimensions.DIVISION_OUTER
TRACK_CORNER_NORTH = TrackDimensions.MAX_COORD - CorridorDimensions.DIVISION_OUTER
TRACK_CORNER_EAST = TrackDimensions.MAX_COORD - CorridorDimensions.DIVISION_OUTER
TRACK_CORNER_WEST = CorridorDimensions.DIVISION_OUTER

# Inner block geometry — the corner region bounds are the block's own extent.
INNER_BLOCK_MIN = TrackDimensions.CORNER_MIN  # m
INNER_BLOCK_MAX = TrackDimensions.CORNER_MAX  # m

# Sign Router Test Config
# A deliberate round FIXTURE value, injected into the router under test so the
# deformation-direction assertions read cleanly. NOT the production offset:
# that is _SIGN_LATERAL_OFFSET (0.2786 at the current chassis), derived from the
# chassis half-DIAGONAL, and pinned separately by
# TestLateralOffsetTracksChassis. This previously carried a comment claiming the
# half-WIDTH derivation, which was superseded when the offset moved to the
# diagonal — and which does not evaluate to 0.20 at the measured 0.194 m width
# anyway. Read as "some offset", not "the offset".
SIGN_LATERAL_OFFSET = 0.20  # m — fixture value only; see comment above
SIGN_ACTIVATION_DIST = _TUNING.sign_router.ACTIVATION_DIST_M  # m
SIGN_PASSED_DIST = _TUNING.sign_router.PASSED_DIST_M  # m

# Sign position grid: every intersection of the three grid rows along the
# corridor with the two division lines across it. Six per corridor, by
# construction rather than by transcription.
SIGN_GRID_POSITIONS: list[tuple[float, float]] = [
    (depth, width)
    for depth in (
        TrafficSignSpecs.GRID_DEPTH_NEAR,
        TrafficSignSpecs.GRID_DEPTH_MIDDLE,
        TrafficSignSpecs.GRID_DEPTH_FAR,
    )
    for width in (TrafficSignSpecs.GRID_WIDTH_OUTER, TrafficSignSpecs.GRID_WIDTH_INNER)
]

# Robot Specs
ROBOT_CHASSIS_WIDTH = RobotSpecs.WIDTH  # m
ROBOT_FOOTPRINT_RADIUS = ROBOT_CHASSIS_WIDTH / 2  # m

# Start Positions — the outer band's spawn offset, in from each wall, at the
# track's midpoint along the corridor. Derived from the starting-zone layout so
# a re-measured chassis moves these with it: the offset is the chassis pushed
# flush against a band edge, not a round number.
_START_OFFSET = STARTING_ZONE_LAYOUT.spawn_offsets[0]
_START_MID = TrackDimensions.CENTER_COORD
_START_FAR = TrackDimensions.MAX_COORD - _START_OFFSET

START_POSITION_SOUTH = (_START_MID, _START_OFFSET)
START_POSITION_NORTH = (_START_MID, _START_FAR)
START_POSITION_EAST = (_START_FAR, _START_MID)
START_POSITION_WEST = (_START_OFFSET, _START_MID)

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
TRACK_MODEL_CORRIDOR_WIDTH_WIDE = CorridorDimensions.WIDE  # m

# Collision detection distances
COLLISION_TEST_NEAR_WALL = 0.05  # m — distance that triggers collision
COLLISION_TEST_FOOTPRINT_CLEARANCE = 0.04  # m — clearance boundary
COLLISION_TEST_INNER_PENETRATION = 1.05  # m — point just inside inner block
COLLISION_TEST_RAYCAST_CLEARANCE = 0.5  # m — raycast distance result

# Parking test configurations (block positions per section)
#
# Spacing is derived from the same rule the Go generator uses
# (``ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH``, see
# ``randomize.go::GenerateParkingLotPositions``) rather than written as a literal. It used
# to be a hardcoded 0.30 m, which is exactly the chassis length — a zero-clearance bay the
# robot can never enter, and 0.15 m narrower than any bay the generator actually emits. Tests
# built on it were parking into geometry that does not occur in a real scenario.
#
# The blocks sit ``ParkingLotSpecs.WALL_OFFSET`` from the outer wall and stand perpendicular
# to it, so the bay they form is ``ParkingLotSpecs.LENGTH`` deep.
PARKING_BLOCK_SPACING = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH  # 0.45 m
_PARK_A = 1.00
_PARK_B = _PARK_A + PARKING_BLOCK_SPACING
_PARK_NEAR = ParkingLotSpecs.WALL_OFFSET  # 0.10 m from the wall
_PARK_FAR = TrackDimensions.MAX_COORD - ParkingLotSpecs.WALL_OFFSET  # 2.90 m

PARKING_SOUTH_BLOCK1 = (_PARK_A, _PARK_NEAR)
PARKING_SOUTH_BLOCK2 = (_PARK_B, _PARK_NEAR)
PARKING_NORTH_BLOCK1 = (_PARK_A, _PARK_FAR)
PARKING_NORTH_BLOCK2 = (_PARK_B, _PARK_FAR)
PARKING_EAST_BLOCK1 = (_PARK_FAR, _PARK_A)
PARKING_EAST_BLOCK2 = (_PARK_FAR, _PARK_B)
PARKING_WEST_BLOCK1 = (_PARK_NEAR, _PARK_A)
PARKING_WEST_BLOCK2 = (_PARK_NEAR, _PARK_B)

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

# Risk Assessment Distances (for speed control) — the tuned clearance zones.
CONTACT_DISTANCE = _TUNING.clearance.CONTACT_DIST  # m — creep forward zone
SLOW_ZONE_DISTANCE = _TUNING.clearance.SLOW_DIST  # m — slow speed zone
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

# Sign Metadata -- shared.domain.models.SignPosition: {x, y, color}
META_SIGN_COLOR = "color"

# WRO Corridor Widths (mm) — the metadata schema carries millimetres.
CORRIDOR_WIDTH_WIDE_MM = round(CorridorDimensions.WIDE * 1000)
CORRIDOR_WIDTH_NARROW_MM = round(CorridorDimensions.NARROW * 1000)

# Parking Metadata -- shared.domain.models.ParkingLot: {block1_position, block2_position},
# each a BlockPosition {x, y}. No zone_start/zone_end or per-block depth in this schema.
META_BLOCK1_POSITION = "block1_position"
META_BLOCK2_POSITION = "block2_position"
META_BLOCK_X = "x"
META_BLOCK_Y = "y"

# Parking Block Positions
PARKING_BLOCK_X = ParkingLotSpecs.WALL_OFFSET  # m — blocks stand this far off the wall
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
