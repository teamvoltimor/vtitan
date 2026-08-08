"""
Pytest configuration and fixtures for robot tests.
"""

import json
import sys
from pathlib import Path
from typing import Any

import pytest

# Ensure shared package is available
_this_dir = Path(__file__).resolve().parent
_shared_src = (_this_dir.parent.parent / "shared" / "src").resolve()
if str(_shared_src) not in sys.path:
    sys.path.insert(0, str(_shared_src))

from shared.config.navigation_tuning import NavigationTuning

from tests.fixtures import LidarScanBuilder
from tests.test_constants import (
    TuningDerivedConstants,
    CHALLENGE_TYPE_OBSTACLES,
    CHALLENGE_TYPE_OPEN,
    CORRIDOR_EAST,
    CORRIDOR_NORTH,
    CORRIDOR_SOUTH,
    CORRIDOR_TYPE_NARROW,
    CORRIDOR_TYPE_WIDE,
    CORRIDOR_WEST,
    CORRIDOR_WIDTH_NARROW_MM,
    CORRIDOR_WIDTH_QUARTER_SOUTH,
    CORRIDOR_WIDTH_WIDE_MM,
    DIRECTION_CLOCKWISE,
    FIXTURES_DIR,
    IMU_ACCEL_OFFSET_X,
    IMU_ACCEL_OFFSET_Y,
    IMU_ACCEL_OFFSET_Z,
    IMU_GYRO_OFFSET_X,
    IMU_GYRO_OFFSET_Y,
    IMU_GYRO_OFFSET_Z,
    LIDAR_DEFAULT_FAR,
    LIDAR_NEAR_WALL,
    LIDAR_WALL_DISTANCE,
    META_BLOCK1_POSITION,
    META_BLOCK2_POSITION,
    META_BLOCK_X,
    META_BLOCK_Y,
    META_CHALLENGE_TYPE,
    META_CORRIDOR_WIDTHS,
    META_DIRECTION,
    META_HAS_PARKING,
    META_NUM_SIGNS,
    META_PARKING_LOT,
    META_POSITION,
    META_SCENARIO_ID,
    META_SECTION,
    META_SIGN_COLOR,
    META_SIGN_POSITIONS,
    META_STARTING_CONDITIONS,
    META_TYPE,
    META_WIDTH_MM,
    META_YAW,
    NUM_RAYS,
    PARKING_BLOCK_X,
    PARKING_BLOCK_Y_MID,
    PARKING_BLOCK_Y_OFFSET,
    ROBOT_CHASSIS_WIDTH,
    SIGN_COLOR_GREEN,
    SIGN_COLOR_RED,
    STEERING_CENTER,
    STEERING_LEFT_LIMIT,
    STEERING_RIGHT_LIMIT,
    TRACK_CENTER_X,
    TRACK_CENTER_Y,
    TRACK_CORNER_EAST,
    TRACK_CORNER_NORTH,
    TRACK_CORNER_SOUTH,
    TRACK_CORNER_WEST,
    YAW_PI_APPROX,
)


@pytest.fixture()
def tuning():
    """NavigationTuning fixture for tests that need to pass tuning to constructors."""
    return NavigationTuning()


@pytest.fixture()
def tuning_constants(tuning):
    """Tuning-derived test constants computed from fixture instead of frozen at module level."""
    return TuningDerivedConstants.from_tuning(tuning)


@pytest.fixture()
def mock_lidar_scan_360():
    """Mock 360° LIDAR scan with clear path ahead."""
    scan = LidarScanBuilder().clear_path(LIDAR_DEFAULT_FAR).build()
    return list(zip(scan.angles, scan.ranges))


@pytest.fixture()
def mock_lidar_scan_obstacle():
    """Mock LIDAR scan with obstacle at 0°."""
    scan = LidarScanBuilder().obstacle_ahead(distance_m=0.15).build()
    return list(zip(scan.angles, scan.ranges))


@pytest.fixture()
def mock_lidar_scan_wall():
    """Mock LIDAR scan with wall very close."""
    scan = LidarScanBuilder().wall_close(near_m=LIDAR_NEAR_WALL, far_m=LIDAR_WALL_DISTANCE).build()
    return list(zip(scan.angles, scan.ranges))


@pytest.fixture()
def sample_metadata_open():
    """Sample metadata JSON for open challenge."""
    return {
        META_SCENARIO_ID: 0,
        META_CHALLENGE_TYPE: CHALLENGE_TYPE_OPEN,
        META_CORRIDOR_WIDTHS: {
            CORRIDOR_NORTH: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_SOUTH: {META_TYPE: CORRIDOR_TYPE_NARROW, META_WIDTH_MM: CORRIDOR_WIDTH_NARROW_MM},
            CORRIDOR_EAST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_WEST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
        },
        META_STARTING_CONDITIONS: {
            META_DIRECTION: DIRECTION_CLOCKWISE,
            META_SECTION: "South",
            META_POSITION: {META_BLOCK_X: TRACK_CENTER_X, META_BLOCK_Y: CORRIDOR_WIDTH_QUARTER_SOUTH},
            META_YAW: YAW_PI_APPROX,
        },
        META_NUM_SIGNS: 0,
        META_SIGN_POSITIONS: [],
        META_PARKING_LOT: None,
    }


@pytest.fixture()
def sample_metadata_obstacles():
    """Sample metadata JSON for obstacles challenge.

    Matches ``shared.domain.models.ScenarioMetadata`` -- the schema
    ``TrackNavigator._plan()`` actually validates against -- rather than the
    legacy ``ScenarioSimulator`` sign/parking shape this fixture used before
    (section/color/depth/lane, blocks/zone_start/zone_end), which fails
    pydantic validation against the current model.
    """
    return {
        META_SCENARIO_ID: 1,
        META_CHALLENGE_TYPE: CHALLENGE_TYPE_OBSTACLES,
        META_CORRIDOR_WIDTHS: {
            CORRIDOR_NORTH: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_SOUTH: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_EAST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_WEST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
        },
        META_STARTING_CONDITIONS: {
            META_DIRECTION: DIRECTION_CLOCKWISE,
            META_SECTION: "South",
            META_POSITION: {META_BLOCK_X: TRACK_CENTER_X, META_BLOCK_Y: 0.5},
            META_YAW: YAW_PI_APPROX,
        },
        META_NUM_SIGNS: 2,
        META_SIGN_POSITIONS: [
            {META_BLOCK_X: TRACK_CENTER_X, META_BLOCK_Y: TRACK_CORNER_NORTH, META_SIGN_COLOR: SIGN_COLOR_GREEN},
            {META_BLOCK_X: TRACK_CORNER_EAST, META_BLOCK_Y: TRACK_CENTER_Y, META_SIGN_COLOR: SIGN_COLOR_RED},
        ],
        META_PARKING_LOT: None,
    }


@pytest.fixture()
def sample_metadata_parking():
    """Sample metadata JSON with parking.

    See ``sample_metadata_obstacles`` -- same schema-correction rationale.
    """
    return {
        META_SCENARIO_ID: 2,
        META_CHALLENGE_TYPE: CHALLENGE_TYPE_OBSTACLES,
        META_CORRIDOR_WIDTHS: {
            CORRIDOR_NORTH: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_SOUTH: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_EAST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
            CORRIDOR_WEST: {META_TYPE: CORRIDOR_TYPE_WIDE, META_WIDTH_MM: CORRIDOR_WIDTH_WIDE_MM},
        },
        META_STARTING_CONDITIONS: {
            META_DIRECTION: DIRECTION_CLOCKWISE,
            META_SECTION: "South",
            META_POSITION: {META_BLOCK_X: TRACK_CENTER_X, META_BLOCK_Y: 0.5},
            META_YAW: YAW_PI_APPROX,
        },
        META_NUM_SIGNS: 1,
        META_SIGN_POSITIONS: [
            {META_BLOCK_X: TRACK_CENTER_X, META_BLOCK_Y: TRACK_CORNER_NORTH, META_SIGN_COLOR: SIGN_COLOR_GREEN},
        ],
        META_HAS_PARKING: True,
        META_PARKING_LOT: {
            META_BLOCK1_POSITION: {META_BLOCK_X: PARKING_BLOCK_X, META_BLOCK_Y: PARKING_BLOCK_Y_MID},
            META_BLOCK2_POSITION: {META_BLOCK_X: PARKING_BLOCK_X, META_BLOCK_Y: PARKING_BLOCK_Y_OFFSET},
        },
    }


@pytest.fixture()
def calibration_data():
    """Sample calibration data."""
    return {
        "steering": {
            "left_limit": STEERING_LEFT_LIMIT,
            "right_limit": STEERING_RIGHT_LIMIT,
            "center": STEERING_CENTER,
        },
        "imu": {
            "accel_offset": [IMU_ACCEL_OFFSET_X, IMU_ACCEL_OFFSET_Y, IMU_ACCEL_OFFSET_Z],
            "gyro_offset": [IMU_GYRO_OFFSET_X, IMU_GYRO_OFFSET_Y, IMU_GYRO_OFFSET_Z],
        },
    }


@pytest.fixture()
def temp_metadata_file(tmp_path, sample_metadata_open):
    """Create temporary metadata JSON file."""
    metadata_file = tmp_path / "test_metadata.json"
    with open(metadata_file, "w") as f:
        json.dump(sample_metadata_open, f)
    return metadata_file
