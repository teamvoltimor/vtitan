"""
Pytest configuration and fixtures for robot tests.
"""

import json
from pathlib import Path
from typing import Any

import pytest


FIXTURES_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def mock_lidar_scan_360():
    """Mock 360° LIDAR scan with clear path ahead."""
    num_points = 360
    angles = [i for i in range(num_points)]
    distances = [2.0] * num_points

    return list(zip(angles, distances))


@pytest.fixture
def mock_lidar_scan_obstacle():
    """Mock LIDAR scan with obstacle at 0°."""
    num_points = 360
    angles = [i for i in range(num_points)]

    distances = []
    for angle in angles:
        if -10 <= angle <= 10:
            distances.append(0.15)
        else:
            distances.append(2.0)

    return list(zip(angles, distances))


@pytest.fixture
def mock_lidar_scan_wall():
    """Mock LIDAR scan with wall very close."""
    num_points = 360
    angles = [i for i in range(num_points)]

    distances = []
    for angle in angles:
        if -5 <= angle <= 5:
            distances.append(0.05)
        else:
            distances.append(1.5)

    return list(zip(angles, distances))


@pytest.fixture
def sample_metadata_open():
    """Sample metadata JSON for open challenge."""
    return {
        "scenario_id": 0,
        "challenge_type": "open",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "narrow", "width_mm": 600},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.3},
            "yaw": 3.14,
        },
        "num_signs": 0,
        "sign_positions": [],
        "parking_lot": None,
    }


@pytest.fixture
def sample_metadata_obstacles():
    """Sample metadata JSON for obstacles challenge."""
    return {
        "scenario_id": 1,
        "challenge_type": "obstacles",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "wide", "width_mm": 1000},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.5},
            "yaw": 3.14,
        },
        "num_signs": 2,
        "sign_positions": [
            {"section": "north", "color": "green", "depth": 1.5, "lane": 0.4},
            {"section": "east", "color": "red", "depth": 2.0, "lane": 0.6},
        ],
        "parking_lot": None,
    }


@pytest.fixture
def sample_metadata_parking():
    """Sample metadata JSON with parking."""
    return {
        "scenario_id": 2,
        "challenge_type": "obstacles",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "wide", "width_mm": 1000},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.5},
            "yaw": 3.14,
        },
        "num_signs": 1,
        "sign_positions": [
            {"section": "north", "color": "green", "depth": 1.5, "lane": 0.4},
        ],
        "parking_lot": {
            "blocks": [
                {"x": 0.1, "y": 0.5, "depth": 1.0},
                {"x": 0.1, "y": 0.85, "depth": 1.5},
            ],
            "zone_start": 0.3,
            "zone_end": 0.7,
        },
    }


@pytest.fixture
def calibration_data():
    """Sample calibration data."""
    return {
        "steering": {
            "left_limit": -45.0,
            "right_limit": 44.5,
            "center": 0.0,
        },
        "imu": {
            "accel_offset": [0.01, -0.02, 0.005],
            "gyro_offset": [0.0, 0.0, 0.0],
        },
    }


@pytest.fixture
def temp_metadata_file(tmp_path, sample_metadata_open):
    """Create temporary metadata JSON file."""
    metadata_file = tmp_path / "test_metadata.json"
    with open(metadata_file, "w") as f:
        json.dump(sample_metadata_open, f)
    return metadata_file
