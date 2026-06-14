"""Unit tests for CollisionAssessment module.

Tests collision risk assessment and clearance measurement logic.
"""

import math

import numpy as np
import pytest

from src.navigation.config import NavigationConfig
from src.navigation.perception.collision_assessment import ClearanceBand, CollisionAssessor


class TestCollisionAssessor:
    """Tests for CollisionAssessor risk assessment calculations."""

    @pytest.fixture()
    def assessor(self):
        config = NavigationConfig.default()
        return CollisionAssessor(config)

    def test_assess_risk_initialization(self, assessor):
        assert assessor.config is not None
        assert hasattr(assessor.config, "collision")

    def test_risk_level_enum(self):
        assert ClearanceBand.CLEAR in ClearanceBand
        assert ClearanceBand.CAUTION in ClearanceBand
        assert ClearanceBand.WARNING in ClearanceBand
        assert ClearanceBand.CRITICAL in ClearanceBand

    def test_measure_clearance_scan_ahead(self, assessor):
        ranges = np.array([1.0, 1.0, 0.3, 1.0, 1.0])
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, direction="forward", fov_degrees=30)

        assert 0.25 < clearance < 0.35

    def test_measure_clearance_off_to_side(self, assessor):
        # Obstacle only at ±90° — outside ±15° forward cone
        ranges = np.array([0.2, 0.2, 1.0, 0.2, 0.2])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, direction="forward", fov_degrees=30)

        assert clearance > 0.9

    def test_measure_clearance_all_clear(self, assessor):
        ranges = np.array([10.0] * 5)
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, direction="forward", fov_degrees=30)

        assert clearance > 9.0

    def test_measure_clearance_inf_values(self, assessor):
        ranges = np.array([10.0, float("inf"), 1.0, float("inf"), 10.0])
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, direction="forward", fov_degrees=30)

        assert isinstance(clearance, (float, np.floating))
        assert not math.isinf(clearance)

    def test_assess_risk_critical(self, assessor):
        # Forward obstacle inside critical_dist (0.05m)
        ranges = np.full(360, 10.0)
        angles = np.linspace(-math.pi, math.pi, 360)
        ranges[180] = 0.03  # dead ahead, < critical_dist

        risk = assessor.assess_risk(ranges, angles)
        assert risk == ClearanceBand.CRITICAL

    def test_assess_risk_clear(self, assessor):
        ranges = np.full(360, 5.0)
        angles = np.linspace(-math.pi, math.pi, 360)

        risk = assessor.assess_risk(ranges, angles)
        assert risk == ClearanceBand.CLEAR

    def test_assess_risk_empty_scan(self, assessor):
        risk = assessor.assess_risk(np.array([]), np.array([]))
        assert risk == ClearanceBand.CLEAR

    # detect_stuck_robot — current API: detect_stuck_robot(odometry_history, move_threshold, min_samples)
    def test_detect_stuck_robot_moving(self, assessor):
        history = [(0.95, 1.95), (1.0, 2.0)]  # moved ~0.07m > threshold 0.03m
        assert not assessor.detect_stuck_robot(history, min_samples=2)

    def test_detect_stuck_robot_not_moving(self, assessor):
        history = [(1.0, 2.0), (1.0, 2.0)]  # moved 0m < threshold
        assert assessor.detect_stuck_robot(history, min_samples=2)

    def test_detect_stuck_robot_minimum_movement(self, assessor):
        history = [(1.02, 2.01), (1.0, 2.0)]  # moved ~0.022m < threshold 0.03m
        assert assessor.detect_stuck_robot(history, min_samples=2)

    def test_detect_stuck_robot_insufficient_samples(self, assessor):
        history = [(1.0, 2.0)]  # only 1 sample, min_samples=3
        assert not assessor.detect_stuck_robot(history, min_samples=3)

    # detect_close_wall — current API: detect_close_wall(ranges, angles, threshold)
    # Returns: "left", "right", "forward", or "none"
    # Angle convention: +90° = left, -90° = right (standard robot frame)
    def test_detect_close_wall_clear(self, assessor):
        ranges = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        result = assessor.detect_close_wall(ranges, angles)
        assert result == "none"

    def test_detect_close_wall_left_side(self, assessor):
        # Obstacle at +90° (left) within threshold
        ranges = np.array([1.0, 1.0, 1.0, 1.0, 0.12])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        result = assessor.detect_close_wall(ranges, angles)
        assert result != "none"

    def test_detect_close_wall_right_side(self, assessor):
        # Obstacle at -90° (right) within threshold
        ranges = np.array([0.12, 1.0, 1.0, 1.0, 1.0])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        result = assessor.detect_close_wall(ranges, angles)
        assert result != "none"
