"""Unit tests for CollisionAssessment module.

Tests collision risk assessment and clearance measurement logic.
"""

import math
import pytest
import numpy as np
from src.navigation.config import NavigationConfig
from src.navigation.collision_assessment import CollisionAssessor, RiskLevel


class TestCollisionAssessor:
    """Tests for CollisionAssessor risk assessment calculations."""

    @pytest.fixture
    def assessor(self):
        """Create a CollisionAssessor instance for testing."""
        config = NavigationConfig.default()
        return CollisionAssessor(config)

    def test_assess_risk_initialization(self, assessor):
        """Test that assessor initializes with valid config."""
        assert assessor.config is not None
        assert hasattr(assessor.config, "collision_avoidance")

    def test_risk_level_enum(self):
        """Test RiskLevel enum values exist."""
        assert RiskLevel.CLEAR in RiskLevel
        assert RiskLevel.CAUTION in RiskLevel
        assert RiskLevel.WARNING in RiskLevel
        assert RiskLevel.CRITICAL in RiskLevel

    def test_measure_clearance_scan_ahead(self, assessor):
        """Test clearance measurement directly ahead."""
        # Scan with obstacle at 0.3m ahead
        ranges = np.array([1.0, 1.0, 0.3, 1.0, 1.0])
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, forward_degree_range=30)

        assert 0.25 < clearance < 0.35, "Should detect obstacle at 0.3m ahead"

    def test_measure_clearance_off_to_side(self, assessor):
        """Test clearance measurement ignores obstacles to the side."""
        # Obstacle only to the side
        ranges = np.array([0.2, 0.2, 1.0, 0.2, 0.2])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, forward_degree_range=30)

        assert clearance > 0.9, "Should ignore side obstacles, measure center as clear"

    def test_measure_clearance_all_clear(self, assessor):
        """Test clearance measurement when path is completely clear."""
        ranges = np.array([10.0] * 5)
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        clearance = assessor.measure_clearance(ranges, angles, forward_degree_range=30)

        assert clearance > 9.0, "Should report full range when clear"

    def test_measure_clearance_inf_values(self, assessor):
        """Test clearance measurement with infinite (invalid) readings."""
        ranges = np.array([10.0, float("inf"), 1.0, float("inf"), 10.0])
        angles = np.array([-45, -22.5, 0, 22.5, 45]) * math.pi / 180

        # Should not crash and should handle inf values
        clearance = assessor.measure_clearance(ranges, angles, forward_degree_range=30)

        assert isinstance(clearance, (float, np.floating)), "Should return valid float"
        assert not math.isinf(clearance), "Should not return infinity"

    def test_detect_stuck_robot_moving(self, assessor):
        """Test stuck detection when robot is moving."""
        current_pos = (1.0, 2.0)
        last_pos = (0.95, 1.95)

        is_stuck = assessor.detect_stuck_robot(
            current_pos=current_pos,
            last_pos=last_pos,
            frames_in_state=5,
        )

        assert not is_stuck, "Robot moving should not be stuck"

    def test_detect_stuck_robot_not_moving(self, assessor):
        """Test stuck detection when robot is stationary."""
        current_pos = (1.0, 2.0)
        last_pos = (1.0, 2.0)

        is_stuck = assessor.detect_stuck_robot(
            current_pos=current_pos,
            last_pos=last_pos,
            frames_in_state=20,
        )

        assert is_stuck, "Robot not moving should be detected as stuck"

    def test_detect_stuck_robot_minimum_movement(self, assessor):
        """Test stuck detection at movement threshold boundary."""
        current_pos = (1.0, 2.0)
        last_pos = (1.02, 2.01)  # Moved ~0.022m

        is_stuck = assessor.detect_stuck_robot(
            current_pos=current_pos,
            last_pos=last_pos,
            frames_in_state=15,
        )

        assert is_stuck, "Movement below threshold should be stuck"

    def test_detect_stuck_robot_insufficient_time(self, assessor):
        """Test stuck detection when not enough time has passed."""
        current_pos = (1.0, 2.0)
        last_pos = (1.0, 2.0)

        is_stuck = assessor.detect_stuck_robot(
            current_pos=current_pos,
            last_pos=last_pos,
            frames_in_state=5,  # Less than threshold
        )

        assert not is_stuck, "Should not be stuck if insufficient frames elapsed"

    def test_detect_close_wall_clear(self, assessor):
        """Test close wall detection when walls are far away."""
        ranges = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        has_wall = assessor.detect_close_wall(ranges, angles, side_degree_range=60)

        assert not has_wall, "Should not detect wall when all sides clear"

    def test_detect_close_wall_left_side(self, assessor):
        """Test close wall detection on left side."""
        ranges = np.array([0.15, 0.15, 1.0, 1.0, 1.0])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        has_wall = assessor.detect_close_wall(ranges, angles, side_degree_range=60)

        assert has_wall, "Should detect wall on left side"

    def test_detect_close_wall_right_side(self, assessor):
        """Test close wall detection on right side."""
        ranges = np.array([1.0, 1.0, 1.0, 0.15, 0.15])
        angles = np.array([-90, -45, 0, 45, 90]) * math.pi / 180

        has_wall = assessor.detect_close_wall(ranges, angles, side_degree_range=60)

        assert has_wall, "Should detect wall on right side"
