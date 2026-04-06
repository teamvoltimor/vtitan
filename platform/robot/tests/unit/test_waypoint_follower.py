"""Unit tests for WaypointFollower module.

Tests pure-pursuit geometry calculations without ROS2 dependencies.
"""

import math
import pytest
from src.navigation.config import NavigationConfig
from src.navigation.waypoint_follower import WaypointFollower


class TestWaypointFollower:
    """Tests for WaypointFollower pure-pursuit calculations."""

    @pytest.fixture
    def follower(self):
        """Create a WaypointFollower instance for testing."""
        config = NavigationConfig.default()
        return WaypointFollower(config, max_steering_angle=math.radians(35))

    def test_lookahead_distance_short_clearance(self, follower):
        """Test short lookahead distance when forward clearance is limited."""
        distance = follower.calculate_lookahead_distance(forward_clearance=0.10)
        assert 0.15 < distance < 0.25, "Should use short lookahead when clearance is limited"

    def test_lookahead_distance_long_clearance(self, follower):
        """Test long lookahead distance when forward clearance is clear."""
        distance = follower.calculate_lookahead_distance(forward_clearance=0.50)
        assert 0.30 < distance < 0.50, "Should use long lookahead when clearance is clear"

    def test_target_point_ahead_on_path(self, follower):
        """Test target point computation for waypoint directly ahead."""
        current_pos = (0.0, 0.0)
        current_yaw = 0.0
        waypoint = (0.5, 0.0)
        lookahead = 0.3

        target = follower.compute_target_point(
            current_pos,
            current_yaw,
            waypoint,
            lookahead,
        )

        # For waypoint directly ahead, target should be lookahead distance away
        assert abs(target[0] - 0.3) < 0.01, "Target x should be lookahead distance"
        assert abs(target[1] - 0.0) < 0.01, "Target y should be on path"

    def test_target_point_off_to_side(self, follower):
        """Test target point computation for waypoint off to the side."""
        current_pos = (0.0, 0.0)
        current_yaw = 0.0
        waypoint = (0.3, 0.2)
        lookahead = 0.3

        target = follower.compute_target_point(
            current_pos,
            current_yaw,
            waypoint,
            lookahead,
        )

        # Target should be at lookahead distance toward the waypoint
        dist_to_target = math.sqrt(target[0] ** 2 + target[1] ** 2)
        assert abs(dist_to_target - lookahead) < 0.01, "Target distance should equal lookahead"

    def test_cross_track_error_on_path(self, follower):
        """Test cross-track error when robot is on the path."""
        current_pos = (0.0, 0.0)
        waypoint_start = (0.0, 0.0)
        waypoint_end = (1.0, 0.0)

        error = follower.compute_cross_track_error(
            current_pos,
            waypoint_start,
            waypoint_end,
        )

        assert abs(error) < 0.001, "Error should be near zero when on path"

    def test_cross_track_error_left_of_path(self, follower):
        """Test cross-track error when robot is left of the path."""
        current_pos = (0.5, 0.1)
        waypoint_start = (0.0, 0.0)
        waypoint_end = (1.0, 0.0)

        error = follower.compute_cross_track_error(
            current_pos,
            waypoint_start,
            waypoint_end,
        )

        assert error > 0, "Error should be positive when left of path"
        assert abs(error - 0.1) < 0.001, "Error magnitude should match offset"

    def test_cross_track_error_right_of_path(self, follower):
        """Test cross-track error when robot is right of the path."""
        current_pos = (0.5, -0.1)
        waypoint_start = (0.0, 0.0)
        waypoint_end = (1.0, 0.0)

        error = follower.compute_cross_track_error(
            current_pos,
            waypoint_start,
            waypoint_end,
        )

        assert error < 0, "Error should be negative when right of path"
        assert abs(error + 0.1) < 0.001, "Error magnitude should match offset"

    def test_steering_angle_straight_path(self, follower):
        """Test steering angle for straight path ahead."""
        current_yaw = 0.0
        target_point = (0.3, 0.0)
        current_pos = (0.0, 0.0)

        angle = follower.compute_steering_angle(
            current_yaw,
            target_point,
            current_pos,
        )

        assert abs(angle) < 0.01, "Steering angle should be near zero for straight path"

    def test_steering_angle_left_turn(self, follower):
        """Test steering angle for left turn."""
        current_yaw = 0.0
        target_point = (0.2, 0.1)  # Above the x-axis
        current_pos = (0.0, 0.0)

        angle = follower.compute_steering_angle(
            current_yaw,
            target_point,
            current_pos,
        )

        assert angle > 0, "Steering angle should be positive (left) for target above"

    def test_steering_angle_right_turn(self, follower):
        """Test steering angle for right turn."""
        current_yaw = 0.0
        target_point = (0.2, -0.1)  # Below the x-axis
        current_pos = (0.0, 0.0)

        angle = follower.compute_steering_angle(
            current_yaw,
            target_point,
            current_pos,
        )

        assert angle < 0, "Steering angle should be negative (right) for target below"

    def test_steering_angle_clamped_to_max(self, follower):
        """Test that steering angle is clamped to max steering angle."""
        current_yaw = 0.0
        target_point = (0.1, 0.5)  # Sharp left turn
        current_pos = (0.0, 0.0)

        angle = follower.compute_steering_angle(
            current_yaw,
            target_point,
            current_pos,
        )

        assert abs(angle) <= follower.max_steering_angle, "Steering angle should be clamped to max"

    def test_steering_angle_respects_heading(self, follower):
        """Test that steering angle accounts for robot heading."""
        current_yaw = math.radians(45)  # Robot facing NE
        target_point = (0.2, 0.0)  # Target ahead in world frame
        current_pos = (0.0, 0.0)

        # When robot is turned, the relative target position changes
        angle = follower.compute_steering_angle(
            current_yaw,
            target_point,
            current_pos,
        )

        # The angle should be well-defined and within bounds
        assert isinstance(angle, float), "Steering angle should be a float"
        assert abs(angle) <= follower.max_steering_angle, "Angle should be within bounds"
