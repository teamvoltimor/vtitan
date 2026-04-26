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
        config = NavigationConfig.default()
        return WaypointFollower(config, max_steering_angle=math.radians(35))

    # calculate_lookahead_distance
    def test_lookahead_distance_short_clearance(self, follower):
        distance = follower.calculate_lookahead_distance(forward_clearance=0.10)
        assert 0.15 < distance < 0.25

    def test_lookahead_distance_long_clearance(self, follower):
        distance = follower.calculate_lookahead_distance(forward_clearance=0.50)
        assert 0.30 < distance < 0.50

    def test_lookahead_threshold_transitions(self, follower):
        la = follower.config.lookahead
        below = follower.calculate_lookahead_distance(la.threshold - 0.01)
        above = follower.calculate_lookahead_distance(la.threshold + 0.01)
        assert below == pytest.approx(la.short)
        assert above == pytest.approx(la.long)

    # compute_target_point — API: (current_pos, waypoints_list, waypoint_index, lookahead_distance)
    def test_target_point_ahead_on_path(self, follower):
        # Waypoint beyond lookahead distance → returns that waypoint
        target = follower.compute_target_point(
            current_pos=(0.0, 0.0),
            waypoints=[(0.5, 0.0)],
            waypoint_index=0,
            lookahead_distance=0.3,
        )
        assert target is not None
        assert target[0] >= 0.3

    def test_target_point_off_to_side(self, follower):
        # Waypoint at angle — returned target is the waypoint itself (first beyond lookahead)
        target = follower.compute_target_point(
            current_pos=(0.0, 0.0),
            waypoints=[(0.4, 0.3)],   # distance = 0.5 > lookahead 0.3
            waypoint_index=0,
            lookahead_distance=0.3,
        )
        assert target is not None
        dist = math.sqrt(target[0] ** 2 + target[1] ** 2)
        assert dist >= 0.3

    def test_target_point_empty_waypoints(self, follower):
        target = follower.compute_target_point(
            current_pos=(0.0, 0.0),
            waypoints=[],
            waypoint_index=0,
            lookahead_distance=0.3,
        )
        assert target is None

    def test_target_point_index_past_end(self, follower):
        target = follower.compute_target_point(
            current_pos=(0.0, 0.0),
            waypoints=[(0.5, 0.0)],
            waypoint_index=5,
            lookahead_distance=0.3,
        )
        assert target is None

    def test_target_point_uses_closest_beyond_lookahead(self, follower):
        # Multiple waypoints — first beyond lookahead is chosen
        target = follower.compute_target_point(
            current_pos=(0.0, 0.0),
            waypoints=[(0.1, 0.0), (0.5, 0.0), (1.0, 0.0)],
            waypoint_index=0,
            lookahead_distance=0.3,
        )
        # First waypoint at 0.1 < 0.3, second at 0.5 >= 0.3 → returns (0.5, 0.0)
        assert target == (0.5, 0.0)

    # compute_cross_track_error — API: (current_pos, waypoint_prev, waypoint_next)
    def test_cross_track_error_on_path(self, follower):
        error = follower.compute_cross_track_error(
            current_pos=(0.5, 0.0),
            waypoint_prev=(0.0, 0.0),
            waypoint_next=(1.0, 0.0),
        )
        assert abs(error) < 0.001

    def test_cross_track_error_left_of_path(self, follower):
        error = follower.compute_cross_track_error(
            current_pos=(0.5, 0.1),
            waypoint_prev=(0.0, 0.0),
            waypoint_next=(1.0, 0.0),
        )
        assert error > 0
        assert abs(error - 0.1) < 0.001

    def test_cross_track_error_right_of_path(self, follower):
        error = follower.compute_cross_track_error(
            current_pos=(0.5, -0.1),
            waypoint_prev=(0.0, 0.0),
            waypoint_next=(1.0, 0.0),
        )
        assert error < 0
        assert abs(error + 0.1) < 0.001

    # compute_steering_angle — API: (current_pos, current_yaw, target_pos)
    def test_steering_angle_straight_path(self, follower):
        angle = follower.compute_steering_angle(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            target_pos=(0.3, 0.0),
        )
        assert abs(angle) < 0.01

    def test_steering_angle_left_turn(self, follower):
        # Target above x-axis → should steer left (positive)
        angle = follower.compute_steering_angle(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            target_pos=(0.2, 0.1),
        )
        assert angle > 0

    def test_steering_angle_right_turn(self, follower):
        # Target below x-axis → should steer right (negative)
        angle = follower.compute_steering_angle(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            target_pos=(0.2, -0.1),
        )
        assert angle < 0

    def test_steering_angle_clamped_to_max(self, follower):
        # Very sharp turn — must be clamped
        angle = follower.compute_steering_angle(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            target_pos=(0.1, 0.5),
        )
        assert abs(angle) <= follower.max_steering_angle

    def test_steering_angle_respects_heading(self, follower):
        angle = follower.compute_steering_angle(
            current_pos=(0.0, 0.0),
            current_yaw=math.radians(45),
            target_pos=(0.2, 0.0),
        )
        assert isinstance(angle, float)
        assert abs(angle) <= follower.max_steering_angle
