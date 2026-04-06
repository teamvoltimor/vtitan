"""Pure-pursuit waypoint following controller.

Computes steering angles and lookahead distances to track a sequence of
waypoints using pure-pursuit geometry (no ROS2 dependencies, unit testable).
"""

import math

import numpy as np

from src.navigation.config import NavigationConfig


class WaypointFollower:
    """Pure-pursuit waypoint following with adaptive lookahead distance.

    Follows a sequence of waypoints using pure-pursuit geometry, which
    computes steering angle based on cross-track error.
    """

    def __init__(self, config: NavigationConfig, max_steering_angle: float = math.pi / 6):
        """Initialize waypoint follower.

        Args:
            config: NavigationConfig with lookahead and steering parameters.
            max_steering_angle: Maximum steering angle in radians.
        """
        self.config = config
        self.max_steering_angle = max_steering_angle

    def calculate_lookahead_distance(self, forward_clearance: float) -> float:
        """Adaptively select lookahead distance based on forward clearance.

        Uses short lookahead near corners (when clearance is low),
        long lookahead on open roads.

        Args:
            forward_clearance: Distance to nearest obstacle ahead (metres).

        Returns:
            Lookahead distance in metres.
        """
        la = self.config.lookahead

        if forward_clearance < la.threshold:
            return la.short  # Near corner — short lookahead
        else:
            return la.long  # Open road — long lookahead

    def compute_target_point(
        self,
        current_pos: tuple[float, float],
        waypoints: list[tuple[float, float]],
        waypoint_index: int,
        lookahead_distance: float,
    ) -> tuple[float, float] | None:
        """Find target point ahead on waypoint path.

        Searches forward from current waypoint index to find the first point
        that is at least lookahead_distance away from robot position.

        Args:
            current_pos: (x, y) robot position in world frame.
            waypoints: Ordered list of (x, y) waypoint positions.
            waypoint_index: Current waypoint index.
            lookahead_distance: Search distance in metres.

        Returns:
            (x, y) target point, or None if insufficient waypoints ahead.
        """
        if not waypoints or waypoint_index >= len(waypoints):
            return None

        cx, cy = current_pos

        # Search forward from current index
        for i in range(waypoint_index, len(waypoints)):
            wx, wy = waypoints[i]
            dist = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)

            if dist >= lookahead_distance:
                return (wx, wy)

        # Fallback: use last waypoint if nothing found at lookahead distance
        return waypoints[-1]

    def compute_steering_angle(
        self,
        current_pos: tuple[float, float],
        current_yaw: float,
        target_pos: tuple[float, float],
    ) -> float:
        """Compute steering angle to target using pure-pursuit geometry.

        Pure-pursuit steers toward a lookahead point such that the steering
        angle magnitude is proportional to the cross-track error.

        Args:
            current_pos: (x, y) robot position.
            current_yaw: Robot heading (radians, 0 = East).
            target_pos: (x, y) target lookahead point.

        Returns:
            Steering angle in radians (negative = left, positive = right).
            Clamped to max_steering_angle.
        """
        cx, cy = current_pos
        tx, ty = target_pos

        # Vector from robot to target
        dx = tx - cx
        dy = ty - cy

        # Target bearing relative to world frame
        target_bearing = math.atan2(dy, dx)

        # Heading error (how far off we are from target direction)
        heading_error = _wrap_angle(target_bearing - current_yaw)

        # Pure-pursuit steering law: angle ∝ cross-track error
        # P-gain from config
        kp = self.config.steering.kp
        steering_angle = kp * heading_error

        # Clamp to max steering angle
        return np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

    def compute_cross_track_error(
        self,
        current_pos: tuple[float, float],
        waypoint_prev: tuple[float, float],
        waypoint_next: tuple[float, float],
    ) -> float:
        """Compute cross-track error between waypoints.

        Perpendicular distance from robot to the line segment between two waypoints.

        Args:
            current_pos: (x, y) robot position.
            waypoint_prev: Previous waypoint.
            waypoint_next: Next waypoint.

        Returns:
            Cross-track error in metres (positive = right of path, negative = left).
        """
        x0, y0 = current_pos
        x1, y1 = waypoint_prev
        x2, y2 = waypoint_next

        # Line segment vector
        dx = x2 - x1
        dy = y2 - y1
        seg_length_sq = dx**2 + dy**2

        if seg_length_sq < 1e-6:
            # Degenerate segment; use distance to point
            return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

        # Project robot onto line segment
        t = ((x0 - x1) * dx + (y0 - y1) * dy) / seg_length_sq
        t = np.clip(t, 0, 1)

        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Signed error (positive = right)
        error = ((x0 - closest_x) * (-dy) + (y0 - closest_y) * dx) / math.sqrt(seg_length_sq)

        return error


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π] radians."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
