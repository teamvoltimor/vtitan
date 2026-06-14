"""Pure pursuit waypoint following controller for Ackermann geometry.

Implements the pure pursuit steering algorithm for waypoint-following
with lookahead distance adaptation based on forward clearance.

Reference: https://www.ri.cmu.edu/pub_files/pub3/coulter_1992_1.pdf
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)


class WaypointController:
    """Pure pursuit steering controller for waypoint following.

    Computes steering angle to intercept a lookahead point on the planned
    path. Adapts lookahead distance based on forward clearance to handle
    corners (short lookahead) vs. straights (long lookahead).

    Attributes:
        max_steering_angle: Physical steering limit in radians
        lookahead_short: Distance for sharp corners (meters)
        lookahead_long: Distance for straight sections (meters)
        lookahead_transition: Forward clearance threshold (meters)
        steer_kp: P-gain for steering control
        max_steering_rate: Maximum steering command rate (rad/s)
    """

    def __init__(
        self,
        max_steering_angle: float = 0.5236,  # ~30 degrees
        lookahead_short: float = 0.20,
        lookahead_long: float = 0.40,
        lookahead_transition: float = 0.30,
        steer_kp: float = 1.5,
        max_steering_rate: float = 2.0,
    ):
        """Initialize pure pursuit controller.

        Args:
            max_steering_angle: Max steering angle in radians
            lookahead_short: Lookahead for corners (m)
            lookahead_long: Lookahead for straights (m)
            lookahead_transition: Crosstrack threshold for mode switch (m)
            steer_kp: P-controller gain
            max_steering_rate: Max steering rate (rad/s)
        """
        self.max_steering_angle = max_steering_angle
        self.lookahead_short = lookahead_short
        self.lookahead_long = lookahead_long
        self.lookahead_transition = lookahead_transition
        self.steer_kp = steer_kp
        self.max_steering_rate = max_steering_rate

    def select_lookahead(self, forward_clearance: float) -> float:
        """Select lookahead distance based on forward clearance.

        Args:
            forward_clearance: Distance to nearest forward obstacle (meters)

        Returns:
            Lookahead distance in meters
        """
        if forward_clearance < self.lookahead_transition:
            return self.lookahead_short
        return self.lookahead_long

    def compute_steering(
        self,
        current_pos: tuple[float, float],
        current_yaw: float,
        target_waypoint: tuple[float, float],
        forward_clearance: float,
    ) -> tuple[float, float]:
        """Compute steering angle and lookahead for next control step.

        Uses pure pursuit: finds the intersection of circle centered at robot
        position (with radius = lookahead distance) with the planned path,
        then steers toward that intersection point.

        Args:
            current_pos: Robot position (x, y)
            current_yaw: Robot heading (radians)
            target_waypoint: Next waypoint (x, y)
            forward_clearance: Distance to forward obstacle (meters)

        Returns:
            Tuple of (steering_angle, lookahead_distance)
            steering_angle: Command in [-1, 1] normalized range
            lookahead_distance: Selected lookahead (for diagnostics)
        """
        lookahead = self.select_lookahead(forward_clearance)

        # Vector from robot to target
        dx = target_waypoint[0] - current_pos[0]
        dy = target_waypoint[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.01:  # Waypoint reached
            return 0.0, lookahead

        # Pure pursuit: steering angle to intercept lookahead circle
        # Simplified version: angle error to target
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - current_yaw

        # Wrap angle to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # P-controller on angle error
        steering_rad = self.steer_kp * angle_error
        steering_rad = max(-self.max_steering_angle, min(self.max_steering_angle, steering_rad))

        # Normalize to [-1, 1]
        steering_normalized = steering_rad / self.max_steering_angle

        return steering_normalized, lookahead
