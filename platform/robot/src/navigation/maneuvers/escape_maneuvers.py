"""Escape maneuver planning for stuck robots and collision avoidance.

Pure functions for computing K-turn commands and recovery strategies.
No ROS2 dependencies. Unit testable.
"""

import math
from dataclasses import dataclass

from geometry_msgs.msg import Twist

from src.navigation.config import NavigationConfig


@dataclass
class EscapeCommand:
    """Output of escape maneuver planning."""

    linear_speed: float  # m/s (negative for reverse)
    angular_steering: float  # radians
    duration_frames: int  # How many control cycles to execute this
    escape_type: str  # "k_turn", "side_push", "obstacle_avoidance", "normal"


class EscapeManager:
    """Plan and execute escape maneuvers for stuck robot recovery."""

    def __init__(self, config: NavigationConfig, max_steering_angle: float = math.pi / 6):
        """Initialize escape manager.

        Args:
            config: NavigationConfig with escape parameters.
            max_steering_angle: Maximum steering angle in radians.
        """
        self.config = config
        self.max_steering_angle = max_steering_angle

    def compute_k_turn_command(self, reverse_direction: int = -1) -> EscapeCommand:
        """Generate K-turn (reverse + steer) maneuver.

        A K-turn is: reverse at angle for some distance, then return to straight
        to escape a stuck position.

        Args:
            reverse_direction: -1 for reverse, +1 for forward K-turn.

        Returns:
            EscapeCommand with reverse speed and steering angle.
        """
        esc = self.config.escape

        # Reverse at max steering angle
        linear_speed = reverse_direction * esc.rev_speed
        steering_angle = reverse_direction * esc.steer_scale * self.max_steering_angle

        # Duration depends on escape parameters
        duration = self._compute_escape_duration(forward_clearance=0.05)

        return EscapeCommand(
            linear_speed=linear_speed,
            angular_steering=steering_angle,
            duration_frames=duration,
            escape_type="k_turn",
        )

    def apply_side_correction(self, side_error: float) -> float:
        """Compute steering correction to center away from walls.

        Args:
            side_error: Lateral error from centerline (metres, positive = right).

        Returns:
            Steering angle correction (radians).
        """
        esc = self.config.escape
        correction = esc.steer_scale * side_error
        return max(-self.max_steering_angle, min(self.max_steering_angle, correction))

    def apply_obstacle_avoidance(
        self,
        forward_clearance: float,
        left_clearance: float,
        right_clearance: float,
    ) -> float:
        """Compute steering to avoid forward obstacle.

        Steers toward the clearer side.

        Args:
            forward_clearance: Distance ahead (metres).
            left_clearance: Distance to left (metres).
            right_clearance: Distance to right (metres).

        Returns:
            Steering angle in radians (negative = left, positive = right).
        """
        esc = self.config.escape
        col = self.config.collision

        if forward_clearance > col.active_fwd_dist:
            return 0.0  # Forward is clear, no obstacle avoidance

        # Steer toward clearer side
        side_diff = left_clearance - right_clearance
        if side_diff > 0:
            # Left is clearer
            steering_angle = -esc.obs_steer_scale * self.max_steering_angle
        else:
            # Right is clearer
            steering_angle = esc.obs_steer_scale * self.max_steering_angle

        return steering_angle

    def _compute_escape_duration(self, forward_clearance: float) -> int:
        """Compute K-turn duration based on forward clearance.

        Shorter K-turns in tight spaces, longer on open roads.

        Args:
            forward_clearance: Distance ahead (metres).

        Returns:
            Duration in control cycles (20 Hz = ~50 ms per cycle).
        """
        esc = self.config.escape
        stk = self.config.stuck

        # Base duration on escape distance step
        distance_steps = int(forward_clearance / esc.obs_fwd_speed) if esc.obs_fwd_speed > 0 else 0
        duration = stk.escape_duration + distance_steps

        # Clamp to configured bounds
        return max(esc.obs_rev_speed, min(duration, stk.escape_duration))

    def transition_to_twist(self, escape_cmd: EscapeCommand) -> Twist:
        """Convert EscapeCommand to ROS2 Twist message.

        Args:
            escape_cmd: EscapeCommand with linear and angular velocities.

        Returns:
            geometry_msgs/Twist message.
        """
        twist = Twist()
        twist.linear.x = escape_cmd.linear_speed
        twist.angular.z = escape_cmd.angular_steering
        return twist


def compute_escape_direction(
    left_clearance: float,
    right_clearance: float,
    close_wall_direction: str,
) -> str:
    """Decide escape direction (left, right, or straight).

    Args:
        forward_clearance: Distance ahead (metres).
        left_clearance: Distance to left (metres).
        right_clearance: Distance to right (metres).
        close_wall_direction: Direction of nearby wall ("left", "right", "none").

    Returns:
        Escape direction: "left", "right", or "straight".
    """
    # Avoid steering into close walls
    if close_wall_direction == "left":
        return "right"
    if close_wall_direction == "right":
        return "left"

    # Steer toward clearer side
    if left_clearance > right_clearance:
        return "left"
    return "right"


def compute_obstacle_avoidance_gain(
    forward_clearance: float,
) -> float:
    """Compute dynamic gain for obstacle avoidance.

    Stronger avoidance closer to obstacles.

    Args:
        forward_clearance: Distance to forward obstacle (metres).
        obstacle_gain: Base gain coefficient.

    Returns:
        Scaled gain (0 to 1).
    """
    if forward_clearance <= 0.1:
        return 1.0  # Maximum avoidance
    if forward_clearance >= 0.5:
        return 0.0  # No avoidance needed
    # Linear interpolation
    return 1.0 - (forward_clearance - 0.1) / (0.5 - 0.1)
