"""Speed scaling and command generation for navigation.

Pure functions that compute speed commands based on:
- Forward clearance zones (contact, slow, medium, fast)
- Heading error thresholds (crawl, slow, medium)
- Target speed fractions from NavigationConfig

No ROS2 dependencies. Unit testable.
"""

import math

import numpy as np

from src.navigation.config import NavigationConfig


class SpeedScaler:
    """Compute scaled speed commands based on clearance and heading error.

    Combines forward clearance zones with heading error zones to produce
    smooth speed commands that respect both obstacle distance and alignment.
    """

    def __init__(self, config: NavigationConfig):
        """Initialize speed scaler with navigation configuration.

        Args:
            config: NavigationConfig with clearance, heading_error, and speed zones.
        """
        self.config = config

    def scale_for_clearance(self, forward_clearance: float) -> float:
        """Compute speed fraction based on forward clearance zone.

        Args:
            forward_clearance: Distance to nearest obstacle ahead (metres).

        Returns:
            Speed fraction 0.0 to 1.0 based on clearance zone.
        """
        c = self.config.clearance
        s = self.config.speed

        if forward_clearance < c.contact_dist:
            return s.contact  # Very close — creep
        elif forward_clearance < c.slow_dist:
            return s.slow  # Slow zone
        elif forward_clearance < c.medium_dist:
            return s.medium  # Medium zone
        elif forward_clearance < c.fast_dist:
            return s.fast  # Fast zone
        else:
            return s.full  # Clear path — full speed

    def scale_for_heading_error(self, heading_error: float) -> float:
        """Compute speed reduction based on heading error magnitude.

        Args:
            heading_error: Absolute heading error (radians).

        Returns:
            Speed fraction 0.0 to 1.0 (1.0 = no reduction).
        """
        abs_error = abs(heading_error)
        h = self.config.heading_error
        s = self.config.speed

        if abs_error >= h.crawl:
            return s.err_crawl  # Worst-case misalignment
        elif abs_error >= h.slow:
            return s.err_slow  # Large heading error
        elif abs_error >= h.medium:
            return s.err_medium  # Moderate heading error
        else:
            return 1.0  # Small error — no reduction

    def scale_combined(self, forward_clearance: float, heading_error: float) -> float:
        """Compute combined speed command (clearance ∩ heading error).

        Takes the minimum of both scaling factors to respect both constraints.

        Args:
            forward_clearance: Distance to nearest obstacle ahead (metres).
            heading_error: Absolute heading error (radians).

        Returns:
            Combined speed fraction 0.0 to 1.0.
        """
        clearance_scale = self.scale_for_clearance(forward_clearance)
        heading_scale = self.scale_for_heading_error(heading_error)
        return min(clearance_scale, heading_scale)

    def compute_speed_command(
        self,
        forward_clearance: float,
        heading_error: float,
        max_linear_speed: float = 1.0,
    ) -> float:
        """Compute final linear speed command in m/s.

        Args:
            forward_clearance: Distance to nearest obstacle (metres).
            heading_error: Absolute heading error (radians).
            max_linear_speed: Maximum achievable speed in m/s.

        Returns:
            Speed command in m/s (clamped to max_linear_speed).
        """
        scale = self.scale_combined(forward_clearance, heading_error)
        return scale * max_linear_speed


def compute_clearance_zone(
    forward_ranges: np.ndarray,
    forward_angles: np.ndarray,
    fov_degrees: float = 60.0,
) -> float:
    """Measure forward clearance within a field of view.

    Args:
        forward_ranges: LIDAR range array (metres).
        forward_angles: LIDAR angle array (radians).
        fov_degrees: Field of view cone width (degrees).

    Returns:
        Minimum range within FOV (metres), or inf if no data.
    """
    if forward_ranges is None or len(forward_ranges) == 0:
        return float("inf")

    fov_rad = math.radians(fov_degrees / 2)

    # Find angles within FOV (±fov_rad from forward direction)
    within_fov = np.abs(forward_angles) <= fov_rad
    if not np.any(within_fov):
        return float("inf")

    # Return minimum range in FOV
    return float(np.min(forward_ranges[within_fov]))
