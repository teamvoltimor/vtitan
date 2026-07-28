"""Pure pursuit waypoint following controller for Ackermann geometry.

Implements the pure pursuit steering algorithm for waypoint-following
with lookahead distance adaptation based on forward clearance.

Reference: https://www.ri.cmu.edu/pub_files/pub3/coulter_1992_1.pdf
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)

_DEFAULT_CONTROL_DT_S: float = 0.05
"""Control-loop tick interval, matching the 20 Hz loop assumed throughout
navigation tuning (see e.g. ``EscapeManeuverParams.STUCK_TIMEOUT_FRAMES``)."""


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
        steer_kp: float = 1.2,
        max_steering_rate: float = 2.0,
        waypoint_reached_distance_m: float = 0.01,
    ):
        """Initialize pure pursuit controller.

        Args:
            max_steering_angle: Max steering angle in radians
            lookahead_short: Lookahead for corners (m)
            lookahead_long: Lookahead for straights (m)
            lookahead_transition: Crosstrack threshold for mode switch (m)
            steer_kp: P-controller gain. Default matches
                ``NavigationTuning.pursuit.STEER_KP`` -- ``CoreNavigator``
                always passes that value explicitly, so this default is only
                ever exercised by a caller that constructs this class
                directly; keep the two in sync rather than letting this one
                drift into unused, misleading dead code again (was 1.5 vs.
                the real wired 1.2 previously).
            max_steering_rate: Max steering rate (rad/s)
            waypoint_reached_distance_m: Distance below which the current
                target waypoint is considered reached (m)
        """
        self.max_steering_angle = max_steering_angle
        self.lookahead_short = lookahead_short
        self.lookahead_long = lookahead_long
        self.lookahead_transition = lookahead_transition
        self.steer_kp = steer_kp
        self.max_steering_rate = max_steering_rate
        self.waypoint_reached_distance_m = waypoint_reached_distance_m
        self._prev_steering_rad = 0.0

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

    def select_target_point(
        self,
        current_pos: tuple[float, float],
        waypoints: list[tuple[float, float]],
        waypoint_index: int,
        lookahead_distance: float,
    ) -> tuple[float, float]:
        """Find the path point at least ``lookahead_distance`` ahead.

        Searches forward from ``waypoint_index`` for the first waypoint whose
        distance from ``current_pos`` reaches the lookahead distance, instead
        of steering directly at the next waypoint (which can be well under the
        lookahead distance and produces weave on straights / corner cutting).

        Args:
            current_pos: Robot position (x, y)
            waypoints: Ordered path waypoints, searched from waypoint_index
            waypoint_index: Index to start the forward search from
            lookahead_distance: Minimum distance from current_pos to target (meters)

        Returns:
            The selected (x, y) target point. Falls back to the last waypoint
            if none in the remaining path reach the lookahead distance.
        """
        cx, cy = current_pos
        for i in range(waypoint_index, len(waypoints)):
            wx, wy = waypoints[i]
            if math.hypot(wx - cx, wy - cy) >= lookahead_distance:
                return waypoints[i]
        return waypoints[-1]

    def reset(self) -> None:
        """Clear the steering-rate-limit memory.

        Call this whenever something other than this controller has just
        driven the steering command (e.g. an escape maneuver just finished),
        so the next pure-pursuit tick isn't rate-limited against a stale
        pre-maneuver angle.
        """
        self._prev_steering_rad = 0.0

    def compute_steering(
        self,
        current_pos: tuple[float, float],
        current_yaw: float,
        target_waypoint: tuple[float, float],
        forward_clearance: float,
        dt: float = _DEFAULT_CONTROL_DT_S,
    ) -> tuple[float, float]:
        """Compute steering angle and lookahead for next control step.

        NOT pure pursuit, despite the lookahead: this is a proportional
        controller on heading error (``steer_kp * angle_error``), with no
        vehicle geometry in it at all. The lookahead only selects *which*
        waypoint to aim at, not the steering law. Real pure pursuit would
        convert a curvature to a steering angle via the wheelbase, as
        ``ParkController._pure_pursuit_steer`` does.

        This matters when tuning: ``steer_kp`` has no physical units, so it
        absorbs whatever the plant does. The simulation previously modelled
        this chassis as a front-steer car when it actually steers both axles
        in counter-phase (twice the yaw rate), so a gain tuned in sim is
        hotter on hardware. Re-tune against the corrected kinematics rather
        than reasoning from the old value.

        Args:
            current_pos: Robot position (x, y)
            current_yaw: Robot heading (radians)
            target_waypoint: Next waypoint (x, y)
            forward_clearance: Distance to forward obstacle (meters)
            dt: Time since the previous call (seconds), used to cap the
                steering delta at ``max_steering_rate * dt``

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

        if distance < self.waypoint_reached_distance_m:
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

        # Rate-limit against the previous tick's command so a large angle
        # error can't demand a full-deflection step in a single tick.
        max_delta = self.max_steering_rate * dt
        steering_rad = max(
            self._prev_steering_rad - max_delta,
            min(self._prev_steering_rad + max_delta, steering_rad),
        )
        self._prev_steering_rad = steering_rad

        # Normalize to [-1, 1]
        steering_normalized = steering_rad / self.max_steering_angle

        return steering_normalized, lookahead
