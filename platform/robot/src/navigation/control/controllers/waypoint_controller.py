"""Pure pursuit waypoint following controller for Ackermann geometry.

Implements the pure pursuit steering algorithm for waypoint-following
with lookahead distance adaptation based on forward clearance.

Reference: https://www.ri.cmu.edu/pub_files/pub3/coulter_1992_1.pdf
"""

from __future__ import annotations

import logging
import math

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.utils import _local_frame, _pure_pursuit_steer

logger = logging.getLogger(__name__)

_DEFAULT_CONTROL_DT_S: float = 1.0 / NavigationTuning.load_default().control.CONTROL_HZ
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
        steer_kp: No longer consumed by ``compute_steering`` (see its
            docstring) -- steering is now curvature-based pure pursuit, not a
            gain on heading error. Kept only so ``NavigationTuning.pursuit.
            STEER_KP`` (still read by diagnostic sweep scripts and exposed via
            the backend API) has somewhere to land without raising; changing
            it no longer has any effect.
        max_steering_rate: Maximum steering command rate (rad/s)
    """

    def __init__(
        self,
        max_steering_angle: float = RobotSpecs.MAX_STEERING_ANGLE,
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
            steer_kp: Accepted for backward compatibility with
                ``NavigationTuning.pursuit.STEER_KP`` and callers that still
                pass it, but no longer read by ``compute_steering`` -- see
                that method's docstring and the class docstring above.
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
        # How much crosstrack the current path can absorb before the chassis
        # reaches an outer wall. None until a path is set, meaning
        # ``lookahead_transition`` stands unmodified.
        self._crosstrack_budget_m: float | None = None

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> WaypointController:
        """Build controller from NavigationTuning parameters.

        Still passes through ``STEER_KP`` even though ``compute_steering`` no
        longer reads it (see its docstring), so the field stays wired rather
        than silently disconnected for whichever caller still sets it.

        Args:
            tuning: NavigationTuning instance (usually from load_default).

        Returns:
            WaypointController with values from tuning.
        """
        return cls(
            max_steering_angle=RobotSpecs.MAX_STEERING_ANGLE,
            lookahead_short=tuning.pursuit.LOOKAHEAD_SHORT,
            lookahead_long=tuning.pursuit.LOOKAHEAD_LONG,
            lookahead_transition=tuning.pursuit.LOOKAHEAD_TRANSITION,
            steer_kp=tuning.pursuit.STEER_KP,
            max_steering_rate=tuning.pursuit.MAX_STEERING_RATE,
            waypoint_reached_distance_m=tuning.waypoints.CONTROLLER_REACHED_DISTANCE_M,
        )

    def select_lookahead(self, crosstrack_error: float) -> float:
        """Select lookahead distance based on how far off-path the robot is.

        Was gated on forward LIDAR clearance instead, despite
        ``lookahead_transition``'s own name and config docstring already
        saying "crosstrack" -- in a wide corridor, forward clearance often
        stays generous through a corner, so the long lookahead never yielded
        to the short one exactly when a tighter turn was needed. With
        curvature-based steering (see ``compute_steering``), a long lookahead
        to a target that is only modestly off-axis commands a much smaller
        curvature than the corner needs, producing a wide, slow arc instead of
        a decisive turn -- measured on real hardware 2026-08-03 (see
        ``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).

        Gating on crosstrack error instead closes the loop correctly: falling
        behind on a turn grows the crosstrack error, which shortens the
        lookahead, which increases the commanded curvature, which corrects the
        error back down.

        The threshold is the smaller of ``lookahead_transition`` and what the
        path itself can afford (see :meth:`set_crosstrack_budget`). The fixed
        value silently assumes at least 0.30 m of room to drift into, which the
        blind narrow prior does not leave: measured on hardware 2026-08-06,
        crosstrack ran 0.09 -> 0.15 through a corner and never crossed 0.30, so
        the lookahead stayed long and the curvature stayed weak the whole way
        into the wall. The loop above is sound; it was armed past the point of
        no return.

        Args:
            crosstrack_error: Perpendicular distance from the planned path (metres)

        Returns:
            Lookahead distance in meters
        """
        if crosstrack_error > self.effective_transition:
            return self.lookahead_short
        return self.lookahead_long

    @property
    def effective_transition(self) -> float:
        """The crosstrack threshold actually in force, after the wall budget."""
        if self._crosstrack_budget_m is None:
            return self.lookahead_transition
        return min(self.lookahead_transition, self._crosstrack_budget_m)

    def set_crosstrack_budget(self, budget_m: float | None) -> None:
        """Declare how far this path may be strayed from before a wall is hit.

        Only ever tightens the threshold -- a path with room to spare keeps
        ``lookahead_transition``, so nothing changes on a confirmed-wide
        corridor. Pass ``None`` to drop back to the configured value.

        Args:
            budget_m: Crosstrack error (m) the path can absorb, or None.
        """
        self._crosstrack_budget_m = budget_m

    def select_target_point(
        self,
        current_pos: tuple[float, float],
        current_yaw: float,
        waypoints: list[tuple[float, float]],
        waypoint_index: int,
        lookahead_distance: float,
    ) -> tuple[float, float]:
        """Find the path point at least ``lookahead_distance`` ahead, and
        geometrically ahead of the chassis right now.

        Searches forward from ``waypoint_index``, wrapping around the end of
        the list back to the start -- at most one full lap. ``waypoints`` is
        the full canonical-lap path, not a pre-sliced remainder: a caller
        slicing it themselves let the search run out of points near the end
        of a lap (or right after a reseek lands close to the tail) and fall
        back to a single fixed final point instead of continuing around the
        loop, producing a long stretch of barely-changing bearing to a point
        that should have already been left behind -- measured on real
        hardware 2026-08-03 as steering pinned near zero for tens of seconds
        while heading drifted 85+ degrees (see
        ``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).

        Also skips any candidate that is behind the chassis in its current
        local frame -- accepting one there previously handed ``compute_steering``
        a target its curvature formula is not valid for, forcing an
        unreliable side-guessing fallback (also measured 2026-08-03 as a
        wrong-direction turn on real hardware; not reproducible in sim, where
        the localizer's pose estimate does not drift between ticks the way
        real sensor noise does). Skipping forward instead of guessing means
        the chosen target is always one the curvature formula actually
        applies to.

        Args:
            current_pos: Robot position (x, y)
            current_yaw: Robot heading (radians) -- used only to skip
                behind-the-chassis candidates, not to compute anything returned.
            waypoints: The full canonical-lap path (not pre-sliced)
            waypoint_index: Index to start the forward search from
            lookahead_distance: Minimum distance from current_pos to target (meters)

        Returns:
            The selected (x, y) target point: the nearest point at least
            ``lookahead_distance`` away that is also ahead of the chassis, or
            (failing that) the nearest ahead point found in one full lap, or
            (only if literally nothing in the entire lap is ahead of the
            chassis -- a degenerate case, e.g. a wildly wrong heading) the
            nearest point found at all.

            Nearest, not farthest, in both fallback tiers: ``compute_steering``'s
            curvature formula divides by the target's actual squared distance, so
            a farther fallback target produces a *weaker* commanded curvature --
            backwards from what a large heading error needs. Picking farthest
            when nothing qualified handed back points 1-3m away (multiple laps'
            worth of waypoint spacing) while the chassis sat 90 deg off the path,
            starving the correction and locking the robot into repeatedly
            re-selecting a similarly distant point forever -- measured on real
            hardware 2026-08-03/04 as a self-reinforcing deadlock: creep speed +
            weak curvature never closes the heading error that caused both (see
            ``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).
            Nearest keeps the fallback target's distance close to a sane
            pure-pursuit lookahead instead.
        """
        cx, cy = current_pos
        cos_yaw, sin_yaw = math.cos(current_yaw), math.sin(current_yaw)
        n = len(waypoints)
        nearest_ahead: tuple[float, float] | None = None
        nearest_ahead_dist = math.inf
        nearest_any = waypoints[waypoint_index % n]
        nearest_any_dist = math.inf
        for offset in range(n):
            wx, wy = waypoints[(waypoint_index + offset) % n]
            dx, dy = wx - cx, wy - cy
            dist = math.hypot(dx, dy)
            if dist < nearest_any_dist:
                nearest_any_dist = dist
                nearest_any = (wx, wy)
            x_local = dx * cos_yaw + dy * sin_yaw
            if x_local <= 0:
                continue
            if dist >= lookahead_distance:
                return (wx, wy)
            if dist < nearest_ahead_dist:
                nearest_ahead_dist = dist
                nearest_ahead = (wx, wy)
        return nearest_ahead if nearest_ahead is not None else nearest_any

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
        crosstrack_error: float,
        dt: float = _DEFAULT_CONTROL_DT_S,
    ) -> tuple[float, float, float]:
        """Compute steering angle, lookahead, and heading error for the next control step.

        Curvature-based pure pursuit (see ``src.navigation.utils._pure_pursuit_steer``
        for the formula and its physical reasoning), not a gain on heading error.
        A bare ``steer_kp * angle_error`` P-term has no physical units, so it
        silently absorbs whatever the plant does -- this chassis was modelled as
        front-steer in the simulator the old gain was tuned against, when it
        actually steers both axles in counter-phase (double the yaw rate for the
        same angle), and that mismatch produced full-lock steering oscillation on
        real hardware (2026-08-03, see
        ``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).
        The lookahead still only selects *which* waypoint to aim at; it is the
        steering law itself that changed.

        Args:
            current_pos: Robot position (x, y)
            current_yaw: Robot heading (radians)
            target_waypoint: Next waypoint (x, y)
            crosstrack_error: Perpendicular distance from the planned path
                (metres), used to select the lookahead -- see ``select_lookahead``.
            dt: Time since the previous call (seconds), used to cap the
                steering delta at ``max_steering_rate * dt``

        Returns:
            Tuple of (steering_angle, lookahead_distance, angle_error_rad).
            steering_angle: Command in [-1, 1] normalized range, after rate limiting.
            lookahead_distance: Selected lookahead (for diagnostics).
            angle_error_rad: Signed bearing error to the target, before rate
                limiting -- lets the caller slow down for a sharp turn instead
                of taking it at whatever speed forward clearance alone selects.
        """
        lookahead = self.select_lookahead(crosstrack_error)

        x_local, y_local = _local_frame(current_pos, current_yaw, target_waypoint)
        distance = math.hypot(x_local, y_local)

        if distance < self.waypoint_reached_distance_m:
            return 0.0, lookahead, 0.0

        angle_error = math.atan2(y_local, x_local)

        if x_local > 0:
            steering_normalized_raw = _pure_pursuit_steer(
                x_local, y_local, self.waypoint_reached_distance_m, self.max_steering_angle
            )
        else:
            # Target behind the robot: the curvature formula is only valid for a
            # roughly-forward target (see _pure_pursuit_steer). Saturate toward
            # whichever side it's on instead of trusting a formula that can look
            # plausible while actually steering away from the target.
            steering_normalized_raw = 1.0 if y_local >= 0 else -1.0

        steering_rad = steering_normalized_raw * self.max_steering_angle

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

        return steering_normalized, lookahead, angle_error
