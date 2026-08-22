"""LIDAR-based absolute position estimation from known track wall geometry.

Real hardware has no wheel odometry (nothing publishes ``nav_msgs/Odometry``),
and pulling in a general-purpose scan-matching/SLAM library is unwarranted for
a track this constrained: the wall layout for the current round is already
known from scenario metadata (see :class:`~src.navigation.track_geometry.TrackWalls`),
so position can be recovered directly by finding the pose whose *predicted*
scan against that known geometry best matches the *real* scan.

The search is local, not a global relocalization: the robot starts at a known
position (from scenario metadata) and moves only a few centimetres between
20 Hz ticks, so each estimate's prior is always a tight, reliable seed for the
next. This also means the same local search handles corners and straight
segments uniformly — there is no special-casing of "which wall is my nearest
wall", which a direct geometric (wall-distance) approach would need.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs

if TYPE_CHECKING:
    from shared.config.navigation_tuning.blind_nav import LocalizationParams

    from src.navigation.track_geometry import TrackWalls


class LidarLocalizer:
    """Estimates (x, y) by matching a LIDAR sweep against known wall geometry.

    Args:
        walls: Wall geometry for the current scenario's corridor widths.
        search_radius_m: Half-width of the initial search window around the
            prior position (metres). Must comfortably exceed the maximum
            per-tick displacement so the true position is never outside it.
        passes: Number of coarse-to-fine grid-search passes.
        grid_points: Candidates per axis per pass (grid is ``grid_points**2``).
        max_speed_mps: Upper bound on real motion between ticks, used to
            reject a candidate that implies impossible speed (see below).
            Deliberately above the measured real top speed (0.156 m/s, see
            ``RobotSpecs.MAX_SPEED_MPS``) to leave headroom for a faster
            drivetrain later without this guard needing to move with it.
        jump_confirm_tolerance_m: How close two consecutive ticks' rejected
            candidates must be to count as the same correction confirming
            itself (see below).
    """

    def __init__(
        self,
        walls: TrackWalls,
        search_radius_m: float = 0.15,
        passes: int = 4,
        grid_points: int = 5,
        residual_clip_m: float = 0.25,
        max_speed_mps: float = 0.25,
        jump_confirm_tolerance_m: float = 0.05,
    ) -> None:
        self._walls = walls
        self._search_radius = search_radius_m
        self._passes = passes
        self._grid_points = grid_points
        self._residual_clip = residual_clip_m
        self._max_speed_mps = max_speed_mps
        self._jump_confirm_tolerance = jump_confirm_tolerance_m
        self._last_estimate_time_s: float | None = None
        self._pending_jump_xy: tuple[float, float] | None = None

    def reset_tracking(self) -> None:
        """Forget everything carried between ticks, for a re-seeded position.

        The speed-bound guard below is a statement about motion *between*
        consecutive estimates. When the caller re-seeds position outright --
        a new race, or blind direction inference overturning the frame every
        creep-time fix was computed in -- there is no such continuity: the
        held candidate was found in the old frame, and the elapsed time since
        it spans a discontinuity rather than real travel. Left in place, the
        very first estimate after a re-seed can have its "impossible" jump
        confirmed by that stale candidate and be accepted immediately, which
        is precisely the corruption the re-seed exists to discard.
        """
        self._pending_jump_xy = None
        self._last_estimate_time_s = None

    def estimate_position(
        self,
        prior_xy: tuple[float, float],
        yaw: float,
        ranges_m: tuple[float, ...] | list[float],
        angles_rad: tuple[float, ...] | list[float],
        now_s: float | None = None,
    ) -> tuple[float, float]:
        """Return the (x, y) that best explains the given LIDAR sweep.

        Args:
            prior_xy: Previous position estimate (or the known scenario start
                position on the very first call) — the search seed.
            yaw: Current heading (radians), taken as accurate (IMU-fused).
            ranges_m: LIDAR range readings (robot frame).
            angles_rad: Per-ray bearings matching ``ranges_m`` (0 = forward).
            now_s: Current clock time (seconds; real or simulated, whichever
                the caller's other timestamps use). Enables the speed-bound
                guard below -- omitted (the default), that guard is skipped
                entirely, matching every call before it existed. The very
                first call also skips it regardless of ``now_s``, since there
                is no prior timestamp yet to compute elapsed time from -- this
                is what lets the deliberate large single-tick correction that
                absorbs a hand-placement error at race start through (see
                test_sensor_errors.py::TestStartPlacement).

        Returns:
            The best-matching (x, y) found within the search window, or
            ``prior_xy`` unchanged if that result fell outside the known
            track, or if it implied impossible speed and was not yet
            confirmed by a second tick agreeing (see below).
        """
        dt = None
        if now_s is not None:
            if self._last_estimate_time_s is not None:
                dt = now_s - self._last_estimate_time_s
            self._last_estimate_time_s = now_s

        ranges = np.asarray(ranges_m, dtype=float)
        angles = np.asarray(angles_rad, dtype=float)

        best_x, best_y = prior_xy
        radius = self._search_radius
        n = self._grid_points

        for _ in range(self._passes):
            offsets = np.linspace(-radius, radius, n)
            # Candidate grid flattened in the same (dx outer, dy inner) order
            # the old nested Python loop visited, purely so a tie (equal
            # cost, picked by whichever came "first") resolves identically.
            grid_dx, grid_dy = np.meshgrid(offsets, offsets, indexing="ij")
            xs = best_x + grid_dx.ravel()
            ys = best_y + grid_dy.ravel()

            # Predict from where the SENSOR is, not where the body centre is.
            # `xs`/`ys` are candidate CHASSIS poses, but the C1 sits
            # LIDAR_MOUNT_X_OFFSET (0.1222 m) forward of centre, flush with the
            # bumper -- so a scan taken there cannot be reproduced by casting
            # from the centre. Until 2026-08-21 it was, which biased every
            # forward ray by the offset and pulled the fit along the corridor
            # axis; the simulator raycast from the centre too, so the two agreed
            # and the error was invisible in sim while present on hardware.
            sensor_xs = xs + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw)
            sensor_ys = ys + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw)
            predicted = self._walls.raycast_grid(sensor_xs, sensor_ys, yaw, angles)
            # Clip each ray's contribution instead of summing raw squares. A
            # plain least-squares fit is dominated by its worst rays, and the
            # worst rays are exactly the ones whose geometry isn't in
            # ``walls``: a traffic sign or parking block standing in the
            # beam, or — when the corridor widths are still being estimated
            # rather than known — a whole stretch of far wall in the wrong
            # place. Those rays then drag the fit toward a pose that
            # "explains" geometry that does not exist. Clipping bounds how
            # far any single ray can pull, so the majority of
            # correctly-modelled rays win.
            residual = np.abs(predicted - ranges[None, :])
            np.minimum(residual, self._residual_clip, out=residual)
            costs = np.sum(residual**2, axis=1)
            best_idx = int(np.argmin(costs))
            best_x, best_y = float(xs[best_idx]), float(ys[best_idx])
            # Refine at the resolution just found, for the next pass.
            radius = 2.0 * radius / (n - 1)

        # The search is a local hill-climb reseeded from prior_xy every call,
        # with no independent check on its own output: search_radius_m is
        # sized generously (0.15m) for search robustness, not as a physical
        # displacement bound, so a wrong-but-locally-cheap match (e.g. during
        # a K-turn's rapid reorientation, when the cost landscape shifts
        # quickly between ticks) can become the new seed and then propagate
        # forever -- nothing else in this call chain ever re-checks it.
        # Confirmed on real hardware 2026-08-04: a CCW run's position snapped
        # from (0.86, ...) to (-0.12, ...) in under a second during a k_turn
        # escape (real motion at that speed is ~1.6cm/tick, see
        # ros2_hardware_gateway.py), then stayed at that physically
        # impossible (off-track, x < 0) position for the rest of the run.
        #
        # point_in_free_space rejects both the outer boundary AND the inner
        # block: a match landing inside the inner block is exactly as
        # physically impossible as one landing outside the outer walls (the
        # robot cannot be inside a solid obstacle), so it gets the same
        # guard rather than a narrower bounds-only check.
        if not self._walls.point_in_free_space(best_x, best_y):
            self._pending_jump_xy = None
            return prior_xy

        if self._reject_implausible_speed((best_x, best_y), prior_xy, dt):
            return prior_xy

        self._pending_jump_xy = None
        return best_x, best_y

    def _reject_implausible_speed(
        self,
        best_xy: tuple[float, float],
        prior_xy: tuple[float, float],
        dt: float | None,
    ) -> bool:
        """Return True when ``best_xy`` implies a physically impossible speed.

        A cost/margin-based ambiguity guard (rejecting a winning candidate
        whose cost margin over its runner-up was too thin) was tried,
        committed, and reverted 2026-08-05 after replaying it against 22 real
        hardware runs (846 sampled ticks): confirmed-bad and genuinely correct
        matches had statistically indistinguishable cost and margin
        distributions on real, noisy scans (median cost ~25-26 either way,
        median margin ~0.02% either way) -- the signal the guard depended on
        does not exist on real data, only in the clean simulator. No threshold
        on it can work; the search's own cost surface cannot tell a real
        correction from an ambiguous flip.

        This guard instead bounds physical plausibility directly: how far the
        candidate is from ``prior_xy`` against how much time actually passed
        and the drivetrain's real top speed (with headroom -- see
        ``max_speed_mps`` above). A single tick implying impossible speed is
        held; if the SAME candidate (within ``jump_confirm_tolerance_m``) wins
        again on the very next tick, it is trusted -- a real correction
        reconverges to nearly the same position from an independent scan,
        while an ambiguous flip (this class's actual observed failure mode)
        does not typically repeat identically.
        """
        if dt is None or dt <= 0:
            return False
        implied_dist = math.hypot(best_xy[0] - prior_xy[0], best_xy[1] - prior_xy[1])
        if implied_dist <= self._max_speed_mps * dt:
            return False
        pending = self._pending_jump_xy
        if pending is not None and math.hypot(best_xy[0] - pending[0], best_xy[1] - pending[1]) <= self._jump_confirm_tolerance:
            return False
        self._pending_jump_xy = best_xy
        return True


def make_localizer(walls: TrackWalls, params: LocalizationParams) -> LidarLocalizer:
    """Build a :class:`LidarLocalizer` from tuning-config :class:`LocalizationParams`.

    Both hardware gateways (sim and ROS2) construct a localizer twice each --
    once at startup and once in ``set_believed_walls`` when blind navigation
    revises its corridor-width belief mid-round -- and every call site was
    hand-spelling the same 6-field mapping, risking one of the four silently
    drifting from the other three.
    """
    return LidarLocalizer(
        walls,
        search_radius_m=params.SEARCH_RADIUS_M,
        passes=params.PASSES,
        grid_points=params.GRID_POINTS,
        residual_clip_m=params.RESIDUAL_CLIP_M,
        max_speed_mps=params.MAX_SPEED_MPS,
        jump_confirm_tolerance_m=params.JUMP_CONFIRM_TOLERANCE_M,
    )
