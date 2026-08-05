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

from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import TrackDimensions

if TYPE_CHECKING:
    from src.navigation.track_geometry import TrackWalls

_TRACK_MIN = TrackDimensions.MIN_COORD
_TRACK_MAX = TrackDimensions.MAX_COORD


class LidarLocalizer:
    """Estimates (x, y) by matching a LIDAR sweep against known wall geometry.

    Args:
        walls: Wall geometry for the current scenario's corridor widths.
        search_radius_m: Half-width of the initial search window around the
            prior position (metres). Must comfortably exceed the maximum
            per-tick displacement so the true position is never outside it.
        passes: Number of coarse-to-fine grid-search passes.
        grid_points: Candidates per axis per pass (grid is ``grid_points**2``).
    """

    def __init__(
        self,
        walls: TrackWalls,
        search_radius_m: float = 0.15,
        passes: int = 4,
        grid_points: int = 5,
        residual_clip_m: float = 0.25,
    ) -> None:
        self._walls = walls
        self._search_radius = search_radius_m
        self._passes = passes
        self._grid_points = grid_points
        self._residual_clip = residual_clip_m

    def estimate_position(
        self,
        prior_xy: tuple[float, float],
        yaw: float,
        ranges_m: tuple[float, ...] | list[float],
        angles_rad: tuple[float, ...] | list[float],
    ) -> tuple[float, float]:
        """Return the (x, y) that best explains the given LIDAR sweep.

        Args:
            prior_xy: Previous position estimate (or the known scenario start
                position on the very first call) — the search seed.
            yaw: Current heading (radians), taken as accurate (IMU-fused).
            ranges_m: LIDAR range readings (robot frame).
            angles_rad: Per-ray bearings matching ``ranges_m`` (0 = forward).

        Returns:
            The best-matching (x, y) found within the search window, or
            ``prior_xy`` unchanged if that result fell outside the known
            track (see below).
        """
        ranges = np.asarray(ranges_m, dtype=float)
        angles = np.asarray(angles_rad, dtype=float)

        best_x, best_y = prior_xy
        radius = self._search_radius
        n = self._grid_points

        for _ in range(self._passes):
            offsets = np.linspace(-radius, radius, n)
            best_cost = np.inf
            cand_x, cand_y = best_x, best_y
            for dx in offsets:
                x = best_x + dx
                for dy in offsets:
                    y = best_y + dy
                    predicted = self._walls.raycast(x, y, yaw, angles)
                    # Clip each ray's contribution instead of summing raw
                    # squares. A plain least-squares fit is dominated by its
                    # worst rays, and the worst rays are exactly the ones whose
                    # geometry isn't in ``walls``: a traffic sign or parking
                    # block standing in the beam, or — when the corridor widths
                    # are still being estimated rather than known — a whole
                    # stretch of far wall in the wrong place. Those rays then
                    # drag the fit toward a pose that "explains" geometry that
                    # does not exist. Clipping bounds how far any single ray can
                    # pull, so the majority of correctly-modelled rays win.
                    residual = np.abs(predicted - ranges)
                    np.minimum(residual, self._residual_clip, out=residual)
                    cost = float(np.sum(residual**2))
                    if cost < best_cost:
                        best_cost = cost
                        cand_x, cand_y = x, y
            best_x, best_y = cand_x, cand_y
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
        # A companion "reject if the jump is larger than real motion could
        # explain" guard was tried and reverted: it also rejected the large,
        # *intentional* single-tick correction this same search performs to
        # absorb a hand-placement error at race start (see
        # test_sensor_errors.py::TestStartPlacement, which starts up to 0.4m
        # off and expects the localizer to converge within ~1s) -- there is
        # no way to tell "wrong snap during a maneuver" apart from "correct
        # snap onto the true start pose" by jump size alone. Track-bounds is
        # weaker (an in-bounds wrong match still slips through) but catches
        # the failure actually observed without that conflict.
        if not (_TRACK_MIN <= best_x <= _TRACK_MAX) or not (_TRACK_MIN <= best_y <= _TRACK_MAX):
            return prior_xy

        return best_x, best_y
