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

if TYPE_CHECKING:
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
    """

    def __init__(
        self,
        walls: TrackWalls,
        search_radius_m: float = 0.15,
        passes: int = 4,
        grid_points: int = 5,
    ) -> None:
        self._walls = walls
        self._search_radius = search_radius_m
        self._passes = passes
        self._grid_points = grid_points

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
            The best-matching (x, y) found within the search window.
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
                    cost = float(np.sum((predicted - ranges) ** 2))
                    if cost < best_cost:
                        best_cost = cost
                        cand_x, cand_y = x, y
            best_x, best_y = cand_x, cand_y
            # Refine at the resolution just found, for the next pass.
            radius = 2.0 * radius / (n - 1)

        return best_x, best_y
