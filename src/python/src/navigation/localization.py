"""LIDAR-based absolute position estimation from known track wall geometry.

Real hardware has no wheel odometry (nothing publishes ``nav_msgs/Odometry``),
and pulling in a general-purpose scan-matching/SLAM library is unwarranted for
a track this constrained: the wall layout for the current round is already
known from scenario metadata (see :class:`~src.navigation.track_geometry.TrackWalls`),
so position can be recovered directly by finding the pose whose *predicted*
scan against that known geometry best matches the *real* scan.

The search is local: the robot starts at a known position (from scenario
metadata) and moves only a few centimetres between 20 Hz ticks, so each
estimate's prior is normally a tight, reliable seed for the next. This also
means the same local search handles corners and straight segments uniformly —
there is no special-casing of "which wall is my nearest wall", which a direct
geometric (wall-distance) approach would need.

A local search reseeded from its own previous answer has no way back once that
answer is wrong, so it is backed by a global relocalization that fires when the
estimate stops explaining the scan (see
:meth:`LidarLocalizer._relocalize_globally`). The physical invariant it rests on
is that the robot cannot leave the track: the walls are known and the car is
always inside them, so a pose whose predicted sweep does not match the real one
is not merely imprecise, it is wrong, and position can be re-solved over the
whole free space without any prior at all.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.domain.models import Waypoint

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.config.navigation_tuning.blind_nav import LocalizationParams

    from src.navigation.track_geometry import TrackWalls


class LidarLocalizer:
    """Estimates (x, y) by matching a LIDAR sweep against known wall geometry.

    Args:
        walls: Wall geometry for the current scenario's corridor widths.
        params: Tuning-derived search parameters. Construct via
            :meth:`from_tuning` (from a ``NavigationTuning``) or pass a
            ``LocalizationParams`` directly; the localizer no longer carries
            its own copy of these defaults.
    """

    def __init__(
        self,
        walls: TrackWalls,
        params: LocalizationParams,
    ) -> None:
        self._walls = walls
        self._search_radius = params.SEARCH_RADIUS_M
        self._passes = params.PASSES
        self._grid_points = params.GRID_POINTS
        self._residual_clip = params.RESIDUAL_CLIP_M
        self._max_speed_mps = params.MAX_SPEED_MPS
        self._jump_confirm_tolerance = params.JUMP_CONFIRM_TOLERANCE_M
        self._relocalize_cost_threshold = params.RELOCALIZE_COST_THRESHOLD
        self._relocalize_after_scans = params.RELOCALIZE_AFTER_SCANS
        self._relocalize_grid_step_m = params.RELOCALIZE_GRID_STEP_M
        self._relocalize_accept_ratio = params.RELOCALIZE_ACCEPT_RATIO
        self._last_estimate_time_s: float | None = None
        self._pending_jump_xy: Waypoint | None = None
        self._bad_fit_streak = 0
        self._relocalization_count = 0
        self._last_fit_cost: float | None = None
        # Built on first use rather than in __init__: a localizer is
        # reconstructed every time the corridor-width belief is revised
        # mid-round (see ros2_hardware_gateway.set_believed_walls), and most
        # instances never need this grid at all.
        self._free_space_grid: tuple[np.ndarray, np.ndarray] | None = None

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning, walls: TrackWalls) -> LidarLocalizer:
        """Build a localizer from a ``NavigationTuning`` (single source of truth).

        Args:
            tuning: Navigation tuning instance (usually from ``load_default``).
            walls: Wall geometry for the current scenario's corridor widths.

        Returns:
            LidarLocalizer with search parameters taken from
            ``tuning.localization``.
        """
        return cls(walls, tuning.localization)

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
        # The streak counts consecutive scans the CURRENT estimate failed to
        # explain. A re-seed replaces that estimate outright, so the count
        # accrued against the old one says nothing about the new one.
        self._bad_fit_streak = 0

    @property
    def relocalization_count(self) -> int:
        """How many times the global search has had to rescue the estimate.

        Diagnostic only. Non-zero means the local search lost the pose and was
        recovered; the value belongs in the debug snapshot because the failure
        it reports (run_20260907_205830) was invisible in every field the
        navigator already published.
        """
        return self._relocalization_count

    @property
    def last_fit_cost(self) -> float | None:
        """Mean clipped squared residual (m^2) of the last accepted match.

        Diagnostic only. Around 0.010 on a healthy hardware run (measured
        median over two clean 3-lap runs, 2026-09-07), 0.043 on the run whose
        estimate had lost the track.
        """
        return self._last_fit_cost

    def estimate_position(
        self,
        prior: Waypoint,
        yaw: float,
        ranges_m: tuple[float, ...] | list[float],
        angles_rad: tuple[float, ...] | list[float],
        now_s: float | None = None,
    ) -> Waypoint:
        """Return the (x, y) that best explains the given LIDAR sweep.

        Args:
            prior: Previous position estimate (or the known scenario start
                position on the very first call) -- the search seed.
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
            The best-matching (x, y) found within the search window as a
            ``Waypoint``, or ``prior`` unchanged if that result fell outside
            the known track, or if it implied impossible speed and was not
            yet confirmed by a second tick agreeing (see below).
        """
        dt = None
        if now_s is not None:
            if self._last_estimate_time_s is not None:
                dt = now_s - self._last_estimate_time_s
            self._last_estimate_time_s = now_s

        ranges = np.asarray(ranges_m, dtype=float)
        angles = np.asarray(angles_rad, dtype=float)

        best_x, best_y = prior.x, prior.y
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

        best_cost = self._fit_cost(best_x, best_y, yaw, ranges, angles)
        self._last_fit_cost = best_cost

        # The search is a local hill-climb reseeded from prior_xy every call,
        # and nothing in it bounds its own output: search_radius_m is
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
        off_track = not self._walls.point_in_free_space(best_x, best_y)

        # Both symptoms count toward one streak, because both say the same
        # thing: the search is no longer anywhere near the truth. An off-track
        # winner is impossible outright -- the car cannot leave the track -- and
        # a winner whose predicted sweep does not resemble the real one has not
        # explained the scan, however cheap it was relative to its neighbours.
        # Returning ``prior_xy`` handles a single bad tick; what it cannot do
        # is end, because the next call reseeds from that same prior and the
        # search never gets a look outside its own basin.
        if off_track or best_cost > self._relocalize_cost_threshold:
            self._bad_fit_streak += 1
        else:
            self._bad_fit_streak = 0

        if self._bad_fit_streak >= self._relocalize_after_scans:
            rescued = self._relocalize_globally(yaw, ranges, angles, best_cost)
            if rescued is not None:
                return rescued

        if off_track:
            self._pending_jump_xy = None
            return prior

        if self._reject_implausible_speed(Waypoint(best_x, best_y), prior, dt):
            return prior

        self._pending_jump_xy = None
        return Waypoint(best_x, best_y)

    def _fit_cost(
        self,
        x: float,
        y: float,
        yaw: float,
        ranges: np.ndarray,
        angles: np.ndarray,
    ) -> float:
        """Mean clipped squared residual (m^2) at one pose, over real returns only.

        Deliberately not the search's own cost. ``sanitize_lidar_ranges``
        substitutes max range for every no-return ray, and on hardware that is
        23-30% of the sweep -- rays carrying no information about where the
        robot is, each contributing a full clipped residual whatever the pose.
        Included, they add a large offset that swamps the very difference this
        number exists to detect: measured over the three runs of 2026-09-07,
        counting them compresses the gap between a healthy fit and a lost one
        from 8x (0.006 vs 0.05) to 2x (0.023 vs 0.051).

        The search's cost is left alone. It only ever compares candidates
        against each other on one sweep, where a constant offset cancels; this
        one is compared against an absolute threshold, where it does not.
        """
        informative = ranges < RobotSpecs.LIDAR_MAX_RANGE
        if not informative.any():
            return 0.0
        predicted = self._walls.raycast_grid(
            np.array([x + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw)]),
            np.array([y + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw)]),
            yaw,
            angles[informative],
        )
        residual = np.abs(predicted[0] - ranges[informative])
        np.minimum(residual, self._residual_clip, out=residual)
        return float(np.mean(residual**2))

    def _relocalize_globally(
        self,
        yaw: float,
        ranges: np.ndarray,
        angles: np.ndarray,
        local_cost: float,
    ) -> Waypoint | None:
        """Re-solve position over the whole track, with no prior at all.

        The local search cannot recover from a wrong seed, because it is
        reseeded from its own previous answer every call. This one is not
        seeded: it scores every free-space candidate on the track against the
        same cost the local search uses, so the answer does not depend on how
        wrong the estimate had become. Yaw is still taken as given -- it is
        corrected against the walls independently, upstream of this class.
        Measured on run_20260907_205830, which the local search lost for 48 s:
        this recovered a pose with a residual 10-15x lower than the latched
        one at every sampled tick, and none of its beams landed off-track.

        The speed guard is deliberately bypassed. It bounds motion between
        consecutive estimates, and this is not motion -- it is the correction
        of an estimate already known to be wrong, so the distance it covers
        carries no information about how fast the robot went.

        Returns ``None`` when the global winner does not fit MATERIALLY better
        than the local one, which is the case that matters most: a cost above
        the threshold does not always mean the estimate is lost, it can equally
        mean the WALL MODEL is wrong -- and during blind operation, while the
        corridor widths are still being estimated, it usually does. A global
        search against a wrong model finds the best explanation of a track that
        is not there, and jumping to it destroys a pose that was fine. Measured
        on the balanced-128 Open sweep: without this check the sweep went
        128/128 -> 127/128 (scenario 94, 1000-600-1000-1000 west/clockwise,
        turned into a reverse-run) and one case lost 17 s. A wrong model raises
        the floor for every candidate, so the global winner cannot beat the
        local one by much -- which is exactly the signal this test reads.
        """
        informative = ranges < RobotSpecs.LIDAR_MAX_RANGE
        gx, gy = self._free_space_candidates()
        sensor_xs = gx + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw)
        sensor_ys = gy + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw)
        predicted = self._walls.raycast_grid(sensor_xs, sensor_ys, yaw, angles[informative])
        residual = np.abs(predicted - ranges[informative][None, :])
        np.minimum(residual, self._residual_clip, out=residual)
        costs = np.mean(residual**2, axis=1)
        best_idx = int(np.argmin(costs))
        best_cost = float(costs[best_idx])

        # Either way the streak restarts: the evidence has been acted on, and
        # leaving it at the trigger would re-run this search on every tick.
        self._bad_fit_streak = 0
        if best_cost > local_cost * self._relocalize_accept_ratio:
            return None

        self._pending_jump_xy = None
        self._relocalization_count += 1
        self._last_fit_cost = best_cost
        return Waypoint(float(gx[best_idx]), float(gy[best_idx]))

    def _free_space_candidates(self) -> tuple[np.ndarray, np.ndarray]:
        """Every on-track position the global search considers, cached.

        Filtered through ``point_in_free_space`` rather than re-deriving the
        free-space rule here, so the inner block stays excluded by the same
        definition the rest of the navigator uses.
        """
        if self._free_space_grid is None:
            axis = np.arange(
                TrackDimensions.MIN_COORD,
                TrackDimensions.MAX_COORD + self._relocalize_grid_step_m,
                self._relocalize_grid_step_m,
            )
            grid_x, grid_y = (a.ravel() for a in np.meshgrid(axis, axis, indexing="ij"))
            free = np.fromiter(
                (self._walls.point_in_free_space(x, y) for x, y in zip(grid_x, grid_y, strict=True)),
                dtype=bool,
                count=grid_x.size,
            )
            self._free_space_grid = (grid_x[free], grid_y[free])
        return self._free_space_grid

    def _reject_implausible_speed(
        self,
        best_xy: Waypoint,
        prior_xy: Waypoint,
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
        implied_dist = best_xy.distance_to(prior_xy)
        if implied_dist <= self._max_speed_mps * dt:
            return False
        pending = self._pending_jump_xy
        if pending is not None and best_xy.distance_to(pending) <= self._jump_confirm_tolerance:
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
    return LidarLocalizer(walls, params)
