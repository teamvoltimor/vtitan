"""Validation of LidarLocalizer against the simulator's exact wall geometry.

Generates a synthetic "real" scan from a known ground-truth pose using the
same TrackWalls raycast the simulator itself uses, then checks the localizer
recovers that pose (a) with clean rays, (b) under realistic LIDAR noise, and
(c) starting from a prior offset by a plausible per-tick displacement rather
than the exact ground truth — the three conditions it will actually face.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from shared.config.constants import RobotSpecs
from shared.config.enums import Section

from src.navigation.localization import LidarLocalizer
from src.navigation.track_geometry import TrackWalls

_ANGLES = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)

_UNIFORM_1000 = {Section.NORTH: 1.0, Section.SOUTH: 1.0, Section.EAST: 1.0, Section.WEST: 1.0}
_MIXED_WIDTHS = {Section.NORTH: 1.0, Section.SOUTH: 0.6, Section.EAST: 1.0, Section.WEST: 0.6}
_NARROW = {Section.NORTH: 0.6, Section.SOUTH: 0.6, Section.EAST: 0.6, Section.WEST: 0.6}

# A spread of ground-truth poses across straight segments and corner regions,
# for the uniform 1.0 m corridor layout (walls at x=y=1.0 / x=y=2.0).
_STRAIGHT_POSES = [
    (1.5, 0.5, 0.0),  # south straight, facing east
    (1.5, 0.5, math.pi),  # south straight, facing west
    (2.5, 1.5, math.pi / 2),  # east straight, facing north
    (0.5, 1.5, -math.pi / 2),  # west straight, facing south
    (1.5, 2.5, 0.3),  # north straight, off-axis heading
]
_CORNER_POSES = [
    (0.6, 0.6, math.pi / 4),  # southwest corner
    (2.4, 0.6, -math.pi / 4),  # southeast corner
    (2.4, 2.4, 3 * math.pi / 4),  # northeast corner
    (0.6, 2.4, -3 * math.pi / 4),  # northwest corner
]


def _localizer_for(widths: dict[Section, float]) -> tuple[LidarLocalizer, TrackWalls]:
    walls = TrackWalls(widths)
    return LidarLocalizer(walls), walls


@pytest.mark.parametrize("x, y, yaw", _STRAIGHT_POSES + _CORNER_POSES)
def test_recovers_exact_pose_from_clean_scan(x, y, yaw):
    localizer, walls = _localizer_for(_UNIFORM_1000)
    ranges = walls.raycast(x, y, yaw, _ANGLES)

    # Prior offset by a plausible per-tick displacement (up to ~5 cm at 10 Hz
    # LIDAR refresh and FAST_SPEED), not the exact ground truth.
    prior = (x - 0.03, y + 0.02)
    est_x, est_y = localizer.estimate_position(prior, yaw, ranges, _ANGLES)

    assert est_x == pytest.approx(x, abs=0.02)
    assert est_y == pytest.approx(y, abs=0.02)


@pytest.mark.parametrize("widths", [_UNIFORM_1000, _MIXED_WIDTHS, _NARROW])
def test_recovers_pose_across_corridor_widths(widths):
    localizer, walls = _localizer_for(widths)
    # A position that is valid across all three width configurations (the
    # narrowest corridor, 0.6 m, still leaves room at the corridor midline).
    x, y, yaw = 1.5, 0.3, 0.2
    ranges = walls.raycast(x, y, yaw, _ANGLES)

    est_x, est_y = localizer.estimate_position((x - 0.03, y - 0.03), yaw, ranges, _ANGLES)

    assert est_x == pytest.approx(x, abs=0.02)
    assert est_y == pytest.approx(y, abs=0.02)


@pytest.mark.parametrize("x, y, yaw", _STRAIGHT_POSES + _CORNER_POSES)
def test_robust_to_realistic_lidar_noise(x, y, yaw):
    """Same poses, but with real Slamtec-C1-level Gaussian range noise."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    rng = np.random.default_rng(0)
    clean = walls.raycast(x, y, yaw, _ANGLES)
    noisy = np.clip(
        clean + rng.normal(0.0, RobotSpecs.LIDAR_NOISE_STDDEV, clean.shape),
        RobotSpecs.LIDAR_MIN_RANGE,
        RobotSpecs.LIDAR_MAX_RANGE,
    )

    prior = (x - 0.03, y + 0.02)
    est_x, est_y = localizer.estimate_position(prior, yaw, noisy, _ANGLES)

    # Noise widens the tolerance a little, but should still be well within
    # the chassis half-width (0.075 m) — good enough to drive on.
    assert est_x == pytest.approx(x, abs=0.05)
    assert est_y == pytest.approx(y, abs=0.05)


def test_tracks_a_moving_pose_tick_by_tick():
    """Simulate ~1s of driving: each tick's estimate seeds the next, like production."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    rng = np.random.default_rng(1)

    x, y, yaw = 0.5, 0.5, 0.0
    speed = 0.3  # m/s
    dt = 0.1  # 10 Hz LIDAR refresh
    est = (x, y)  # first estimate seeded from the known scenario start position

    for _ in range(20):
        x += speed * dt
        clean = walls.raycast(x, y, yaw, _ANGLES)
        noisy = clean + rng.normal(0.0, RobotSpecs.LIDAR_NOISE_STDDEV, clean.shape)
        est = localizer.estimate_position(est, yaw, noisy, _ANGLES)
        assert est[0] == pytest.approx(x, abs=0.05)
        assert est[1] == pytest.approx(y, abs=0.05)


class TestPlausibilityGuards:
    """The search is a local hill-climb reseeded from prior_xy every call with
    no other check on its own output -- confirmed on real hardware 2026-08-04
    to snap to a physically impossible (off-track) position during a k_turn
    escape and stay there for the rest of the run. Guard: reject a result
    outside the known track, holding prior_xy instead.

    A companion "reject a jump larger than real motion could explain" guard
    was tried and reverted -- it also rejected the deliberate large single-tick
    correction this same search performs to absorb a hand-placement error at
    race start (see test_sensor_errors.py::TestStartPlacement, which starts up
    to 0.4m off and expects convergence within ~1s); there's no way to tell a
    wrong snap apart from a correct one by jump size alone.

    Replaying real hardware captures (2026-08-05) against this exact search
    showed the bounds guard's actual blind spot: an in-bounds wrong match from
    a nearly flat cost landscape, where the winner beats the runner-up by well
    under 1% of cost and a scan just as ambiguous a moment later flips which
    one wins. Guard: reject a result whose winning margin over the runner-up
    is too thin to trust, holding prior_xy instead -- unless the winner's own
    absolute cost is already excellent, since a well-converged match
    legitimately has a tiny gap to its own neighbours too.
    """

    def test_rejects_result_outside_track_bounds(self):
        localizer, walls = _localizer_for(_UNIFORM_1000)
        prior = (0.05, 1.5)
        # A scan generated from a position outside the track (x < 0):
        # mathematically valid raycast geometry, physically impossible.
        ranges = walls.raycast(-0.2, 1.5, 0.0, _ANGLES)

        est = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est == prior

    def test_rejects_result_inside_inner_block(self):
        """The inner block (the 1x1 m island in the middle, x/y in [1, 2] for
        the uniform 1.0 m corridor layout) is just as physically impossible
        to be inside as being outside the outer walls -- the robot cannot be
        inside a solid obstacle. point_in_free_space rejects both.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        prior = (0.9, 1.5)
        # A scan generated from a position inside the inner block:
        # mathematically valid raycast geometry, physically impossible.
        ranges = walls.raycast(1.5, 1.5, 0.0, _ANGLES)

        est = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est == prior

    def test_accepts_a_large_in_bounds_correction(self):
        """The start-placement-absorption case: a big single-tick jump is
        legitimate as long as it lands inside the track.

        15cm, well beyond the old (reverted) max_step_m=0.05 that broke this
        case, but within one call's actual reach (default search_radius_m=0.15
        across 4 shrinking passes tops out around ~0.28m) -- the multi-tick
        convergence over ~1s that TestStartPlacement exercises is a separate,
        gradual process, not one call doing the whole correction.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        prior = (1.35, 0.5)
        true_x, true_y = 1.5, 0.5
        ranges = walls.raycast(true_x, true_y, 0.0, _ANGLES)

        est_x, est_y = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est_x == pytest.approx(true_x, abs=0.02)
        assert est_y == pytest.approx(true_y, abs=0.02)

    def test_rejects_an_ambiguous_match_even_in_bounds(self):
        """The actual failure mode found on real hardware: not a wrong-but-
        confident match, but a flat cost landscape with no real winner.

        A stub whose predicted range is the same constant regardless of (x, y)
        makes every candidate in the search tie exactly -- the most extreme
        case of the ambiguity confirmed on real CCW/CW captures, where the
        winner led the runner-up by well under 1% of cost. Distinct from
        ``test_accepts_a_large_in_bounds_correction`` (a real, unambiguous
        global minimum far below every alternative): here there is no
        alternative that is actually worse, so the result must not be trusted.
        """

        class _FlatWalls:
            def raycast(self, x, y, yaw, angles_rad):
                return np.full(len(angles_rad), 1.0)

            def point_in_free_space(self, x, y):
                return True

        localizer = LidarLocalizer(_FlatWalls())
        prior = (1.5, 0.5)
        # Every candidate predicts 1.0m; a uniform 1.3m "scan" disagrees by the
        # same clipped 0.3m on every ray, everywhere in the search window --
        # cost is well above the floor (large, real disagreement) but tied
        # between every candidate (zero distinctiveness).
        ranges = np.full(RobotSpecs.LIDAR_SAMPLES, 1.3)

        est = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est == prior


def test_estimate_runs_within_control_tick_budget():
    """A single estimate must comfortably fit inside a 50 ms (20 Hz) control tick."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    x, y, yaw = 1.5, 0.5, 0.0
    ranges = walls.raycast(x, y, yaw, _ANGLES)

    start = time.perf_counter()
    for _ in range(10):
        localizer.estimate_position((x - 0.03, y + 0.02), yaw, ranges, _ANGLES)
    elapsed_per_call = (time.perf_counter() - start) / 10

    assert elapsed_per_call < 0.05, f"estimate_position took {elapsed_per_call * 1000:.1f} ms, over the 50 ms budget"
