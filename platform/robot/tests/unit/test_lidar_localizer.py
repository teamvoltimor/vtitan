"""Validation of LidarLocalizer against the simulator's exact wall geometry.

Generates a synthetic "real" scan from a known ground-truth pose using the
same TrackWalls raycast the simulator itself uses -- cast from the LIDAR mount
rather than the chassis centre, matching both the simulator and the localizer's
own prediction since 2026-08-21 -- then checks the localizer recovers that pose (a) with clean rays, (b) under realistic LIDAR noise, and
(c) starting from a prior offset by a plausible per-tick displacement rather
than the exact ground truth — the three conditions it will actually face.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from shared.config.constants import RobotSpecs
from shared.domain.enums import Section

from src.navigation.localization import LidarLocalizer
from src.navigation.track_geometry import TrackWalls

_ANGLES = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)


def _sensor_scan(walls: TrackWalls, x: float, y: float, yaw: float, angles: np.ndarray) -> np.ndarray:
    """Ranges a robot at ``(x, y, yaw)`` would measure.

    Cast from the LIDAR, which sits ``LIDAR_MOUNT_X_OFFSET`` forward of the
    chassis centre, not from the centre itself. Casting from the centre is what
    these tests did until 2026-08-21, and it agreed with the simulator and the
    localizer because all three shared the omission -- so the suite passed while
    the modelled sensor sat 12.2 cm behind the real one.
    """
    return walls.raycast(
        x + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(yaw),
        y + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(yaw),
        yaw,
        angles,
    )

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
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

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
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

    est_x, est_y = localizer.estimate_position((x - 0.03, y - 0.03), yaw, ranges, _ANGLES)

    assert est_x == pytest.approx(x, abs=0.02)
    assert est_y == pytest.approx(y, abs=0.02)


@pytest.mark.parametrize("x, y, yaw", _STRAIGHT_POSES + _CORNER_POSES)
def test_robust_to_realistic_lidar_noise(x, y, yaw):
    """Same poses, but with real Slamtec-C1-level Gaussian range noise."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    rng = np.random.default_rng(0)
    clean = _sensor_scan(walls, x, y, yaw, _ANGLES)
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
        clean = _sensor_scan(walls, x, y, yaw, _ANGLES)
        noisy = clean + rng.normal(0.0, RobotSpecs.LIDAR_NOISE_STDDEV, clean.shape)
        est = localizer.estimate_position(est, yaw, noisy, _ANGLES)
        assert est[0] == pytest.approx(x, abs=0.05)
        assert est[1] == pytest.approx(y, abs=0.05)


class TestPlausibilityGuards:
    """The search is a local hill-climb reseeded from prior_xy every call with
    no other check on its own output -- confirmed on real hardware 2026-08-04
    to snap to a physically impossible (off-track) position during a k_turn
    escape and stay there for the rest of the run. Guard: reject a result
    outside the known track (or inside the inner block -- equally impossible),
    holding prior_xy instead.

    A cost/margin ambiguity guard (reject a winning candidate whose margin
    over its runner-up was too thin) was tried, committed, and reverted
    2026-08-05 after replaying it against 22 real hardware runs (846 sampled
    ticks): confirmed-bad and genuinely correct matches had statistically
    indistinguishable cost and margin distributions on real, noisy scans. The
    signal it depended on only existed in the clean simulator.

    Guard instead: bound physical plausibility directly, from elapsed time and
    the drivetrain's real top speed (with headroom for a future faster
    drivetrain). A single tick implying impossible speed is held; the same
    candidate winning again on the very next tick is trusted, since a real
    correction reconverges to nearly the same position from an independent
    scan while an ambiguous flip does not typically repeat identically. This
    needs the caller to pass ``now_s`` -- omitted, the guard (and the
    hand-placement-absorption exemption on a localizer's very first call) does
    not apply at all, matching every call before it existed.
    """

    def test_rejects_result_outside_track_bounds(self):
        localizer, walls = _localizer_for(_UNIFORM_1000)
        prior = (0.05, 1.5)
        # A scan generated from a position outside the track (x < 0):
        # mathematically valid raycast geometry, physically impossible.
        ranges = _sensor_scan(walls, -0.2, 1.5, 0.0, _ANGLES)

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
        ranges = _sensor_scan(walls, 1.5, 1.5, 0.0, _ANGLES)

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
        ranges = _sensor_scan(walls, true_x, true_y, 0.0, _ANGLES)

        est_x, est_y = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est_x == pytest.approx(true_x, abs=0.02)
        assert est_y == pytest.approx(true_y, abs=0.02)

    def test_rejects_an_implausibly_fast_single_tick_jump(self):
        """A candidate implying far more speed than the drivetrain can produce
        is held on its first appearance, even though it is otherwise a clean,
        unambiguous match (within the search's actual per-call reach) --
        distinct from ``test_accepts_a_large_in_bounds_correction``, which is
        the SAME kind of jump but on a localizer's very first call, where
        there is no elapsed-time baseline yet and the guard does not apply.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        x0, y0 = 1.5, 0.5
        ranges0 = _sensor_scan(walls, x0, y0, 0.0, _ANGLES)
        est0 = localizer.estimate_position((x0, y0), 0.0, ranges0, _ANGLES, now_s=0.0)

        # 0.15m in 0.05s implies 3 m/s -- far beyond max_speed_mps (0.25 default).
        far_x, far_y = x0 + 0.15, y0
        ranges1 = _sensor_scan(walls, far_x, far_y, 0.0, _ANGLES)
        est1 = localizer.estimate_position(est0, 0.0, ranges1, _ANGLES, now_s=0.05)

        assert est1 == est0

    def test_confirms_a_repeated_jump_on_the_next_tick(self):
        """The same implausible jump winning again on the very next tick is
        trusted: a real correction reconverges to nearly the same position
        from an independent scan, unlike the ambiguous-flip failure mode this
        guard replaced (see class docstring).
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        x0, y0 = 1.5, 0.5
        ranges0 = _sensor_scan(walls, x0, y0, 0.0, _ANGLES)
        est0 = localizer.estimate_position((x0, y0), 0.0, ranges0, _ANGLES, now_s=0.0)

        far_x, far_y = x0 + 0.15, y0
        ranges1 = _sensor_scan(walls, far_x, far_y, 0.0, _ANGLES)
        est1 = localizer.estimate_position(est0, 0.0, ranges1, _ANGLES, now_s=0.05)
        assert est1 == est0  # held on first appearance

        est2 = localizer.estimate_position(est1, 0.0, ranges1, _ANGLES, now_s=0.10)

        assert est2[0] == pytest.approx(far_x, abs=0.02)
        assert est2[1] == pytest.approx(far_y, abs=0.02)


def test_estimate_runs_within_control_tick_budget():
    """A single estimate must comfortably fit inside a 50 ms (20 Hz) control tick."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    x, y, yaw = 1.5, 0.5, 0.0
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

    start = time.perf_counter()
    for _ in range(10):
        localizer.estimate_position((x - 0.03, y + 0.02), yaw, ranges, _ANGLES)
    elapsed_per_call = (time.perf_counter() - start) / 10

    assert elapsed_per_call < 0.05, f"estimate_position took {elapsed_per_call * 1000:.1f} ms, over the 50 ms budget"
