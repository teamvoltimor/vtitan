"""Validation of LidarLocalizer against the simulator's exact wall geometry.

Generates a synthetic "real" scan from a known ground-truth pose using the
same TrackWalls raycast the simulator itself uses, cast from the LIDAR mount
rather than the chassis centre so it matches both the simulator and the
localizer's own prediction (see adr:0080-lidar-mount-and-scan-plane) -- then
checks the localizer recovers that pose (a) with clean rays, (b) under realistic
LIDAR noise, and (c) starting from a prior offset by a plausible per-tick
displacement rather than the exact ground truth, the three conditions it will
actually face.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import shipped_group
from shared.config.navigation_tuning.blind_nav import LocalizationParams
from shared.domain.enums import Section
from shared.domain.models import Waypoint

from src.navigation.localization import LidarLocalizer
from src.navigation.track_geometry import TrackWalls

_ANGLES = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)


def _sensor_scan(walls: TrackWalls, x: float, y: float, yaw: float, angles: np.ndarray) -> np.ndarray:
    """Ranges a robot at ``(x, y, yaw)`` would measure.

    Cast from the LIDAR, which sits ``LIDAR_MOUNT_X_OFFSET`` forward of the
    chassis centre, not from the centre itself. Casting from the centre is what
    these tests once did, and it agreed with the simulator and the localizer
    because all three shared the omission, so the suite passed while the
    modelled sensor sat behind the real one. See
    adr:0080-lidar-mount-and-scan-plane.
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
    return LidarLocalizer(walls, shipped_group(LocalizationParams)), walls


@pytest.mark.parametrize("x, y, yaw", _STRAIGHT_POSES + _CORNER_POSES)
def test_recovers_exact_pose_from_clean_scan(x, y, yaw):
    localizer, walls = _localizer_for(_UNIFORM_1000)
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

    # Prior offset by a plausible per-tick displacement (up to ~5 cm at 10 Hz
    # LIDAR refresh and FAST_SPEED), not the exact ground truth.
    prior = Waypoint(x - 0.03, y + 0.02)
    est = localizer.estimate_position(prior, yaw, ranges, _ANGLES)

    assert est.x == pytest.approx(x, abs=0.02)
    assert est.y == pytest.approx(y, abs=0.02)


@pytest.mark.parametrize("widths", [_UNIFORM_1000, _MIXED_WIDTHS, _NARROW])
def test_recovers_pose_across_corridor_widths(widths):
    localizer, walls = _localizer_for(widths)
    # A position that is valid across all three width configurations (the
    # narrowest corridor, 0.6 m, still leaves room at the corridor midline).
    x, y, yaw = 1.5, 0.3, 0.2
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

    est = localizer.estimate_position(Waypoint(x - 0.03, y - 0.03), yaw, ranges, _ANGLES)

    assert est.x == pytest.approx(x, abs=0.02)
    assert est.y == pytest.approx(y, abs=0.02)


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

    prior = Waypoint(x - 0.03, y + 0.02)
    est = localizer.estimate_position(prior, yaw, noisy, _ANGLES)

    # Noise widens the tolerance a little, but should still be well within
    # the chassis half-width (0.075 m) — good enough to drive on.
    assert est.x == pytest.approx(x, abs=0.05)
    assert est.y == pytest.approx(y, abs=0.05)


def test_tracks_a_moving_pose_tick_by_tick():
    """Simulate ~1s of driving: each tick's estimate seeds the next, like production."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    rng = np.random.default_rng(1)

    x, y, yaw = 0.5, 0.5, 0.0
    speed = 0.3  # m/s
    dt = 0.1  # 10 Hz LIDAR refresh
    est = Waypoint(x, y)  # first estimate seeded from the known scenario start position

    for _ in range(20):
        x += speed * dt
        clean = _sensor_scan(walls, x, y, yaw, _ANGLES)
        noisy = clean + rng.normal(0.0, RobotSpecs.LIDAR_NOISE_STDDEV, clean.shape)
        est = localizer.estimate_position(est, yaw, noisy, _ANGLES)
        assert est.x == pytest.approx(x, abs=0.05)
        assert est.y == pytest.approx(y, abs=0.05)


class TestPlausibilityGuards:
    """The search is a local hill-climb reseeded from prior_xy every call with
    no other check on its own output -- confirmed on real hardware to snap to a
    physically impossible (off-track) position during a k_turn escape and stay
    there for the rest of the run. Guard: reject a result outside the known
    track (or inside the inner block, equally impossible), holding prior_xy
    instead.

    A cost/margin ambiguity guard (reject a winning candidate whose margin
    over its runner-up was too thin) was tried, committed, and reverted after
    replaying it against real hardware runs: confirmed-bad and genuinely correct
    matches had statistically indistinguishable cost and margin distributions on
    real, noisy scans. The signal it depended on only existed in the clean
    simulator. See adr:0084-localizer-divergence-and-relocalization.

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
        prior = Waypoint(0.05, 1.5)
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
        prior = Waypoint(0.9, 1.5)
        # A scan generated from a position inside the inner block:
        # mathematically valid raycast geometry, physically impossible.
        ranges = _sensor_scan(walls, 1.5, 1.5, 0.0, _ANGLES)

        est = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est == prior

    def test_accepts_a_large_in_bounds_correction(self):
        """The start-placement-absorption case: a big single-tick jump is
        legitimate as long as it lands inside the track.

        A jump well beyond the old (reverted) ``max_step_m`` that broke this
        case, but within one call's actual reach (the default ``search_radius_m``
        across its shrinking passes); the multi-tick convergence over about a
        second that TestStartPlacement exercises is a separate, gradual process,
        not one call doing the whole correction. See
        adr:0084-localizer-divergence-and-relocalization.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        prior = Waypoint(1.35, 0.5)
        true_x, true_y = 1.5, 0.5
        ranges = _sensor_scan(walls, true_x, true_y, 0.0, _ANGLES)

        est = localizer.estimate_position(prior, 0.0, ranges, _ANGLES)

        assert est.x == pytest.approx(true_x, abs=0.02)
        assert est.y == pytest.approx(true_y, abs=0.02)

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
        est0 = localizer.estimate_position(Waypoint(x0, y0), 0.0, ranges0, _ANGLES, now_s=0.0)

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
        est0 = localizer.estimate_position(Waypoint(x0, y0), 0.0, ranges0, _ANGLES, now_s=0.0)

        far_x, far_y = x0 + 0.15, y0
        ranges1 = _sensor_scan(walls, far_x, far_y, 0.0, _ANGLES)
        est1 = localizer.estimate_position(est0, 0.0, ranges1, _ANGLES, now_s=0.05)
        assert est1 == est0  # held on first appearance

        est2 = localizer.estimate_position(est1, 0.0, ranges1, _ANGLES, now_s=0.10)

        assert est2.x == pytest.approx(far_x, abs=0.02)
        assert est2.y == pytest.approx(far_y, abs=0.02)


def test_estimate_runs_within_control_tick_budget():
    """A single estimate must comfortably fit inside a 50 ms (20 Hz) control tick."""
    localizer, walls = _localizer_for(_UNIFORM_1000)
    x, y, yaw = 1.5, 0.5, 0.0
    ranges = _sensor_scan(walls, x, y, yaw, _ANGLES)

    start = time.perf_counter()
    for _ in range(10):
        localizer.estimate_position(Waypoint(x - 0.03, y + 0.02), yaw, ranges, _ANGLES)
    elapsed_per_call = (time.perf_counter() - start) / 10

    assert elapsed_per_call < 0.05, f"estimate_position took {elapsed_per_call * 1000:.1f} ms, over the 50 ms budget"


class TestGlobalRelocalization:
    """Recovery from a seed the local search cannot walk back from.

    The local search is a hill-climb reseeded from its own previous answer, so
    a wrong seed is self-sustaining: a hardware run latched metres off and held
    that position for the rest of the round, driving the navigator into walls
    for a string of escape manoeuvres. These cover the escape hatch added for
    it, and the far more important half -- that a correctly-tracking localizer
    never takes it. See
    adr:0084-localizer-divergence-and-relocalization.

    ``_MIXED_WIDTHS`` throughout, not ``_UNIFORM_1000``: see
    ``test_a_symmetric_layout_cannot_be_disambiguated_by_cost``.
    """

    @staticmethod
    def _drive(localizer, walls, truth, seed, ticks, dt=0.05):
        """Feed ``ticks`` copies of the scan taken at ``truth``, seeded at ``seed``."""
        scan = _sensor_scan(walls, *truth, _ANGLES)
        estimate = seed
        for i in range(ticks):
            estimate = localizer.estimate_position(estimate, truth[2], scan, _ANGLES, now_s=i * dt)
        return estimate

    def test_recovers_from_a_seed_the_local_search_cannot_reach(self) -> None:
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        truth = (2.5, 1.5, math.pi / 2)
        # The opposite corridor, ~2 m away and far outside search_radius_m
        # (0.15 m) -- the situation the local search has no answer for, and the
        # one the hardware run was in.
        seed = Waypoint(0.3, 1.5)

        params = shipped_group(LocalizationParams)
        estimate = self._drive(localizer, walls, truth, seed, params.relocalize_after_scans + 1)

        assert localizer.relocalization_count == 1
        assert math.hypot(estimate.x - truth[0], estimate.y - truth[1]) <= params.relocalize_grid_step_m

    def test_does_not_fire_while_the_estimate_is_tracking(self) -> None:
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        truth = (2.5, 1.5, math.pi / 2)
        # A plausible per-tick displacement, i.e. exactly what the local search
        # exists to absorb. Firing here would throw away a good estimate.
        seed = Waypoint(truth[0] - 0.02, truth[1] - 0.02)

        self._drive(localizer, walls, truth, seed, shipped_group(LocalizationParams).relocalize_after_scans * 3)

        assert localizer.relocalization_count == 0
        assert localizer.last_fit_cost is not None
        assert localizer.last_fit_cost < shipped_group(LocalizationParams).relocalize_cost_threshold

    def test_the_streak_has_to_be_consecutive(self) -> None:
        """One explained scan resets the count, so scattered bad ticks cannot accumulate."""
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        truth = (2.5, 1.5, math.pi / 2)
        good = _sensor_scan(walls, *truth, _ANGLES)
        bad = _sensor_scan(walls, 0.3, 1.5, math.pi / 2, _ANGLES)

        estimate = Waypoint(truth[0], truth[1])
        for i in range(shipped_group(LocalizationParams).relocalize_after_scans * 4):
            estimate = localizer.estimate_position(
                estimate, truth[2], good if i % 3 == 0 else bad, _ANGLES, now_s=i * 0.05
            )

        assert localizer.relocalization_count == 0

    def test_reset_tracking_clears_the_streak(self) -> None:
        """A re-seed voids evidence accrued against the estimate it replaces."""
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        truth = (2.5, 1.5, math.pi / 2)
        params = shipped_group(LocalizationParams)

        self._drive(localizer, walls, truth, Waypoint(0.3, 1.5), params.relocalize_after_scans - 1)
        assert localizer.relocalization_count == 0
        localizer.reset_tracking()
        self._drive(localizer, walls, truth, Waypoint(0.3, 1.5), params.relocalize_after_scans - 1)

        assert localizer.relocalization_count == 0

    def test_a_symmetric_layout_cannot_be_disambiguated_by_cost(self) -> None:
        """The known limit of this guard, asserted rather than left to be rediscovered.

        On a uniform layout the four corridors are congruent, so a pose in the
        wrong one predicts very nearly the scan the right one produces, under
        the cost threshold. The detector stays silent, correctly: nothing in a
        single sweep distinguishes those poses, and relocalizing would be a coin
        flip, not a correction. What rescues the real robot is that a WRO Open
        layout has unequal corridors, which is what the failing hardware run
        had too. See adr:0084-localizer-divergence-and-relocalization.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        truth = (2.5, 1.5, math.pi / 2)

        self._drive(localizer, walls, truth, Waypoint(0.5, 1.5), shipped_group(LocalizationParams).relocalize_after_scans * 2)

        assert localizer.relocalization_count == 0
        assert localizer.last_fit_cost is not None
        assert localizer.last_fit_cost < shipped_group(LocalizationParams).relocalize_cost_threshold

    def test_a_symmetric_layout_is_refused_even_when_the_cost_is_bad(self) -> None:
        """The hardware case the cost-based protection above does not cover.

        ``test_a_symmetric_layout_cannot_be_disambiguated_by_cost`` keeps the
        search silent on a uniform layout by relying on the cost STAYING LOW:
        the wrong corridor predicts nearly the right sweep, so the streak never
        builds. That holds for a scan the wall model can explain. It does not
        hold in the parking bay, where the sweep is full of returns from the lot
        -- which is the one feature that would break the symmetry and is absent
        from the model. MEASURED on run_20260915_140358: the best cost anywhere
        sits at 0.0295-0.0388 against a 0.03 threshold, so the streak builds
        every 1.5 s, the search runs, and the coin flip the other test describes
        is then taken for real. It teleported the estimate to the 180 degree
        rotational copy of the truth and flipped yaw by 179 degrees.

        So the protection is made explicit here rather than inherited from the
        cost: a symmetric model must refuse the search BECAUSE it is symmetric,
        not because the arithmetic happened to stay quiet.
        """
        localizer, walls = _localizer_for(_UNIFORM_1000)
        truth = (2.5, 1.5, math.pi / 2)
        unexplainable = np.full(len(_ANGLES), 1.0)

        estimate = Waypoint(truth[0], truth[1])
        for i in range(shipped_group(LocalizationParams).relocalize_after_scans * 3):
            estimate = localizer.estimate_position(estimate, truth[2], unexplainable, _ANGLES, now_s=i * 0.05)

        assert localizer.last_fit_cost is not None
        assert localizer.last_fit_cost > shipped_group(LocalizationParams).relocalize_cost_threshold, (
            "test is void unless the scan really does score badly everywhere -- "
            "otherwise the cost guard is what kept the search silent, not the symmetry guard"
        )
        assert localizer.relocalization_count == 0
        assert walls.point_in_free_space(estimate.x, estimate.y)

    def test_an_asymmetric_layout_still_rescues_under_the_same_conditions(self) -> None:
        """The other half of the guard: it must not silence the rescue it was added around.

        Same seed and same truth as
        ``test_recovers_from_a_seed_the_local_search_cannot_reach``, stated
        separately so that a future change to the symmetry threshold shows up
        as a failure HERE -- as "the rescue stopped working" -- rather than as
        a number quietly drifting past 0.40 m, which is what the Open layouts
        and the lost hardware run actually separate their corridors by.
        """
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        assert walls.geometry.width_spread_m >= shipped_group(LocalizationParams).relocalize_min_width_spread_m
        truth = (2.5, 1.5, math.pi / 2)

        params = shipped_group(LocalizationParams)
        estimate = self._drive(localizer, walls, truth, Waypoint(0.3, 1.5), params.relocalize_after_scans + 1)

        assert localizer.relocalization_count == 1
        assert math.hypot(estimate.x - truth[0], estimate.y - truth[1]) <= params.relocalize_grid_step_m

    def test_no_jump_when_the_global_winner_is_no_better(self) -> None:
        """A high cost does not always mean the POSE is wrong.

        It can equally mean the WALL MODEL is wrong -- routine during blind
        operation, while corridor widths are still being estimated. Then every
        candidate fits badly, including the correct one, and the global search
        returns the best explanation of a track that is not there. Letting that
        replace a pose that was fine cost the balanced Open sweep a case before
        RELOCALIZE_ACCEPT_RATIO was added. See
        adr:0084-localizer-divergence-and-relocalization.

        Stood up here with a sweep no pose on the track can produce, which is
        the same condition -- nowhere fits, so nowhere is materially better.
        """
        localizer, walls = _localizer_for(_MIXED_WIDTHS)
        truth = (2.5, 1.5, math.pi / 2)
        unexplainable = np.full(len(_ANGLES), 1.0)

        estimate = Waypoint(truth[0], truth[1])
        for i in range(shipped_group(LocalizationParams).relocalize_after_scans * 3):
            estimate = localizer.estimate_position(estimate, truth[2], unexplainable, _ANGLES, now_s=i * 0.05)

        assert localizer.last_fit_cost is not None
        assert localizer.last_fit_cost > shipped_group(LocalizationParams).relocalize_cost_threshold, (
            "test is void unless the scan really does score badly everywhere"
        )
        assert localizer.relocalization_count == 0
        assert walls.point_in_free_space(estimate.x, estimate.y)
