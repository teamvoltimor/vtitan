"""Unit cover for the in-bay start manoeuvre.

Everything in ``src.navigation.maneuvers.bay_exit`` was measured only through
``scripts/sim/diag_bay_start.py`` until now -- a whole-simulator probe that takes
minutes and answers "did the round score", not "did the manoeuvre do what it
says". The Go port has had unit tests since it landed; this is the Python side of
that pair, and it exists because three separate defects in here (the falsy-zero
leg bound, the unsigned reverse odometry, the worst-case clearance bound) all
presented as a SWEEP THAT CAME BACK FLAT rather than as a visible failure.

The closed loop below is a MODEL-level check: it drives the manoeuvre against the
same rectangle geometry the guard reasons with, so it proves the guard is
self-consistent -- it never steers the pose it believes in into a fin -- and not
that the physical chassis stays clear. The physical claim needs the simulator's
contact model and lives in the diagnostic script.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import ParkingLotSpecs, RobotSpecs

from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.maneuvers.bay_exit import (
    BayExit,
    _along_slack_m,
    _fin_rects,
    _gap,
    _rect_corners,
    _wall_feasible_yaw_rad,
)

_EFFECTIVE_WHEELBASE_M = RobotSpecs.WHEELBASE / (1.0 + RobotSpecs.REAR_STEER_RATIO)
# Rays the manoeuvre reads to decide which way is out: a single pair at +-90 deg,
# with the open corridor on the left and the outer wall a hair away on the right.
_ANGLES_RAD = (math.pi / 2, -math.pi / 2, 0.0)
_RANGES_M = (1.0, ParkingLotSpecs.WALL_OFFSET, 0.2)


def _guard_tuning(**changes: object):
    """Shipped tuning with the clearance guard on, plus any per-test overrides."""
    return tuning_with_overrides({"BAY_EXIT_CLEARANCE_GUARD": True, **changes})


def _true_fin_gap(along: float, out: float, yaw: float) -> float:
    """Smallest separation between the chassis at this pose and either fin."""
    corners = _rect_corners(along, out, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
    return min(_gap(corners, fin) for fin in _fin_rects())


class _Pocket:
    """The pocket's kinematics, clipped by the wall exactly as the sim clips them.

    ``allowed_step`` keeps the full translation and takes whatever fraction of
    the turn fits, which for a chassis parallel to a wall behind it is the
    closed-form bound in ``_wall_feasible_yaw_rad``. Reproduced here rather than
    imported so the test states the physics it assumes instead of inheriting it.
    """

    def __init__(self) -> None:
        self.along = 0.0
        self.out = 0.0
        self.yaw = 0.0
        self.wheel_rad = 0.0
        self.travelled_m = 0.0

    def step(self, speed_mps: float, steering_norm: float, tuning) -> None:
        dt = 1.0 / tuning.control.CONTROL_HZ
        slew = tuning.pursuit.MAX_STEERING_RATE * dt
        target = steering_norm * math.radians(RobotSpecs.MAX_WHEEL_ANGLE_DEG)
        self.wheel_rad += max(-slew, min(slew, target - self.wheel_rad))
        step_m = speed_mps * dt
        self.yaw += step_m * math.tan(self.wheel_rad) / _EFFECTIVE_WHEELBASE_M * RobotSpecs.YAW_GAIN
        limit = _wall_feasible_yaw_rad(self.out)
        self.yaw = max(-limit, min(limit, self.yaw))
        self.along += step_m * math.cos(self.yaw)
        self.out += step_m * math.sin(self.yaw)
        # Signed, like a quadrature encoder: the reverse leg counts DOWN.
        self.travelled_m += step_m


def _drive(ticks: int, tuning) -> tuple[_Pocket, list[float]]:
    """Run the manoeuvre in the pocket, returning the pose and every steering command."""
    exit_maneuver = BayExit()
    pocket = _Pocket()
    steering: list[float] = []
    for _ in range(ticks):
        command = exit_maneuver.command(
            _RANGES_M,
            _ANGLES_RAD,
            pocket.travelled_m,
            tuning.speed.medium_mps(),
            tuning,
        )
        steering.append(command.steering_norm)
        pocket.step(command.speed_mps, command.steering_norm, tuning)
    return pocket, steering


@pytest.mark.parametrize(
    ("out_m", "expected_deg"),
    [(0.0, 1.15), (0.005, 3.11), (0.010, 5.12), (0.020, 9.31)],
)
def test_wall_feasible_yaw_matches_the_closed_form(out_m: float, expected_deg: float) -> None:
    """The pocket's depth pins rotation, and by how much is not a free parameter."""
    assert math.degrees(_wall_feasible_yaw_rad(out_m)) == pytest.approx(expected_deg, abs=0.02)


def test_wall_stops_bounding_the_yaw_once_the_body_is_clear() -> None:
    """Past the swept-diagonal reach the wall constrains nothing, so the ratchet ends."""
    free_at_m = math.hypot(RobotSpecs.LENGTH, RobotSpecs.WIDTH) / 2.0 - ParkingLotSpecs.WALL_OFFSET
    assert free_at_m == pytest.approx(0.0786, abs=0.001)
    assert _wall_feasible_yaw_rad(free_at_m + 1e-6) == pytest.approx(math.pi / 2)


def test_guarded_exit_holds_one_steering_angle_across_leg_changes() -> None:
    """The reverse leg must HOLD the forward lock, not mirror it.

    Mirroring drives the yaw the same sense on both legs, so it saturates against
    the wall clip and never reaches the far side of it -- which is where the
    reverse leg's outward gain lives. It also charges a full servo swing per leg
    change. Both were true until 2026-09-04, and the manoeuvre measured 0.06 m of
    travel for no net gain.
    """
    _, steering = _drive(400, _guard_tuning(BAY_EXIT_SPEED_SCALE=0.35))
    assert steering, "the manoeuvre issued no commands"
    assert len({math.copysign(1.0, s) for s in steering if s != 0.0}) == 1
    assert len(set(steering)) == 1, "the guarded exit should command one angle throughout"


def test_guarded_exit_ratchets_outward_against_the_wall() -> None:
    """Outward displacement accumulates, which no free-space shuffle can do.

    At constant steering magnitude ``y`` is a state function of the heading, so a
    cycle that returns the heading returns the displacement with it. The wall
    clip is what breaks that, and this is the test that says so: remove the clip
    from ``_Pocket.step`` and the assertion below fails.
    """
    pocket, _ = _drive(600, _guard_tuning(BAY_EXIT_SPEED_SCALE=0.35))
    assert pocket.out > 0.002
    assert abs(pocket.along) < ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH / 2.0


def test_guarded_exit_never_steers_the_modelled_pose_into_a_fin() -> None:
    """9.24.7 ends the round on lot contact, so the guard's own pose must stay clear.

    Checked every tick rather than at the end: the previous exits ended their
    legs ON a stall, which is contact, and a final-pose test cannot see a touch
    the chassis has since backed away from.
    """
    tuning = _guard_tuning(BAY_EXIT_SPEED_SCALE=0.35)
    exit_maneuver = BayExit()
    pocket = _Pocket()
    worst = math.inf
    for _ in range(600):
        command = exit_maneuver.command(
            _RANGES_M,
            _ANGLES_RAD,
            pocket.travelled_m,
            tuning.speed.medium_mps(),
            tuning,
        )
        pocket.step(command.speed_mps, command.steering_norm, tuning)
        worst = min(worst, _true_fin_gap(pocket.along, pocket.out, pocket.yaw))
    assert worst > 0.0


def test_the_margin_band_that_froze_the_ratchet_is_closed() -> None:
    """The clearance margin was opening a deadlock, not only absorbing error.

    Every leg is bounded by ``reach``, and there is a band of outward positions
    where a step of that size lands just under the margin in BOTH directions.
    Both legs are then refused while the modelled pose is still perfectly clear,
    so nothing moves, so the dead-reckoned pose never changes, so the refusal is
    permanent. The band opens at ``_dr_out`` = 0.0365 m and the ratchet drives
    ``out`` straight through it by design.

    Pinned rather than inherited on the failing arm, because the fix is a
    SHIPPED VALUE (0.005 -> 0.001) and a test that read the shipped value would
    stop covering the bug the moment it was tuned back.
    """
    frozen, _ = _drive(900, _guard_tuning(BAY_EXIT_CLEARANCE_MARGIN_M=0.005))
    settled, _ = _drive(200, _guard_tuning(BAY_EXIT_CLEARANCE_MARGIN_M=0.005))
    # Stalled: four and a half times the ticks buy nothing at all.
    assert frozen.out == pytest.approx(settled.out, abs=1e-6)

    escaped, _ = _drive(900, _guard_tuning())
    assert escaped.out > frozen.out * 10
    # ...and it got out by clearing the fins, not by pushing through them.
    assert _true_fin_gap(escaped.along, escaped.out, escaped.yaw) > 0.0


def test_a_forward_arc_with_no_returns_is_blocked_not_clear() -> None:
    """The state that ended a real exit mid-reverse, facing the wall.

    ``_forward_clearance`` reports ``inf`` when nothing in the forward arc
    survives ``MIN_VALID_RANGE_M``, and ``inf`` compares as clear against any
    threshold. In a pocket that is exactly backwards: no returns means the wall
    is inside the sensor's minimum range. Measured on run_20260906_094342 --
    the arc read 0.050-0.052 m (self-detection) for seconds, then dropped out
    for one tick, and that tick released the manoeuvre.
    """
    tuning = tuning_with_overrides({})
    blind = (float("inf"),) * 3
    assert not BayExit.is_clear(blind, _ANGLES_RAD, tuning)


def test_a_genuinely_open_arc_still_releases_the_maneuver() -> None:
    """The no-return rule must not swallow the real exit condition."""
    tuning = tuning_with_overrides({})
    open_ahead = (1.0, 1.0, tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M + 0.5)
    assert BayExit.is_clear(open_ahead, _ANGLES_RAD, tuning)


def test_a_blocked_arc_inside_the_threshold_is_not_clear() -> None:
    """Unchanged behaviour: a real return below the threshold still holds."""
    tuning = tuning_with_overrides({})
    boxed = (1.0, 1.0, tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M - 0.05)
    assert not BayExit.is_clear(boxed, _ANGLES_RAD, tuning)


def _contact_ranges(forward_m: float) -> tuple[float, ...]:
    """Side rays unchanged, forward ray at ``forward_m``."""
    return (1.0, ParkingLotSpecs.WALL_OFFSET, forward_m)


def test_a_silent_forward_arc_backs_off_instead_of_steering() -> None:
    """No returns means the wall is INSIDE minimum range, not that it is gone.

    This is the state that ended run_20260906_112613 with the nose buried and
    normal driving then accelerating into the wall.
    """
    tuning = tuning_with_overrides({"BAY_EXIT_CONTACT_RECOVERY_TICKS": 12})
    exit_maneuver = BayExit()
    command = exit_maneuver.command((float("inf"),) * 3, _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
    assert command.speed_mps < 0.0
    assert command.steering_norm == pytest.approx(0.0)
    assert exit_maneuver.contact_recoveries == 1


def test_contact_range_backs_off_straight() -> None:
    """A reading below BAY_EXIT_CONTACT_DIST_M is contact, and reverses STRAIGHT."""
    tuning = tuning_with_overrides({"BAY_EXIT_CONTACT_RECOVERY_TICKS": 12})
    contact = tuning.corridor_follower.BAY_EXIT_CONTACT_DIST_M - 0.01
    exit_maneuver = BayExit()
    command = exit_maneuver.command(_contact_ranges(contact), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
    assert command.speed_mps < 0.0
    assert command.steering_norm == pytest.approx(0.0)


def test_the_recovery_is_held_for_its_full_count() -> None:
    """Held for a fixed count, because the arc is SILENT while in contact --
    'reverse until it reads clear' waits on the sensor that just went blind."""
    tuning = tuning_with_overrides({"BAY_EXIT_CONTACT_RECOVERY_TICKS": 12})
    ticks = tuning.corridor_follower.BAY_EXIT_CONTACT_RECOVERY_TICKS
    exit_maneuver = BayExit()
    exit_maneuver.command((float("inf"),) * 3, _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
    # Clear ahead from here on; the recovery must still run out its count.
    for _ in range(ticks - 1):
        command = exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
        assert command.speed_mps < 0.0
        assert command.steering_norm == pytest.approx(0.0)
    assert exit_maneuver.contact_recoveries == 1


def test_a_clear_arc_does_not_trigger_recovery() -> None:
    """The normal manoeuvre is untouched when nothing is against the nose."""
    tuning = tuning_with_overrides({"BAY_EXIT_CONTACT_RECOVERY_TICKS": 12})
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
    assert exit_maneuver.contact_recoveries == 0


def test_rotation_is_measured_from_placement_and_unwraps() -> None:
    """Accumulated, not differenced: a wrap at +/-pi must not read as 360 deg."""
    tuning = tuning_with_overrides({})
    exit_maneuver = BayExit()
    for yaw in (3.0, 3.1, -3.1, -3.0):  # crosses +pi going one way
        exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning, yaw_rad=yaw)
    # 3.0->3.1 is +0.1, 3.1->-3.1 CROSSES +pi and is +(2pi - 6.2), -3.1->-3.0 is +0.1.
    # Differencing the endpoints instead would read -6.0 rad: the bug being guarded.
    expected = 0.1 + (2.0 * math.pi - 6.2) + 0.1
    assert exit_maneuver.rotation_deg == pytest.approx(math.degrees(expected), abs=1e-6)


def test_turning_far_enough_drives_straight_out_instead_of_steering() -> None:
    """Past the target the nose comes back around toward the outer wall."""
    tuning = tuning_with_overrides({})
    target = math.radians(tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG)
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning, yaw_rad=0.0)
    assert not exit_maneuver.rotation_complete(tuning)
    command = exit_maneuver.command(
        _contact_ranges(2.0),
        _ANGLES_RAD,
        0.0,
        tuning.speed.medium_mps(),
        tuning,
        yaw_rad=-(target + 0.05),
    )
    assert exit_maneuver.rotation_complete(tuning)
    assert command.speed_mps > 0.0
    assert command.steering_norm == pytest.approx(0.0)


def test_rotation_release_needs_a_measurement() -> None:
    """No yaw supplied means no claim: the manoeuvre keeps its old behaviour."""
    tuning = tuning_with_overrides({})
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
    assert not exit_maneuver.rotation_complete(tuning)
    assert exit_maneuver.rotation_deg == pytest.approx(0.0)


def test_contact_recovery_still_wins_over_a_completed_rotation() -> None:
    """Backing off the wall comes first -- driving out of it forward does not work."""
    tuning = tuning_with_overrides({"BAY_EXIT_CONTACT_RECOVERY_TICKS": 12})
    target = math.radians(tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG)
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(2.0), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning, yaw_rad=0.0)
    command = exit_maneuver.command(
        (float("inf"),) * 3,
        _ANGLES_RAD,
        0.0,
        tuning.speed.medium_mps(),
        tuning,
        yaw_rad=-(target + 0.05),
    )
    assert exit_maneuver.rotation_complete(tuning)
    assert command.speed_mps < 0.0
    assert exit_maneuver.contact_recoveries == 1


def test_turning_far_enough_is_not_enough_if_the_way_out_is_blocked() -> None:
    """70 deg says the chassis is no longer across the pocket, NOT that it is
    aimed down the corridor. Measured on run_20260906_145909: the exit released
    and normal driving took forward clearance 0.54 -> 0.08 m into the outer
    wall. Keep ratcheting instead -- the next reverse buys more angle.
    """
    tuning = tuning_with_overrides({})
    target = math.radians(tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG)
    blocked = tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M - 0.05
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(blocked), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning, yaw_rad=0.0)
    command = exit_maneuver.command(
        _contact_ranges(blocked),
        _ANGLES_RAD,
        0.0,
        tuning.speed.medium_mps(),
        tuning,
        yaw_rad=-(target + 0.05),
    )
    assert exit_maneuver.rotation_complete(tuning)
    # Still ratcheting: a steered leg, not the straight drive-out.
    assert command.steering_norm != pytest.approx(0.0)


def test_turned_and_clear_drives_straight_out() -> None:
    """Both conditions together are what release means."""
    tuning = tuning_with_overrides({})
    target = math.radians(tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG)
    clear = tuning.corridor_follower.MIN_FORWARD_CLEARANCE_M + 0.5
    exit_maneuver = BayExit()
    exit_maneuver.command(_contact_ranges(clear), _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning, yaw_rad=0.0)
    command = exit_maneuver.command(
        _contact_ranges(clear),
        _ANGLES_RAD,
        0.0,
        tuning.speed.medium_mps(),
        tuning,
        yaw_rad=-(target + 0.05),
    )
    assert command.speed_mps > 0.0
    assert command.steering_norm == pytest.approx(0.0)


def _fan(left_m: float, right_m: float, rays: int = 24) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """A scan fan with a uniform range each side, as the pocket presents them.

    Wide enough to fill ``BAY_EXIT_OPEN_SIDE_SECTOR_DEG`` either side, because
    the open-side test is a SECTOR now and a three-ray fixture cannot exercise
    the valid-fraction half of the score at all.
    """
    angles: list[float] = []
    ranges: list[float] = []
    for i in range(rays):
        offset = math.radians(-20.0 + 40.0 * i / (rays - 1))
        angles.append(math.pi / 2 + offset)
        ranges.append(left_m)
        angles.append(-math.pi / 2 + offset)
        ranges.append(right_m)
    return tuple(ranges), tuple(angles)


def _drop(ranges: tuple[float, ...], angles: tuple[float, ...], side_sign: float) -> tuple[float, ...]:
    """Replace one side's returns with the gateway's no-return substitution."""
    return tuple(
        RobotSpecs.LIDAR_MAX_RANGE if math.copysign(1.0, a) == side_sign else r
        for r, a in zip(ranges, angles, strict=True)
    )


def test_a_dropped_wall_ray_does_not_invert_the_open_side() -> None:
    """The defect that steered a real round into the wall for its whole exit.

    The gateway substitutes ``LIDAR_MAX_RANGE`` for every no-return, and the
    pocket wall at 0.08-0.13 m is close enough that the C1 returns nothing on
    21-37% of ticks -- against 0-5% for the ray facing open space. Compared as
    ranges, the substituted 12 m beats the corridor's real 0.84 m and the side
    reads BACKWARDS. Measured on run_20260906_192424: the open side was
    demonstrably RIGHT (0.839 m against 0.128 m) and the manoeuvre ratcheted at
    ``steering_norm=+1.0``, toward the wall, for 11.7 s.
    """
    tuning = _guard_tuning()
    ranges, angles = _fan(left_m=0.13, right_m=0.84)
    assert not BayExit()._resolve_open_side(ranges, angles, tuning)
    # Same pocket, but the near wall returns nothing at all. The verdict must
    # not move: no return AT the wall is the wall, not open corridor.
    blinded = _drop(ranges, angles, side_sign=1.0)
    assert not BayExit()._resolve_open_side(blinded, angles, tuning)


def test_the_open_side_latch_survives_one_corrupt_frame() -> None:
    """A latch taken on tick 1 rests the round on a frame no bag can show.

    On run_20260906_192315 and _192424 recording began 2.6 s and 1.9 s AFTER
    the exit did, so the deciding tick is absent from both. The vote makes the
    latch depend on several frames instead of the first one.
    """
    tuning = _guard_tuning()
    ranges, angles = _fan(left_m=0.13, right_m=0.84)
    exit_maneuver = BayExit()
    # One wholly corrupt opening frame, then the truth.
    exit_maneuver._resolve_open_side(_drop(ranges, angles, side_sign=1.0), angles, tuning)
    for _ in range(tuning.corridor_follower.BAY_EXIT_OPEN_SIDE_VOTES):
        exit_maneuver._resolve_open_side(ranges, angles, tuning)
    assert exit_maneuver._open_is_left is False


def test_dead_reckoned_along_cannot_leave_the_pocket() -> None:
    """A modelled pose the bay forbids is not one the guard may act on.

    ``_dr_along`` integrates wheel travel, and a wheel spinning against a
    chassis the wall is holding reports travel the body never made. Measured on
    run_20260906_192358 it reached 0.106 m -- 63% beyond the entire 65 mm of
    along-wall slack the pocket has.
    """
    tuning = _guard_tuning()
    exit_maneuver = BayExit()
    # Feed a metre of pure wheel travel: far more than the pocket can absorb.
    travelled = 0.0
    for _ in range(200):
        exit_maneuver._dead_reckon(travelled, 0.0, tuning)
        travelled += 0.005
    assert abs(exit_maneuver._dr_along) <= _along_slack_m() + 1e-9


def test_the_guard_releases_a_leg_that_improves_an_overlapping_pose() -> None:
    """Both legs refused at a negative gap is a trap, not a safe state.

    ``_predicted_gap`` takes the ``min`` over BOTH fins, so once the modelled
    body overlaps one, the fin the manoeuvre is moving AWAY from vetoes the leg
    exactly as hard as the one ahead. No travel means the pose never changes,
    so the refusal is permanent. Measured on run_20260906_192358: 285
    consecutive zero-speed ticks, 14.2 s of a 16.6 s exit, at a frozen 44 mm
    overlap, net rotation 7.5 deg against the 60-70 the escaping runs turned.
    """
    tuning = _guard_tuning()
    ranges, angles = _fan(left_m=1.0, right_m=ParkingLotSpecs.WALL_OFFSET)
    exit_maneuver = BayExit()
    exit_maneuver.command(ranges, angles, 0.0, tuning.speed.medium_mps(), tuning)
    # Drop the modelled pose inside a fin, which is the state the hardware
    # reached and which no admissible leg exists from under the margin test.
    exit_maneuver._dr_along = _along_slack_m()
    exit_maneuver._dr_yaw = math.radians(12.0)
    exit_maneuver._dr_out = 0.05
    assert exit_maneuver._predicted_gap(0.0, 1.0, tuning) <= 0.0

    moved = False
    travelled = 0.0
    for _ in range(40):
        command = exit_maneuver.command(ranges, angles, travelled, tuning.speed.medium_mps(), tuning)
        if command.speed_mps != 0.0:
            moved = True
            break
        travelled += 0.0
    assert moved, "the guard refused every leg from an overlapping pose"


def test_overlap_recovery_still_refuses_a_leg_that_makes_the_gap_worse() -> None:
    """The recovery is narrower than it sounds: improvement only, never depth.

    The guard's whole purpose is that it never approves a step toward a fin.
    Relaxing the test from "clears the margin" to "improves the gap" must not
    cost that, so every command the manoeuvre issues from an overlapping pose
    is checked against the gap it was issued at.
    """
    tuning = _guard_tuning()
    ranges, angles = _fan(left_m=1.0, right_m=ParkingLotSpecs.WALL_OFFSET)
    exit_maneuver = BayExit()
    exit_maneuver.command(ranges, angles, 0.0, tuning.speed.medium_mps(), tuning)
    exit_maneuver._dr_along = _along_slack_m()
    exit_maneuver._dr_yaw = math.radians(12.0)
    exit_maneuver._dr_out = 0.05
    margin = tuning.corridor_follower.BAY_EXIT_CLEARANCE_MARGIN_M

    admitted = 0
    for _ in range(40):
        held = exit_maneuver._predicted_gap(0.0, 1.0, tuning)
        command = exit_maneuver.command(ranges, angles, 0.0, tuning.speed.medium_mps(), tuning)
        if command.speed_mps == 0.0:
            continue
        admitted += 1
        step = command.speed_mps / tuning.control.CONTROL_HZ
        reach = step + math.copysign(abs(command.speed_mps) * RobotSpecs.SPEED_RESPONSE_TAU_S, step)
        gap = exit_maneuver._predicted_gap(reach, 1.0, tuning)
        assert gap > margin or gap > held, "admitted a leg that did not improve an overlapping gap"
    assert admitted, "no leg was ever admitted, so the property is untested"


def test_the_guard_hands_over_once_it_has_refused_every_leg_for_long_enough() -> None:
    """The escape hatch the guarded exit did not have, exercised at its cost.

    ``_guarded_command`` was answered above the ``BAY_EXIT_FALLBACK_FRAMES``
    switch, and that constant ships at 0, so a trapped guard could not be timed
    out by anything -- ``BAY_EXIT_MAX_FRAMES`` (45 s) fired in none of the
    2026-09-06 hardware runs, one of which stood still for 14.2 s.

    Ships OFF, and nothing needs it at the shipped margin: the handover buys
    motion by spending fin contact, which 9.24.7 ends the round on. It is kept
    reachable because it is the only bound on a guard that has trapped itself,
    so the trap is pinned back on here to prove the hatch still opens.
    """
    trapped = {"BAY_EXIT_CLEARANCE_MARGIN_M": 0.005}
    shut, _ = _drive(400, _guard_tuning(**trapped, BAY_EXIT_GUARD_BLOCK_TICKS=0))
    opened, _ = _drive(400, _guard_tuning(**trapped, BAY_EXIT_GUARD_BLOCK_TICKS=40))
    assert opened.out > shut.out


def _drive_stalled(ticks: int, tuning) -> list[bool]:
    """Run the manoeuvre with the wheel STALLED, returning the leg flag per tick.

    ``_Pocket`` always moves, and the hardware failure this covers is the case
    where it does not: measured 2026-09-08, the bay-exit wheel reads zero on a
    median 73% of ticks, and stall rate by time since the last reversal runs
    3.9% in the first 0.2 s to 99.7% by 4-8 s. Freezing ``travelled_m`` is that
    condition exactly -- the manoeuvre commands motion and no travel arrives.

    It is the CONJUNCTION that traps the guarded exit: ``_guarded_command`` ends
    a leg only when the dead-reckoned fin gap closes, and the dead reckoning
    integrates wheel travel, so a stalled leg cannot generate the evidence that
    would end it. ``BAY_EXIT_LEG_STALL_TICKS`` covers this for
    ``_cycle_command``, which ``BAY_EXIT_CLEARANCE_GUARD`` makes unreachable.
    """
    exit_maneuver = BayExit()
    legs: list[bool] = []
    for _ in range(ticks):
        exit_maneuver.command(_RANGES_M, _ANGLES_RAD, 0.0, tuning.speed.medium_mps(), tuning)
        legs.append(bool(exit_maneuver._leg_is_reverse))  # noqa: SLF001 - the flag under test
    return legs


def test_a_stalled_leg_still_reverses_within_the_time_bound() -> None:
    """A leg whose wheel never turns must still end, or the exit deadlocks.

    Without the bound the guarded exit holds ONE leg forever against a stalled
    wheel -- legs reached 15.25 s on hardware and 68% of all bay-exit ticks sat
    in legs older than 2 s, where the wheel is stalled 91-99.7% of the time.
    """
    tuning = _guard_tuning(BAY_EXIT_LEG_MAX_S=0.5)
    legs = _drive_stalled(200, tuning)
    assert any(legs) and not all(legs), "the ratchet never reversed against a stalled wheel"

    flips = sum(1 for a, b in zip(legs, legs[1:]) if a != b)
    bound_ticks = math.ceil(0.5 * tuning.control.CONTROL_HZ)
    # 200 ticks of 10-tick legs, each paying a computed servo settle, so the
    # count is bounded above by the bound alone and below by half of it.
    assert flips >= len(legs) // (2 * bound_ticks), f"only {flips} reversals in 200 stalled ticks"


def test_without_the_time_bound_a_stalled_leg_never_ends() -> None:
    """The defect itself, pinned so the bound cannot be quietly removed."""
    legs = _drive_stalled(200, _guard_tuning(BAY_EXIT_LEG_MAX_S=1000.0))
    flips = sum(1 for a, b in zip(legs, legs[1:]) if a != b)
    assert flips == 0, "the stalled-leg deadlock is gone; this test no longer pins anything"
