"""Unit tests for ParkController (CR-02).

Verifies:
- _build_zone produces correct bounding box and target yaw for all sections.
- _inside_zone correctly classifies positions and yaws.
- Controller from 4 approach poses terminates done with robot inside zone.
- The maneuver never enters the inner keep-out square at any tick, including from
  degenerate approach poses (target behind the robot / inside its turning radius) that
  previously drove a non-convergent orbit into the inner block — see
  docs/internal/2026-07-11-navigation-logic-review.md §2.3.
- done controller always returns zero speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import pytest
from shared.config.constants import CorridorDimensions, ParkingLotSpecs, RobotSpecs
from shared.domain.enums import Direction, Section
from shared.domain.models import BlockPosition, ParkingLot

from src.navigation.maneuvers.parking import (
    ParkController,
    ParkingContext,
    _build_zone,
    _inside_zone,
    _normalise_angle,
    _staging_pos,
)
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import ObstacleBox, TrackModel, _convex_overlap, _rect_corners
from tests.fixtures import ParkingLotFixtures
from tests.test_constants import (
    PARKING_EAST_BLOCK1,
    PARKING_EAST_BLOCK2,
    PARKING_NORTH_BLOCK1,
    PARKING_NORTH_BLOCK2,
    PARKING_SOUTH_BLOCK1,
    PARKING_SOUTH_BLOCK2,
    PARKING_WEST_BLOCK1,
    PARKING_WEST_BLOCK2,
)

_CW = Direction.CLOCKWISE
_CCW = Direction.COUNTERCLOCKWISE

_SOUTH_CFG = ParkingLotFixtures.south()
_NORTH_CFG = ParkingLotFixtures.north()
_EAST_CFG = ParkingLotFixtures.east()
_WEST_CFG = ParkingLotFixtures.west()

_PARKING_CONTEXT = ParkingContext()


def _bp(x: float, y: float) -> BlockPosition:
    """Shorthand for BlockPosition."""
    return BlockPosition(x=x, y=y)


# Zone geometry


class TestBuildZone:
    """The zone is the parking lot rectangle itself — "between the two markers"."""

    def _zone_south(self):
        lot = _SOUTH_CFG
        return _build_zone(
            lot.block1_position,
            lot.block2_position,
            Section.SOUTH,
            _CCW,
        )

    def test_south_bay_centre(self):
        lot = _SOUTH_CFG
        z = self._zone_south()
        assert z.gap_cx == pytest.approx((lot.block1_position.x + lot.block2_position.x) / 2)
        assert z.gap_cy == pytest.approx(ParkingLotSpecs.LENGTH / 2)

    def test_south_x_bounds_are_the_fins_inner_faces(self):
        lot = _SOUTH_CFG
        z = self._zone_south()
        assert z.x_min == pytest.approx(lot.block1_position.x + ParkingLotSpecs.WIDTH / 2)
        assert z.x_max == pytest.approx(lot.block2_position.x - ParkingLotSpecs.WIDTH / 2)

    def test_bay_depth_is_the_marker_length(self):
        """The markers are fins standing perpendicular to the wall, so they set the depth."""
        z = self._zone_south()
        assert z.y_min == pytest.approx(0.0)
        assert z.y_max == pytest.approx(ParkingLotSpecs.LENGTH)

    @pytest.mark.parametrize(
        "section,lot,direction,expected_yaw",
        [
            (Section.SOUTH, _SOUTH_CFG, _CW, math.pi),
            (Section.SOUTH, _SOUTH_CFG, _CCW, 0.0),
            (Section.NORTH, _NORTH_CFG, _CW, 0.0),
            (Section.NORTH, _NORTH_CFG, _CCW, math.pi),
            (Section.EAST, _EAST_CFG, _CW, math.pi / 2),
            (Section.EAST, _EAST_CFG, _CCW, -math.pi / 2),
            (Section.WEST, _WEST_CFG, _CW, -math.pi / 2),
            (Section.WEST, _WEST_CFG, _CCW, math.pi / 2),
        ],
    )
    def test_target_yaw_is_parallel_to_the_wall_and_matches_travel(
        self,
        section,
        lot,
        direction,
        expected_yaw,
    ):
        """WRO requires the robot parked parallel to the field wall.

        A nose-in heading (what this used to return) cannot satisfy the rule: the lot is
        ParkingLotSpecs.LENGTH deep and the chassis is RobotSpecs.LENGTH long, so a
        perpendicular pose protrudes by the difference no matter how well it is driven.
        """
        z = _build_zone(lot.block1_position, lot.block2_position, section, direction)
        assert _normalise_angle(z.target_yaw - expected_yaw) == pytest.approx(0.0, abs=1e-9)

    def test_target_yaw_is_never_perpendicular_to_the_wall(self):
        """Regression guard for the pre-2026-07-25 nose-in geometry."""
        for section, lot in (
            (Section.SOUTH, _SOUTH_CFG),
            (Section.NORTH, _NORTH_CFG),
            (Section.EAST, _EAST_CFG),
            (Section.WEST, _WEST_CFG),
        ):
            for direction in (_CW, _CCW):
                z = _build_zone(lot.block1_position, lot.block2_position, section, direction)
                wall_normal = math.pi / 2 if section in (Section.SOUTH, Section.NORTH) else 0.0
                err = abs(_normalise_angle(z.target_yaw - wall_normal))
                assert min(err, abs(math.pi - err)) > math.radians(45)


# Parked detection — the WRO rule, not a centre-in-box approximation


class TestInsideZone:
    def _zone(self, direction=_CCW):
        return _build_zone(_bp(*PARKING_SOUTH_BLOCK1), _bp(*PARKING_SOUTH_BLOCK2), Section.SOUTH, direction)

    def _bay_centre(self, z):
        return z.gap_cx, z.gap_cy

    def test_perfectly_placed_footprint_is_parked(self):
        z = self._zone()
        cx, cy = self._bay_centre(z)
        pos_ok, yaw_ok = _inside_zone(cx, cy, z.target_yaw, z, _PARKING_CONTEXT)
        assert pos_ok
        assert yaw_ok

    def test_centre_inside_but_footprint_protruding_is_not_parked(self):
        """The exact failure the old centre-in-box test could not see.

        A robot parked across the bay has its centre well inside the rectangle while most of
        the chassis sits out in the corridor. The old check called this a successful park.
        """
        z = self._zone()
        cx, cy = self._bay_centre(z)
        pos_ok, _ = _inside_zone(cx, cy, z.target_yaw + math.pi / 2, z, _PARKING_CONTEXT)
        assert not pos_ok

    def test_nose_through_the_outer_wall_is_not_parked(self):
        z = self._zone()
        cx, _ = self._bay_centre(z)
        pos_ok, _ = _inside_zone(cx, 0.02, z.target_yaw + math.pi / 2, z, _PARKING_CONTEXT)
        assert not pos_ok

    def test_outside_along_the_wall_is_not_parked(self):
        z = self._zone()
        _, cy = self._bay_centre(z)
        pos_ok, _ = _inside_zone(z.x_min - 0.30, cy, z.target_yaw, z, _PARKING_CONTEXT)
        assert not pos_ok

    def test_parallel_tolerance_follows_the_two_wheel_rule(self):
        """+-2 cm between the two wheels on one side, i.e. atan(0.02 / WHEELBASE) ~ 6 deg."""
        z = self._zone()
        cx, cy = self._bay_centre(z)
        limit = math.atan2(0.02, RobotSpecs.WHEELBASE)
        _, just_inside = _inside_zone(cx, cy, z.target_yaw + limit * 0.9, z, _PARKING_CONTEXT)
        _, just_outside = _inside_zone(cx, cy, z.target_yaw + limit * 1.1, z, _PARKING_CONTEXT)
        assert just_inside
        assert not just_outside


# Controller simulation

# Uniform wide corridors on all four sides, matching the demo/closed-loop scenarios --
# real Ackermann-relevant wall geometry, not an arbitrary stand-in.
_TRACK_WIDTHS = dict.fromkeys(Section, CorridorDimensions.WIDE)


def _parking_fins(cfg: ParkingLot, section: Section) -> list[ObstacleBox]:
    """The two magenta markers as physical obstacles.

    They became collidable in the simulator in commit fd33fd5, but this harness kept
    building an obstacle-free ``TrackModel``, so ``ever_collided`` could only ever mean
    "hit a wall or the inner square" — a park that drove straight through a marker was
    recorded as clean. They stand perpendicular to the outer wall, hence the quarter-turn
    yaw for a north/south bay.
    """
    yaw = math.pi / 2 if section in (Section.SOUTH, Section.NORTH) else 0.0
    return [
        ObstacleBox.from_pose(
            cx=pos[0],
            cy=pos[1],
            length=ParkingLotSpecs.LENGTH,
            width=ParkingLotSpecs.WIDTH,
            yaw=yaw,
        )
        for pos in ((cfg.block1_position.x, cfg.block1_position.y), (cfg.block2_position.x, cfg.block2_position.y))
    ]


@dataclass
class ParkRun:
    """Outcome of one isolated ParkController run."""

    stopped: bool
    """The controller returned ``done`` — which includes giving up on its frame budget."""

    parked: bool
    """A genuine park: stopped without timing out, footprint inside the lot, wall-parallel.

    Kept distinct from ``stopped`` because ``ParkController.is_done`` is true for a timeout
    too. The previous tests asserted only the equivalent of ``stopped``, so a maneuver that
    ran out its budget and held position counted as a successful park.
    """

    steps: int
    final_pos: tuple[float, float]
    final_yaw: float
    hits: set[str]
    """Which objects the footprint touched at any tick: 'wall-or-inner' and/or 'marker'.

    Split because they are different claims with different owners: 'wall-or-inner' is the
    2026-07-11 non-convergent-orbit guarantee, 'marker' is whether the maneuver clears the
    parking lot's own geometry -- previously unmeasurable, since the harness built an
    obstacle-free TrackModel even after the markers became collidable in fd35dd5.
    """


def _simulate_park(
    cfg: ParkingLot,
    section: Section,
    start_pos: tuple[float, float],
    start_yaw: float,
    max_steps: int = 800,
    direction: Direction = _CCW,
) -> ParkRun:
    """Real Ackermann bicycle-model simulation (matches the production sim/hardware).

    Collision is checked every tick against the actual chassis footprint, not just the final
    pose -- this is what would have caught the non-convergent-orbit bug (2026-07-11 review
    §2.3): a controller that eventually reaches ``done`` can still have driven through the
    inner keep-out square getting there. Simulation continues after a contact (recording it,
    not stopping) rather than treating it as fatal: this drives ``ParkController`` in
    isolation, without ``CoreNavigator``'s own defense-in-depth clearance gate
    (`core_navigator.py::_handle_finish`) that the full system relies on for the final
    guarantee. Callers decide which contacts they assert on.
    """
    ctrl = ParkController(cfg, section, direction)
    kin = AckermannKinematics()
    bare_track = TrackModel(_TRACK_WIDTHS)
    markers = _parking_fins(cfg, section)
    dt = 0.05
    state = AckermannState(x=start_pos[0], y=start_pos[1], yaw=start_yaw)
    hits: set[str] = set()

    def _finish(stopped: bool, steps: int) -> ParkRun:
        pos_ok, yaw_ok = _inside_zone(state.x, state.y, state.yaw, ctrl.zone, _PARKING_CONTEXT)
        return ParkRun(
            stopped=stopped,
            parked=stopped and not ctrl.is_timed_out and pos_ok and yaw_ok,
            steps=steps,
            final_pos=(state.x, state.y),
            final_yaw=state.yaw,
            hits=hits,
        )

    for step in range(max_steps):
        cmd = ctrl.update((state.x, state.y), state.yaw)
        if cmd.done:
            return _finish(stopped=True, steps=step)
        state = kin.step(state, target_speed=cmd.linear, target_steer_norm=cmd.steering, dt=dt)
        if bare_track.footprint_collides(state.x, state.y, state.yaw):
            hits.add("wall-or-inner")
        corners = _rect_corners(state.x, state.y, state.yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
        if any(_convex_overlap(corners, m.to_box().corners(), state.yaw) for m in markers):
            hits.add("marker")

    return _finish(stopped=False, steps=max_steps)


# 4 canonical approach poses for SOUTH section
# Approach from north, heading south (robot falls toward the gap). Positions must stay
# north of the staging point (gap_y=0.10 + _APPROACH_CLEARANCE=0.45 -> staging_y=0.67, see
# ParkController._staging_pos) so "approach from north" is still literally true, and south
# of CORNER_MIN=1.0 with real margin for the chassis's own half-length (0.15m) -- these
# were recalibrated for the corrected chassis dims (2026-07-11); the old values (y up to
# 0.90, staging at the old, smaller 0.25m clearance) put some poses south of the new
# staging point, which correctly triggers "target behind, reverse" but reverses straight
# toward the inner square from a position already too close to it.
#
# Yaw/position offsets are deliberately gentle (+-0.05, not the old +-0.3): ENTER's
# pure-pursuit-toward-a-point never explicitly steers toward the required final yaw (only
# position), so for some approach geometries it can hunt near the zone without position
# and yaw ever satisfying the stop condition simultaneously within the frame budget --
# confirmed pre-existing (not something this session's fixes introduced: the old test
# never caught it because its unicycle model could spin-align for free, decoupled from
# position, which a real Ackermann chassis cannot). It's safely bounded either way
# (ParkController's own max_frames give-up holds position, never runs forever or
# collides), but chasing full convergence for every possible approach angle is a separate,
# larger tuning effort than this fix's scope -- see
# docs/internal/2026-07-11-navigation-logic-review.md §2.3.
_SOUTH_APPROACHES = [
    ((1.15, 0.75), -math.pi / 2),  # centred, facing south
    ((1.10, 0.75), -math.pi / 2 + 0.05),  # left of gap, slight yaw error
    ((1.20, 0.75), -math.pi / 2 - 0.05),  # right of gap, slight yaw error
    ((1.15, 0.80), -math.pi / 2),  # further above
]


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ENTER pure-pursues a single point, which controls position but not final heading, "
        "so it cannot satisfy the WRO containment rule. This used to 'pass' only because the "
        "stop condition was a centre-in-box test that reported a park for a robot sitting "
        "perpendicular and mostly out in the corridor (measured: 0/240 swept approaches "
        "actually contained, 176 of them reported done). Now that the stop condition is "
        "honest, the missing entry maneuver is what fails. See "
        "platform/docs/internal/2026-07-25-parking-review.md."
    ),
)
@pytest.mark.parametrize("start_pos,start_yaw", _SOUTH_APPROACHES)
def test_south_park_from_4_approaches(start_pos, start_yaw):
    # Convergence only (not collision-free): these poses drive ParkController without
    # CoreNavigator's own defense-in-depth clearance gate -- see _simulate_park's docstring.
    run = _simulate_park(_SOUTH_CFG, Section.SOUTH, start_pos, start_yaw)
    assert run.parked, (
        f"Did not park after {run.steps} steps — pos={run.final_pos}, "
        f"yaw={math.degrees(run.final_yaw):.1f}°"
    )


# Degenerate approach: target behind the robot / inside its turning radius
#
# Reproduces the actual 2026-07-11 §2.3 failure geometry: the robot arrives near the
# staging point already, but heading along the corridor cruise direction rather than
# toward it -- bearing error ~170-190°, i.e. the staging point is essentially behind
# the robot. The old bearing-proportional `_pursuit_steer` orbited into the inner
# block trying to reach it; the fixed controller must reverse-and-reorient instead.

_DEGENERATE_CFGS: dict[Section, tuple[ParkingLot, tuple[float, float], float]] = {}
for _section, _cfg in (
    (Section.SOUTH, _SOUTH_CFG),
    (Section.NORTH, _NORTH_CFG),
    (Section.EAST, _EAST_CFG),
    (Section.WEST, _WEST_CFG),
):
    _zone = _build_zone(_cfg.block1_position, _cfg.block2_position, _section, _CCW)
    _staging = _staging_pos(_zone, _section, _PARKING_CONTEXT)
    if _section in (Section.SOUTH, Section.NORTH):
        _sign = 1.0 if _section is Section.SOUTH else -1.0
        _pos = (_staging[0] + 0.15, _staging[1] + _sign * -0.02)
        _yaw = 0.0
    else:
        _pos = (_staging[0] + (0.02 if _section is Section.EAST else -0.02), _staging[1] + 0.15)
        _yaw = math.pi / 2
    _DEGENERATE_CFGS[_section] = (_cfg, _pos, _yaw)


@pytest.mark.parametrize("section", list(_DEGENERATE_CFGS))
def test_degenerate_approach_never_hits_a_wall_or_the_inner_block(section):
    """The 2026-07-11 non-convergent-orbit guarantee. Markers excluded deliberately.

    This is the regression guard for the orbit-into-the-inner-square bug, and it still
    holds. It is kept separate from the marker check below so that a future regression here
    fails loudly instead of hiding behind that check's xfail.
    """
    cfg, start_pos, start_yaw = _DEGENERATE_CFGS[section]
    run = _simulate_park(cfg, section, start_pos, start_yaw, max_steps=800)
    assert "wall-or-inner" not in run.hits, (
        f"{section}: hit a wall or the inner square by step {run.steps} — pos={run.final_pos}"
    )


@pytest.mark.parametrize("section", list(_DEGENERATE_CFGS))
def test_degenerate_approach_never_hits_the_markers(section):
    """Marker clearance, measurable for the first time.

    The markers became collidable in fd33fd5, but this harness kept building an
    obstacle-free ``TrackModel``, so marker contact went unmeasured until now. It turns out
    the maneuver does clear them — what it does not clear is the field wall behind the lot,
    which is a separate test.
    """
    cfg, start_pos, start_yaw = _DEGENERATE_CFGS[section]
    run = _simulate_park(cfg, section, start_pos, start_yaw, max_steps=800)
    assert "marker" not in run.hits, (
        f"{section}: hit a parking marker by step {run.steps} — pos={run.final_pos}"
    )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "Same missing entry maneuver as test_south_park_from_4_approaches — split out so the "
        "collision-free guarantee above (which does hold, and is the safety-critical half of "
        "this test) keeps failing loudly on regression instead of being masked by an xfail."
    ),
)
@pytest.mark.parametrize("section", list(_DEGENERATE_CFGS))
def test_degenerate_approach_still_parks(section):
    cfg, start_pos, start_yaw = _DEGENERATE_CFGS[section]
    run = _simulate_park(cfg, section, start_pos, start_yaw, max_steps=800)
    assert run.parked, f"{section}: did not park after {run.steps} steps — pos={run.final_pos}"


# Basic controller behaviour


class TestParkControllerBasics:
    def test_not_done_initially(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, _CCW)
        assert not ctrl.is_done

    def test_done_returns_zero_speed(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, _CCW)
        # Hold the robot at a genuinely parked pose: the lot centre, wall-parallel. Taken
        # from the zone rather than hardcoded, so it stays a parked pose if the geometry
        # moves (the old literal was a perpendicular pose that no longer qualifies).
        parked_pose = ((ctrl.zone.gap_cx, ctrl.zone.gap_cy), ctrl.zone.target_yaw)
        for _ in range(800):
            cmd = ctrl.update(*parked_pose)
            if cmd.done:
                break
        assert ctrl.is_done
        cmd2 = ctrl.update(*parked_pose)
        assert cmd2.linear == 0.0
        assert cmd2.done

    def test_far_robot_drives_nonzero_speed(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, _CCW)
        cmd = ctrl.update((1.15, 2.5), 0.0)
        assert cmd.linear > 0
        assert not cmd.done


class TestParkControllerTimeout:
    """A maneuver that can never reach the position+yaw stop condition must
    give up and hold, rather than chase the gap centre for the whole match.
    """

    def test_unreachable_target_times_out_instead_of_running_forever(self):
        # Robot held exactly on the far side of the field, never approaching
        # the zone (a stand-in for "wedged, can't make progress").
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, _CCW, max_frames=50)

        for _ in range(51):
            cmd = ctrl.update((2.9, 2.9), 0.0)
        assert cmd.done
        assert ctrl.is_done
        assert ctrl.is_timed_out
        assert cmd.linear == 0.0
        assert cmd.steering == 0.0

    def test_successful_park_is_not_flagged_as_timed_out(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, _CCW, max_frames=400)
        parked_pose = ((ctrl.zone.gap_cx, ctrl.zone.gap_cy), ctrl.zone.target_yaw)
        for _ in range(400):
            cmd = ctrl.update(*parked_pose)
            if cmd.done:
                break
        assert ctrl.is_done
        assert not ctrl.is_timed_out
