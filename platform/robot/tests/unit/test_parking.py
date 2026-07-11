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

import pytest
from shared.config.constants import CorridorDimensions
from shared.config.enums import Section

from src.navigation.maneuvers.parking import (
    ParkController,
    _build_zone,
    _inside_zone,
    _normalise_angle,
    _staging_pos,
)
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import TrackModel
from tests.test_constants import (
    PARKING_EAST_BLOCK1,
    PARKING_EAST_BLOCK2,
    PARKING_NORTH_BLOCK1,
    PARKING_NORTH_BLOCK2,
    PARKING_SOUTH_BLOCK1,
    PARKING_SOUTH_BLOCK2,
    PARKING_WEST_BLOCK1,
    PARKING_WEST_BLOCK2,
    YAW_EAST,
    YAW_NORTH,
    YAW_SOUTH,
    YAW_WEST,
)

# Test configs

_SOUTH_CFG = {"block1_pos": PARKING_SOUTH_BLOCK1, "block2_pos": PARKING_SOUTH_BLOCK2}
_NORTH_CFG = {"block1_pos": PARKING_NORTH_BLOCK1, "block2_pos": PARKING_NORTH_BLOCK2}
_EAST_CFG = {"block1_pos": PARKING_EAST_BLOCK1, "block2_pos": PARKING_EAST_BLOCK2}
_WEST_CFG = {"block1_pos": PARKING_WEST_BLOCK1, "block2_pos": PARKING_WEST_BLOCK2}


# Zone geometry


class TestBuildZone:
    def test_south_gap_centre(self):
        z = _build_zone(PARKING_SOUTH_BLOCK1, PARKING_SOUTH_BLOCK2, Section.SOUTH)
        assert z.gap_cx == pytest.approx(1.15)
        assert z.gap_cy == pytest.approx(0.10)

    def test_south_x_bounds_inside_blocks(self):
        z = _build_zone(PARKING_SOUTH_BLOCK1, PARKING_SOUTH_BLOCK2, Section.SOUTH)
        assert z.x_min > PARKING_SOUTH_BLOCK1[0]
        assert z.x_max < PARKING_SOUTH_BLOCK2[0]
        assert z.x_min < z.x_max

    def test_south_target_yaw(self):
        z = _build_zone(PARKING_SOUTH_BLOCK1, PARKING_SOUTH_BLOCK2, Section.SOUTH)
        assert z.target_yaw == pytest.approx(YAW_SOUTH)

    def test_north_target_yaw(self):
        z = _build_zone(PARKING_NORTH_BLOCK1, PARKING_NORTH_BLOCK2, Section.NORTH)
        assert z.target_yaw == pytest.approx(YAW_NORTH)

    def test_east_target_yaw(self):
        z = _build_zone(PARKING_EAST_BLOCK1, PARKING_EAST_BLOCK2, Section.EAST)
        assert z.target_yaw == pytest.approx(YAW_EAST)

    def test_west_target_yaw(self):
        z = _build_zone(PARKING_WEST_BLOCK1, PARKING_WEST_BLOCK2, Section.WEST)
        assert abs(_normalise_angle(z.target_yaw)) == pytest.approx(YAW_WEST)


# Inside-zone detection


class TestInsideZone:
    def _zone(self):
        return _build_zone(PARKING_SOUTH_BLOCK1, PARKING_SOUTH_BLOCK2, Section.SOUTH)

    def test_inside_pos_and_yaw(self):
        z = self._zone()
        pos_ok, yaw_ok = _inside_zone(1.15, 0.08, YAW_SOUTH, z)
        assert pos_ok
        assert yaw_ok

    def test_outside_x(self):
        z = self._zone()
        pos_ok, _ = _inside_zone(0.80, 0.08, YAW_SOUTH, z)
        assert not pos_ok

    def test_bad_yaw(self):
        z = self._zone()
        _, yaw_ok = _inside_zone(1.15, 0.08, 0.0, z)
        assert not yaw_ok

    def test_yaw_within_10_degrees(self):
        z = self._zone()
        _, yaw_ok = _inside_zone(1.15, 0.08, -math.pi / 2 + math.radians(9), z)
        assert yaw_ok

    def test_yaw_outside_10_degrees(self):
        z = self._zone()
        _, yaw_ok = _inside_zone(1.15, 0.08, -math.pi / 2 + math.radians(11), z)
        assert not yaw_ok


# ── Controller simulation ─────────────────────────────────────────────────────

# Uniform wide corridors on all four sides, matching the demo/closed-loop scenarios --
# real Ackermann-relevant wall geometry, not an arbitrary stand-in.
_TRACK_WIDTHS = dict.fromkeys(Section, CorridorDimensions.WIDE)


def _simulate_park(
    cfg: dict,
    section: Section,
    start_pos: tuple[float, float],
    start_yaw: float,
    max_steps: int = 800,
) -> tuple[bool, int, tuple[float, float], float, bool]:
    """Real Ackermann bicycle-model simulation (matches the production sim/hardware).

    Returns (done, steps, final_pos, final_yaw, ever_collided). ``ever_collided`` is
    checked every tick against the actual chassis footprint, not just the final pose --
    this is what would have caught the non-convergent-orbit bug (2026-07-11 review §2.3):
    a controller that eventually reaches ``done`` can still have driven through the inner
    keep-out square getting there. Simulation continues even after a collision (recording
    it, not stopping) rather than treating it as fatal: this drives ``ParkController`` in
    isolation, without ``CoreNavigator``'s own defense-in-depth clearance gate
    (`core_navigator.py::_handle_finish`) that the full system relies on for the final
    guarantee -- ParkController alone threading the WRO-regulation gap (only ~4cm wider
    than the chassis per side) at the very end of ENTER can still clip on its own, which
    the gated system's tests (`test_obstacles_challenge_sim.py`) confirm doesn't happen
    end-to-end. Callers decide whether ``ever_collided`` is asserted on.
    """
    ctrl = ParkController(cfg, section)
    kin = AckermannKinematics()
    track = TrackModel(_TRACK_WIDTHS)
    dt = 0.05
    state = AckermannState(x=start_pos[0], y=start_pos[1], yaw=start_yaw)
    ever_collided = False

    for step in range(max_steps):
        cmd = ctrl.update((state.x, state.y), state.yaw)
        if cmd.done:
            return True, step, (state.x, state.y), state.yaw, ever_collided
        state = kin.step(state, target_speed=cmd.linear, target_steer_norm=cmd.steering, dt=dt)
        ever_collided = ever_collided or track.footprint_collides(state.x, state.y, state.yaw)

    return False, max_steps, (state.x, state.y), state.yaw, ever_collided


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


@pytest.mark.parametrize("start_pos,start_yaw", _SOUTH_APPROACHES)
def test_south_park_from_4_approaches(start_pos, start_yaw):
    # Convergence only (not collision-free): these poses drive ParkController without
    # CoreNavigator's own defense-in-depth clearance gate -- see _simulate_park's
    # docstring. TestDegenerateApproach below is the one that targets and confirms the
    # actual reported bug (non-convergent orbit into the inner square).
    done, steps, final_pos, final_yaw, _ever_collided = _simulate_park(
        _SOUTH_CFG,
        Section.SOUTH,
        start_pos,
        start_yaw,
    )
    assert done, f"Did not park after {steps} steps — pos={final_pos}, yaw={math.degrees(final_yaw):.1f}°"
    zone = _build_zone(
        _SOUTH_CFG["block1_pos"],
        _SOUTH_CFG["block2_pos"],
        Section.SOUTH,
    )
    pos_ok, yaw_ok = _inside_zone(final_pos[0], final_pos[1], final_yaw, zone)
    assert pos_ok, f"Final pos {final_pos} not inside zone"
    assert yaw_ok, f"Final yaw {math.degrees(final_yaw):.1f}° not within ±10° of target"


# ── Degenerate approach: target behind the robot / inside its turning radius ──────
#
# Reproduces the actual 2026-07-11 §2.3 failure geometry: the robot arrives near the
# staging point already, but heading along the corridor cruise direction rather than
# toward it -- bearing error ~170-190°, i.e. the staging point is essentially behind
# the robot. The old bearing-proportional `_pursuit_steer` orbited into the inner
# block trying to reach it; the fixed controller must reverse-and-reorient instead.

_DEGENERATE_CFGS: dict[Section, tuple[dict, tuple[float, float], float]] = {}
for _section, _cfg in (
    (Section.SOUTH, _SOUTH_CFG),
    (Section.NORTH, _NORTH_CFG),
    (Section.EAST, _EAST_CFG),
    (Section.WEST, _WEST_CFG),
):
    _zone = _build_zone(_cfg["block1_pos"], _cfg["block2_pos"], _section)
    _staging = _staging_pos(_zone, _section)
    if _section in (Section.SOUTH, Section.NORTH):
        _sign = 1.0 if _section is Section.SOUTH else -1.0
        _pos = (_staging[0] + 0.15, _staging[1] + _sign * -0.02)
        _yaw = 0.0
    else:
        _pos = (_staging[0] + (0.02 if _section is Section.EAST else -0.02), _staging[1] + 0.15)
        _yaw = math.pi / 2
    _DEGENERATE_CFGS[_section] = (_cfg, _pos, _yaw)


@pytest.mark.parametrize("section", list(_DEGENERATE_CFGS))
def test_degenerate_approach_never_collides_and_still_parks(section):
    cfg, start_pos, start_yaw = _DEGENERATE_CFGS[section]
    done, steps, final_pos, final_yaw, collided = _simulate_park(
        cfg, section, start_pos, start_yaw, max_steps=800,
    )
    assert not collided, (
        f"{section}: collided with the inner keep-out square at step {steps} — pos={final_pos}"
    )
    assert done, f"{section}: did not park after {steps} steps — pos={final_pos}"


# ── Basic controller behaviour ────────────────────────────────────────────────


class TestParkControllerBasics:
    def test_not_done_initially(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH)
        assert not ctrl.is_done

    def test_done_returns_zero_speed(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH)
        # Force done by placing robot perfectly inside zone
        for _ in range(800):
            cmd = ctrl.update((1.15, 0.08), -math.pi / 2)
            if cmd.done:
                break
        assert ctrl.is_done
        cmd2 = ctrl.update((1.15, 0.08), -math.pi / 2)
        assert cmd2.linear == 0.0
        assert cmd2.done

    def test_far_robot_drives_nonzero_speed(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH)
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
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, max_frames=50)

        for _ in range(51):
            cmd = ctrl.update((2.9, 2.9), 0.0)
        assert cmd.done
        assert ctrl.is_done
        assert ctrl.is_timed_out
        assert cmd.linear == 0.0
        assert cmd.steering == 0.0

    def test_successful_park_is_not_flagged_as_timed_out(self):
        ctrl = ParkController(_SOUTH_CFG, Section.SOUTH, max_frames=400)
        for _ in range(400):
            cmd = ctrl.update((1.15, 0.08), -math.pi / 2)
            if cmd.done:
                break
        assert ctrl.is_done
        assert not ctrl.is_timed_out
