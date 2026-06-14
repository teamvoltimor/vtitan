"""Unit tests for ParkController (CR-02).

Verifies:
- _build_zone produces correct bounding box and target yaw for all sections.
- _inside_zone correctly classifies positions and yaws.
- Controller from 4 approach poses terminates done with robot inside zone.
- done controller always returns zero speed.
"""

from __future__ import annotations

import math

import pytest
from shared.config.enums import Section

from src.navigation.maneuvers.parking import (
    ParkController,
    _build_zone,
    _inside_zone,
    _normalise_angle,
)

# ── Test configs ──────────────────────────────────────────────────────────────

_SOUTH_CFG = {"block1_pos": (1.00, 0.10), "block2_pos": (1.30, 0.10)}
_NORTH_CFG = {"block1_pos": (1.00, 2.90), "block2_pos": (1.30, 2.90)}
_EAST_CFG = {"block1_pos": (2.90, 1.00), "block2_pos": (2.90, 1.30)}
_WEST_CFG = {"block1_pos": (0.10, 1.00), "block2_pos": (0.10, 1.30)}


# ── Zone geometry ─────────────────────────────────────────────────────────────


class TestBuildZone:
    def test_south_gap_centre(self):
        z = _build_zone((1.00, 0.10), (1.30, 0.10), Section.SOUTH)
        assert z.gap_cx == pytest.approx(1.15)
        assert z.gap_cy == pytest.approx(0.10)

    def test_south_x_bounds_inside_blocks(self):
        z = _build_zone((1.00, 0.10), (1.30, 0.10), Section.SOUTH)
        assert z.x_min > 1.00
        assert z.x_max < 1.30
        assert z.x_min < z.x_max

    def test_south_target_yaw(self):
        z = _build_zone((1.00, 0.10), (1.30, 0.10), Section.SOUTH)
        assert z.target_yaw == pytest.approx(-math.pi / 2)

    def test_north_target_yaw(self):
        z = _build_zone((1.00, 2.90), (1.30, 2.90), Section.NORTH)
        assert z.target_yaw == pytest.approx(math.pi / 2)

    def test_east_target_yaw(self):
        z = _build_zone((2.90, 1.00), (2.90, 1.30), Section.EAST)
        assert z.target_yaw == pytest.approx(0.0)

    def test_west_target_yaw(self):
        z = _build_zone((0.10, 1.00), (0.10, 1.30), Section.WEST)
        assert abs(_normalise_angle(z.target_yaw)) == pytest.approx(math.pi)


# ── Inside-zone detection ─────────────────────────────────────────────────────


class TestInsideZone:
    def _zone(self):
        return _build_zone((1.00, 0.10), (1.30, 0.10), Section.SOUTH)

    def test_inside_pos_and_yaw(self):
        z = self._zone()
        pos_ok, yaw_ok = _inside_zone(1.15, 0.08, -math.pi / 2, z)
        assert pos_ok
        assert yaw_ok

    def test_outside_x(self):
        z = self._zone()
        pos_ok, _ = _inside_zone(0.80, 0.08, -math.pi / 2, z)
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


def _simulate_park(
    cfg: dict,
    section: Section,
    start_pos: tuple[float, float],
    start_yaw: float,
    max_steps: int = 800,
) -> tuple[bool, int, tuple[float, float], float]:
    """Unicycle kinematic simulation. Returns (done, steps, final_pos, final_yaw)."""
    ctrl = ParkController(cfg, section, speed=0.15, steer_kp=3.0)
    dt = 0.05
    x, y = start_pos
    yaw = start_yaw

    for step in range(max_steps):
        cmd = ctrl.update((x, y), yaw)
        if cmd.done:
            return True, step, (x, y), yaw
        v = cmd.linear
        omega = cmd.steering * 2.0  # steer → rad/s
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = _normalise_angle(yaw + omega * dt)
        x = max(0.01, min(2.99, x))
        y = max(0.01, min(2.99, y))

    return False, max_steps, (x, y), yaw


# 4 canonical approach poses for SOUTH section
# Approach from north, heading south (robot falls toward the gap).
_SOUTH_APPROACHES = [
    ((1.15, 0.60), -math.pi / 2),  # centred, facing south
    ((0.95, 0.65), -math.pi / 2 + 0.3),  # left of gap, slight yaw error
    ((1.35, 0.65), -math.pi / 2 - 0.3),  # right of gap, slight yaw error
    ((1.15, 0.90), -math.pi / 2),  # further above
]


@pytest.mark.parametrize("start_pos,start_yaw", _SOUTH_APPROACHES)
def test_south_park_from_4_approaches(start_pos, start_yaw):
    done, steps, final_pos, final_yaw = _simulate_park(
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
