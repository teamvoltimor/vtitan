"""Unit tests for the WRO parking scorer (``score_park``).

The repo scored parking as a boolean -- "whole footprint inside, wall-parallel" --
which is only rule 1.8.2, worth 15. Rule 1.8.3 pays 7 for "parking partly or not
parallel in the parking area", and nothing here could express it, so every sweep
reported 0 for poses a judge would pay for.

The cases below pin the three outcomes and the veto:

* the chassis CANNOT reach full credit by geometry -- 0.194 m in a 0.20 m bay is a
  3 mm window -- but it can reach partial credit with 0.118 m of slack per side,
  which is the entire reason this scorer exists;
* contact voids BOTH tiers ("the robot is stopped and no points for the parking
  can be scored"), so it is checked first and short-circuits.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import ParkingLotSpecs, RobotSpecs
from shared.domain.enums import Direction, Section
from shared.domain.models import BlockPosition

from src.navigation.maneuvers.parking.scoring import (
    FULL_PARK_POINTS,
    PARTIAL_PARK_POINTS,
    score_park,
)
from src.navigation.maneuvers.parking.zone import build_zone

_LOT_CENTRE_X = 1.5
_WALL_Y = 0.0
"""SOUTH section: the outer wall is y = 0 and the lot runs along x."""


@pytest.fixture()
def zone():
    """A SOUTH lot: fins ``BLOCK_SPACING_FACTOR x LENGTH`` apart, standing off the wall."""
    half_span = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH / 2
    return build_zone(
        BlockPosition(x=_LOT_CENTRE_X - half_span, y=ParkingLotSpecs.WALL_OFFSET),
        BlockPosition(x=_LOT_CENTRE_X + half_span, y=ParkingLotSpecs.WALL_OFFSET),
        Section.SOUTH,
        Direction.COUNTERCLOCKWISE,
    )


def _along_centre(zone) -> float:
    lo, hi = zone.bounds_along()
    return (lo + hi) / 2


def test_parallel_and_contained_scores_full(zone):
    """Mid-depth, wall-parallel: the 15-point park, if the chassis can ever get there."""
    depth_lo, depth_hi = zone.bounds_depth()
    score = score_park(_along_centre(zone), (depth_lo + depth_hi) / 2, zone.target_yaw, zone)
    assert score.points == FULL_PARK_POINTS
    assert score.contained
    assert score.parallel
    assert not score.touched


def test_perpendicular_nose_in_scores_partial(zone):
    """The strategy the 0/240 sweep could not see.

    Nose-in leaves 0.10 m of a 0.30 m chassis outside a 0.20 m bay, so it can never
    be CONTAINED -- but it overlaps, touches nothing, and is exactly the "partly or
    not parallel" the rule pays 7 for.
    """
    depth_lo, _ = zone.bounds_depth()
    standoff = 0.02
    nose_depth = depth_lo + standoff
    score = score_park(
        _along_centre(zone), nose_depth + RobotSpecs.LENGTH / 2, math.pi / 2, zone
    )
    assert score.points == PARTIAL_PARK_POINTS
    assert score.overlaps
    assert not score.contained
    assert not score.touched


def test_perpendicular_entry_has_room_to_spare(zone):
    """The lateral tolerance that makes nose-in worth simulating at all.

    A 0.194 m chassis across a 0.430 m gap clears each fin by 0.118 m -- twenty
    times the 6 mm the parallel park has. Scored across the whole band, not
    asserted from arithmetic.
    """
    along_lo, along_hi = zone.bounds_along()
    depth_lo, _ = zone.bounds_depth()
    centre_depth = depth_lo + 0.02 + RobotSpecs.LENGTH / 2
    slack = (along_hi - along_lo - RobotSpecs.WIDTH) / 2
    for offset in (-slack + 0.005, 0.0, slack - 0.005):
        score = score_park(_along_centre(zone) + offset, centre_depth, math.pi / 2, zone)
        assert score.points == PARTIAL_PARK_POINTS, f"offset {offset:.3f} should still score"
        assert not score.touched


def test_touching_a_fin_voids_all_points(zone):
    """Contact is a veto, not a deduction -- and it outranks a perfect pose."""
    along_lo, along_hi = zone.bounds_along()
    depth_lo, _ = zone.bounds_depth()
    beyond_fin = along_hi + RobotSpecs.WIDTH / 2 - 0.01
    score = score_park(
        beyond_fin, depth_lo + 0.02 + RobotSpecs.LENGTH / 2, math.pi / 2, zone
    )
    assert score.touched
    assert score.points == 0


def test_out_in_the_corridor_scores_nothing(zone):
    """No overlap, no contact, no points -- the ordinary miss."""
    _, depth_hi = zone.bounds_depth()
    score = score_park(_along_centre(zone), depth_hi + RobotSpecs.LENGTH, zone.target_yaw, zone)
    assert score.points == 0
    assert not score.overlaps
    assert not score.touched


def test_parallel_test_accepts_either_heading(zone):
    """A judge measures wheel-to-wall distances, which do not know which way it faces.

    ``zone.target_yaw`` names only the heading matching the direction of travel, so
    scoring against it alone would fail a robot parked perfectly but facing the
    other way.
    """
    depth_lo, depth_hi = zone.bounds_depth()
    reversed_yaw = zone.target_yaw + math.pi
    score = score_park(_along_centre(zone), (depth_lo + depth_hi) / 2, reversed_yaw, zone)
    assert score.parallel
    assert score.points == FULL_PARK_POINTS
