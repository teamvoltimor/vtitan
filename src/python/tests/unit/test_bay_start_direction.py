"""Unit cover for the in-bay placement read that names the travel direction.

The rule itself is one comparison and has never been hard; what was hard is that
it only ever got ONE look, taken with a single ray per side, on the first scan of
the round. Both halves of that were measured to fail on the eighteen recorded
in-bay starts:

* two rounds had the wall side reading the substituted ``LIDAR_MAX_RANGE``
  because the C1 dropped the near return, which reads as "open corridor" and
  would have answered one of them BACKWARDS had the clearance gate not abstained
  first;
* one round had the whole forward arc substituted for the first four scans, so
  the single look was spent before the LIDAR was returning at all.

Sector median takes the rule 15/18 -> 17/18 and the bounded retry 17/18 -> 18/18,
both with zero wrong answers. See ``adr:0053-direction-inference-and-start-pose``.
"""

from __future__ import annotations

import math

from shared.config.constants import RobotSpecs
from shared.domain.enums import Direction

from src.config.tuning_helpers import get_tuning, tuning_with_overrides
from src.navigation.direction_estimator import direction_from_parking_bay

_MAX = RobotSpecs.LIDAR_MAX_RANGE


def _scan(left_m: float, right_m: float, *, fan_deg: int = 20, forward_m: float = 0.15):
    """A boxed-in placement: forward blocked, a wall one side, corridor the other."""
    angles: list[float] = []
    ranges: list[float] = []
    for deg in range(-180, 180, 2):
        rad = math.radians(deg)
        angles.append(rad)
        if abs(deg) <= 10:
            ranges.append(forward_m)
        elif abs(deg - 90) <= fan_deg:
            ranges.append(left_m)
        elif abs(deg + 90) <= fan_deg:
            ranges.append(right_m)
        else:
            ranges.append(1.5)
    return tuple(ranges), tuple(angles)


def _drop(ranges, angles, target_deg: float) -> tuple[float, ...]:
    """Substitute the max range for the one ray nearest ``target_deg``."""
    target = math.radians(target_deg)
    i = min(range(len(angles)), key=lambda k: abs(angles[k] - target))
    out = list(ranges)
    out[i] = _MAX
    return tuple(out)


def test_the_open_side_on_the_left_reads_counterclockwise() -> None:
    """Open side, inner side and turn side are the same side by track design."""
    ranges, angles = _scan(left_m=0.85, right_m=0.12)
    assert direction_from_parking_bay(ranges, angles) is Direction.COUNTERCLOCKWISE


def test_the_open_side_on_the_right_reads_clockwise() -> None:
    ranges, angles = _scan(left_m=0.12, right_m=0.85)
    assert direction_from_parking_bay(ranges, angles) is Direction.CLOCKWISE


def test_a_dropped_wall_ray_no_longer_decides_the_round() -> None:
    """The measured failure: one absent return on the wall side.

    Single-ray, the wall side reads 12 m and the placement looks like open
    corridor on BOTH sides, which the clearance gate can only answer by
    abstaining. The sector median ignores the substituted max and reports the
    wall that is still there on every other ray of the fan.
    """
    ranges, angles = _scan(left_m=0.85, right_m=0.12)
    ranges = _drop(ranges, angles, -90.0)
    assert direction_from_parking_bay(ranges, angles) is Direction.COUNTERCLOCKWISE


def test_a_zero_width_sector_restores_the_single_ray() -> None:
    """The knob's off position is the behaviour that shipped, not an approximation."""
    tuning = tuning_with_overrides({"bay_start_sector_deg": 0.0})
    ranges, angles = _scan(left_m=0.85, right_m=0.12)
    ranges = _drop(ranges, angles, -90.0)
    assert direction_from_parking_bay(ranges, angles, tuning) is None


def test_a_wholly_absent_fan_abstains_rather_than_guessing() -> None:
    """A LIDAR that has not started returning must not be read as a wide corridor.

    This is the case the retry covers: the answer here is ``None``, and the
    caller's job is to look again rather than to settle a direction off it.
    """
    ranges, angles = _scan(left_m=_MAX, right_m=_MAX, forward_m=_MAX)
    assert direction_from_parking_bay(ranges, angles) is None


def test_an_ordinary_centreline_start_is_still_not_a_bay() -> None:
    """Widening the read must not let a parallel start claim to be boxed in."""
    ranges, angles = _scan(left_m=0.5, right_m=0.5, forward_m=2.0)
    assert direction_from_parking_bay(ranges, angles) is None


def test_the_retry_budget_is_configured_and_bounded() -> None:
    """Bounded is the whole safety argument: unbounded is the mid-creep fault."""
    follower = get_tuning(None).corridor_follower
    assert follower.bay_start_max_checks >= 1
    # Half a second at the control rate. Long enough for the LIDAR to start
    # returning, short enough that a creep from standstill cannot reach a corner.
    assert follower.bay_start_max_checks <= 10
