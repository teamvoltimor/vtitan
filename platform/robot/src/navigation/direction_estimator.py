"""Infer which way round the loop the robot is travelling, from LIDAR alone.

Travel direction is the last thing the robot had to be told. Corridor widths it
learns (:mod:`src.navigation.corridor_estimator`) and the starting section it
can simply assume, because assuming it only rotates the robot's private world
frame (:mod:`src.navigation.start_conditions`). Direction is different: it is a
reflection, and getting it wrong puts the inner block on the wrong side of every
plan.

## The signal

Every corridor has the outer wall on one side and the inner block on the other.
The block is finite and the outer wall is not, so driving toward the end of a
corridor the block *ends* while the wall continues -- and the sideways ray on
that side stops coming back at corridor width and instead runs off down the next
corridor.

That side is where the track turns, which fixes the rotational sense:

* opening on the **right** -> the block is on the right -> **clockwise**
* opening on the **left**  -> the block is on the left  -> **counterclockwise**

This is the same corner leakage that :mod:`corridor_estimator` filters out as
nonsense, read the other way round. There it is noise polluting a width; here it
is the whole measurement.

## Why it is easy

The discrimination is between "a wall at 0.6-1.0 m" and "no wall for at least
the next corridor", so the gap is metres against ~3 cm of LIDAR noise. It does
not need the robot to be centred, or to know where it is, or even to know the
corridor width -- only that one side stopped returning a wall and the other did
not.

The robot cannot follow a planned path before this resolves, since the path
depends on the answer. It only has to drive *along* its corridor, which is a
purely reactive behaviour needing no map.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction

if TYPE_CHECKING:
    from collections.abc import Sequence

_WIDE = CorridorDimensions.WIDE

_MAX_PLAUSIBLE_SPAN_M = _WIDE + 0.25
"""Beyond this, left + right is no longer two walls of one corridor.

The widest legal corridor is 1.0 m and the LIDAR sits at the chassis centre, so
the two side rays sum to the corridor width wherever the robot sits across it.
A sum past this has to mean one ray missed the inner block and ran off down the
next corridor. The margin absorbs scanning slightly off-axis; it is the same
plausibility bound :mod:`src.navigation.corridor_estimator` uses to reject the
readings this module is looking for.
"""

_ALIGNMENT_TOLERANCE_RAD = math.radians(25.0)
"""Beyond this off the corridor axis the side rays cut a diagonal and mean little."""

_MIN_VALID_RANGE_M = 0.01
"""Below this a return is the driver's invalid-reading sentinel, not a wall."""

CORNER_CLEARANCE_M = 1.00
"""Forward clearance below which the corridor counts as ending, for inference.

Deliberately *larger* than the clearance at which
:mod:`src.navigation.corridor_follower` starts turning. The two must not
coincide. Turning swings the heading past the alignment gate below, which then
refuses every reading -- so a robot that begins its turn at the same instant
the comparison becomes decisive rotates straight through its only measurement
window and comes out the far side with a wall on both sides again and nothing
learned. Measured with both at 0.75 m: three fixtures never settled at all and
two settled wrong after 20-plus seconds of wandering.

The gap between this and the turn threshold is the window in which the robot is
still square to the corridor and the way ahead is visibly closing.
"""

_FORWARD_ARC_RAD_FWD = math.radians(8.0)
"""Narrow, so the forward check sees ahead rather than the near side wall.

In a 0.6 m corridor a ray 25 degrees off the nose already returns the side
wall, which reads as an obstacle in front when the way ahead is clear.
"""

_MIN_ASYMMETRY_M = 0.30
"""How much further the open side must see than the closed one.

Guards the case where both sides read long -- at the very corner the robot can
briefly see past the block on one side and down the finishing corridor on the
other, and a marginal difference there is not evidence.
"""


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _nearest_ray(ranges_m: Sequence[float], angles_rad: Sequence[float], target: float) -> float:
    index = min(range(len(angles_rad)), key=lambda i: abs(_wrap(angles_rad[i] - target)))
    return ranges_m[index]


def _forward_clearance(ranges_m: Sequence[float], angles_rad: Sequence[float]) -> float:
    forward = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(_wrap(a)) <= _FORWARD_ARC_RAD_FWD and r > _MIN_VALID_RANGE_M
    ]
    return min(forward) if forward else math.inf


def infer_direction(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    yaw: float,
) -> Direction | None:
    """Which way round the loop this scan implies, or ``None`` if it cannot say.

    Args:
        ranges_m: LIDAR ranges.
        angles_rad: Matching robot-frame bearings (0 = forward).
        yaw: Current heading (radians, world frame). Only used to check the
            chassis is roughly aligned with a corridor; no map is consulted.

    Returns:
        The inferred :class:`Direction`, or ``None`` while both sides still
        look like walls -- which is the normal state until the robot nears the
        end of its corridor.
    """
    # Off-axis the side rays cut a diagonal and can read long for no good reason.
    axis_error = _wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2))
    if abs(axis_error) > _ALIGNMENT_TOLERANCE_RAD:
        return None

    left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
    right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)

    # Decide on the SPAN, not on either range alone. Two walls span the
    # corridor wherever the chassis sits between them, so left + right stays at
    # the corridor width until one side stops being a wall -- at which point it
    # jumps by a corridor length. That makes the test immune to being
    # off-centre, which is the whole difficulty:
    #
    # Comparing the two ranges directly does not work. Drifted toward the inner
    # block, a robot reads 0.27 m to the block on its left and 0.72 m to the
    # outer wall on its right, and "the larger side is open" then picks the
    # outer wall and returns exactly the wrong answer. That is which wall is
    # *nearer*, not which side is *open*, and it cost two fixtures a confident
    # wrong direction inside six seconds.
    if left + right <= _MAX_PLAUSIBLE_SPAN_M:
        return None
    if abs(left - right) < _MIN_ASYMMETRY_M:
        return None

    # The inner block is on the side that opened, and the block's side fixes
    # the rotational sense: block on the right means going clockwise.
    return Direction.CLOCKWISE if right > left else Direction.COUNTERCLOCKWISE


class DirectionEstimator:
    """Running direction estimate, settled by agreeing observations.

    Votes rather than trusting a single scan, for the same reason
    :class:`~src.navigation.corridor_estimator.CorridorWidthEstimator` does: a
    ray slipping past a block corner produces brief, clustered misreadings, and
    one of those arriving first should not decide the round.
    """

    def __init__(self, min_votes: int = 5) -> None:
        self._min_votes = min_votes
        self._votes: dict[Direction, int] = dict.fromkeys(Direction, 0)
        self._settled: Direction | None = None

    @property
    def direction(self) -> Direction | None:
        """The settled direction, or ``None`` until enough evidence agrees."""
        return self._settled

    @property
    def is_settled(self) -> bool:
        """True once the direction has been decided."""
        return self._settled is not None

    def observe(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        yaw: float,
    ) -> bool:
        """Fold one scan in; return True if this observation settled the direction."""
        if self._settled is not None:
            return False
        inferred = infer_direction(ranges_m, angles_rad, yaw)
        if inferred is None:
            return False

        self._votes[inferred] += 1
        if self._votes[inferred] < self._min_votes:
            return False
        self._settled = inferred
        return True
