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

OPENING_RANGE_M = _WIDE + 0.35
"""A side ray beyond this cannot be the far wall of any legal corridor.

The widest corridor is 1.0 m and the LIDAR sits at the chassis centre, so a
side ray in a corridor returns at most about 1.0 m. Anything substantially past
that has missed the inner block and is looking down the next corridor. The
margin absorbs the robot sitting off-centre and scanning slightly off-axis.
"""

_ALIGNMENT_TOLERANCE_RAD = math.radians(25.0)
"""Beyond this off the corridor axis the side rays cut a diagonal and mean little."""

_MIN_VALID_RANGE_M = 0.01
"""Below this a return is the driver's invalid-reading sentinel, not a wall."""

CORNER_CLEARANCE_M = 0.75
"""Forward clearance below which the corridor is treated as ending.

Shared with :mod:`src.navigation.corridor_follower`, which starts turning at
the same point -- the corner rule below and the turn have to agree about when
a corner has arrived, or the robot commits to a turn the estimator has not
justified.
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

    left_open = left > OPENING_RANGE_M
    right_open = right > OPENING_RANGE_M
    if left_open != right_open and abs(left - right) >= _MIN_ASYMMETRY_M:
        # The inner block is on the side that opened, and the block's side is
        # the rotational sense: block on the right means going clockwise.
        return Direction.CLOCKWISE if right_open else Direction.COUNTERCLOCKWISE

    # Corner rule. The absolute test above wants a side to see clear down the
    # next corridor, which does not always happen before the wall ahead
    # arrives: from far enough off-centre the block-side ray can clear the
    # block and still land on the next corridor's far wall inside the
    # threshold. Once the corridor is visibly ending, the *comparison* is still
    # decisive even when neither side passes the absolute bar -- one ray is
    # looking along a corridor and the other at a wall a corridor-width away.
    #
    # Without this the robot reaches the corner undecided, and having no plan
    # and no corner behaviour it simply stops: measured, that deadlock was
    # every one of the 7 closed-loop failures, none of them a wrong answer.
    if _forward_clearance(ranges_m, angles_rad) < CORNER_CLEARANCE_M and abs(left - right) >= _MIN_ASYMMETRY_M:
        return Direction.CLOCKWISE if right > left else Direction.COUNTERCLOCKWISE
    return None


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
