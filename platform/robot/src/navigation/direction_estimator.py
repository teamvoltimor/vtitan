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

from shared.domain.enums import Direction

from src.config.tuning_helpers import get_tuning
from src.navigation.utils import _nearest_ray, axis_error_rad

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning


def infer_direction(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    yaw: float,
    tuning: NavigationTuning | None = None,
) -> Direction | None:
    """Which way round the loop this scan implies, or ``None`` if it cannot say.

    Args:
        ranges_m: LIDAR ranges.
        angles_rad: Matching robot-frame bearings (0 = forward).
        yaw: Current heading (radians, world frame). Only used to check the
            chassis is roughly aligned with a corridor; no map is consulted.
        tuning: Navigation tuning instance. Defaults to the default tuning profile.

    Returns:
        The inferred :class:`Direction`, or ``None`` while both sides still
        look like walls -- which is the normal state until the robot nears the
        end of its corridor.
    """
    tuning = get_tuning(tuning)

    alignment_tol = tuning.direction_estimator.ALIGNMENT_TOLERANCE_RAD
    max_in_track = tuning.direction_estimator.MAX_IN_TRACK_RANGE_M
    plausible_span = tuning.direction_estimator.PLAUSIBLE_SPAN_THRESHOLD_M
    min_asymmetry = tuning.direction_estimator.MIN_ASYMMETRY_M

    # Off-axis the side rays cut a diagonal and can read long for no good reason.
    if axis_error_rad(yaw) > alignment_tol:
        return None

    left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
    right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)

    # A dropout carries no information about whether a side is open, and the
    # span test below cannot tell one from a corridor running away: both read
    # long. Reject rather than guess.
    if left > max_in_track or right > max_in_track:
        return None

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
    if left + right <= plausible_span:
        return None
    if abs(left - right) < min_asymmetry:
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

    @property
    def votes(self) -> dict[Direction, int]:
        """Current vote tally per direction, for telemetry.

        A copy -- callers cannot perturb the real count through it.
        """
        return dict(self._votes)

    def observe(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        yaw: float,
        tuning: NavigationTuning | None = None,
    ) -> bool:
        """Fold one scan in; return True if this observation settled the direction."""
        if self._settled is not None:
            return False
        inferred = infer_direction(ranges_m, angles_rad, yaw, tuning)
        if inferred is None:
            return False

        self._votes[inferred] += 1
        if self._votes[inferred] < self._min_votes:
            return False
        self._settled = inferred
        return True
