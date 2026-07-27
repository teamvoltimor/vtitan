"""Drive along a corridor without a map, a plan, or a travel direction.

There is a gap at the start of a blind round: the robot cannot follow a planned
path until it knows which way round the loop it is going
(:mod:`src.navigation.direction_estimator`), and it cannot learn that without
driving. Something has to move the robot in between.

Following the corridor needs none of the missing information. Both walls are
visible, so steering to sit between them is purely reactive -- no position, no
layout, no direction. It is deliberately not a fallback for a lost robot: it
holds the corridor for the metre or so before the direction resolves, and then
hands over to the real path.

Starting on the assumed direction instead does not work. The path for the wrong
direction runs the opposite way down the same corridor, so its lookahead point
is *behind* the robot and pure pursuit turns it around inside the corridor.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, RobotSpecs

from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, _wrap

TURN_CLEARANCE_M = 0.60
"""Forward clearance at which to start turning the corner.

Strictly below :data:`~src.navigation.direction_estimator.CORNER_CLEARANCE_M`,
and the gap matters. Turning swings the heading past the direction estimator's
alignment gate, so beginning the turn as soon as the corner is detectable
rotates the robot straight through the only window in which it can read which
side is open. Hold the line for that window first, then turn.
"""

if TYPE_CHECKING:
    from collections.abc import Sequence

_CENTERING_GAIN = 0.8
_MAX_CENTERING_STEER = 0.25
"""Steering per metre of lateral offset, and a hard cap on the result.

Both are deliberately timid. The chassis is counter-phase four-wheel steering
with a 0.034 m minimum turn radius, so it responds violently -- the same reason
``STEER_KP`` came down to 1.2. A hot gain here does not merely wander: the
resulting oscillation swings the heading past the alignment gate in
:mod:`src.navigation.direction_estimator`, which then refuses every reading and
the direction never settles at all. Measured at gain 2.0, that cost 12 of 28
fixtures their direction and put 9 into a wall.

Sitting off-centre for one metre costs nothing. Oscillating costs the round.
"""

_CORNER_SPEED_SCALE = 0.6
"""Fraction of creep speed while turning a corner blind. Slower than straight
running, because the turn is committed on one comparison rather than a plan."""

_MIN_FORWARD_CLEARANCE_M = RobotSpecs.LENGTH
"""Stop creeping when the wall ahead is this close.

The direction should have settled long before this -- measured, it resolves
after about 0.8 m of travel with roughly 0.5 m to spare. Reaching here means it
did not, so stopping is better than driving into the corner with no plan.
"""


def follow_corridor(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    speed_mps: float,
) -> DriveCommand:
    """Creep along the corridor, centred between whatever walls are visible.

    Args:
        ranges_m: LIDAR ranges.
        angles_rad: Matching robot-frame bearings (0 = forward).
        speed_mps: Speed to creep at while the direction is unknown.

    Returns:
        A drive command centring the chassis, or a stop if the corridor ends
        before the direction resolved.
    """
    forward = _forward_clearance(ranges_m, angles_rad)
    left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
    right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)

    if forward < TURN_CLEARANCE_M:
        # The corridor is ending. Turn toward the side with more room, which is
        # where the track continues -- and is the same observation the
        # direction estimator settles on, so the turn and the answer agree.
        #
        # Stopping here instead is a deadlock: with no direction there is no
        # plan to hand over to, so the robot would sit at the corner until the
        # round expired. That was every closed-loop failure of this feature.
        steering = _MAX_CENTERING_STEER if left > right else -_MAX_CENTERING_STEER
        crawl = 0.0 if forward < _MIN_FORWARD_CLEARANCE_M else speed_mps * _CORNER_SPEED_SCALE
        return DriveCommand(speed_mps=crawl, steering_norm=steering)

    # Once a side has opened past the end of the inner block it is no longer a
    # corridor wall, and centring against it would steer into the other one.
    # Hold the line instead; the direction estimator is about to settle on the
    # very reading that disqualified it.
    limit = CorridorDimensions.WIDE + 0.35
    if left > limit or right > limit:
        return DriveCommand(speed_mps=speed_mps, steering_norm=0.0)

    # More room on the left means the chassis sits right of centre, so steer
    # left to correct -- and steering_norm is +1 = full left (see
    # ``shared.domain.steering``).
    offset = (left - right) / 2.0
    steering = max(-_MAX_CENTERING_STEER, min(_MAX_CENTERING_STEER, _CENTERING_GAIN * offset))
    return DriveCommand(speed_mps=speed_mps, steering_norm=steering)
