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

Tried and reverted (2026-08-02): a dead zone on the turn-below's ``left >
right`` comparison, holding straight instead of committing to a side when the
two were within a few centimetres -- meant to filter the occasional noisy scan
that steers the wrong way for a tick before the direction estimator's own
(much stricter, 5-vote) test corrects it. Even sized to real LIDAR noise
(~3 cm), it regressed multiple blind Open Challenge fixtures into the 180 s
round limit or stuck oscillating near a corner: in a narrow (0.6 m) corridor
the asymmetry signal grows slowly approaching a turn, so any dead zone here
eats into the same margin the deadlock-avoidance back-off branch below
depends on, disproportionately to the noise it was filtering. The wrong-side
steer this was meant to fix is now largely absorbed by
:mod:`src.navigation.core_navigator`'s heading-aware ``replace_path`` reseek
instead (a bad blind-phase guess gets a correctly-sized correction once the
direction estimator settles, rather than an oversized one) -- do not
re-attempt a dead zone here without re-measuring against the full Open
Challenge sim battery, not just the fixture that motivated it.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, _wrap

# One load, reused by the module-level constants below. These feed free
# functions with no instance to inject tuning into, but that is not a reason to
# restate a configured number -- see corridor_follower.toml.
_TUNING = NavigationTuning.load_default()
_FOLLOWER = _TUNING.corridor_follower

# Both read from the group that owns them rather than restated here: the
# no-return floor is a LIDAR fact, and the in-track ceiling is the same 3 m-mat
# plausibility bound direction_estimator.MAX_IN_TRACK_RANGE_M documents at
# length. A second copy of either would be a second thing to keep in step.
_MIN_VALID_RANGE_M = _TUNING.lidar_sectors.MIN_VALID_RANGE_M
_MAX_IN_TRACK_RANGE_M = _TUNING.direction_estimator.MAX_IN_TRACK_RANGE_M

TURN_CLEARANCE_M = _FOLLOWER.TURN_CLEARANCE_M
"""Forward clearance at which to start turning the corner.

Strictly below :data:`~src.navigation.direction_estimator.CORNER_CLEARANCE_M`,
and the gap matters. Turning swings the heading past the direction estimator's
alignment gate, so beginning the turn as soon as the corner is detectable
rotates the robot straight through the only window in which it can read which
side is open. Hold the line for that window first, then turn.

``NavigationTuning`` enforces that ordering at load time, so the two can no
longer be edited into agreement by accident.
"""

if TYPE_CHECKING:
    from collections.abc import Sequence

_CENTERING_GAIN = _FOLLOWER.CENTERING_GAIN
_MAX_CENTERING_STEER = _FOLLOWER.MAX_CENTERING_STEER
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

_CORNER_SPEED_SCALE = _FOLLOWER.CORNER_SPEED_SCALE
"""Fraction of creep speed while turning a corner blind. Slower than straight
running, because the turn is committed on one comparison rather than a plan."""

_TURN_ARC_HALF_FOV_RAD = math.radians(_FOLLOWER.TURN_ARC_HALF_FOV_DEG)
_TURN_OPEN_RANGE_M = _FOLLOWER.TURN_OPEN_RANGE_M
"""Arc and range for the second opinion on whether the corridor has ended.

:data:`TURN_CLEARANCE_M` is applied to ``_forward_clearance``, which is the
*minimum* over a +/-8 deg cone. A minimum over a narrow cone answers "is
anything close ahead", which is not the same question as "has the corridor
ended", and the two come apart exactly when the chassis is oblique: 0.24 m off
a wall at 30 deg puts the whole cone on that wall at 0.24/sin(30) = 0.48 m,
below a 0.60 m threshold, in the middle of a perfectly open corridor.

That is not hypothetical. On run_20260806_162008 it held the corner branch --
and with it hard-over steering -- for 53% of a 305 s round, at 47% precision
against a 45% base rate: no better than chance. Hard-over steering swings the
heading past the direction estimator's alignment gate, so the round never
inferred its travel direction, never planned a path, and scored no laps at all.
That is the failure this module's own header warns about ("Oscillating costs
the round"), reached at a centring gain well below the one it was measured at.

So ask the complementary question, as a *maximum* over a *wider* arc: is there
any bearing ahead with real room left. At a real corner the end wall blocks
every bearing in the arc. An oblique chassis still has the corridor's own axis
inside it, reading metres. Scored against run_20260806_161659, a healthy
three-lap round, requiring both tests fires 12 times -- one episode per corner
per lap -- at 96% precision, where the clearance test alone fired 13 times at
88%. On the failing round it cuts the hard-over ticks by a sixth even before
the loop closes; the remainder is geometry the weave itself created.
"""

_MIN_FORWARD_CLEARANCE_M = RobotSpecs.LENGTH
"""Back off when the wall ahead is this close.

The direction should have settled long before this -- measured, it resolves
after about 0.8 m of travel with roughly 0.5 m to spare. Reaching here means it
did not, so driving on into the corner with no plan is not an option.

Stopping is not either, and used to be what happened. With no direction there is
no plan to hand over to and nothing else is steering, so a stopped robot stays
stopped: go_open_0020 sat at zero speed for 400 ticks with the wall 0.13 m away
and the round expired around it. The corner branch below carries a comment
warning of exactly that deadlock; this branch reintroduced it.
"""

_MIN_REVERSE_CLEARANCE_M = RobotSpecs.LENGTH
"""Room needed behind before backing off is allowed.

Backing blindly into whatever is behind trades one wall for another. With less
than this the robot is boxed at both ends and holding still is genuinely all
that is left.
"""

_REVERSE_SPEED_SCALE = _FOLLOWER.REVERSE_SPEED_SCALE
"""Fraction of creep speed to back off at.

Reverse is for realigning the nose over a few centimetres, not for travelling.
The robot must never cover ground backwards: the round is driven in the
direction drawn on the day, and a robot reversing down a corridor is going the
wrong way regardless of which way it is pointing. Clearance recovers within a
few ticks, at which point the forward branches take over again.
"""


def _way_through(ranges_m: Sequence[float], angles_rad: Sequence[float]) -> bool:
    """Is any bearing in the forward arc still open enough to drive down?

    The maximum, over an arc wide enough to contain the corridor's own axis
    when the chassis is oblique -- see :data:`_TURN_OPEN_RANGE_M` for why the
    forward *minimum* cannot answer this.

    Dropouts are excluded on the same reasoning, and against the same bound, as
    :mod:`src.navigation.direction_estimator` uses: the gateway substitutes max
    range (12 m) for a no-return, and nothing on a 3 m mat can be further than
    its diagonal. Left in, a single dropped beam would read as wide-open track
    and veto every corner turn on the round.
    """
    open_ranges = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(_wrap(a)) <= _TURN_ARC_HALF_FOV_RAD and _MIN_VALID_RANGE_M < r < _MAX_IN_TRACK_RANGE_M
    ]
    if not open_ranges:
        return False
    return max(open_ranges) >= _TURN_OPEN_RANGE_M


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

    # Something close ahead is a fact about safety, not about the layout, so it
    # is answered first and on the forward minimum -- whether or not the
    # corridor has ended, there is no room to drive on. Hoisted out of the
    # corner branch below, which no longer fires on every close wall and so can
    # no longer be relied on to reach this.
    if forward < _MIN_FORWARD_CLEARANCE_M:
        # Back off far enough to point somewhere useful, steering the mirror of
        # the turn: reversing swings the nose away from the steer direction, so
        # the inverted sign walks the nose toward the open side instead of
        # further into the wall it is against.
        steering = _MAX_CENTERING_STEER if left > right else -_MAX_CENTERING_STEER
        rear = _nearest_ray(ranges_m, angles_rad, math.pi)
        if rear > _MIN_REVERSE_CLEARANCE_M:
            return DriveCommand(
                speed_mps=-speed_mps * _REVERSE_SPEED_SCALE,
                steering_norm=-steering,
            )
        return DriveCommand(speed_mps=0.0, steering_norm=steering)

    if forward < TURN_CLEARANCE_M and not _way_through(ranges_m, angles_rad):
        # The corridor is ending -- close ahead AND nothing open across the
        # wider arc, so this is a wall spanning the track rather than one seen
        # at an angle. Turn toward the side with more room, which is where the
        # track continues, and is the same observation the direction estimator
        # settles on, so the turn and the answer agree.
        #
        # Stopping here instead is a deadlock: with no direction there is no
        # plan to hand over to, so the robot would sit at the corner until the
        # round expired. That was every closed-loop failure of this feature.
        steering = _MAX_CENTERING_STEER if left > right else -_MAX_CENTERING_STEER
        return DriveCommand(speed_mps=speed_mps * _CORNER_SPEED_SCALE, steering_norm=steering)

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
