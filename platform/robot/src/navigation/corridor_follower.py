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
from enum import StrEnum
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, RobotSpecs

from src.config.tuning_helpers import get_tuning
from src.navigation.corridor_estimator import classify_width
from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, axis_offset_rad, clamp, wrap_angle

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning


class TurnSide(StrEnum):
    """Which side to turn toward, overriding the clearance-based heuristic."""

    LEFT = "left"
    RIGHT = "right"

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

The robot must never cover ground backwards: the round is driven in the
direction drawn on the day, and a robot reversing down a corridor is going the
wrong way regardless of which way it is pointing. Clearance recovers within a
few ticks, at which point the forward branches take over again.
"""


def _way_through(ranges_m: Sequence[float], angles_rad: Sequence[float], tuning: NavigationTuning | None = None) -> bool:
    """Is any bearing in the forward arc still open enough to drive down?

    The maximum, over an arc wide enough to contain the corridor's own axis
    when the chassis is oblique -- uses tuning.corridor_follower.TURN_OPEN_RANGE_M
    and related fields.

    Dropouts are excluded on the same reasoning, and against the same bound, as
    :mod:`src.navigation.direction_estimator` uses: the gateway substitutes max
    range (12 m) for a no-return, and nothing on a 3 m mat can be further than
    its diagonal. Left in, a single dropped beam would read as wide-open track
    and veto every corner turn on the round.

    Uses tuning: corridor_follower.TURN_ARC_HALF_FOV_DEG, TURN_OPEN_RANGE_M;
        lidar_sectors.MIN_VALID_RANGE_M;
        direction_estimator.MAX_IN_TRACK_RANGE_M
    """
    tuning = get_tuning(tuning)
    follower = tuning.corridor_follower
    turn_arc_rad = math.radians(follower.TURN_ARC_HALF_FOV_DEG)
    min_valid = tuning.lidar_sectors.MIN_VALID_RANGE_M
    max_in_track = tuning.direction_estimator.MAX_IN_TRACK_RANGE_M
    open_ranges = [
        r
        for r, a in zip(ranges_m, angles_rad, strict=False)
        if abs(wrap_angle(a)) <= turn_arc_rad and min_valid < r < max_in_track
    ]
    if not open_ranges:
        return False
    return max(open_ranges) >= follower.TURN_OPEN_RANGE_M


def follow_corridor(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    speed_mps: float,
    yaw: float | None = None,
    tuning: NavigationTuning | None = None,
    forced_turn_side: TurnSide | None = None,
    believed_width_m: float | None = None,
) -> DriveCommand:
    """Creep along the corridor, centred between whatever walls are visible.

    Args:
        ranges_m: LIDAR ranges.
        angles_rad: Matching robot-frame bearings (0 = forward).
        speed_mps: Speed to creep at while the direction is unknown.
        yaw: Current heading (world frame), for the damping term on the
            centring branch. Optional because the value is only ever used
            against the nearest 90-degree axis -- the track is a Manhattan
            world, so this stays as map-free as the rest of the module and
            consults no plan. Omitted, centring falls back to
            offset-proportional, which oscillates; see :data:`_HEADING_GAIN`.
        tuning: Navigation tuning instance. Defaults to the default tuning profile.
        forced_turn_side: Override which side the "back off" and "corner"
            branches below turn toward. Both branches otherwise pick the side
            with more LIDAR clearance, which is correct for a plain wall but
            not for a red/green traffic sign -- those have a fixed WRO
            pass-side rule (red outward, green inward) that has nothing to do
            with which side looks more open. The caller resolves that rule
            (see ``planning.sign_router.outward_lateral_axis``) from a world-
            frame sign observation, something this function has no access to
            since it only ever sees robot-frame LIDAR. ``None`` preserves the
            plain clearance-based behaviour.
        believed_width_m: The corridor currently being creep-followed, from
            the caller's own running average of :func:`~src.navigation.corridor_estimator.measure_corridor_width`
            readings taken this same creep phase (no direction or section
            attribution needed -- see that module). When this classifies as
            NARROW, the corner turn commits at
            ``tuning.corridor_follower.NARROW_TURN_CLEARANCE_M`` instead of
            ``TURN_CLEARANCE_M`` -- see that field's docstring for why the
            wide-corridor threshold leaves no direction-settling window in a
            narrow one. ``None`` (no readings yet) keeps the plain
            ``TURN_CLEARANCE_M`` behaviour.

    Returns:
        A drive command centring the chassis, or a stop if the corridor ends
        before the direction resolved.
    """
    tuning = get_tuning(tuning)

    follower = tuning.corridor_follower
    max_centering = follower.MAX_CENTERING_STEER
    reverse_scale = follower.REVERSE_SPEED_SCALE
    corner_scale = follower.CORNER_SPEED_SCALE
    centering_gain = follower.CENTERING_GAIN
    heading_gain = follower.HEADING_GAIN
    turn_clearance = follower.TURN_CLEARANCE_M
    if believed_width_m is not None and classify_width(believed_width_m) == CorridorDimensions.NARROW:
        turn_clearance = follower.NARROW_TURN_CLEARANCE_M

    forward = _forward_clearance(ranges_m, angles_rad, tuning)
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
        turn_left = forced_turn_side == TurnSide.LEFT if forced_turn_side is not None else left > right
        steering = max_centering if turn_left else -max_centering
        rear = _nearest_ray(ranges_m, angles_rad, math.pi)
        if rear > _MIN_REVERSE_CLEARANCE_M:
            return DriveCommand(
                speed_mps=-speed_mps * reverse_scale,
                steering_norm=-steering,
            )
        return DriveCommand(speed_mps=0.0, steering_norm=steering)

    if forward < turn_clearance and not _way_through(ranges_m, angles_rad, tuning):
        # The corridor is ending -- close ahead AND nothing open across the
        # wider arc, so this is a wall spanning the track rather than one seen
        # at an angle. Turn toward the side with more room, which is where the
        # track continues, and is the same observation the direction estimator
        # settles on, so the turn and the answer agree.
        #
        # Stopping here instead is a deadlock: with no direction there is no
        # plan to hand over to, so the robot would sit at the corner until the
        # round expired. That was every closed-loop failure of this feature.
        turn_left = forced_turn_side == TurnSide.LEFT if forced_turn_side is not None else left > right
        steering = max_centering if turn_left else -max_centering
        return DriveCommand(speed_mps=speed_mps * corner_scale, steering_norm=steering)

    # Once a side has opened past the end of the inner block it is no longer a
    # corridor wall, and centring against it would steer into the other one.
    # Hold the line instead; the direction estimator is about to settle on the
    # very reading that disqualified it.
    limit = CorridorDimensions.WIDE + follower.CORNER_LEAK_MARGIN_M
    if left > limit or right > limit:
        return DriveCommand(speed_mps=speed_mps, steering_norm=0.0)

    # More room on the left means the chassis sits right of centre, so steer
    # left to correct -- and steering_norm is +1 = full left (see
    # ``shared.domain.steering``).
    offset = (left - right) / 2.0
    demand = centering_gain * offset
    # Damping. Offset alone is 90 degrees out of phase with the control the
    # chassis actually has -- steering sets yaw rate, yaw integrates to heading,
    # heading integrates to position -- so correcting position without regard to
    # heading always overshoots and comes back. Subtracting the heading error
    # takes the corner off that: pointing left of the corridor axis is a reason
    # to steer right even while still left of centre. Positive axis offset means
    # the nose is left of the axis, and +1 steering is full left, hence minus.
    if yaw is not None:
        demand -= heading_gain * axis_offset_rad(yaw)
    steering = clamp(demand, -max_centering, max_centering)
    return DriveCommand(speed_mps=speed_mps, steering_norm=steering)
