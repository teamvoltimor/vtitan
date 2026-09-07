"""Sign-avoidance deformation math for the WRO 2026 obstacles challenge.

Pure functions that turn a sign, its corridor, and a routing context into a
laterally-deformed waypoint: the pass-side offset, the depth pin that holds the
commanded point abeam the sign, and the camera-color match. No router state.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from shared.domain.enums import Axis, Direction, Section
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_router.config import (
    _DEFAULT_SIGN_ROUTER_CONTEXT,
    SignRouterConfig,
    SignRouterContext,
)
from src.navigation.planning.sign_router.routing import (
    ROUTING_TABLE,
    clamp_lateral,
    is_squarely_in_corridor,
)
from src.navigation.utils import _dist2d

if TYPE_CHECKING:
    from shared.domain.models import TrafficSignObservation

    from src.navigation.planning.sign_discovery import SignSpec


def apply_deformation(
    waypoint: tuple[float, float],
    sign: SignSpec,
    color: str,
    corridor: Section,
    direction: Direction,
    lateral_offset: float,
    robot_pos: tuple[float, float] | None = None,
    context: SignRouterContext | None = None,
    yaw_drift: float | None = None,
) -> tuple[float, float]:
    """Compute the laterally deformed waypoint for a given sign and corridor.

    The result is clamped so it can't land inside the restricted inner square or
    beyond the outer wall (WP-1) -- a sign positioned near a corridor edge would
    otherwise deform the waypoint straight into a hazard.

    Only the LATERAL coordinate carries the avoidance; the depth coordinate is
    whatever the lookahead search picked, which sits 0.2-0.4 m further along the
    corridor every tick. Passing that through unchanged is what makes the offset
    arrive late: the commanded point holds a constant lateral value but keeps
    receding, so the slope the chassis must follow to reach it flattens tick by
    tick and the lateral error is only ever asymptotically closed -- traced on
    go_obstacles_0000, the chassis needed 0.324 m of lateral travel over the 0.42
    m of runway left and achieved 0.163 m of it, arriving level with the pillar
    still half a chassis width inside the line it was given. Ramping to full offset
    sooner does NOT fix that lag -- measured and rejected, see the taper comment in
    ``deform_waypoint`` -- because the target the offset is attached to is the
    thing running away.

    So while the sign lies between the robot and the lookahead point, pin the depth
    coordinate to the SIGN's own depth. The commanded point stops receding and
    becomes a fixed gate abeam the pillar, which the chassis has to be on by the
    time it gets there. ``robot_pos`` is optional so callers testing the pure
    pass-side mapping can keep asking for it alone.

    Args:
        waypoint: Original target waypoint (x, y).
        sign: Traffic sign spec (position + color).
        color: Effective sign color (may be camera-confirmed).
        corridor: Current track section.
        direction: Travel direction (CW/CCW) -- selects the pass-side mapping.
        lateral_offset: Lateral deformation magnitude (m).
        robot_pos: Current robot position (x, y); enables the depth pin.
        context: Tuning-derived constants for the wall-clearance clamp. Defaults
            to the checked-in tuning.
        yaw_drift: Absolute heading change (rad) since the pin engaged on this
            sign; releases the pin past ``PIN_HEADING_GUARD_DEG`` when
            ``PIN_HEADING_GUARD`` is set. See ``pin_depth``.

    Returns:
        Deformed waypoint (x, y).
    """
    if (corridor, direction) not in ROUTING_TABLE:
        return waypoint

    entry = ROUTING_TABLE[(corridor, direction)]
    # Explicit colour match, not `if RED else GREEN`. An UNKNOWN sign is a
    # position without a colour (a LIDAR proposal the camera has not confirmed);
    # it has no pass side, so the waypoint is returned UNDEFORMED and the robot
    # holds its line. Generic obstacle avoidance still applies -- declining to
    # choose a side is not declining to avoid the object.
    axis = entry.axis
    if color == SignColor.RED:
        mult = entry.red_mult
    elif color == SignColor.GREEN:
        mult = entry.green_mult
    else:
        return waypoint

    wx, wy = waypoint
    if axis == Axis.Y:
        return (
            pin_depth(wx, sign.x, robot_pos[0] if robot_pos else None, robot_pos, corridor, context, yaw_drift),
            clamp_lateral(sign.y + mult * lateral_offset, corridor, context),
        )
    return (
        clamp_lateral(sign.x + mult * lateral_offset, corridor, context),
        pin_depth(wy, sign.y, robot_pos[1] if robot_pos else None, robot_pos, corridor, context, yaw_drift),
    )


def pin_depth(
    waypoint_depth: float,
    sign_depth: float,
    robot_depth: float | None,
    robot_pos: tuple[float, float] | None,
    corridor: Section,
    context: SignRouterContext | None = None,
    yaw_drift: float | None = None,
) -> float:
    """Hold the commanded point abeam the sign instead of letting it recede.

    Applies only while the sign is genuinely between the chassis and the lookahead
    point, in whichever direction the robot is travelling along the corridor. Once
    the robot is level with the sign the condition lapses on its own and the
    ordinary lookahead resumes -- there is no separate "release" to get wrong, and
    a sign already behind never pulls the target backwards.

    ``deform_waypoint`` only checks ``is_squarely_in_corridor`` once, upstream,
    against the raw (pre-deformation) WAYPOINT -- not against where the robot
    itself actually is. The lookahead target runs 0.2-0.4 m ahead of the robot, so
    the robot can already have curved out of the straight-corridor assumption this
    pin depends on (e.g. mid corner-arc) while the waypoint still reads squarely in
    the corridor. Pinning to the sign's depth in that state drove the commanded
    point into a wall -- measured as 11 wall collisions with the pin on against 0
    with it off, all corner-adjacent. Re-checking squareness here, against the
    robot's own real (x, y), closes that gap: the pin only fires when both ends of
    its own logic actually hold.

    That re-check rode in on an unrelated commit ten days after the 11 was measured
    and was never attributed on its own, so it carries its own toggle
    (``PIN_CORNER_GUARD``) -- both arms belong in one harness invocation.

    ``PIN_CORNER_GUARD`` re-checks the robot's POSITION but not its HEADING.
    Traced on go_obstacles_0049 (subset64, sighted): the robot entered a corner turn
    -- yaw rotating 67 deg to 127 deg over 46 ticks -- while its raw waypoint
    position still read squarely in the corridor the whole time, so the position
    guard never released the pin. The commanded point stayed frozen abeam a sign for
    2.3 s while the chassis was actually mid-turn, steering saturated chasing it,
    and the chassis crashed into a wall. ``PIN_HEADING_GUARD`` releases the pin once
    the robot's heading has drifted more than ``PIN_HEADING_GUARD_DEG`` from where
    it stood when the pin first engaged on this sign, which is what the position
    check misses.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    if robot_depth is None or robot_pos is None:
        return waypoint_depth
    if context.constants.pin_corner_guard and not is_squarely_in_corridor(
        robot_pos[0], robot_pos[1], corridor, context
    ):
        return waypoint_depth
    if (
        context.constants.pin_heading_guard
        and yaw_drift is not None
        and yaw_drift > context.constants.pin_heading_guard_rad
    ):
        return waypoint_depth
    if min(robot_depth, waypoint_depth) < sign_depth < max(robot_depth, waypoint_depth):
        return sign_depth
    return waypoint_depth


def match_detection_to_sign(
    observations: list[TrafficSignObservation],
    expected_world_pos: tuple[float, float],
    config: SignRouterConfig,
) -> SignColor | None:
    """Try to confirm sign color using world-coordinate observations.

    Matches each observation against the expected sign world position.
    TrafficSignObservation already carries world coordinates, so no pixel-to-world
    projection is needed.

    Args:
        observations: Current frame observations.
        expected_world_pos: Expected (x, y) world position of the sign.
        config: Router config (confidence threshold, match distance).

    Returns:
        Confirmed SignColor, or None if no confident match.
    """
    best_match_dist = float("inf")
    best_color: SignColor | None = None

    for obs in observations:
        if obs.confidence < config.min_confidence:
            continue
        if obs.color not in (SignColor.RED, SignColor.GREEN):
            continue

        world = (obs.world_x_m, obs.world_y_m)
        d = _dist2d(Waypoint(*world), Waypoint(*expected_world_pos))
        if d < config.detection_match_dist and d < best_match_dist:
            best_match_dist = d
            best_color = obs.color

    return best_color
