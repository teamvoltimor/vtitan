"""Sign-routing table and pure routing helpers for the WRO 2026 obstacles challenge.

The pass-side rule is absolute, tied to track geometry, not travel direction:
a red obstacle is cleared on its OUTWARD side (toward the outer wall) and a green
on its INWARD side. This module holds the per-(corridor, direction) routing table
and the pure functions that look the rule up, clamp a deformed lateral
coordinate, test corridor squareness, and extract signs from metadata. No router
state.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from shared.config.constants import DictKeys, TrackDimensions
from shared.domain.enums import Axis, Direction, Section
from shared.domain.models import RoutingEntry, ScenarioMetadata, SignColor

from src.navigation.geometry import behind_tolerance_m
from src.navigation.planning.sign_discovery import SignSpec
from src.navigation.planning.sign_router.config import (
    _CHASSIS_HALF_DIAGONAL,
    _DEFAULT_SIGN_ROUTER_CONTEXT,
    SignRouterContext,
)

if TYPE_CHECKING:
    from collections.abc import Mapping

_BEHIND_TOLERANCE = behind_tolerance_m()

# Per-(corridor, direction) routing table.
# axis: Axis.Y means deform the y-coordinate; Axis.X deforms x.
# red_mult / green_mult: +1 or -1 multiplier applied to the LATERAL offset,
# chosen so red always moves the deformed waypoint OUTWARD (away from the
# inner square) and green always moves it INWARD -- identically for CW and
# CCW, since outward/inward is a fixed property of the corridor, not the
# travel direction. (An earlier version of this table made the CW rows the
# world-frame negation of the CCW rows, which instead pinned "red on the
# robot's right" -- a travel-RELATIVE rule that flips outward/inward between
# CW and CCW. That was wrong: the official rule is the absolute one above.)
_ROUTING_TABLE: dict[tuple[Section, Direction], RoutingEntry] = {
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.Y, -1, +1),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.Y, +1, -1),
    (Section.EAST, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.X, +1, -1),
    (Section.WEST, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.X, -1, +1),
    (Section.SOUTH, Direction.CLOCKWISE): RoutingEntry(Axis.Y, -1, +1),
    (Section.NORTH, Direction.CLOCKWISE): RoutingEntry(Axis.Y, +1, -1),
    (Section.EAST, Direction.CLOCKWISE): RoutingEntry(Axis.X, +1, -1),
    (Section.WEST, Direction.CLOCKWISE): RoutingEntry(Axis.X, -1, +1),
}


def outward_lateral_axis(corridor: Section, color: SignColor) -> tuple[Axis, int] | None:
    """World-frame axis and sign of the pass-side rule for ``corridor``, direction-agnostic.

    The CLOCKWISE and COUNTERCLOCKWISE rows of ``_ROUTING_TABLE`` are identical
    for every section (see module docstring), so looking the rule up under a fixed
    direction is exactly as correct as knowing the real one. This lets a caller
    that hasn't inferred the travel direction yet -- e.g. ``corridor_follower``
    during BLIND_CREEP -- still apply "red outward, green inward" instead of
    falling back to generic obstacle avoidance.

    Returns:
        ``(axis, multiplier)`` where a positive multiplier along ``axis`` points
        OUTWARD (away from the inner square) for red, INWARD for green. ``None``
        if ``corridor`` has no routing entry.
    """
    entry = _ROUTING_TABLE.get((corridor, Direction.CLOCKWISE))
    if entry is None:
        return None
    return entry.axis, entry.red_mult if color == SignColor.RED else entry.green_mult


def clamp_lateral(value: float, corridor: Section, context: SignRouterContext | None = None) -> float:
    """Clamp a deformed lateral coordinate clear of the inner square and outer wall.

    SOUTH/WEST corridors border the inner square on their high side (the
    coordinate must stay below ``CORNER_MIN``); NORTH/EAST border it on their low
    side (must stay above ``CORNER_MAX``). Every corridor is also bounded on its
    outer side by the track wall.

    This clamp is asymmetric by nature and that is deliberate but NOT free: it
    keeps the full boundary clearance and hands whatever squeeze remains entirely
    to the sign. On 646 of the corpus's 1282 signs it binds, and the resulting
    lane clears its sign only within +/-28.2 deg of the corridor axis (the
    simulator collides via exact SAT on the oriented chassis, so clearance is
    yaw-dependent -- do not model it as a flat half-diagonal threshold).

    Rebalancing it has been measured and REFUTED. A ``pass_lateral`` that
    interpolated a squeezed plateau toward the midpoint of its free gap -- the
    maximin placement, clear of both sides at every yaw -- moved the sign column
    exactly as predicted (199 collisions to 168) and the wall column far more (3
    to 61), for 229/256 against a 202/256 baseline; swept at 0.25/0.40/0.55/0.70
    it was worse at every value. Do not re-try a placement change here without
    first reducing that tracking error.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    wall_clearance = _CHASSIS_HALF_DIAGONAL + context.constants.wall_clearance_margin_m
    low_side = corridor in (Section.SOUTH, Section.WEST)
    if low_side:
        value = min(value, TrackDimensions.CORNER_MIN - wall_clearance)
        value = max(value, TrackDimensions.MIN_COORD + wall_clearance)
    else:
        value = max(value, TrackDimensions.CORNER_MAX + wall_clearance)
        value = min(value, TrackDimensions.MAX_COORD - wall_clearance)
    return value


def _is_squarely_in_corridor(x: float, y: float, corridor: Section, context: SignRouterContext | None = None) -> bool:
    """True if this waypoint is still a reasonable candidate for straight-corridor deformation.

    The deformation model holds the depth axis (whatever value the raw path
    already gives it) and overrides only the lateral axis with a value derived
    from the sign's position, then clamps that result into the corridor's own
    free-space band. Two independent checks:

    * Lateral axis (the one being overridden) must still read as this corridor,
      not already the opposite wall.
    * Depth axis (held, never touched) must stay within ``DEFORM_DEPTH_BUFFER_M``
      of the inner square's own span -- not the exact ``[CORNER_MIN, CORNER_MAX]``
      window ``corridor_for_position()`` uses for its own robot-position
      classification, which is far too strict here: the lookahead target runs
      0.2-0.4m ahead of the robot, so it's often already past that window well
      before the robot itself is anywhere near a corner, and requiring it anyway
      silently killed deformation through most of a sign's real engagement. But
      with no depth check at all, deformation can keep firing long after the robot
      has geometrically left this corridor for the next one, building up an offset
      that snaps back hard once the sign finally disengages by corridor mismatch --
      this buffer catches that case without reintroducing the original
      over-strict cutoff.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    deform_depth_buffer = context.constants.deform_depth_buffer_m
    depth_min = TrackDimensions.CORNER_MIN - deform_depth_buffer
    depth_max = TrackDimensions.CORNER_MAX + deform_depth_buffer
    if corridor is Section.SOUTH:
        return y < TrackDimensions.CORNER_MIN and depth_min <= x <= depth_max
    if corridor is Section.NORTH:
        return y > TrackDimensions.CORNER_MAX and depth_min <= x <= depth_max
    if corridor is Section.EAST:
        return x > TrackDimensions.CORNER_MAX and depth_min <= y <= depth_max
    if corridor is Section.WEST:
        return x < TrackDimensions.CORNER_MIN and depth_min <= y <= depth_max
    return False


def signs_from_metadata(metadata: ScenarioMetadata | Mapping[Any, Any]) -> list[SignSpec]:
    """Extract sign specs from scenario metadata.

    Args:
        metadata: Scenario metadata (Pydantic model or coercible dict).

    Returns:
        List of SignSpec for all signs in the scenario.
    """
    if isinstance(metadata, ScenarioMetadata):
        return [SignSpec(x=s.x, y=s.y, color=s.color) for s in metadata.sign_positions]
    sign_positions = metadata.get(DictKeys.SIGN_POSITIONS, [])
    return [
        SignSpec(x=entry[DictKeys.X], y=entry[DictKeys.Y], color=entry[DictKeys.COLOR]) for entry in sign_positions
    ]
