"""Sign-routing table and pure routing helpers for the WRO 2026 obstacles challenge.

The pass-side rule is TRAVEL-RELATIVE: the vehicle passes to its own RIGHT of a
red pillar and to its own LEFT of a green one, in the direction the round is
being driven. Rules 2026 9.19: "The vehicle must pass the traffic sign
represented by the red pillar on the right ... and the green pillar on the
left", restated for back-to-front driving on p38. This module holds the
per-(corridor, direction) routing table and the pure functions that look the
rule up, clamp a deformed lateral coordinate, test corridor squareness, and
extract signs from metadata. No router state.

Because the vehicle's right is the OUTER wall when driving counterclockwise and
the INNER square when driving clockwise, the same rule inverts in world terms
between the two directions -- so the CW rows are the negation of the CCW rows,
and **the rule cannot be evaluated without knowing the travel direction**.

Corrected 2026-09-03 after reading the official PDF. Between 2026-07-05 and then
this table was ABSOLUTE (red always outward, CW rows identical to CCW), which is
right for counterclockwise and backwards for every clockwise round. It was
invisible because ``scenario_simulator/scoring.py`` scored the same absolute
convention the router drove, so the simulator graded itself against its own
mistake -- 256-corpus pass-side counts recorded before this date are scored on
the absolute rule and mean nothing under the real one.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from shared.config.constants import DictKeys, TrackDimensions
from shared.domain.enums import Axis, Direction, Section
from shared.domain.models import RoutingEntry, ScenarioMetadata, SignColor

from src.navigation.geometry import behind_tolerance_m
from src.navigation.planning.sign_discovery import SignSpec
from src.navigation.planning.sign_router.config import (
    _DEFAULT_SIGN_ROUTER_CONTEXT,
    CHASSIS_HALF_DIAGONAL,
    SignRouterContext,
)

if TYPE_CHECKING:
    from collections.abc import Mapping

BEHIND_TOLERANCE = behind_tolerance_m()
# Fewest overlapping corridors a position must sit in before the depth-violation
# tie-break has anything to choose between.
_MIN_CANDIDATES_TO_DISAMBIGUATE = 2

# Per-(corridor, direction) routing table.
# axis: Axis.Y means deform the y-coordinate; Axis.X deforms x.
# red_mult / green_mult: +1 or -1 multiplier applied to the LATERAL offset,
# chosen so the deformed waypoint moves to the vehicle's own RIGHT of a red
# pillar and to its own LEFT of a green one, for the direction actually driven.
#
# Worked example, so the polarity can be re-derived rather than trusted: on the
# SOUTH straight a counterclockwise round heads EAST, and the right hand of an
# east-facing chassis points SOUTH, which is OUTWARD (the south corridor sits at
# low y, away from the inner square). So CCW red = -1 on Y = outward. A
# clockwise round drives the same straight heading WEST, whose right hand points
# NORTH = inward, so CW red = +1. Every CW row is the negation of its CCW
# partner for exactly this reason -- "the vehicle's right" is a fixed rule that
# names opposite world directions depending on which way it drives.
ROUTING_TABLE: dict[tuple[Section, Direction], RoutingEntry] = {
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.Y, -1, +1),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.Y, +1, -1),
    (Section.EAST, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.X, +1, -1),
    (Section.WEST, Direction.COUNTERCLOCKWISE): RoutingEntry(Axis.X, -1, +1),
    (Section.SOUTH, Direction.CLOCKWISE): RoutingEntry(Axis.Y, +1, -1),
    (Section.NORTH, Direction.CLOCKWISE): RoutingEntry(Axis.Y, -1, +1),
    (Section.EAST, Direction.CLOCKWISE): RoutingEntry(Axis.X, -1, +1),
    (Section.WEST, Direction.CLOCKWISE): RoutingEntry(Axis.X, +1, -1),
}


def pass_side_lateral_axis(
    corridor: Section, color: SignColor, direction: Direction | None
) -> tuple[Axis, int] | None:
    """World-frame axis and sign of the pass-side rule for ``corridor``.

    ``direction`` is REQUIRED and may not be guessed. Until 2026-09-03 this was
    ``outward_lateral_axis(corridor, color)``, which looked the rule up under a
    fixed CLOCKWISE key and documented itself as direction-agnostic -- true only
    while the table's CW and CCW rows were identical, which was itself the bug.
    Under the real travel-relative rule the two rows are negations, so a caller
    without a settled direction cannot evaluate the rule at all.

    Returns ``None`` when ``direction`` is unknown, which callers must treat as
    "the pass-side rule is unavailable on this tick" and fall back to generic
    obstacle avoidance. That is a real capability loss for BLIND_CREEP, where the
    direction has not been inferred yet -- but a coin-flip between two opposite
    answers is worse than declining to answer, because a wrong-side pass ENDS
    THE ROUND under rule 9.24.5 while merely avoiding the sign does not.

    Returns:
        ``(axis, multiplier)`` where a positive multiplier along ``axis`` points
        to the vehicle's own RIGHT for red and its own LEFT for green, expressed
        in world coordinates for ``direction``. ``None`` if ``direction`` is
        ``None`` or ``corridor`` has no routing entry.
    """
    if direction is None:
        return None
    entry = ROUTING_TABLE.get((corridor, direction))
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
    wall_clearance = CHASSIS_HALF_DIAGONAL + context.constants.wall_clearance_margin_m
    low_side = corridor in (Section.SOUTH, Section.WEST)
    if low_side:
        value = min(value, TrackDimensions.CORNER_MIN - wall_clearance)
        value = max(value, TrackDimensions.MIN_COORD + wall_clearance)
    else:
        value = max(value, TrackDimensions.CORNER_MAX + wall_clearance)
        value = min(value, TrackDimensions.MAX_COORD - wall_clearance)
    return value


def candidate_corridors(x: float, y: float) -> list[Section]:
    """Corridors a point could plausibly belong to, nearest-face order aside.

    On a straight this is one section. In a CORNER -- both coordinates outside
    the inner square -- it is the two adjacent faces, which is exactly the
    ambiguity ``corridor_for_position`` resolves by picking the nearest one.
    Two-thirds of legal WRO grid positions sit on a corner boundary.
    """
    candidates: list[Section] = []
    if y < TrackDimensions.CORNER_MIN:
        candidates.append(Section.SOUTH)
    if y > TrackDimensions.CORNER_MAX:
        candidates.append(Section.NORTH)
    if x > TrackDimensions.CORNER_MAX:
        candidates.append(Section.EAST)
    if x < TrackDimensions.CORNER_MIN:
        candidates.append(Section.WEST)
    return candidates


def _depth_violation(x: float, y: float, corridor: Section) -> float:
    """How far outside its corridor's straight this point's DEPTH falls.

    Zero anywhere along the straight. The along-corridor axis is x for
    SOUTH/NORTH and y for EAST/WEST -- the axis ``pass_side_lateral_axis`` does
    NOT use, since lateral and depth are perpendicular by definition.
    """
    depth = x if corridor in (Section.SOUTH, Section.NORTH) else y
    return max(0.0, TrackDimensions.CORNER_MIN - depth, depth - TrackDimensions.CORNER_MAX)


def depth_consistent_corridor(x: float, y: float, fallback: Section) -> Section:
    """The candidate face whose straight this point actually lies along.

    ``corridor_for_position`` resolves a corner by NEAREST FACE, which is the
    wrong axis to decide it on. A sign at depth exactly 2.00 -- and 1211 of 1282
    corpus signs sit at depth 1.00 or 2.00, right where the corner arc meets the
    straight -- needs only a millimetre of estimate error to tip past
    ``CORNER_MAX``. It is then millimetres from the perpendicular face and 0.6 m
    from its own, so nearest-face hands it the perpendicular one. There the
    sign's LATERAL offset becomes its depth, the lane target is computed on the
    wrong axis, and ``clamp_lateral`` clamps against the wrong bound.

    Measured 2026-08-26 over the 256 corpus with the belief offset removed, so
    this is the tie-break and not localization: 42.1% of published specs are
    filed against the perpendicular face, and the resulting "distance past the
    corner" is not a distribution but two spikes at 0.40 m (x568) and 0.60 m
    (x483) -- exactly the lateral offsets signs are allowed to take. Confined to
    boundary signs: 44.9% at depth 1.00 and 44.8% at 2.00 against 0 of 153
    mid-straight signs, which is what an unresolved exact tie looks like.

    Deciding on depth instead makes the tie answerable, because 0.40 m is not a
    legal depth: the winner is simply the candidate least outside its own
    straight. Ties keep ``fallback`` so a genuine diagonal is left where
    ``corridor_for_position`` put it.
    """
    candidates = candidate_corridors(x, y)
    # One candidate is already unambiguous; disambiguating by depth violation
    # needs at least two to choose between.
    if len(candidates) < _MIN_CANDIDATES_TO_DISAMBIGUATE:
        return fallback
    best = min(candidates, key=lambda section: _depth_violation(x, y, section))
    if _depth_violation(x, y, best) < _depth_violation(x, y, fallback):
        return best
    return fallback


def target_clearance(spec: SignSpec, corridor: Section, lateral_offset: float,
                     direction: Direction | None,
                     context: SignRouterContext | None = None) -> float | None:
    """Clearance this corridor's CLAMPED lane target leaves from the sign itself.

    Positive is the permitted side. Negative means the instruction is
    unsatisfiable: ``clamp_lateral`` has capped the target at the corridor bound
    and that bound is on the FORBIDDEN side of the sign, so every point of the
    resulting plateau violates the rule the lane exists to obey.
    """
    rule = pass_side_lateral_axis(corridor, spec.color, direction)
    if rule is None:
        return None
    axis, permitted = rule
    lateral = spec.y if axis == Axis.Y else spec.x
    target = clamp_lateral(lateral + permitted * lateral_offset, corridor, context)
    return (target - lateral) * permitted


def satisfiable_corridor(spec: SignSpec, corridor: Section, lateral_offset: float,
                         direction: Direction | None,
                         context: SignRouterContext | None = None) -> Section:
    """``corridor``, or the corner's OTHER face when this one cannot be satisfied.

    A sign discovered near a corner diagonal is ambiguous between two faces.
    Under the wrong one it reads as past that corridor's straight and hard
    against the inner square, so its clamped target lands on the forbidden side
    of the sign and the planner lays a line that violates its own rule by
    construction. Measured blind over the 256 corpus: specs inside their
    corridor's straight plan wrong-side 5% of the time against 29% for specs
    past the corner, and EVERY inverted spec (45/45) is satisfiable under the
    other face, with ~21.7 cm of clearance available there.

    Restricted to ``candidate_corridors`` on purpose. Any section that makes the
    arithmetic positive would satisfy the check -- including one on the far side
    of the track -- and that would plant a geometrically absurd lane while
    scoring well on the metric.
    """
    clearance = target_clearance(spec, corridor, lateral_offset, direction, context)
    if clearance is None or clearance > 0.0:
        return corridor
    for alternative in candidate_corridors(spec.x, spec.y):
        if alternative == corridor:
            continue
        other = target_clearance(spec, alternative, lateral_offset, direction, context)
        if other is not None and other > 0.0:
            return alternative
    return corridor


def is_squarely_in_corridor(x: float, y: float, corridor: Section, context: SignRouterContext | None = None) -> bool:
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
