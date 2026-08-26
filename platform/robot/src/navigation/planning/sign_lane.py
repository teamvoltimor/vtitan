"""Obstacles-only lane planning: shift the PATH past a sign, not just the carrot.

``SignRouter.deform_waypoint`` avoids a sign by overriding the lateral
coordinate of whatever point the pure-pursuit search picked, within
``ACTIVATION_DIST_M`` of the sign. The planned waypoint polyline itself never
moves. Two consequences follow from that, and both are measured:

* ``cross_track_error`` -- the controller's own error signal, and the input
  ``select_lookahead`` gates on -- is computed against the undeformed path, so
  it stays near zero for the whole pass. The tracker never learns it is
  supposed to be somewhere else; it only ever sees a carrot that has been
  nudged sideways.
* The correction therefore has to be produced entirely by pure pursuit chasing
  an off-path point over ~1.4 m of runway, which closes the offset only
  asymptotically: a consistent ~6.3-6.6 cm shortfall between the commanded
  line and the chassis at the moment it draws level with the sign, across four
  independently traced scenarios (subset64 go_obstacles_0009/0011/0020/0046).

Seven levers that change WHEN or HOW HARD that carrot-chase happens have all
been measured flat or worse (activation distance, offset magnitude, lookahead,
steering gain, sign-aware lookahead x2, sign-aware speed x3) -- see
``signs.py``'s ``SIGN_AWARE_LOOKAHEAD``/``SIGN_AWARE_SPEED`` docstrings and
``docs/sign-avoidance-investigation.md``.

This module changes the maneuver instead of its tuning. It rewrites the
corridor's straight-segment waypoints onto a pass-side LANE: the robot
transitions onto the lane over ``SIGN_LANE_RAMP_M`` of approach, holds it
across the sign, and transitions back. Because the polyline moves, crosstrack,
the lookahead gate and the target search all agree the lane is the path, and
the lateral travel is spread over the whole corridor straight instead of being
demanded in the last 1.4 m.

Obstacles-only by construction: the transform is driven by the routed sign
list, and the Open Challenge has no ``SignRouter``, so with no signs the
returned path is the input path -- byte for byte, not merely equivalent.

The transform is 1:1 -- same waypoint count, same order, only lateral
coordinates change. Every index-keyed invariant in ``CoreNavigator``
(``_waypoint_index`` advance, the lap-seam wrap, ``replace_path``'s re-seek)
therefore survives it unchanged.

WRO layout invariants this planner leans on
-------------------------------------------
Measured over all 256 corpus scenarios (1282 signs), not assumed:

* Along-corridor sign depths take exactly three values: 1.00, 1.50, 2.00 --
  the section's start, middle and end. A section is a 1 m x 1 m square.
* A section holds 0, 1 or 2 signs, never 3. The only depth combinations that
  occur are ``(1.0,)``, ``(1.5,)``, ``(2.0,)`` and ``(1.0, 2.0)``.
* A MIDDLE sign is always the only sign in its section (0 counter-examples).
  Two signs therefore always sit at the section boundaries, exactly 1.00 m
  apart, which is what gives the S-bend above its runway.
* No sign ever occupies a corner. Corner arcs are guaranteed free space --
  which is what makes ``corner_entry_m`` below safe to use as ramp room.
* Signs sit only 0.10 m off the corridor centreline (lateral 0.4/0.6 against
  a 0.5 centre). Since the pass offset is 0.28 m, ``clamp_lateral`` binds on
  essentially every sign -- the lane runs at its clearance limit by
  construction, not by mis-tuning. Counted exactly: 646 of the 1282 signs,
  in 248 of the 256 scenarios.

  Do not read that as an available lever; it has been measured and it is
  not. The simulator collides via exact SAT on the ORIENTED chassis, so a
  squeezed lane's clearance is yaw-dependent: the clamped placement clears
  its sign only within +/-28.2 deg of the corridor axis, while the midpoint
  of the free gap clears at any yaw. Moving the plateau from one to the
  other (``SIGN_LANE_GAP_CENTRE_FRAC``, since reverted -- see ``66fa5f2e``
  for the implementation) does exactly what that geometry predicts to the
  SIGN column, 199 collisions down to 168, and loses far more to the wall,
  3 up to 61: 229/256 against a 202/256 baseline. Swept at 0.25/0.40/0.55/
  0.70 it is worse at every value (213/211/217/220), the sign column is not
  even monotonic (WORSE than baseline at 0.25 and 0.40), and laps>=3 falls
  monotonically 56 -> 32.

  The reason is the number in the second bullet at the top of this
  docstring: every one of those arms is geometrically wall-immune at any
  yaw, so the wall strikes are not the plan reaching the wall -- they are
  the chassis failing to be on the plan. The entire adjustable range is
  3.1 cm (0.2186 -> 0.1875) against a 6.3-6.6 cm crosstrack shortfall. The
  lever is half the size of the error it is fighting, so no placement can
  win. Reduce the tracking error before revisiting the geometry.

The third and fourth points together drive ``corner_entry_m``: 1211 of the
1282 signs sit at a section BOUNDARY, where the straight offers no runway on
the near side at all (a plateau centred on depth 1.00 already starts at 0.75,
outside the straight). Confining the lane to the straight therefore put the
first straight waypoint at full offset hard against an untouched corner arc --
an abrupt lateral step exactly at the corner exit, beside the inner block.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

from shared.config.constants import TrackDimensions
from shared.domain.enums import Section
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_router import Axis, SignSpec, clamp_lateral, outward_lateral_axis

__all__ = ["SignLaneParams", "apply_sign_lanes"]


@dataclass(frozen=True, slots=True)
class SignLaneParams:
    """Geometry of the lane transition. See ``SignRouterParams`` for the tunables."""

    lateral_offset: float
    """Lateral distance from the sign's own centre to the lane (m).

    The same magnitude ``SignRouter`` deforms by (chassis half-diagonal + sign
    half-width + margin), so the lane and the carrot ask for the same line --
    they differ in WHEN the robot is asked to be on it, not WHERE it is.
    """

    ramp_m: float
    """Along-corridor distance to transition on and off the lane (m)."""

    hold_m: float
    """Along-corridor half-width of the full-offset hold around a sign (m)."""

    split_overlap: bool = False
    """Split overlapping plateaux at their midpoint instead of letting one dip through another.

    Legal WRO geometry never overlaps -- a section holds at most two signs, 1.00 m
    apart, against a 0.50 m plateau -- so an overlap is always a DISCOVERY
    artifact. See ``SIGN_LANE_SPLIT_OVERLAP``.
    """

    skip_unsatisfiable: bool = False
    """Drop a sign whose clamped target is on the FORBIDDEN side of it.

    Such a sign cannot be satisfied by any lane geometry -- the instruction
    itself violates the rule -- so the choice is between planning a violating
    line and planning none. See ``SIGN_LANE_SKIP_UNSATISFIABLE``.
    """

    corner_entry_m: float = 0.0
    """How far past the corridor's straight the lane may extend, into the
    corner arcs either side (m). ``0.0`` confines it to the straight.

    Non-zero because 1211 of the corpus's 1282 signs sit at a section
    BOUNDARY, where the straight offers no near-side runway whatsoever: the
    lane reaches full offset on its very first waypoint, against a corner arc
    still exactly on the centreline. The step that creates lands at the corner
    exit, beside the inner square, which is where the lane's wall collisions
    were traced.

    Safe to spend corner arc as runway specifically because no sign ever
    occupies a corner (0 of 1282), so nothing is being routed around there --
    the arc is free space whose only job is to deliver the chassis into the
    next corridor, and delivering it already on the lane is strictly closer to
    what the robot must end up doing. ``clamp_lateral`` still bounds every
    point it moves.
    """


def _axis_coords(wp: Waypoint, axis: Axis) -> tuple[float, float]:
    """Return ``(lateral, depth)`` for a waypoint under ``axis``."""
    return (wp.y, wp.x) if axis is Axis.Y else (wp.x, wp.y)


def _rebuild(wp: Waypoint, axis: Axis, lateral: float) -> Waypoint:
    """Rebuild a waypoint with its lateral coordinate replaced."""
    return Waypoint(wp.x, lateral) if axis is Axis.Y else Waypoint(lateral, wp.y)


def _lane_span(corner_entry_m: float) -> tuple[float, float]:
    """Depth window the lane may rewrite, straight plus any borrowed corner."""
    return (
        TrackDimensions.CORNER_MIN - corner_entry_m,
        TrackDimensions.CORNER_MAX + corner_entry_m,
    )


def _in_lane_span(wp: Waypoint, corridor: Section, axis: Axis, corner_entry_m: float) -> bool:
    """True if ``wp`` is a point this corridor's lane may move.

    The depth test widens by ``corner_entry_m`` to borrow corner-arc runway
    (see ``SignLaneParams.corner_entry_m``); the LATERAL test does not widen,
    and that is what keeps the widening safe. A corner arc leaves this
    corridor's side of the track partway round, so once its points have
    crossed into the neighbouring corridor's band they stop qualifying on
    their own -- the borrowed region self-limits to the part of the arc still
    geometrically belonging to this corridor, rather than running away around
    the whole turn.

    A variant that widened the lateral test too (out to the far edge of the
    corner square, gated to only apply at a sign's own plateau depth) was
    tried, to rescue a waypoint the corner arc's own curvature had already
    swept past the near edge before its depth exited the borrowed window
    (traced as the mechanism behind go_obstacles_0004's collision). It
    measured catastrophically WORSE on the full 256-scenario corpus (249/256
    collisions vs the 202/256 baseline, laps>=3 6 vs 70) and was reverted: the
    shipped ``corner_entry_m`` default is 0.50 (not 0.90 -- an earlier version
    of this note cited the wrong value, never having checked the shipped
    ``SignRouterParams.SIGN_LANE_CORNER_ENTRY_M``/TOML directly), which still
    doubles the depth window (0.5 m either side of the straight, `_lane_span`
    spanning 2.0 m against the corridor's own 1.0 m). A widened lateral bound
    at that depth pulls in swaths of the NEIGHBOURING corridor's own arc
    points too, corrupting geometry across corridors with a boundary sign
    rather than rescuing the one intended waypoint. Do not re-try without
    bounding the widened window far more tightly than "the far edge of the
    corner square" -- and verify any cited default against the actual shipped
    Field/TOML value before writing it down, not from memory of an earlier
    investigation.
    """
    lateral, depth = _axis_coords(wp, axis)
    low, high = _lane_span(corner_entry_m)
    if not low <= depth <= high:
        return False
    if corridor in (Section.SOUTH, Section.WEST):
        return lateral < TrackDimensions.CORNER_MIN
    return lateral > TrackDimensions.CORNER_MAX


def _control_points(
    signs: list[tuple[SignSpec, Section]],
    corridor: Section,
    axis: Axis,
    base_lateral: float,
    params: SignLaneParams,
) -> list[tuple[float, float]]:
    """Piecewise-linear ``(depth, lateral)`` profile for one corridor's lane.

    Each sign contributes a full-offset plateau spanning ``hold_m`` either side
    of its own depth, and the profile ramps to the corridor's base lateral
    ``ramp_m`` beyond the outermost plateau at each end.

    Two signs in the same corridor requiring OPPOSITE sides simply produce two
    plateaux at opposite laterals with a straight interpolation between them:
    an S-bend, which is the maneuver a driver actually makes. Averaging their
    influence instead would command the centreline between two obstacles the
    robot must pass on opposite sides, i.e. straight into both. This is the
    common case, not an edge one -- 342 of the 514 two-sign sections in the
    256-scenario corpus are opposite-coloured.

    The S-bend is comfortable rather than tight, and that is a property of the
    WRO layout rather than luck: a section holding two signs ALWAYS has them
    exactly 1.00 m apart (see the layout invariants in the module docstring),
    so the lane has a full metre to cross the corridor rather than the 0.50 m
    an earlier version of this comment wrongly assumed.
    """
    plateaux: list[tuple[float, float]] = []
    for spec, sign_corridor in signs:
        rule = outward_lateral_axis(sign_corridor, SignColor(spec.color))
        if rule is None:
            continue
        _, mult = rule
        sign_lateral, sign_depth = _axis_coords(Waypoint(spec.x, spec.y), axis)
        target = clamp_lateral(sign_lateral + mult * params.lateral_offset, corridor)
        if params.skip_unsatisfiable and (target - sign_lateral) * mult <= 0.0:
            # The clamp put this sign's own target on the forbidden side of it,
            # so every point of its plateau would violate the rule it exists to
            # obey. Emit nothing rather than a line that is wrong by
            # construction; the corridor's other signs still get their plateaux.
            continue
        plateaux.append((sign_depth, target))

    if not plateaux:
        return []

    points: list[tuple[float, float]] = []
    plateaux.sort(key=lambda entry: entry[0])
    for index, (sign_depth, target) in enumerate(plateaux):
        low = sign_depth - params.hold_m
        high = sign_depth + params.hold_m
        if params.split_overlap:
            # A plateau nested inside another one puts a HOLE in the enclosing
            # sign's hold window: the profile dips to the neighbour's target
            # exactly where the robot is abeam this sign. Traced on a WEST
            # corridor holding three specs -- our plateau ran 2.15..2.65 at
            # lateral 0.781 while a neighbour's endpoints sat inside it at 2.22
            # and 2.24 at 0.325, and the plan passed the sign on that dip.
            #
            # Legal WRO geometry cannot produce this: a section holds at most
            # two signs and they sit 1.00 m apart, against a 0.50 m plateau. The
            # overlaps come from DISCOVERY emitting several specs per physical
            # sign (measured 2.50x). Splitting the overlap at the midpoint gives
            # each sign the half nearer itself, so every sign keeps a flat hold
            # over the stretch where it is actually passed.
            if index > 0:
                low = max(low, (plateaux[index - 1][0] + sign_depth) / 2.0)
            if index + 1 < len(plateaux):
                high = min(high, (sign_depth + plateaux[index + 1][0]) / 2.0)
            if high < low:
                low = high = sign_depth
        points.append((low, target))
        points.append((high, target))

    if not points:
        return []

    points.sort(key=lambda p: p[0])
    # Ramp endpoints are clamped into the corridor's own straight span rather
    # than allowed to fall outside it. A sign near the middle of a 1.45 m
    # straight puts the nominal ramp start behind CORNER_MIN, where there are
    # no straight waypoints to carry it -- the profile would then be truncated
    # at the boundary and the first straight waypoint would already sit a
    # sizeable step off the centreline, while the corner arc immediately
    # before it (deliberately untouched, see _on_corridor_straight) is still
    # exactly on it. That step is a kink in the path at precisely the place
    # the chassis is finishing a turn. Clamping instead COMPRESSES the ramp:
    # it still reaches the centreline, just sooner and more steeply, so the
    # lane meets the arc continuously.
    # Overlapping plateaux (signs closer together than 2 * hold_m) would
    # otherwise leave the profile non-monotonic in depth, which the
    # interpolation below reads as a fold-back. Collapsing ties to a shared
    # depth turns the fold into an instantaneous transition at that depth --
    # ugly, but it commands a real line rather than an undefined one, and the
    # ramps on either side still carry the chassis onto it.
    for i in range(1, len(points)):
        if points[i][0] < points[i - 1][0]:
            points[i] = (points[i - 1][0], points[i][1])

    low, high = _lane_span(params.corner_entry_m)
    entry = min(max(points[0][0] - params.ramp_m, low), points[0][0])
    exit_ = max(min(points[-1][0] + params.ramp_m, high), points[-1][0])
    return [(entry, base_lateral), *points, (exit_, base_lateral)]


def _interpolate(profile: list[tuple[float, float]], depth: float) -> float | None:
    """Lateral value of ``profile`` at ``depth``; ``None`` outside its span."""
    if depth < profile[0][0] or depth > profile[-1][0]:
        return None
    for (d0, l0), (d1, l1) in pairwise(profile):
        if d0 <= depth <= d1:
            if d1 == d0:
                return l1
            return l0 + (l1 - l0) * (depth - d0) / (d1 - d0)
    return profile[-1][1]


def apply_sign_lanes(
    waypoints: list[Waypoint],
    signs: list[tuple[SignSpec, Section]],
    params: SignLaneParams,
) -> list[Waypoint]:
    """Return ``waypoints`` with each signed corridor's straight shifted onto its pass-side lane.

    Args:
        waypoints: The planned path (one canonical lap). Not mutated.
        signs: ``(spec, corridor)`` for every sign still being routed around,
            corridor as the router itself labels it (``_sign_corridors``) --
            not recomputed here, so a discovery estimate that has been held
            steady against corner jitter stays steady in the lane too.
        params: Lane geometry.

    Returns:
        A new list of the same length and order. Identical to the input when
        ``signs`` is empty, which is every Open Challenge run.
    """
    if not signs or not waypoints:
        return list(waypoints)

    result = list(waypoints)
    by_corridor: dict[Section, list[tuple[SignSpec, Section]]] = {}
    for entry in signs:
        by_corridor.setdefault(entry[1], []).append(entry)

    for corridor, corridor_signs in by_corridor.items():
        rule = outward_lateral_axis(corridor, SignColor(corridor_signs[0][0].color))
        if rule is None:
            continue
        axis, _ = rule

        indices = [i for i, wp in enumerate(result) if _in_lane_span(wp, corridor, axis, params.corner_entry_m)]
        if not indices:
            continue
        # The corridor's own centreline as PLANNED, which already carries
        # CENTER_BIAS_M and this scenario's estimated corridor width -- taking
        # it from the waypoints rather than recomputing it keeps the lane's
        # ramp endpoints on the path the tracker would otherwise follow, so the
        # transform is continuous at both ends by construction.
        #
        # Measured over the STRAIGHT only, even when corner runway is being
        # borrowed: an arc's lateral coordinate sweeps away from the
        # centreline as it turns, so including arc points would drag this
        # median off the centreline it is supposed to represent.
        straight = [i for i in indices if _in_lane_span(result[i], corridor, axis, 0.0)]
        laterals = sorted(_axis_coords(result[i], axis)[0] for i in (straight or indices))
        base_lateral = laterals[len(laterals) // 2]

        profile = _control_points(corridor_signs, corridor, axis, base_lateral, params)
        if not profile:
            continue

        for i in indices:
            lateral, depth = _axis_coords(result[i], axis)
            lane = _interpolate(profile, depth)
            if lane is None:
                continue
            # Apply the profile as a SHIFT from the centreline, not as an
            # absolute lateral. On the straight the two are identical (every
            # point already sits at base_lateral), but on borrowed corner
            # runway they are not: an arc point's own lateral is partway
            # through the turn, and assigning it the profile's absolute value
            # would snap it back onto the corridor centreline -- destroying
            # the turn instead of offsetting it. Shifting translates the arc
            # while leaving its shape intact, which is exactly the "exit the
            # corner already on the lane" behaviour this borrows runway for.
            shifted = clamp_lateral(lateral + (lane - base_lateral), corridor)
            if shifted == lateral:
                continue
            result[i] = _rebuild(result[i], axis, shifted)

    return result
