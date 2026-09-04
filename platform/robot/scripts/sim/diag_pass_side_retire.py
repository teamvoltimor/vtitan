"""Ground-truth audit of the Obstacles pass-side rule.

Scores every sign from the TRUE layout against the TRUE trajectory, sharing
nothing with what the robot believes, and reports it beside the router's own
believed-frame record. The gap between the two is discovery error; the truth
column is what a judge would call.

Also carries the checks that qualify that number:

- **Retreat vs pass.** ``SignRouter`` retires a sign on distance ALONE
  (``d > passed_dist``), unlike the candidate filter directly below it, which
  also requires the sign not to be behind the chassis -- so a robot that backs
  away from a sign should cross the same threshold as one that drove past.
  REFUTED at 0 retreats in 181 retirements (4 in 1275 untruncated); every
  retirement has the sign genuinely behind at median -1.2 m. Kept because it
  is cheap and would catch the mechanism if escape behaviour ever changed.
- **Verdict stability.** Whether a violation survives nudging the sign's
  corridor label -- see ``_verdict_is_ambiguous``.
- **Escape attribution.** Whether the chassis was mid-escape at the closest
  approach, which is the first bucket of the wrong-side investigation.

``--no-terminate`` lets runs continue past a violation, so the rate is measured
over the full three laps rather than on a trajectory the verdict truncated.

``--known-start`` is the discriminator for the plan-wrong bucket. Blind seeds
the believed pose from a fixed SOUTH guess, so three quarters of the corpus
plan in a frame rotated a multiple of 90 deg from truth. The track is 4-fold
symmetric and the pass-side rule is rotation-invariant, so that rotation
*should* be harmless to lane construction; handing the true start back tells
which it is. If plan-wrong largely vanishes, lane construction mixes the
believed frame with absolute truth. If it survives, the frame is innocent and
the fault is in the routing geometry itself.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_pass_side_retire.py --scenarios-dir <dir> [--limit 64] [--no-terminate] [--known-start]
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.provenance import environment
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from shared.config.constants import TrackDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Axis, Direction
from src.navigation.planning import sign_router as sign_router_module
from src.navigation.planning.sign_lane import _axis_coords, _in_lane_span, _lane_span
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.utils import wrap_angle
from src.simulation.scenario_simulator import ScenarioSimulator

# After the simulator: importing shared.domain.models first hits the
# models<->enums cycle that enums.py resolves by deferring its re-export.
from shared.domain.models import ScenarioMetadata, Waypoint  # noqa: E402

_BEHIND_EPS_M = 0.0
"""Along-track sign that separates a pass from a retreat; 0 is the geometric split."""

_PASSED_NEAR_M = 0.60
"""Closest approach within which the run is treated as having passed a sign.

Wide enough to include a deliberately wide berth (the lane's own spec is
+27.86 cm on the free side) and short enough to exclude a sign in the opposite
corridor, which is metres away."""


_EARLY: list[tuple[bool, float]] = []
"""Per-run early-window control: (run had a true violation, median pose error)."""

_VIOLATIONS: list[tuple[str, str, float, bool, str]] = []
"""Per-violation detail from ``_truth_violations``: (colour, corridor, margin
onto the forbidden side in m, verdict flips under a corridor nudge, navigator
phase at closest approach)."""

_CORRIDOR_PROBE_M = 0.10
"""Nudge used to test whether a violation's verdict depends on where the corner
is drawn. See ``_verdict_is_ambiguous`` for why the naive form of this test --
asking whether the LABEL is stable -- measures nothing at all."""


_EARLY_WINDOW_TICKS = 400
"""Ticks of the run treated as the "before it diverged" window (20 s at 20 Hz).

The escape<->timeout link stayed circular for a session because a robot that
is failing escapes more BY DEFINITION, so escape counts measured at the
failure proved nothing; it broke only when the rate was measured early, before
runs diverge. Pose error and wrong-side passes are circular in exactly the same
way -- a robot that has wandered onto the wrong side of a sign is, for that
reason, somewhere it did not plan to be. Measuring pose error in a window that
precedes the approaches asks whether it PREDICTS the violation instead."""

def _to_belief(sim: object, x: float, y: float, true_x: float, true_y: float, true_yaw: float) -> tuple[float, float] | None:
    """Map a TRUE position into the frame the navigator believes it is in.

    Everything the router publishes -- its sign list, its planned polyline --
    lives in the believed frame, so comparing any of it against a true position
    mixes two frames. Blind always assumes a SOUTH start, so for the other
    three start sections the belief frame is rotated by a multiple of 90 deg
    and the gap is METRES by construction rather than by drift.

    Gating those runs out was tried first and is not the answer: the pose-error
    gate turned out to select the start section exactly (13 south in, all 31
    non-south out), so every attribution it produced spoke for a quarter of the
    corpus. Rotating instead of discarding keeps all four sections, because the
    offset is a rigid transform and both sides of the comparison move together.
    """
    believed = sim.gateway.get_current_pose()  # type: ignore[attr-defined]
    if believed is None:
        return None
    dyaw = wrap_angle(believed.yaw - true_yaw)
    cos_d, sin_d = math.cos(dyaw), math.sin(dyaw)
    dx, dy = x - true_x, y - true_y
    return (believed.x + dx * cos_d - dy * sin_d, believed.y + dx * sin_d + dy * cos_d)


@dataclass(frozen=True, slots=True)
class _BeliefSign:
    """A true sign rotated into the believed frame, for comparison against the plan."""

    x: float
    y: float
    color: object


def _side_of(sign: object, x: float, y: float, direction: Direction | None) -> bool | None:
    """Is ``(x, y)`` on the forbidden side of ``sign``? ``None`` if no rule applies.

    ``direction`` is the round's TRUE travel direction, not the router's belief.
    The pass-side rule is travel-relative, so a judge must evaluate it under the
    direction actually driven -- scoring it under the direction the robot merely
    believed is the shared-convention mistake that hid the routing bug for two
    months. ``None`` direction means the rule cannot be evaluated, not "legal".
    """
    rule = sign_router_module.pass_side_lateral_axis(
        corridor_for_position(sign.x, sign.y),  # type: ignore[attr-defined]
        sign.color,  # type: ignore[attr-defined]
        direction,
    )
    if rule is None:
        return None
    axis, permitted = rule
    robot_lat = x if axis == Axis.X else y
    sign_lat = sign.x if axis == Axis.X else sign.y  # type: ignore[attr-defined]
    if robot_lat == sign_lat:
        return False
    return (1 if robot_lat > sign_lat else -1) != permitted


def _signed_clearance(sign: object, x: float, y: float, direction: Direction | None) -> float | None:
    """Signed lateral clearance of ``(x, y)`` from ``sign``, + on the permitted side.

    The bucket label says only WHICH side the plan is on. The magnitude says what
    kind of mistake it is, and the two point at different code: a plan sitting a
    centimetre over the line is a lane that under-delivered its offset, while one
    sitting a full lane width onto the forbidden side (spec is +18.14/+27.86 cm)
    is a lane built on the wrong side to begin with.
    """
    rule = sign_router_module.pass_side_lateral_axis(
        corridor_for_position(sign.x, sign.y),  # type: ignore[attr-defined]
        sign.color,  # type: ignore[attr-defined]
        direction,
    )
    if rule is None:
        return None
    axis, permitted = rule
    robot_lat = x if axis == Axis.X else y
    sign_lat = sign.x if axis == Axis.X else sign.y  # type: ignore[attr-defined]
    return (robot_lat - sign_lat) * permitted


def _nearest_on_path(path: list, sign: object) -> tuple[float, float] | None:
    """Closest point to ``sign`` on the planned polyline, projected onto segments.

    Segments, not vertices: the planned line passes a sign between waypoints
    far more often than at one, and reading the plan at its nearest VERTEX is
    the same measure-at-an-index error that cost this investigation three
    hypotheses in a day.
    """
    if not path or len(path) < 2:
        return None
    best: tuple[float, float] | None = None
    best_d = float("inf")
    for start, end in zip(path, path[1:], strict=False):
        vx, vy = end.x - start.x, end.y - start.y
        span = vx * vx + vy * vy
        if span <= 0.0:
            continue
        t = ((sign.x - start.x) * vx + (sign.y - start.y) * vy) / span  # type: ignore[attr-defined]
        t = max(0.0, min(1.0, t))
        px, py = start.x + t * vx, start.y + t * vy
        d = math.hypot(sign.x - px, sign.y - py)  # type: ignore[attr-defined]
        if d < best_d:
            best_d, best = d, (px, py)
    return best


_SPEC_MATCH_M = 0.50
"""Radius for matching a published lane spec to a true sign, for the depth test.

Deliberately NOT the tight ``_SIGN_MATCH_M``: the quantity being measured IS the
error between the two, so a tight gate would truncate exactly the tail the test
is about -- the same trap as the pose gate that turned out to select the start
section. 0.50 m is unambiguous because two signs in one section are always
1.00 m apart (see the layout invariants in ``sign_lane``), and passes with no
spec inside it are counted separately rather than dropped silently.
"""


def _spec_depth_error(sim: object, believed_sign: object) -> tuple[float, float] | None:
    """Signed along-corridor error between the PUBLISHED lane spec and the true sign.

    ``_control_points`` centres each sign's full-offset plateau on the DISCOVERED
    sign's depth and holds it for ``SIGN_LANE_HOLD_M`` either side. So a plateau
    is only over the real sign if discovery has its depth right to within that
    half-width. This measures that directly: if the error routinely exceeds the
    hold, the lane is centred in the wrong place and the 63% "phased-off" share
    is a DISCOVERY defect, not a too-narrow plateau -- and widening the hold
    would paper over it while spending wall clearance.
    """
    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    specs = list(router.lane_specs) if router is not None else []
    if not specs:
        return None
    corridor = corridor_for_position(believed_sign.x, believed_sign.y)  # type: ignore[attr-defined]
    # The router's OWN believed direction, not the truth: these helpers measure
    # whether the planner delivered the instruction it built, so they must read
    # the rule under the direction it planned with. Judging helpers take the
    # true direction instead -- see ``_side_of``.
    rule = sign_router_module.pass_side_lateral_axis(corridor, believed_sign.color, router.direction)  # type: ignore[attr-defined]
    if rule is None:
        return None
    lateral_axis, _permitted = rule
    # Depth is the axis the lane does NOT deform: the one it holds the plateau along.
    def depth_of(x: float, y: float) -> float:
        return x if lateral_axis == Axis.Y else y

    def lateral_of(x: float, y: float) -> float:
        return y if lateral_axis == Axis.Y else x

    nearest = min(
        specs,
        key=lambda entry: math.hypot(entry[0].x - believed_sign.x, entry[0].y - believed_sign.y),  # type: ignore[attr-defined]
    )[0]
    if math.hypot(nearest.x - believed_sign.x, nearest.y - believed_sign.y) > _SPEC_MATCH_M:
        return math.inf, math.inf
    # Lateral is signed toward the PERMITTED side, so a systematic bias shows up
    # as a consistent sign rather than averaging itself away across corridors.
    depth_err = depth_of(nearest.x, nearest.y) - depth_of(believed_sign.x, believed_sign.y)  # type: ignore[attr-defined]
    lateral_err = (lateral_of(nearest.x, nearest.y) - lateral_of(believed_sign.x, believed_sign.y)) * _permitted  # type: ignore[attr-defined]
    return depth_err, lateral_err


def _plateau_stats(sim: object, believed_sign: object) -> tuple[int, int, bool, float] | None:
    """Do any waypoints inside the sign's plateau actually receive the lane shift?

    ``apply_sign_lanes`` only moves waypoints passing ``_in_lane_span``, which
    confines the lane to the corridor's straight plus ``SIGN_LANE_CORNER_ENTRY_M``.
    1211 of the corpus's 1282 signs sit at a section BOUNDARY, where the straight
    offers no near-side runway -- so the profile can hold a perfect full-offset
    plateau centred on the sign while no waypoint in the span ever samples it,
    and the offset that shows up further along is the ramp, not the plateau.

    Returns ``(in_plateau, shifted_in_plateau, clamp_binds, boundary_distance)``:
    how many waypoints fall inside the plateau at all, how many of those actually
    moved, whether the clamp binds for this sign (the FREE/CLAMPED split, which
    ran 20% against 94% delivered at sign collisions), and how far the sign sits
    from the nearest section boundary.
    """
    corridor = corridor_for_position(believed_sign.x, believed_sign.y)  # type: ignore[attr-defined]
    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    if router is None:
        return None
    # The router's OWN believed direction, not the truth: these helpers measure
    # whether the planner delivered the instruction it built, so they must read
    # the rule under the direction it planned with. Judging helpers take the
    # true direction instead -- see ``_side_of``.
    rule = sign_router_module.pass_side_lateral_axis(corridor, believed_sign.color, router.direction)  # type: ignore[attr-defined]
    if rule is None:
        return None
    lateral_axis, permitted = rule
    lane = sim.navigator._waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    base = sim.navigator._lane_base_waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    if len(lane) != len(base):
        return None

    def depth_of(wp: object) -> float:
        return wp.x if lateral_axis == Axis.Y else wp.y  # type: ignore[attr-defined]

    def lateral_of(wp: object) -> float:
        return wp.y if lateral_axis == Axis.Y else wp.x  # type: ignore[attr-defined]

    sign_depth = depth_of(believed_sign)
    in_plateau = shifted = 0
    for lane_wp, base_wp in zip(lane, base, strict=False):
        if abs(depth_of(base_wp) - sign_depth) > _HOLD_M:
            continue
        in_plateau += 1
        if abs(lateral_of(lane_wp) - lateral_of(base_wp)) > 0.01:
            shifted += 1

    sign_lateral = lateral_of(believed_sign)
    want = sign_lateral + permitted * _LANE_SPEC_FAR_M
    got = sign_router_module.clamp_lateral(want, corridor)
    boundary = min(
        abs(sign_depth - TrackDimensions.CORNER_MIN), abs(sign_depth - TrackDimensions.CORNER_MAX)
    )
    return in_plateau, shifted, abs(got - want) > 1e-3, boundary


def _shift_reference_error(sim: object, believed_sign: object) -> tuple[float, float] | None:
    """Deviation of the pass waypoint from the corridor median, against the shortfall.

    ``apply_sign_lanes`` moves each waypoint by ``target - base_lateral``, where
    ``base_lateral`` is the MEDIAN lateral of the corridor's straight -- one
    number for the whole corridor -- rather than setting the waypoint to the
    lane's absolute lateral. So a waypoint whose own lateral differs from that
    median lands short of the target by exactly that difference, and the
    waypoints abeam a sign are the deviant ones: 1211 of the corpus's 1282 signs
    sit at a section BOUNDARY, where the path is entering or leaving a corner arc.

    Returns ``(deviation, shortfall)``, both signed toward the permitted side and
    both measured independently -- ``deviation`` from the base path and the
    corridor median, ``shortfall`` from the clamped target and where the lane
    waypoint actually ended up. If the mechanism holds they agree; reporting both
    is what distinguishes a confirmed identity from an assumed one.
    """
    corridor = corridor_for_position(believed_sign.x, believed_sign.y)  # type: ignore[attr-defined]
    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    if router is None:
        return None
    # The router's OWN believed direction, not the truth: these helpers measure
    # whether the planner delivered the instruction it built, so they must read
    # the rule under the direction it planned with. Judging helpers take the
    # true direction instead -- see ``_side_of``.
    rule = sign_router_module.pass_side_lateral_axis(corridor, believed_sign.color, router.direction)  # type: ignore[attr-defined]
    if rule is None:
        return None
    lateral_axis, permitted = rule
    lane = sim.navigator._waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    base = sim.navigator._lane_base_waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    if len(lane) != len(base):
        return None

    # base_lateral exactly as apply_sign_lanes computes it: the median lateral of
    # the corridor's STRAIGHT waypoints (corner runway excluded), off the base path.
    straight = [wp for wp in base if _in_lane_span(wp, corridor, lateral_axis, 0.0)]
    if not straight:
        return None
    laterals = sorted(_axis_coords(wp, lateral_axis)[0] for wp in straight)
    base_lateral = laterals[len(laterals) // 2]

    sign_lateral, sign_depth = _axis_coords(believed_sign, lateral_axis)  # type: ignore[arg-type]
    # Only waypoints in THIS corridor's lane span are candidates. Picking the
    # nearest by depth across the whole loop instead lands on the OPPOSITE
    # corridor, which sits at the same depth on a closed track and is
    # legitimately unshifted -- that contaminated the first run of this test
    # with metre-scale deviations.
    span = [
        i for i in range(len(base)) if _in_lane_span(base[i], corridor, lateral_axis, _CORNER_ENTRY_M)
    ]
    if not span:
        return None
    nearest = min(span, key=lambda i: abs(_axis_coords(base[i], lateral_axis)[1] - sign_depth))
    wp_lateral = _axis_coords(base[nearest], lateral_axis)[0]
    lane_lateral = _axis_coords(lane[nearest], lateral_axis)[0]

    target = sign_router_module.clamp_lateral(sign_lateral + permitted * _LANE_SPEC_FAR_M, corridor)
    deviation = (base_lateral - wp_lateral) * permitted
    shortfall = (target - lane_lateral) * permitted
    return deviation, shortfall


def _spec_frame_clearance(sim: object) -> list[tuple]:
    """Clearance of the plan from each ROUTER SPEC, wholly inside the believed frame.

    Every other clearance number in this script is measured against a TRUE sign
    mapped through ``_to_belief`` and then matched to a spec by proximity. Both
    steps are instrument, and a single-pass reconciliation put the lane at 98% of
    spec where the aggregate says 12-18% -- so the aggregate has to be re-taken
    without either step before it can be trusted.

    This asks the narrower, unimpeachable question: did the lane end up where the
    router asked it to, relative to the sign the router thinks it is avoiding? No
    frame mapping, no nearest-true-sign matching, no scoring against ground truth.
    It cannot say whether the robot passed the real sign correctly -- only whether
    the planner delivered its own instruction.

    Returns ``(key, clearance, commanded, distance)`` per spec currently within a
    pass of the BELIEVED pose, keyed by rounded position so a spec keeps its
    identity while discovery refines it tick to tick.
    """
    pose = sim.gateway.get_current_pose()  # type: ignore[attr-defined]
    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    if pose is None or router is None:
        return []
    lane = sim.navigator._waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    rows: list[tuple] = []
    for spec, corridor in router.lane_specs:
        distance = math.hypot(spec.x - pose.x, spec.y - pose.y)
        if distance > _PASSED_NEAR_M:
            continue
        rule = sign_router_module.pass_side_lateral_axis(corridor, spec.color, router.direction)
        if rule is None:
            continue
        lateral_axis, permitted = rule
        point = _nearest_on_path(lane, spec)
        if point is None:
            continue
        plan_lateral = point[1] if lateral_axis == Axis.Y else point[0]
        sign_lateral = spec.y if lateral_axis == Axis.Y else spec.x
        target = sign_router_module.clamp_lateral(sign_lateral + permitted * _LANE_SPEC_FAR_M, corridor)
        # Conditioning for the tail. The wrong-side passes are a MINORITY, so an
        # aggregate over all of them reads as a base rate (that has happened to
        # every candidate this session); what is needed is what separates them.
        #
        # in_span is the leading hypothesis, straight out of section B3: the lane
        # is only applied inside _in_lane_span, so a polyline whose closest
        # approach falls OUTSIDE it -- on the corner arc -- passes the sign on
        # whatever side the untouched centreline happens to give.
        # DEPTH ONLY. _in_lane_span also tests the lateral against the corridor
        # bound, and a lane point shifted INWARD (green) legitimately crosses it,
        # so the combined predicate answered 'outside' for ~80% of ALL passes --
        # uninformative rather than discriminating. The B3 claim is purely about
        # along-track position: does the polyline come closest to the sign on a
        # segment the lane was never allowed to rewrite?
        span_low, span_high = _lane_span(_CORNER_ENTRY_M)
        plan_depth = point[0] if lateral_axis == Axis.Y else point[1]
        in_span = span_low <= plan_depth <= span_high
        boundary = min(
            abs((spec.x if lateral_axis == Axis.Y else spec.y) - TrackDimensions.CORNER_MIN),
            abs((spec.x if lateral_axis == Axis.Y else spec.y) - TrackDimensions.CORNER_MAX),
        )
        # Is the spec inside its corridor's own STRAIGHT, or past the corner?
        # `boundary` above is distance to the NEAREST bound, which conflates a
        # genuinely mid-straight sign (depth 1.5, bdist 0.5) with one sitting
        # past the corner (depth 2.39, bdist 0.39) -- almost certainly why the
        # dose-response came out non-monotonic.
        spec_depth = spec.x if lateral_axis == Axis.Y else spec.y
        inside_straight = TrackDimensions.CORNER_MIN <= spec_depth <= TrackDimensions.CORNER_MAX
        # Did SIGN_LANE_RELABEL_UNSATISFIABLE move this spec off the face
        # corridor_for_position picked? If relabelled specs are over-represented
        # among what is still wrong, the relabel is incomplete -- it takes the
        # other face when the first is unsatisfiable without checking the new
        # face is otherwise sane.
        relabelled = corridor_for_position(spec.x, spec.y) != corridor
        # Whose plateau governs the point where the plan passes THIS sign?
        # _control_points emits one plateau per spec in the corridor and
        # interpolates between them, so a sign can be passed on a stretch the
        # profile is holding for a NEIGHBOUR. That is the precise form of the
        # interleaving seen in the traced dump (plateau endpoints at 2.14 /
        # 2.22 / 2.24 for three different specs), and unlike "corridor holds
        # opposite colours" it also catches same-coloured neighbours.
        plan_depth_axis = point[0] if lateral_axis == Axis.Y else point[1]
        nearest_spec = min(
            (entry for entry in router.lane_specs if entry[1] == corridor),
            key=lambda entry: abs(
                (entry[0].x if lateral_axis == Axis.Y else entry[0].y) - plan_depth_axis
            ),
            default=None,
        )
        governed_by_other = nearest_spec is not None and (
            abs(nearest_spec[0].x - spec.x) > 1e-6 or abs(nearest_spec[0].y - spec.y) > 1e-6
        )
        # Does the CLAMPED target itself sit on the forbidden side of this spec?
        # Traced on a green WEST spec at x=0.993: the clamp caps the target at
        # 0.781, which is 0.212 m the wrong side of the sign. No lane geometry
        # can satisfy the rule for such a spec.
        target_wrong_side = (target - sign_lateral) * permitted < 0
        same_corridor = [entry for entry in router.lane_specs if entry[1] == corridor]
        opposite_colours = len({str(entry[0].color) for entry in same_corridor}) > 1
        rows.append(
            (
                (round(spec.x, 1), round(spec.y, 1)),
                (plan_lateral - sign_lateral) * permitted,
                (target - sign_lateral) * permitted,
                distance,
                in_span,
                boundary,
                len(same_corridor),
                opposite_colours,
                abs(target - (sign_lateral + permitted * _LANE_SPEC_FAR_M)) > 1e-3,
                inside_straight,
                target_wrong_side,
                relabelled,
                governed_by_other,
                len(same_corridor),
            )
        )
    return rows


def _lane_shape(sim: object, believed_sign: object) -> str:
    """Split the lane's missing offset into WHERE it went missing.

    "18% of spec delivered" covers three defects that live in different code, and
    the aggregate cannot tell them apart:

    * ``A no-spec`` -- the router never published a lane for this sign, so
      nothing downstream could apply one.
    * ``B phased-off`` -- the lane reaches most of its offset somewhere near the
      sign, just not where the robot is abeam it. ``SIGN_LANE_HOLD_M`` is 0.25 m,
      so full offset exists over a narrow plateau that the pass can miss.
    * ``C built-shallow`` -- the lane never reaches its offset anywhere near the
      sign, so it was clamped or scaled at construction.

    ``apply_sign_lanes`` returns a list of the same length and order as its input
    (see ``CoreNavigator``'s lane docstring), so lane and base are compared
    index-wise rather than by projection -- no nearest-point search to get wrong.
    """
    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    specs = list(router.lane_specs) if router is not None else []
    near_spec = any(
        math.hypot(spec.x - believed_sign.x, spec.y - believed_sign.y) <= _SIGN_MATCH_M  # type: ignore[attr-defined]
        for spec, _corridor in specs
    )
    if not near_spec:
        return "A no-spec"
    lane = sim.navigator._waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    base = sim.navigator._lane_base_waypoints  # type: ignore[attr-defined]  # noqa: SLF001
    if len(lane) != len(base):
        return "unknown"
    peak = 0.0
    for lane_wp, base_wp in zip(lane, base, strict=False):
        if math.hypot(base_wp.x - believed_sign.x, base_wp.y - believed_sign.y) <= _PASSED_NEAR_M:  # type: ignore[attr-defined]
            peak = max(peak, math.hypot(lane_wp.x - base_wp.x, lane_wp.y - base_wp.y))
    if peak < _LANE_SPEC_FAR_M / 2:
        return "C built-shallow"
    return "B phased-off"


def _attribute(
    sim: object, sign: object, chassis_wrong: bool, true_pose: tuple[float, float, float],
    direction: Direction | None,
) -> tuple[str, float | None, float | None, str | None, tuple | None, tuple | None, tuple | None]:
    """Bucket one true violation: whose mistake was it?

    Splits the plan from the chassis. If the PLANNED line was already on the
    forbidden side, the robot drove where it meant to and the fault is in
    routing -- either the colour it believed or the lane it built from it. If
    the plan was correct and only the chassis ended up wrong, it is tracking.

    Returns the bucket alongside the plan's own signed clearance from the sign
    (+ on the permitted side), which separates an inverted lane from an
    under-delivered one -- see ``_signed_clearance``.
    """
    moved = _to_belief(sim, sign.x, sign.y, *true_pose)  # type: ignore[attr-defined]
    if moved is None:
        return "no-pose", None, None, None, None, None, None
    believed_sign = _BeliefSign(moved[0], moved[1], sign.color)  # type: ignore[attr-defined]
    plan_point = _nearest_on_path(sim.navigator._waypoints, believed_sign)  # type: ignore[attr-defined]  # noqa: SLF001
    if plan_point is None:
        return "no-plan", None, None, None, None, None, None
    plan_wrong = _side_of(believed_sign, *plan_point, direction)
    if plan_wrong is None:
        return "no-rule", None, None, None, None, None, None
    plan_clearance = _signed_clearance(believed_sign, *plan_point, direction)
    # How much of the lane's specified offset actually reaches the pass. The
    # base path is the same polyline before apply_sign_lanes moved it sideways,
    # so the difference is the delivered offset -- against a spec of 0.2786 m at
    # the shipped SIGN_LANE_OFFSET_FRAC.
    base_point = _nearest_on_path(sim.navigator._lane_base_waypoints, believed_sign)  # type: ignore[attr-defined]  # noqa: SLF001
    base_clearance = None if base_point is None else _signed_clearance(believed_sign, *base_point, direction)
    delivered = None if base_clearance is None or plan_clearance is None else plan_clearance - base_clearance
    lane_shape = _lane_shape(sim, believed_sign)
    depth_error = _spec_depth_error(sim, believed_sign)
    plateau = _plateau_stats(sim, believed_sign)
    shift_ref = _shift_reference_error(sim, believed_sign)
    if not plan_wrong:
        return ("plan-ok/chassis-wrong (tracking)" if chassis_wrong else "plan-ok"), plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref

    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    routed = list(router.signs) if router is not None else []
    if not routed:
        return "plan-wrong/never-routed", plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref
    nearest = min(routed, key=lambda r: math.hypot(r.x - believed_sign.x, r.y - believed_sign.y))
    if math.hypot(nearest.x - believed_sign.x, nearest.y - believed_sign.y) > _SIGN_MATCH_M:
        return "plan-wrong/never-routed", plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref
    if str(nearest.color) != str(sign.color):  # type: ignore[attr-defined]
        return "plan-wrong/colour-misread", plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref
    return "plan-wrong/colour-ok (routing)", plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref


_SIGN_MATCH_M = 0.25
"""How close a routed sign must be to a true one to count as the same sign."""


_LANE_SPEC_NEAR_M = 0.1814
"""Near edge of the sign-lane spec (+18.14/+27.86 cm off the sign).

A plan sitting further onto the forbidden side than this is on the wrong side by
a full lane width -- a lane built the wrong way round rather than one that fell
short of its offset."""


_CORNER_ENTRY_M = NavigationTuning.load_default().sign_router.SIGN_LANE_CORNER_ENTRY_M
"""How far past the straight the lane may reach, from the shipped tuning."""


_HOLD_M = NavigationTuning.load_default().sign_router.SIGN_LANE_HOLD_M
"""Plateau half-width the lane holds full offset over, from the shipped tuning."""


_LANE_SPEC_FAR_M = 0.2786
"""Lateral offset the lane planner asks for at the shipped config.

Read back from ``NavigationTuning.load_default()`` on 2026-08-25 as
``(chassis_half_diagonal + sign_width/2 + SIGN_CLEARANCE_MARGIN_M) *
SIGN_LANE_OFFSET_FRAC`` with ``SIGN_LANE_PLANNER`` True -- restated here only as
the denominator of the delivery ratio."""


def _verdict_is_ambiguous(sign: object, near_x: float, near_y: float, direction: Direction | None) -> bool:
    """Would nudging the sign's corridor label change the wrong-side VERDICT?

    Label instability on its own means nothing: 94.9% of corpus signs sit
    within 10 cm of some corridor boundary, so "is the label unstable" is not
    a selective question and the first version of this check, which asked
    exactly that, returned 100% for violations against a 94.9% base rate --
    a control that was missing until it was measured.

    What matters is whether the instability reaches the answer. A south<->west
    flip also flips the comparison AXIS (``ROUTING_TABLE`` gives N/S the Y
    axis and E/W the X axis), so it genuinely can. A violation whose verdict
    survives every neighbouring label is one the scorer can be trusted on
    regardless of where the corner is drawn.
    """
    verdicts = set()
    for dx, dy in ((0.0, 0.0), (_CORRIDOR_PROBE_M, 0.0), (-_CORRIDOR_PROBE_M, 0.0), (0.0, _CORRIDOR_PROBE_M), (0.0, -_CORRIDOR_PROBE_M)):
        rule = sign_router_module.pass_side_lateral_axis(
            corridor_for_position(sign.x + dx, sign.y + dy),  # type: ignore[attr-defined]
            sign.color,  # type: ignore[attr-defined]
            direction,
        )
        if rule is None:
            continue
        axis, permitted = rule
        robot_lat = near_x if axis == Axis.X else near_y
        sign_lat = sign.x if axis == Axis.X else sign.y  # type: ignore[attr-defined]
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        verdicts.add(side != 0 and side != permitted)
    return len(verdicts) > 1


def _truth_violations(
    metadata: ScenarioMetadata,
    trail: list[tuple[float, float]],
    phases: list[str],
) -> tuple[int, int]:
    """Score every sign the way a judge would: true layout, true trajectory.

    Deliberately consults NOTHING the robot believes. Matching a retirement to
    a true sign by proximity was tried first and is not sound -- the belief
    frame is offset, so the nearest true sign to the believed pose need not be
    the sign that was retired, and the resulting colour-agreement rate came out
    near chance, which is what a random matching looks like rather than a
    finding.

    Instead each TRUE sign is scored at the robot's closest approach along its
    TRUE path -- the point at which the choice of side is actually made, per
    the standing method lesson that a pass must be measured on the polyline
    and not at an index. Signs the run never reached are not scored.

    Returns ``(passed, wrong_side)`` counts for this scenario.
    """
    signs = sign_router_module.signs_from_metadata(metadata)
    if not signs or not trail:
        return 0, 0
    passed = wrong = 0
    for sign in signs:
        near_i = min(range(len(trail)), key=lambda i: math.hypot(sign.x - trail[i][0], sign.y - trail[i][1]))
        near_x, near_y = trail[near_i]
        if math.hypot(sign.x - near_x, sign.y - near_y) > _PASSED_NEAR_M:
            continue
        rule = sign_router_module.pass_side_lateral_axis(
            corridor_for_position(sign.x, sign.y), sign.color, metadata.starting_conditions.direction
        )
        if rule is None:
            continue
        axis, permitted = rule
        robot_lat = near_x if axis == Axis.X else near_y
        sign_lat = sign.x if axis == Axis.X else sign.y
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        passed += 1
        if side != 0 and side != permitted:
            wrong += 1
            # Margin: how far onto the forbidden side, and whether the corridor
            # this verdict was read through is stable. A violation that is both
            # marginal AND corridor-ambiguous is one the scorer should not be
            # trusted on.
            _VIOLATIONS.append(
                (
                    str(sign.color),
                    str(corridor_for_position(sign.x, sign.y)),
                    abs(robot_lat - sign_lat),
                    _verdict_is_ambiguous(sign, near_x, near_y, metadata.starting_conditions.direction),
                    phases[near_i] if near_i < len(phases) else "unknown",
                )
            )
    return passed, wrong


def _instrument(true_pose: list[tuple[float, float]]) -> list[tuple[float, bool, float, float, str]]:
    """Patch ``SignRouter`` so every retirement records its along-track geometry.

    ``true_pose`` is a one-element holder the caller refreshes each tick with
    the simulator's ground-truth position, so each retirement can be scored
    against the layout as well as against the robot's belief.

    Returns the list the patched methods append to: one entry per retirement,
    ``(along_track_m, scored_violation, true_x, true_y, believed_colour)``.
    """
    records: list[tuple[float, bool, float, float, str]] = []
    router_cls = sign_router_module.SignRouter
    original_candidates = router_cls._active_sign_candidates  # noqa: SLF001
    original_record = router_cls._record_pass_side  # noqa: SLF001

    def candidates(self, robot_pos, robot_yaw, corridor):  # type: ignore[no-untyped-def]
        # The yaw the retirement happened under is not passed to
        # _record_pass_side, so latch it on the way past.
        self._diag_yaw = robot_yaw  # noqa: SLF001
        return original_candidates(self, robot_pos, robot_yaw, corridor)

    def record(self, index, robot_pos):  # type: ignore[no-untyped-def]
        before = index in self._wrong_side  # noqa: SLF001
        original_record(self, index, robot_pos)
        after = index in self._wrong_side  # noqa: SLF001
        sign = self._signs[index]  # noqa: SLF001
        yaw = getattr(self, "_diag_yaw", 0.0)
        dx, dy = sign.x - robot_pos.x, sign.y - robot_pos.y
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        tx, ty = true_pose[0]
        records.append((along, after and not before, tx, ty, str(sign.color)))

    router_cls._active_sign_candidates = candidates  # type: ignore[assignment]  # noqa: SLF001
    router_cls._record_pass_side = record  # type: ignore[assignment]  # noqa: SLF001
    return records


def _disable_termination() -> None:
    """Stop the simulator ending a run on the router's pass-side verdict.

    Without this the ground-truth rate is measured on trajectories the verdict
    itself truncated -- the run stops at the first believed violation, so the
    signs counted are only the ones reached before it, and mostly the earliest
    ones. Letting the run continue measures the rate over the whole three laps
    the challenge actually scores.
    """
    ScenarioSimulator._check_pass_side_violation = lambda self, nav: None  # type: ignore[assignment]  # noqa: ARG005, SLF001


def _run_one(args_tuple: tuple[str, bool, bool]) -> tuple[Counter[str], list[float], list, list, list, list, list, list, list, list, list, list]:
    """Run one scenario and cross-tabulate each retirement's two verdicts.

    Returns ``(counts, alongs, colour_disagreements)`` where ``counts`` keys
    are ``"<router>/<truth>"`` over ``ok``/``wrong``, plus ``retreat`` and
    ``pass`` for the along-track split.
    """
    path_str, no_terminate, known_start = args_tuple
    logging.disable(logging.CRITICAL)
    if no_terminate:
        _disable_termination()
    true_pose = [(0.0, 0.0)]
    records = _instrument(true_pose)
    _VIOLATIONS.clear()
    _EARLY.clear()
    trail: list[tuple[float, float]] = []
    phases: list[str] = []
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path_str).read_text()))
    # The round's TRUE direction. Every wrong-side verdict below is scored against
    # this, never against the router's belief -- the scorer must not share its
    # convention with the thing it scores.
    true_direction = metadata.starting_conditions.direction
    sim = ScenarioSimulator(metadata, num_laps=3, seed=0, blind=True, known_start=known_start)

    signs = sign_router_module.signs_from_metadata(metadata)
    # Per sign: the closest approach seen so far, and what the navigator was
    # doing AT that tick. Attribution has to be taken live -- the plan and the
    # router's sign list are rebuilt continuously, so nothing about the moment
    # of the pass survives to the end of the run to be read off afterwards.
    best: dict[int, tuple[float, str, float, float | None, float | None, str | None, tuple | None, tuple | None]] = {}
    # Spec hygiene, sampled at the tick the router holds the most lane specs.
    # Duplication and corridor mislabelling are INTERNAL to the believed frame,
    # so unlike a comparison against true positions they are not confounded by
    # blind's frame rotation.
    hygiene = {"peak": 0, "mislabelled": 0}
    # Believed-frame delivery, keyed per spec at its own closest approach.
    spec_frame: dict[tuple[float, float], tuple[float, float, float]] = {}
    early_pose_errors: list[float] = []

    def _on_step(state, _scan) -> None:  # type: ignore[no-untyped-def]
        true_pose[0] = (state.x, state.y)
        router_now = sim.navigator.sign_router
        specs_now = list(router_now.lane_specs) if router_now is not None else []
        for row in _spec_frame_clearance(sim):
            previous = spec_frame.get(row[0])
            if previous is None or row[3] < previous[3]:
                spec_frame[row[0]] = row
        if len(specs_now) > hygiene["peak"]:
            hygiene["peak"] = len(specs_now)
            hygiene["mislabelled"] = sum(
                1 for spec, label in specs_now if corridor_for_position(spec.x, spec.y) != label
            )
        trail.append((state.x, state.y))
        phases.append(str(getattr(sim.navigator.debug_snapshot, "phase", "unknown")))
        if len(trail) <= _EARLY_WINDOW_TICKS:
            early = sim.gateway.get_current_pose()
            if early is not None:
                early_pose_errors.append(math.hypot(early.x - state.x, early.y - state.y))
        for index, sign in enumerate(signs):
            distance = math.hypot(sign.x - state.x, sign.y - state.y)
            # Only near a pass, so the per-tick cost stays off the hot path.
            if distance > _PASSED_NEAR_M or distance >= best.get(index, (math.inf,))[0]:
                continue
            believed = sim.gateway.get_current_pose()
            pose_error = math.inf if believed is None else math.hypot(believed.x - state.x, believed.y - state.y)
            chassis_wrong = bool(_side_of(sign, state.x, state.y, true_direction))
            bucket, plan_clearance, delivered, lane_shape, depth_error, plateau, shift_ref = _attribute(
                sim, sign, chassis_wrong, (state.x, state.y, state.yaw), true_direction
            )
            best[index] = (
                distance, bucket, pose_error, plan_clearance, delivered, lane_shape, depth_error,
                plateau, shift_ref,
            )

    sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=_on_step)

    counts: Counter[str] = Counter()
    alongs: list[float] = []
    for along, violation, _tx, _ty, _colour in records:
        alongs.append(along)
        counts["retreat" if along > _BEHIND_EPS_M else "pass"] += 1
        counts["router_wrong"] += int(violation)
    truth_passed, truth_wrong = _truth_violations(metadata, trail, phases)
    counts["truth_passed"] += truth_passed
    counts["truth_wrong"] += truth_wrong
    counts["run_ended_by_router"] += int(any(v for _a, v, *_ in records))
    counts["run_truly_violated"] += int(truth_wrong > 0)
    if early_pose_errors:
        _EARLY.append((truth_wrong > 0, statistics.median(early_pose_errors)))
    hygiene_row = (len(signs), hygiene["peak"], hygiene["mislabelled"], truth_wrong > 0)
    spec_rows = [row[1:] for row in spec_frame.values()]

    # Attribution, restricted to passes where the believed frame nearly
    # coincides with the true one. Outside that gate the router's own output
    # cannot be matched to a true sign at all (see _POSE_GATE_M).
    plan_clearances: list[tuple[str, float]] = []
    deliveries: list[tuple[str, float]] = []
    shapes: list[tuple[str, str]] = []
    depth_errors: list[tuple[str, float, float]] = []
    plateaux: list[tuple[int, int, bool, float, float]] = []
    shift_refs: list[tuple[float, float]] = []
    for index, (
        _distance, bucket, pose_error, plan_clearance, delivered, lane_shape, depth_error,
        plateau, shift_ref,
    ) in best.items():
        near_x, near_y = min(trail, key=lambda p: math.hypot(signs[index].x - p[0], signs[index].y - p[1]))
        violated = bool(_side_of(signs[index], near_x, near_y, true_direction))
        # Delivery is recorded for CLEAN passes too. Attribution runs only on
        # violations, so measuring delivery there alone asks whether the runs
        # that went wrong had a weak lane -- which is the circularity that kept
        # the escape<->timeout link alive for a session. The clean column is
        # what turns "7% of spec" into a finding or a base rate.
        if delivered is not None:
            deliveries.append((f"{'VIOLATION' if violated else 'clean pass'}: {bucket}", delivered))
        if lane_shape is not None:
            shapes.append((f"{'VIOLATION' if violated else 'clean pass'}", lane_shape))
        if depth_error is not None:
            depth_errors.append((lane_shape or "unknown", depth_error[0], depth_error[1]))
        if plateau is not None and delivered is not None:
            plateaux.append((*plateau, delivered))
        if shift_ref is not None:
            shift_refs.append(shift_ref)
        if not violated:
            continue
        counts["attributed_total"] += 1
        # Which start sections survive the gate? Blind always assumes a SOUTH
        # start, so for the other three the belief frame is rotated by a
        # multiple of 90 deg and the believed-vs-true gap is metres by
        # construction, not by drift. If the gate is really just selecting
        # South starts then every bucket below speaks for one quarter of the
        # corpus, and saying so is the difference between a result and a
        # sampling artifact.
        start = str(corridor_for_position(*trail[0])) if trail else "unknown"
        counts[f"start::{start}"] += 1
        counts[f"bucket::{bucket}"] += 1
        if plan_clearance is not None:
            plan_clearances.append((bucket, plan_clearance))
    return counts, alongs, list(_VIOLATIONS), list(_EARLY), plan_clearances, deliveries, shapes, depth_errors, [hygiene_row], plateaux, shift_refs, spec_rows




def _report_retirement_summary(
    paths: list, known_start: bool, counts: Counter[str], alongs: list[float]
) -> None:
    """Print the top-line retirement/along-track summary."""
    total = counts["retreat"] + counts["pass"]
    arm = "blind+known_start" if known_start else "blind"
    print(f"scenarios {len(paths)}  retirements {total}  arm {arm}")
    print(f"  ENV {environment()}")
    print(f"  along-track: RETREAT (sign ahead) {counts['retreat']:>4}   PASS (sign behind) {counts['pass']:>4}")
    if alongs:
        print(f"    median {statistics.median(alongs):+.2f} m")
    print(f"  ROUTER (believed frame, discovered colour): {counts['router_wrong']:>4} wrong-side of {total} retirements")
    print(f"  TRUTH  (true layout, true trajectory):      {counts['truth_wrong']:>4} wrong-side of {counts['truth_passed']} signs actually passed")
    print("  per-run:")
    print(f"    runs the router ended on pass-side  {counts['run_ended_by_router']:>4}/{len(paths)}")
    print(f"    runs with a REAL wrong-side pass    {counts['run_truly_violated']:>4}/{len(paths)}")


def _report_violation_detail(violations: list) -> None:
    """Print margin/ambiguity/phase detail for every scored violation."""
    if not violations:
        return
    ambiguous = [v for v in violations if v[3]]
    margins = sorted(v[2] for v in violations)
    print(f"  violation detail ({len(violations)}):")
    print(f"    VERDICT flips under a {_CORRIDOR_PROBE_M:.2f} m corridor nudge  {len(ambiguous)}  ({len(ambiguous) / len(violations):.1%})")
    print(f"    margin onto forbidden side: p10 {margins[len(margins) // 10]:.3f} m  median {statistics.median(margins):.3f} m  p90 {margins[-max(len(margins) // 10, 1)]:.3f} m")
    print(f"    marginal (<0.05 m) {sum(1 for m in margins if m < 0.05)}   by corridor {dict(Counter(v[1] for v in violations))}   by colour {dict(Counter(v[0] for v in violations))}")
    print(f"    phase at closest approach {dict(Counter(v[4] for v in violations).most_common())}")


def _report_attribution(counts: Counter[str]) -> None:
    """Print the attribution-bucket breakdown and the by-start-section split."""
    buckets = {k[len("bucket::"):]: v for k, v in counts.items() if k.startswith("bucket::")}
    print(f"  attribution (violations {counts['attributed_total']}, all start sections, belief-frame matched):")
    for name, count in sorted(buckets.items(), key=lambda kv: -kv[1]):
        print(f"    {name:<38} {count:>4}")
    by_start = {k[len("start::"):]: v for k, v in counts.items() if k.startswith("start::")}
    if by_start:
        print(f"    violations by start section {dict(sorted(by_start.items()))}")


def _report_plan_clearance(plan_clearances: list[tuple[str, float]]) -> None:
    """Print PLAN clearance at the pass, by attribution bucket."""
    if not plan_clearances:
        return
    print("  PLAN clearance at the pass (+ = permitted side), by bucket:")
    for name in sorted({b for b, _ in plan_clearances}):
        values = sorted(c for b, c in plan_clearances if b == name)
        inverted = sum(1 for c in values if c <= -_LANE_SPEC_NEAR_M)
        print(
            f"    {name:<38} n={len(values):>4}  p10 {values[len(values) // 10]:+.3f}  "
            f"median {statistics.median(values):+.3f}  p90 {values[-max(len(values) // 10, 1)]:+.3f}  "
            f"beyond -{_LANE_SPEC_NEAR_M:.2f} m {inverted}"
        )


def _report_lane_delivery(deliveries: list[tuple[str, float]]) -> None:
    """Print LANE DELIVERY (lane path minus base path) by outcome/bucket."""
    if not deliveries:
        return
    print(f"  LANE DELIVERY at the pass (lane path minus base path, spec {_LANE_SPEC_FAR_M:.4f} m):")
    for name in sorted({b for b, _ in deliveries}):
        values = sorted(d for b, d in deliveries if b == name)
        never = sum(1 for d in values if abs(d) < 0.01)
        print(
            f"    {name:<38} n={len(values):>4}  median {statistics.median(values):+.3f}  "
            f"({statistics.median(values) / _LANE_SPEC_FAR_M:>4.0%} of spec)  never-applied (<1 cm) {never}"
        )


def _report_lane_shape(
    shapes: list[tuple[str, str]], depth_errors: list[tuple[str, float, float]]
) -> None:
    """Print WHERE THE OFFSET GOES and the LANE CENTRING depth/lateral errors."""
    if shapes:
        print("  WHERE THE OFFSET GOES (A no lane published / B lane peaks but not at the pass / C lane built shallow):")
        for outcome in sorted({o for o, _ in shapes}):
            tally = Counter(sh for o, sh in shapes if o == outcome)
            total_shape = sum(tally.values())
            detail = "  ".join(f"{k} {v:>4} ({v / total_shape:.0%})" for k, v in tally.most_common())
            print(f"    {outcome:<12} n={total_shape:>4}   {detail}")
    if depth_errors:
        finite = [d for _shape, d, _lat in depth_errors if math.isfinite(d)]
        unmatched = sum(1 for _shape, d, _lat in depth_errors if not math.isfinite(d))
        print(
            f"  LANE CENTRING: published spec depth minus TRUE sign depth, against the "
            f"{_HOLD_M:.2f} m plateau half-width:"
        )
        if finite:
            magnitudes = sorted(abs(d) for d in finite)
            beyond = sum(1 for m in magnitudes if m > _HOLD_M)
            print(
                f"    n={len(finite)}  median |err| {statistics.median(magnitudes):.3f} m  "
                f"p90 {magnitudes[-max(len(magnitudes) // 10, 1)]:.3f} m  "
                f"BEYOND the hold {beyond} ({beyond / len(magnitudes):.0%})"
            )
            for shape in sorted({sh for sh, d, _lat in depth_errors if math.isfinite(d)}):
                values = sorted(abs(d) for sh, d, _lat in depth_errors if sh == shape and math.isfinite(d))
                past = sum(1 for m in values if m > _HOLD_M)
                print(
                    f"    {shape:<18} n={len(values):>4}  median |err| {statistics.median(values):.3f} m  "
                    f"beyond hold {past} ({past / len(values):.0%})"
                )
        print(f"    no spec within {_SPEC_MATCH_M:.2f} m of the true sign: {unmatched}")
    laterals = sorted(lat for _sh, _d, lat in depth_errors if math.isfinite(lat))
    if laterals:
        print("  LANE CENTRING, LATERAL axis (spec minus true, + = toward the PERMITTED side):")
        print(
            f"    n={len(laterals)}  p10 {laterals[len(laterals) // 10]:+.3f}  "
            f"median {statistics.median(laterals):+.3f}  p90 {laterals[-max(len(laterals) // 10, 1)]:+.3f}"
        )
        print(
            f"    a consistent + median means the lane is built off a sign the router believes is "
            f"further toward the permitted side than it truly is, which eats the commanded clearance"
        )


def _report_spec_hygiene(hygiene_rows: list) -> None:
    """Print router spec hygiene: peak specs held and mislabelling rate."""
    if not hygiene_rows:
        return
    ratios = sorted(peak / true_n for true_n, peak, _bad, _v in hygiene_rows if true_n)
    dirty = [row for row in hygiene_rows if row[2] > 0]
    print("  ROUTER SPEC HYGIENE (peak lane specs held, against the true sign count):")
    print(
        f"    specs per true sign: median {statistics.median(ratios):.2f}x  "
        f"p90 {ratios[-max(len(ratios) // 10, 1)]:.2f}x  runs at >1.5x {sum(1 for r in ratios if r > 1.5)}/{len(ratios)}"
    )
    print(
        f"    runs holding a spec whose settled corridor disagrees with its own believed "
        f"position: {len(dirty)}/{len(hygiene_rows)}"
    )
    # Prevalence first, then whether it SELECTS failures -- the base-rate
    # check every other candidate this session has failed.
    if dirty and len(dirty) < len(hygiene_rows):
        clean_rows = [row for row in hygiene_rows if row[2] == 0]
        bad_rate = sum(1 for row in dirty if row[3]) / len(dirty)
        clean_rate = sum(1 for row in clean_rows if row[3]) / len(clean_rows)
        print(
            f"    runs with a REAL violation: {bad_rate:.0%} of mislabelled runs (n={len(dirty)}) "
            f"vs {clean_rate:.0%} of clean ones (n={len(clean_rows)})"
        )


def _report_plateau_coverage(plateaux: list) -> None:
    """Print plateau coverage: waypoints inside the hold that actually shifted."""
    if not plateaux:
        return
    print(f"  PLATEAU COVERAGE (waypoints inside the +/-{_HOLD_M:.2f} m plateau that actually received the shift):")
    for label, want_clamped in (("CLAMPED (18.14cm)", True), ("FREE (27.86cm)", False)):
        rows = [row for row in plateaux if row[2] is want_clamped]
        if not rows:
            continue
        present = sorted(row[0] for row in rows)
        moved = sorted(row[1] for row in rows)
        none_moved = sum(1 for row in rows if row[1] == 0)
        empty = sum(1 for row in rows if row[0] == 0)
        delivered = sorted(row[4] for row in rows)
        boundary = sorted(row[3] for row in rows)
        print(
            f"    {label:<18} n={len(rows):>4}  in-plateau median {statistics.median(present):.0f}  "
            f"shifted median {statistics.median(moved):.0f}  ZERO shifted {none_moved} ({none_moved / len(rows):.0%})  "
            f"empty plateau {empty} ({empty / len(rows):.0%})"
        )
        print(
            f"    {'':<18}       delivered median {statistics.median(delivered):+.3f} m  "
            f"({statistics.median(delivered) / _LANE_SPEC_FAR_M:.0%} of spec)  "
            f"sign-to-boundary median {statistics.median(boundary):.2f} m"
        )


def _report_shift_reference(shift_refs: list[tuple[float, float]]) -> None:
    """Print the shift-reference identity check: median-based deviation vs shortfall."""
    if not shift_refs:
        return
    deviations = sorted(d for d, _sf in shift_refs)
    shortfalls = sorted(sf for _d, sf in shift_refs)
    agree = sum(1 for d, sf in shift_refs if abs(d - sf) < 0.02)
    print("  SHIFT REFERENCE (the lane moves each waypoint by target-minus-CORRIDOR-MEDIAN, not to the target):")
    print(
        f"    n={len(shift_refs)}  waypoint deviation from the median: median {statistics.median(deviations):+.3f} m  "
        f"p90 {deviations[-max(len(deviations) // 10, 1)]:+.3f} m"
    )
    print(
        f"    shortfall of the lane from its own target:  median {statistics.median(shortfalls):+.3f} m  "
        f"p90 {shortfalls[-max(len(shortfalls) // 10, 1)]:+.3f} m"
    )
    print(
        f"    the two agree within 2 cm on {agree}/{len(shift_refs)} ({agree / len(shift_refs):.0%}) "
        f"-- measured independently, so agreement is the mechanism, not an identity assumed"
    )


def _report_believed_delivery(spec_rows: list) -> None:
    """Print believed-frame delivery: plan vs the router's own published spec."""
    if not spec_rows:
        return
    clearances = sorted(row[0] for row in spec_rows)
    ratios = sorted(row[0] / row[1] for row in spec_rows if row[1] > 1e-6)
    print("  BELIEVED-FRAME DELIVERY (plan vs the ROUTER'S OWN spec -- no frame mapping, no true-sign matching):")
    print(
        f"    n={len(clearances)}  clearance p10 {clearances[len(clearances) // 10]:+.3f}  "
        f"median {statistics.median(clearances):+.3f}  p90 {clearances[-max(len(clearances) // 10, 1)]:+.3f} m"
    )
    if ratios:
        print(
            f"    as a fraction of what the router COMMANDED: median {statistics.median(ratios):.0%}  "
            f"below 50% {sum(1 for r in ratios if r < 0.5)}/{len(ratios)}  "
            f"on the WRONG side {sum(1 for r in ratios if r < 0)}/{len(ratios)}"
        )
    print("    this asks only whether the planner delivered its OWN instruction, not whether it was right")


def _report_tail_conditioning(spec_rows: list) -> None:
    """Print the tail-conditioning comparison between wrong-side and correct specs."""
    if not spec_rows:
        return
    wrong = [row for row in spec_rows if row[0] < 0]
    right = [row for row in spec_rows if row[0] >= 0]
    if not (wrong and right):
        return
    print(f"  TAIL CONDITIONING ({len(wrong)} wrong-side of its own spec vs {len(right)} correct):")
    for name, index in (("plan point INSIDE the lane span (depth)", 3), ("clamp binds", 7), ("corridor holds opposite colours", 6),
        ("spec INSIDE the corridor straight", 8),
        ("clamped TARGET on the forbidden side", 9),
        ("spec was RELABELLED", 10),
        ("plan governed by ANOTHER spec's plateau", 11)):
        wrong_rate = sum(1 for row in wrong if row[index]) / len(wrong)
        right_rate = sum(1 for row in right if row[index]) / len(right)
        lift = wrong_rate / right_rate if right_rate else float("inf")
        print(f"    {name:<34} wrong {wrong_rate:>5.0%}  correct {right_rate:>5.0%}  lift {lift:>4.1f}x")
    wrong_boundary = statistics.median(row[4] for row in wrong)
    right_boundary = statistics.median(row[4] for row in right)
    print(f"    {'sign-to-boundary median (m)':<34} wrong {wrong_boundary:>5.2f}  correct {right_boundary:>5.2f}")
    # Dose-response, not a median split: a 0.40-vs-0.06 m median gap can
    # be produced by one over-represented cluster. If the rate climbs
    # with distance from the boundary the population is real.
    for name, index in (("relabelled", 10),):
        sub = [row for row in spec_rows if row[index]]
        if sub:
            bad = sum(1 for row in sub if row[0] < 0)
            print(f"    {name + ' specs':<34} n={len(sub):>5}  wrong {bad:>4} ({bad / len(sub):>5.0%})")
        rest = [row for row in spec_rows if not row[index]]
        if rest:
            bad = sum(1 for row in rest if row[0] < 0)
            print(f"    {'not ' + name:<34} n={len(rest):>5}  wrong {bad:>4} ({bad / len(rest):>5.0%})")
    print("    wrong-side rate by specs held in the corridor:")
    for count in sorted({row[12] for row in spec_rows}):
        sub = [row for row in spec_rows if row[12] == count]
        bad = sum(1 for row in sub if row[0] < 0)
        print(f"      {count} spec(s)   n={len(sub):>5}  wrong {bad:>4} ({bad / len(sub):>5.0%})")
    inside = [row for row in spec_rows if row[8]]
    outside = [row for row in spec_rows if not row[8]]
    for name, rows in (("spec inside the straight", inside), ("spec PAST the corner", outside)):
        if rows:
            bad = sum(1 for row in rows if row[0] < 0)
            print(f"    {name:<34} n={len(rows):>5}  wrong {bad:>4} ({bad / len(rows):>5.0%})")
    print("    wrong-side rate by sign-to-boundary distance:")
    buckets = ((0.0, 0.10), (0.10, 0.25), (0.25, 0.50), (0.50, 9.9))
    for low, high in buckets:
        inside = [row for row in spec_rows if low <= row[4] < high]
        if not inside:
            continue
        bad = sum(1 for row in inside if row[0] < 0)
        print(
            f"      {low:.2f}-{high if high < 9 else float('inf'):.2f} m  n={len(inside):>5}  "
            f"wrong {bad:>4} ({bad / len(inside):>5.0%})"
        )


def _report_early_window(early: list[tuple[bool, float]]) -> None:
    """Print the early-window pose-error control comparing violated vs clean runs."""
    offenders = [e for hit, e in early if hit]
    clean = [e for hit, e in early if not hit]
    if offenders and clean:
        print(f"  EARLY-WINDOW CONTROL (median pose error over the first {_EARLY_WINDOW_TICKS} ticks, before the approaches):")
        print(f"    runs that later violated  n={len(offenders):>3}  median {statistics.median(offenders):.3f} m")
        print(f"    runs that never violated  n={len(clean):>3}  median {statistics.median(clean):.3f} m")


def main() -> None:
    """Aggregate retirement geometry across a scenario directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios-dir", required=True)
    parser.add_argument("--limit", type=int, default=64)
    parser.add_argument("--workers", type=int, default=14)
    parser.add_argument(
        "--no-terminate",
        action="store_true",
        help="let runs continue past a believed pass-side violation (untruncated truth rate)",
    )
    parser.add_argument(
        "--known-start",
        action="store_true",
        help="seed the believed pose from the true start, keeping every other blind handicap",
    )
    args = parser.parse_args()

    paths = sorted(Path(args.scenarios_dir).glob("*_metadata.json"))[: args.limit]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_run_one, [(str(p), args.no_terminate, args.known_start) for p in paths]))

    counts: Counter[str] = Counter()
    for result in results:
        counts.update(result[0])
    alongs = [a for r in results for a in r[1]]
    violations = [v for r in results for v in r[2]]
    early = [e for r in results for e in r[3]]
    plan_clearances = [c for r in results for c in r[4]]
    deliveries = [d for r in results for d in r[5]]
    shapes = [sh for r in results for sh in r[6]]
    depth_errors = [d for r in results for d in r[7]]
    hygiene_rows = [h for r in results for h in r[8]]
    plateaux = [pl for r in results for pl in r[9]]
    shift_refs = [sr for r in results for sr in r[10]]
    spec_rows = [sr for r in results for sr in r[11]]

    _report_retirement_summary(paths, args.known_start, counts, alongs)
    _report_violation_detail(violations)
    _report_attribution(counts)
    _report_plan_clearance(plan_clearances)
    _report_lane_delivery(deliveries)
    _report_lane_shape(shapes, depth_errors)
    _report_spec_hygiene(hygiene_rows)
    _report_plateau_coverage(plateaux)
    _report_shift_reference(shift_refs)
    _report_believed_delivery(spec_rows)
    _report_tail_conditioning(spec_rows)
    _report_early_window(early)


if __name__ == "__main__":
    main()
