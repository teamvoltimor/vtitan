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
import os
import statistics
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Axis
from src.navigation.planning import sign_router as sign_router_module
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.utils import wrap_angle
from src.simulation.scenario_simulator import ScenarioSimulator

# After the simulator: importing shared.domain.models first hits the
# models<->enums cycle that enums.py resolves by deferring its re-export.
from shared.domain.models import ScenarioMetadata  # noqa: E402

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


def _side_of(sign: object, x: float, y: float) -> bool | None:
    """Is ``(x, y)`` on the forbidden side of ``sign``? ``None`` if no rule applies."""
    rule = sign_router_module.outward_lateral_axis(
        corridor_for_position(sign.x, sign.y),  # type: ignore[attr-defined]
        sign.color,  # type: ignore[attr-defined]
    )
    if rule is None:
        return None
    axis, permitted = rule
    robot_lat = x if axis == Axis.X else y
    sign_lat = sign.x if axis == Axis.X else sign.y  # type: ignore[attr-defined]
    if robot_lat == sign_lat:
        return False
    return (1 if robot_lat > sign_lat else -1) != permitted


def _signed_clearance(sign: object, x: float, y: float) -> float | None:
    """Signed lateral clearance of ``(x, y)`` from ``sign``, + on the permitted side.

    The bucket label says only WHICH side the plan is on. The magnitude says what
    kind of mistake it is, and the two point at different code: a plan sitting a
    centimetre over the line is a lane that under-delivered its offset, while one
    sitting a full lane width onto the forbidden side (spec is +18.14/+27.86 cm)
    is a lane built on the wrong side to begin with.
    """
    rule = sign_router_module.outward_lateral_axis(
        corridor_for_position(sign.x, sign.y),  # type: ignore[attr-defined]
        sign.color,  # type: ignore[attr-defined]
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
    rule = sign_router_module.outward_lateral_axis(corridor, believed_sign.color)  # type: ignore[attr-defined]
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
    sim: object, sign: object, chassis_wrong: bool, true_pose: tuple[float, float, float]
) -> tuple[str, float | None, float | None, str | None, float | None]:
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
        return "no-pose", None, None, None, None
    believed_sign = _BeliefSign(moved[0], moved[1], sign.color)  # type: ignore[attr-defined]
    plan_point = _nearest_on_path(sim.navigator._waypoints, believed_sign)  # type: ignore[attr-defined]  # noqa: SLF001
    if plan_point is None:
        return "no-plan", None, None, None, None
    plan_wrong = _side_of(believed_sign, *plan_point)
    if plan_wrong is None:
        return "no-rule", None, None, None, None
    plan_clearance = _signed_clearance(believed_sign, *plan_point)
    # How much of the lane's specified offset actually reaches the pass. The
    # base path is the same polyline before apply_sign_lanes moved it sideways,
    # so the difference is the delivered offset -- against a spec of 0.2786 m at
    # the shipped SIGN_LANE_OFFSET_FRAC.
    base_point = _nearest_on_path(sim.navigator._lane_base_waypoints, believed_sign)  # type: ignore[attr-defined]  # noqa: SLF001
    base_clearance = None if base_point is None else _signed_clearance(believed_sign, *base_point)
    delivered = None if base_clearance is None or plan_clearance is None else plan_clearance - base_clearance
    lane_shape = _lane_shape(sim, believed_sign)
    depth_error = _spec_depth_error(sim, believed_sign)
    if not plan_wrong:
        return ("plan-ok/chassis-wrong (tracking)" if chassis_wrong else "plan-ok"), plan_clearance, delivered, lane_shape, depth_error

    router = sim.navigator.sign_router  # type: ignore[attr-defined]
    routed = list(router.signs) if router is not None else []
    if not routed:
        return "plan-wrong/never-routed", plan_clearance, delivered, lane_shape, depth_error
    nearest = min(routed, key=lambda r: math.hypot(r.x - believed_sign.x, r.y - believed_sign.y))
    if math.hypot(nearest.x - believed_sign.x, nearest.y - believed_sign.y) > _SIGN_MATCH_M:
        return "plan-wrong/never-routed", plan_clearance, delivered, lane_shape, depth_error
    if str(nearest.color) != str(sign.color):  # type: ignore[attr-defined]
        return "plan-wrong/colour-misread", plan_clearance, delivered, lane_shape, depth_error
    return "plan-wrong/colour-ok (routing)", plan_clearance, delivered, lane_shape, depth_error


_SIGN_MATCH_M = 0.25
"""How close a routed sign must be to a true one to count as the same sign."""


_LANE_SPEC_NEAR_M = 0.1814
"""Near edge of the sign-lane spec (+18.14/+27.86 cm off the sign).

A plan sitting further onto the forbidden side than this is on the wrong side by
a full lane width -- a lane built the wrong way round rather than one that fell
short of its offset."""


_HOLD_M = NavigationTuning.load_default().sign_router.SIGN_LANE_HOLD_M
"""Plateau half-width the lane holds full offset over, from the shipped tuning."""


_LANE_SPEC_FAR_M = 0.2786
"""Lateral offset the lane planner asks for at the shipped config.

Read back from ``NavigationTuning.load_default()`` on 2026-08-25 as
``(chassis_half_diagonal + sign_width/2 + SIGN_CLEARANCE_MARGIN_M) *
SIGN_LANE_OFFSET_FRAC`` with ``SIGN_LANE_PLANNER`` True -- restated here only as
the denominator of the delivery ratio."""


def _verdict_is_ambiguous(sign: object, near_x: float, near_y: float) -> bool:
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
        rule = sign_router_module.outward_lateral_axis(
            corridor_for_position(sign.x + dx, sign.y + dy),  # type: ignore[attr-defined]
            sign.color,  # type: ignore[attr-defined]
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
        rule = sign_router_module.outward_lateral_axis(corridor_for_position(sign.x, sign.y), sign.color)
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
                    _verdict_is_ambiguous(sign, near_x, near_y),
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


def _run_one(args_tuple: tuple[str, bool, bool]) -> tuple[Counter[str], list[float], list, list, list, list, list, list, list]:
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
    sim = ScenarioSimulator(metadata, num_laps=3, seed=0, blind=True, known_start=known_start)

    signs = sign_router_module.signs_from_metadata(metadata)
    # Per sign: the closest approach seen so far, and what the navigator was
    # doing AT that tick. Attribution has to be taken live -- the plan and the
    # router's sign list are rebuilt continuously, so nothing about the moment
    # of the pass survives to the end of the run to be read off afterwards.
    best: dict[int, tuple[float, str, float, float | None, float | None, str | None, float | None]] = {}
    # Spec hygiene, sampled at the tick the router holds the most lane specs.
    # Duplication and corridor mislabelling are INTERNAL to the believed frame,
    # so unlike a comparison against true positions they are not confounded by
    # blind's frame rotation.
    hygiene = {"peak": 0, "mislabelled": 0}
    early_pose_errors: list[float] = []

    def _on_step(state, _scan) -> None:  # type: ignore[no-untyped-def]
        true_pose[0] = (state.x, state.y)
        router_now = sim.navigator.sign_router
        specs_now = list(router_now.lane_specs) if router_now is not None else []
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
            chassis_wrong = bool(_side_of(sign, state.x, state.y))
            bucket, plan_clearance, delivered, lane_shape, depth_error = _attribute(
                sim, sign, chassis_wrong, (state.x, state.y, state.yaw)
            )
            best[index] = (distance, bucket, pose_error, plan_clearance, delivered, lane_shape, depth_error)

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

    # Attribution, restricted to passes where the believed frame nearly
    # coincides with the true one. Outside that gate the router's own output
    # cannot be matched to a true sign at all (see _POSE_GATE_M).
    plan_clearances: list[tuple[str, float]] = []
    deliveries: list[tuple[str, float]] = []
    shapes: list[tuple[str, str]] = []
    depth_errors: list[tuple[str, float, float]] = []
    for index, (_distance, bucket, pose_error, plan_clearance, delivered, lane_shape, depth_error) in best.items():
        near_x, near_y = min(trail, key=lambda p: math.hypot(signs[index].x - p[0], signs[index].y - p[1]))
        violated = bool(_side_of(signs[index], near_x, near_y))
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
    return counts, alongs, list(_VIOLATIONS), list(_EARLY), plan_clearances, deliveries, shapes, depth_errors, [hygiene_row]


def _environment() -> str:
    """Hardware profile and tree state, stamped on every sweep.

    On 2026-08-25 a 122/68 attribution split reported the day before turned out
    to be unreproducible from the very commit that reported it -- the same code
    gives 91/95 -- so the difference was environmental. The worktree it ran in
    had already been auto-removed, which left no way to recover which profile or
    working-tree edits produced it, and an unreproducible ratio had by then
    chosen the investigation's target for a day. A corpus number is not a result
    unless what produced it is written down beside it.
    """
    profile = os.environ.get("VTITAN_HARDWARE_PROFILE", "UNSET")
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = "unknown", ""
    return f"profile={profile}  rev={revision}{'+dirty' if dirty else ''}"


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

    total = counts["retreat"] + counts["pass"]
    arm = "blind+known_start" if args.known_start else "blind"
    print(f"scenarios {len(paths)}  retirements {total}  arm {arm}")
    print(f"  ENV {_environment()}")
    print(f"  along-track: RETREAT (sign ahead) {counts['retreat']:>4}   PASS (sign behind) {counts['pass']:>4}")
    if alongs:
        print(f"    median {statistics.median(alongs):+.2f} m")
    print(f"  ROUTER (believed frame, discovered colour): {counts['router_wrong']:>4} wrong-side of {total} retirements")
    print(f"  TRUTH  (true layout, true trajectory):      {counts['truth_wrong']:>4} wrong-side of {counts['truth_passed']} signs actually passed")
    print("  per-run:")
    print(f"    runs the router ended on pass-side  {counts['run_ended_by_router']:>4}/{len(paths)}")
    print(f"    runs with a REAL wrong-side pass    {counts['run_truly_violated']:>4}/{len(paths)}")
    if violations:
        ambiguous = [v for v in violations if v[3]]
        margins = sorted(v[2] for v in violations)
        print(f"  violation detail ({len(violations)}):")
        print(f"    VERDICT flips under a {_CORRIDOR_PROBE_M:.2f} m corridor nudge  {len(ambiguous)}  ({len(ambiguous) / len(violations):.1%})")
        print(f"    margin onto forbidden side: p10 {margins[len(margins) // 10]:.3f} m  median {statistics.median(margins):.3f} m  p90 {margins[-max(len(margins) // 10, 1)]:.3f} m")
        print(f"    marginal (<0.05 m) {sum(1 for m in margins if m < 0.05)}   by corridor {dict(Counter(v[1] for v in violations))}   by colour {dict(Counter(v[0] for v in violations))}")
        print(f"    phase at closest approach {dict(Counter(v[4] for v in violations).most_common())}")
    buckets = {k[len("bucket::"):]: v for k, v in counts.items() if k.startswith("bucket::")}
    print(f"  attribution (violations {counts['attributed_total']}, all start sections, belief-frame matched):")
    for name, count in sorted(buckets.items(), key=lambda kv: -kv[1]):
        print(f"    {name:<38} {count:>4}")
    if plan_clearances:
        print("  PLAN clearance at the pass (+ = permitted side), by bucket:")
        for name in sorted({b for b, _ in plan_clearances}):
            values = sorted(c for b, c in plan_clearances if b == name)
            inverted = sum(1 for c in values if c <= -_LANE_SPEC_NEAR_M)
            print(
                f"    {name:<38} n={len(values):>4}  p10 {values[len(values) // 10]:+.3f}  "
                f"median {statistics.median(values):+.3f}  p90 {values[-max(len(values) // 10, 1)]:+.3f}  "
                f"beyond -{_LANE_SPEC_NEAR_M:.2f} m {inverted}"
            )
    if deliveries:
        print(f"  LANE DELIVERY at the pass (lane path minus base path, spec {_LANE_SPEC_FAR_M:.4f} m):")
        for name in sorted({b for b, _ in deliveries}):
            values = sorted(d for b, d in deliveries if b == name)
            never = sum(1 for d in values if abs(d) < 0.01)
            print(
                f"    {name:<38} n={len(values):>4}  median {statistics.median(values):+.3f}  "
                f"({statistics.median(values) / _LANE_SPEC_FAR_M:>4.0%} of spec)  never-applied (<1 cm) {never}"
            )
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
    if hygiene_rows:
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
    by_start = {k[len("start::"):]: v for k, v in counts.items() if k.startswith("start::")}
    if by_start:
        print(f"    violations by start section {dict(sorted(by_start.items()))}")
    offenders = [e for hit, e in early if hit]
    clean = [e for hit, e in early if not hit]
    if offenders and clean:
        print(f"  EARLY-WINDOW CONTROL (median pose error over the first {_EARLY_WINDOW_TICKS} ticks, before the approaches):")
        print(f"    runs that later violated  n={len(offenders):>3}  median {statistics.median(offenders):.3f} m")
        print(f"    runs that never violated  n={len(clean):>3}  median {statistics.median(clean):.3f} m")


if __name__ == "__main__":
    main()
