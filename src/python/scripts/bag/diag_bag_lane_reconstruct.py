r"""Re-run ``apply_sign_lanes`` offline and compare the lane it PLANS with the line the robot CHASED.

``diag_bag_cross_attempt.py`` measured, over 129 bags, that on a CROSSING pass
that failed the observed ``steer_target`` reached the legal side of the pillar
on only 37.4% of passes and peaked at +0.130 m against an intended +0.28 m,
while the already-legal control reached it 80.2% of the time at +0.316 m. That
reading cannot say WHERE the 0.28 was lost, because it only ever saw the
commanded line -- never the lane the planner built.

This script closes that gap. For every committed pillar it reconstructs the
planner's own answer offline and prints it beside the bag's:

* ``apply_sign_lanes`` is re-run on a freshly planned base path with the
  router's OWN ``lane_specs`` and settled corridor labels, exactly as
  ``CoreNavigator._refresh_sign_lanes`` calls it (same ``SignLaneParams``, same
  ``router.direction``).  ``SIGN_LANE_COMMIT_AHEAD_M`` is 0.0 on the shipped
  tuning, so ``_hold_committed_path`` is a no-op and the laned path IS the
  path; ``_apply_path_wall_budget`` touches the controller, not the waypoints.
* The base path is the nominal one-lap centreline rather than the run's own
  believed-width path, and that is SOUND rather than a shortcut: on a straight
  every base point already sits at ``base_lateral``, so
  ``shifted = clamp_lateral(lateral + (lane - base_lateral))`` collapses to
  ``clamp_lateral(lane)`` and the plateau value is independent of the belief.
  The belief only moves the ramp endpoints and the borrowed corner arc, which
  is reported separately (``RECON corridor max``) from the plateau.
* The ANALYTIC plateau, ``clamp_lateral(sign_lat + mult*offset) - sign_lat``,
  is printed alongside, so a disagreement between it and the reconstructed
  waypoints is itself a finding (it means the indices/interpolation, not the
  clamp, ate the offset).

The two hypotheses this separates, stated before the numbers:

  A. PLANNER.  The reconstructed plateau is itself only ~+0.13 m. Then the
     0.28 never existed and the cause is inside ``apply_sign_lanes`` --
     ``clamp_lateral``, the ``indices`` selection, ``_interpolate``, or the
     corridor label the sign was filed under.
  B. TRACKER/CARROT.  The reconstruction is a full +0.28 while the bag's
     ``steer_target`` reads +0.13. Then the lane is fine and the loss is in
     ``select_target_point``: the lookahead carrot sits outside the
     +/-``hold_m`` plateau at closest approach and aims at a point on the RAMP.
     ``target depth offset`` prices exactly that -- the along-corridor distance
     from the carrot to the pillar, against ``hold_m``.

Healthy prediction, to be checked on the controls in the same run: for an
already-legal pass the reconstructed plateau and the observed lane should both
read near the analytic value, and the carrot should sit inside the plateau.
If the controls do not give that, the query is broken, not the robot.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_lane_reconstruct.py \
        data/live/runs/run_2026090* data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from shared.config.constants import TrackDimensions  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from shared.domain.enums import Axis, Direction, Section  # noqa: E402
from shared.domain.models import SignColor, Waypoint  # noqa: E402

from scripts.bag.diag_bag_cross_attempt import Pillar, _replay  # noqa: E402
from scripts.common.bag_io import create_bags_parser, read_vision_rows_and_scans  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.sign_discovery import SignSpec  # noqa: E402
from src.navigation.planning.sign_lane import (  # noqa: E402
    SignLaneParams,
    _control_points,
    _in_lane_span,
    apply_sign_lanes,
)
from src.navigation.planning.sign_router.routing import clamp_lateral, pass_side_lateral_axis  # noqa: E402
from src.navigation.planning.waypoints.generation import calculate_waypoints  # noqa: E402
from src.simulation.scenario_builder import build_open_metadata, uniform_widths  # noqa: E402

# The chassis half-diagonal the clamp is built from. Imported rather than
# retyped so a robot.toml edit moves this reading with it.
_HALF_DIAG = math.hypot(RobotSpecs.LENGTH, RobotSpecs.WIDTH) / 2.0


def _base_path(direction: Direction, tuning) -> list[Waypoint]:  # noqa: ANN001
    """One canonical lap of the nominal Obstacles centreline.

    Nominal widths, and the Obstacles centre bias the node passes
    (``OBSTACLES_CENTER_BIAS_M``). See the module docstring for why the
    believed-width path is not needed for the plateau.
    """
    metadata = build_open_metadata(uniform_widths(1000), Section.canonical(), direction)
    return calculate_waypoints(
        metadata,
        num_laps=1,
        tuning=tuning,
        center_bias_m=tuning.waypoints.OBSTACLES_CENTER_BIAS_M,
    )


@dataclass
class Recon:
    """One pillar: what the planner would have built vs what the robot chased."""

    verdict: str
    crossing: bool
    corridor_ok: bool
    analytic: float
    """clamp_lateral(sign + mult*offset) - sign, on the legal side. The most the
    clamp allows the plateau to be."""
    clamp_bound: bool
    unclamped_excess: float
    """How much offset the clamp removed (0.0 when it did not bind)."""
    recon_plateau: float
    """Best legal-side offset among reconstructed waypoints INSIDE the hold window."""
    recon_corridor: float
    """Best legal-side offset among reconstructed waypoints anywhere in the corridor."""
    n_plateau_wp: int
    """Reconstructed waypoints whose depth falls inside the hold window."""
    n_moved: int
    """Waypoints apply_sign_lanes actually moved in this corridor."""
    n_indices: int
    profile_ok: bool
    observed_lane: float
    """Best legal-side offset of the bag's own steer_target over the approach."""
    observed_at_closest: float
    target_depth_off: float
    """|carrot depth - sign depth| at closest approach, against hold_m."""
    carrot_on_plateau: bool
    recon_at_carrot_depth: float
    """What the reconstructed profile commands AT the carrot's own depth -- the
    value the carrot would have carried had it been taken off the lane."""
    pose_at_closest: float
    sign_lat_from_centre: float
    """Believed pillar lateral, measured from the corridor centreline TOWARD the
    legal side. A legal WRO pillar reads -0.10 or +0.10; nothing else is on the
    lattice, so this separates 'the clamp is too tight' from 'the map is wrong'."""
    lattice_residual: float
    """|sign_lat_from_centre| - 0.10: how far off a legal lattice cell the belief is."""
    headroom: list[float]
    """Analytic plateau the clamp would allow at each swept wall clearance."""


# Wall clearances swept in the counterfactual. The shipped value is the chassis
# HALF-DIAGONAL + margin, i.e. the footprint at 45 deg of yaw; on a corridor
# straight the body is near axis-aligned and its true half-extent is WIDTH/2.
# The sweep prices the band between those two, which is a different lever from
# the REFUTED one (moving the plateau inside a FIXED band -- see
# SIGN_LANE_GAP_CENTRE_FRAC in sign_lane's module docstring).
_CLEARANCE_SWEEP = (0.2186, 0.1900, 0.1600, 0.1370)


def _reconstruct(p: Pillar, direction: Direction, params: SignLaneParams, base: list[Waypoint]) -> Recon | None:
    rule = pass_side_lateral_axis(p.corridor, p.colour, direction)
    committed = [t for t in p.ticks if t.committed]
    if rule is None or not committed or p.corridor is None:
        return None
    axis, want = rule
    idx = 0 if axis is Axis.X else 1
    sign_lat = p.sign_x if idx == 0 else p.sign_y
    sign_depth = p.sign_y if idx == 0 else p.sign_x

    def lat(x: float, y: float) -> float:
        """Legal-side offset of a world point from the pillar, the pass-side convention."""
        return ((x if idx == 0 else y) - sign_lat) * want

    def depth(x: float, y: float) -> float:
        return y if idx == 0 else x

    centre = (
        TrackDimensions.CORNER_MIN / 2.0
        if p.corridor in (Section.SOUTH, Section.WEST)
        else (TrackDimensions.CORNER_MAX + TrackDimensions.MAX_COORD) / 2.0
    )

    # --- what the clamp allows at all -------------------------------------
    unclamped = sign_lat + want * params.lateral_offset
    clamped = clamp_lateral(unclamped, p.corridor)
    analytic = (clamped - sign_lat) * want

    # --- re-run the shipped transform on the shipped inputs ----------------
    specs = [(SignSpec(x=e[1], y=e[2], color=SignColor(e[0])), e[3]) for e in p.lane_specs]
    laned = apply_sign_lanes(base, specs, params, direction)

    same_corridor = [e for e in specs if e[1] == p.corridor]
    indices = [i for i, wp in enumerate(base) if _in_lane_span(wp, p.corridor, axis, params.corner_entry_m or 0.0)]
    straight = [i for i in indices if _in_lane_span(base[i], p.corridor, axis, 0.0)]
    laterals = sorted((base[i].x if idx == 0 else base[i].y) for i in (straight or indices)) if indices else []
    base_lateral = laterals[len(laterals) // 2] if laterals else 0.0
    profile = (
        _control_points(same_corridor, p.corridor, axis, base_lateral, params, direction) if same_corridor else []
    )

    plateau_vals = [lat(laned[i].x, laned[i].y) for i in indices if abs(depth(laned[i].x, laned[i].y) - sign_depth) <= (params.hold_m + 1e-9)]
    corridor_vals = [lat(laned[i].x, laned[i].y) for i in indices]
    n_moved = sum(1 for i in indices if laned[i] != base[i])

    # --- what the robot actually chased ------------------------------------
    approach_first = committed[0]
    closest = min(committed, key=lambda t: t.rng)
    approach = [t for t in p.ticks if approach_first.rel <= t.rel <= closest.rel and t.steer_x is not None]
    observed = [lat(t.steer_x, t.steer_y) for t in approach]
    carrot_depth_off = (
        abs(depth(closest.steer_x, closest.steer_y) - sign_depth) if closest.steer_x is not None else float("nan")
    )
    recon_at_carrot = float("nan")
    if profile and closest.steer_x is not None:
        from src.navigation.planning.sign_lane import _interpolate  # noqa: PLC0415

        val = _interpolate(profile, depth(closest.steer_x, closest.steer_y))
        if val is not None:
            recon_at_carrot = (clamp_lateral(val, p.corridor) - sign_lat) * want

    achieved = lat(closest.pose_x, closest.pose_y)
    commanded = lat(closest.deform_x, closest.deform_y) if closest.deform_x is not None else achieved
    verdict = "routing" if commanded < 0 else ("execution" if achieved < 0 else "ok")

    return Recon(
        verdict=verdict,
        crossing=lat(approach_first.pose_x, approach_first.pose_y) < 0,
        corridor_ok=any(abs(e[0].x - p.sign_x) < 1e-9 and abs(e[0].y - p.sign_y) < 1e-9 for e in same_corridor),
        analytic=analytic,
        clamp_bound=abs(clamped - unclamped) > 1e-9,
        unclamped_excess=abs(clamped - unclamped),
        recon_plateau=max(plateau_vals) if plateau_vals else float("-inf"),
        recon_corridor=max(corridor_vals) if corridor_vals else float("-inf"),
        n_plateau_wp=len(plateau_vals),
        n_moved=n_moved,
        n_indices=len(indices),
        profile_ok=bool(profile),
        observed_lane=max(observed) if observed else float("-inf"),
        observed_at_closest=lat(closest.steer_x, closest.steer_y) if closest.steer_x is not None else float("nan"),
        target_depth_off=carrot_depth_off,
        carrot_on_plateau=carrot_depth_off <= params.hold_m,
        recon_at_carrot_depth=recon_at_carrot,
        pose_at_closest=achieved,
        sign_lat_from_centre=(sign_lat - centre) * want,
        lattice_residual=abs(sign_lat - centre) - 0.10,
        headroom=[_headroom(sign_lat, want, p.corridor, c, params.lateral_offset) for c in _CLEARANCE_SWEEP],
    )


def _headroom(sign_lat: float, want: int, corridor: Section, clearance: float, offset: float) -> float:
    """Analytic plateau offset if ``clamp_lateral`` used ``clearance`` instead of the shipped one."""
    target = sign_lat + want * offset
    if corridor in (Section.SOUTH, Section.WEST):
        target = min(target, TrackDimensions.CORNER_MIN - clearance)
        target = max(target, TrackDimensions.MIN_COORD + clearance)
    else:
        target = max(target, TrackDimensions.CORNER_MAX + clearance)
        target = min(target, TrackDimensions.MAX_COORD - clearance)
    return (target - sign_lat) * want


def _mean(vals: list[float]) -> float:
    finite = [v for v in vals if math.isfinite(v)]
    return sum(finite) / len(finite) if finite else float("nan")


def _report(label: str, rows: list[Recon], params: SignLaneParams) -> None:
    n = len(rows)
    print(f"  {label:<32} n={n}")
    if not n:
        return

    def pct(pred) -> float:  # noqa: ANN001
        return 100.0 * sum(1 for r in rows if pred(r)) / n

    print(
        f"      ANALYTIC plateau (clamp ceiling)   {_mean([r.analytic for r in rows]):+.3f} m"
        f"   clamp bound {pct(lambda r: r.clamp_bound):5.1f}%"
        f"   mean removed {_mean([r.unclamped_excess for r in rows]):.3f} m"
        f"   UNSATISFIABLE (<=0) {pct(lambda r: r.analytic <= 0.0):5.1f}%"
    )
    print(
        f"      RECON plateau (waypoints, hold)    {_mean([r.recon_plateau for r in rows]):+.3f} m"
        f"   reached >0.15 {pct(lambda r: r.recon_plateau > 0.15):5.1f}%"
        f"   no wp in hold {pct(lambda r: r.n_plateau_wp == 0):5.1f}%"
        f"   mean wp in hold {_mean([float(r.n_plateau_wp) for r in rows]):4.1f}"
    )
    print(
        f"      RECON corridor max (anywhere)      {_mean([r.recon_corridor for r in rows]):+.3f} m"
        f"   indices empty {pct(lambda r: r.n_indices == 0):5.1f}%"
        f"   profile empty {pct(lambda r: not r.profile_ok):5.1f}%"
        f"   nothing moved {pct(lambda r: r.n_moved == 0):5.1f}%"
        f"   sign not in its corridor group {pct(lambda r: not r.corridor_ok):5.1f}%"
    )
    print(
        f"      OBSERVED steer_target best         {_mean([r.observed_lane for r in rows]):+.3f} m"
        f"   at closest {_mean([r.observed_at_closest for r in rows]):+.3f} m"
        f"   >0.15 on some tick {pct(lambda r: r.observed_lane > 0.15):5.1f}%"
    )
    print(
        f"      CARROT depth from pillar           {_mean([r.target_depth_off for r in rows]):.3f} m"
        f"   inside hold ({params.hold_m:.2f}) {pct(lambda r: r.carrot_on_plateau):5.1f}%"
        f"   recon value AT carrot depth {_mean([r.recon_at_carrot_depth for r in rows]):+.3f} m"
    )
    gap_plan = [r.recon_plateau - r.observed_lane for r in rows if math.isfinite(r.recon_plateau) and math.isfinite(r.observed_lane)]
    print(
        f"      GAP recon_plateau - observed       {_mean(gap_plan):+.3f} m"
        f"   |   pose at closest {_mean([r.pose_at_closest for r in rows]):+.3f} m"
    )


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--per-lap", action="store_true")
    args = parser.parse_args()
    tuning = get_tuning(None)
    sr = tuning.sign_router
    offset = (_HALF_DIAG + 0.05 / 2 + sr.SIGN_CLEARANCE_MARGIN_M) * sr.SIGN_LANE_OFFSET_FRAC
    params = SignLaneParams(
        lateral_offset=offset,
        ramp_m=sr.SIGN_LANE_RAMP_M,
        hold_m=sr.SIGN_LANE_HOLD_M,
        skip_unsatisfiable=sr.SIGN_LANE_SKIP_UNSATISFIABLE,
        split_overlap=sr.SIGN_LANE_SPLIT_OVERLAP,
        corner_entry_m=sr.SIGN_LANE_CORNER_ENTRY_M,
    )
    wall_clear = _HALF_DIAG + sr.WALL_CLEARANCE_MARGIN_M
    print(
        f"== shipped: offset={offset:.4f} ramp={params.ramp_m} hold={params.hold_m} "
        f"corner_entry={params.corner_entry_m} skip_unsat={params.skip_unsatisfiable}"
    )
    print(
        f"== clamp: half_diag={_HALF_DIAG:.4f} wall_margin={sr.WALL_CLEARANCE_MARGIN_M} "
        f"-> lane band [{TrackDimensions.MIN_COORD + wall_clear:.4f}, {TrackDimensions.CORNER_MIN - wall_clear:.4f}] "
        f"(low-side corridors); headroom from a 0.5 centreline = {TrackDimensions.CORNER_MIN - wall_clear - 0.5:+.4f} m"
    )
    print()

    bases: dict[Direction, list[Waypoint]] = {}
    rows: list[Recon] = []
    read = skipped = 0
    for bag in args.bag_dirs:
        try:
            data, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        read += 1
        pillars, direction = _replay(Path(bag).name.replace("run_", ""), data, frames, scans, tuning, args.per_lap)
        if direction not in bases:
            bases[direction] = _base_path(direction, tuning)
        for p in pillars:
            r = _reconstruct(p, direction, params, bases[direction])
            if r is not None:
                rows.append(r)

    print(f"== {read} bags read, {skipped} unreadable, {len(rows)} pillars reconstructed")
    correct = [r for r in rows if r.verdict != "routing"]
    cross = [r for r in correct if r.crossing]
    hold = [r for r in correct if not r.crossing]
    print()
    _report("crossing FAIL", [r for r in cross if r.verdict == "execution"], params)
    print()
    _report("crossing SUCCESS (control)", [r for r in cross if r.verdict == "ok"], params)
    print()
    _report("already-legal SUCCESS (control)", [r for r in hold if r.verdict == "ok"], params)
    print()
    _report("already-legal FAIL", [r for r in hold if r.verdict == "execution"], params)
    print()

    print("== WHERE THE OFFSET IS LOST (each stage in metres, mean)")
    print("   AXIS AGREES = the sign's settled corridor label is the one the pass is judged in.")
    print("   Where it disagrees the lane is laid on the PERPENDICULAR axis, so its offset")
    print("   measured in the pass's own frame is not a shortfall, it is a different quantity.")
    for label, sub in (
        ("crossing FAIL", [r for r in cross if r.verdict == "execution"]),
        ("crossing SUCCESS", [r for r in cross if r.verdict == "ok"]),
        ("already-legal SUCCESS", [r for r in hold if r.verdict == "ok"]),
    ):
        for axis_label, pred in (("axis AGREES", True), ("axis DISAGREES", False)):
            part = [r for r in sub if r.corridor_ok is pred]
            if not part:
                continue
            print(
                f"   {label:<22} {axis_label:<15} n={len(part):4d}"
                f"  intent {offset:+.3f}"
                f" -> clamp {_mean([r.analytic for r in part]):+.3f}"
                f" -> waypoints {_mean([r.recon_plateau for r in part]):+.3f}"
                f" -> carrot {_mean([r.observed_lane for r in part]):+.3f}"
                f" -> chassis {_mean([r.pose_at_closest for r in part]):+.3f}"
            )
    print()
    print("== IS THE CLAMP REFUSING A LEGAL GEOMETRY, OR AN IMPOSSIBLE BELIEF?")
    print("   A legal WRO pillar sits 0.10 m off its corridor centreline, so |lattice residual|")
    print("   should be ~0.00 if the belief is on the grid. Positive = believed FURTHER out.")
    for label, sub in (
        ("crossing FAIL", [r for r in cross if r.verdict == "execution"]),
        ("crossing SUCCESS", [r for r in cross if r.verdict == "ok"]),
        ("already-legal SUCCESS", [r for r in hold if r.verdict == "ok"]),
    ):
        part = [r for r in sub if r.corridor_ok]
        if not part:
            continue
        lats = sorted(r.sign_lat_from_centre for r in part)
        res = sorted(r.lattice_residual for r in part)
        print(
            f"   {label:<22} n={len(part):4d}  sign lat from centre TOWARD legal side"
            f"  p10 {lats[len(lats) // 10]:+.3f}  p50 {lats[len(lats) // 2]:+.3f}"
            f"  p90 {lats[int(len(lats) * 0.9)]:+.3f}"
            f"  |  lattice residual p50 {res[len(res) // 2]:+.3f}"
            f"  off-grid (>0.05) {100.0 * sum(1 for v in res if v > 0.05) / len(res):5.1f}%"
        )
    print()
    print("== COUNTERFACTUAL: what a wider clamp band would allow (analytic plateau)")
    print(f"   shipped wall clearance = {wall_clear:.4f} m (chassis half-DIAGONAL + margin).")
    print("   0.1370 = chassis half-WIDTH + the same margin, i.e. the axis-aligned footprint.")
    for label, sub in (
        ("crossing FAIL", [r for r in cross if r.verdict == "execution"]),
        ("crossing SUCCESS", [r for r in cross if r.verdict == "ok"]),
        ("already-legal SUCCESS", [r for r in hold if r.verdict == "ok"]),
    ):
        part = [r for r in sub if r.corridor_ok]
        if not part:
            continue
        print(f"   {label}  n={len(part)}")
        for j, c in enumerate(_CLEARANCE_SWEEP):
            vals = [r.headroom[j] for r in part]
            print(
                f"      clearance {c:.4f}  mean plateau {_mean(vals):+.3f} m"
                f"   unsatisfiable {100.0 * sum(1 for v in vals if v <= 0.0) / len(vals):5.1f}%"
                f"   >=0.15 m {100.0 * sum(1 for v in vals if v >= 0.15) / len(vals):5.1f}%"
                f"   full offset {100.0 * sum(1 for v in vals if v >= offset - 1e-6) / len(vals):5.1f}%"
            )
    print()
    print("== CARROT PLACEMENT: crossing FAIL vs its controls (branch B)")
    for label, sub in (
        ("crossing FAIL", [r for r in cross if r.verdict == "execution"]),
        ("crossing SUCCESS", [r for r in cross if r.verdict == "ok"]),
        ("already-legal SUCCESS", [r for r in hold if r.verdict == "ok"]),
    ):
        if not sub:
            continue
        offs = sorted(r.target_depth_off for r in sub if math.isfinite(r.target_depth_off))
        if not offs:
            continue
        p50 = offs[len(offs) // 2]
        p90 = offs[int(len(offs) * 0.9)]
        inside = 100.0 * sum(1 for v in offs if v <= params.hold_m) / len(offs)
        print(f"   {label:<24} carrot |depth-sign| p50 {p50:.3f}  p90 {p90:.3f}  inside hold {inside:5.1f}%")


if __name__ == "__main__":
    main()
