r"""In a CROSSING pass, does the planner ever ask for the crossing?

Background. ``diag_bag_pass_side.py`` splits sign passes into ROUTING (commanded
the wrong side), EXECUTION (commanded right, chassis went wrong) and ok. Over
129 bags the EXECUTION bucket is 31% of correctly-commanded passes, and 215 of
its 248 members are passes that had to CROSS from the wrong side to the legal
one. Crossings fail ~65% regardless of speed, commit range or the geometrically
available arc -- that margin test is FLAT. So the failure is not geometry.

This script asks the next question: what did the wheel actually do, and did the
planner ever ask for the crossing at all.

The mechanism it is built to measure, read out of the shipped code rather than
assumed:

* ``SIGN_LANE_SUPPRESS_DEFORM`` is TRUE on the shipped tuning, so
  ``CoreNavigator`` computes ``deform_waypoint`` and then THROWS IT AWAY --
  ``steer_target`` stays the raw point selected from ``self._waypoints``. The
  only thing that can move the commanded line is therefore the LANE, i.e.
  ``apply_sign_lanes`` having rewritten those waypoints.
* ``/nav_debug`` records both: ``steer_target_x/y`` is what steering actually
  chased (the lane path), ``sign_target_x_m/y_m`` is the deformed point that was
  discarded. Their difference is directly readable per tick.
* A latched manoeuvre exits ``CoreNavigator.step`` BEFORE the router is called,
  so those ticks carry no steer target and no commitment. They are folded back
  into each pass's window by TIME (see ``_replay``); without that they simply
  vanish from the trace and question 3 cannot be priced at all.

So for each committed pillar the trace below reports, in the pass-side
convention (positive = the legal side of the sign):

    lane_off    steer_target's lateral offset from the sign  -> DID THE LANE MOVE?
    deform_off  sign_target's offset from the sign           -> what the router wanted
    pose_off    the chassis's own offset                     -> where it actually was

A sign sits 0.10 m off the corridor centreline and the pass offset is 0.28 m, so
an unlaned (centreline) path reads |lane_off| ~ 0.10 with an arbitrary sign,
while a materialised lane reads lane_off ~ +0.20..+0.28. The threshold used to
call the lane PRESENT is ``--lane-thresh`` (default 0.15 m), which no
centreline path can reach on the legal side.

CONTROL, and the whole point of the script: crossing passes that SUCCEEDED are
reported beside crossing passes that FAILED. Same geometry, opposite outcome. If
both populations show the same lane presence, the lane is not the separator and
this reading is a null -- which is only meaningful because the same query also
reports the already-legal population, where the number is known to be high.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_cross_attempt.py \
        data/live/runs/run_2026090* data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Axis, Direction  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402

from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.stats import nearest_by_time  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402
from src.navigation.planning.sign_router.routing import clamp_lateral, pass_side_lateral_axis  # noqa: E402


@dataclass
class Tick:
    """One tick of a pass, raw. Projected onto the pass-side axis only at the end."""

    rel: float
    rng: float | None
    committed: bool
    pose_x: float
    pose_y: float
    yaw: float
    steer_x: float | None
    steer_y: float | None
    deform_x: float | None
    deform_y: float | None
    steering: float | None
    maneuver: str | None
    man_steering: float | None
    speed: float | None
    crosstrack: float | None
    lap: int


@dataclass
class Pillar:
    run: str
    ticks: list[Tick] = field(default_factory=list)
    colour: SignColor = SignColor.UNKNOWN
    corridor: object = None
    sign_x: float = 0.0
    sign_y: float = 0.0
    best_rng: float = 1e9
    lane_specs: list = field(default_factory=list)
    """router.lane_specs as it stood at closest approach -- what
    apply_sign_lanes would have been handed on that tick."""


def _replay(run: str, rows, frames, scans, tuning, per_lap: bool = False):  # noqa: ANN001,ANN201,FBT001,FBT002
    """Replay the shipped router over one bag, keeping every tick of every pass.

    per_lap keys a pillar by (position, lap) instead of position alone. The
    shipped diag_bag_pass_side keys by position, so one physical pillar
    passed on three laps collapses into ONE verdict taken at its best approach
    -- which is what makes the default here reproduce that script's counts, and
    also what makes the default UNABLE to test anything lap-dependent. Under
    per_lap each lap's pass is judged on its own, which is the only form in
    which 'the lane is already built from lap 2' is a testable claim.
    """
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    frame_i = 0
    scan_times = [t for t, _ in scans]
    pillars: dict[tuple[float, float], Pillar] = {}
    orphans: list[Tick] = []

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            obs.extend(
                o
                for det in decode_detections(frames[frame_i][1])
                if (o := detection_to_observation(det, pose, tuning, ranges, angles)) is not None
            )
            frame_i += 1
        if d.steer_target_x is None or d.steer_target_y is None:
            # Manoeuvre-owned (or otherwise pre-router) tick. Parked here and
            # folded back into whichever pass's window it falls inside, below.
            orphans.append(
                Tick(
                    rel=rel,
                    rng=None,
                    committed=False,
                    pose_x=d.pose_x,
                    pose_y=d.pose_y,
                    yaw=d.pose_yaw,
                    steer_x=None,
                    steer_y=None,
                    deform_x=None,
                    deform_y=None,
                    steering=d.commanded_steering_norm,
                    maneuver=str(d.active_maneuver_type) if d.active_maneuver_type is not None else None,
                    man_steering=d.maneuver_steering,
                    speed=d.commanded_speed_mps,
                    crosstrack=d.crosstrack_error_m,
                    lap=d.laps_completed,
                )
            )
            continue
        deformed = router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )
        committed = router.committed_sign_position
        if committed is None:
            continue
        rng = math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
        key = (round(committed.x, 1), round(committed.y, 1), d.laps_completed if per_lap else 0)
        p = pillars.setdefault(key, Pillar(run=run))
        p.ticks.append(
            Tick(
                rel=rel,
                rng=rng,
                committed=True,
                pose_x=d.pose_x,
                pose_y=d.pose_y,
                yaw=d.pose_yaw,
                # The bag's OWN recorded steer target: what steering really chased.
                steer_x=d.steer_target_x,
                steer_y=d.steer_target_y,
                # The replayed deformation, which the shipped navigator discards
                # under SIGN_LANE_SUPPRESS_DEFORM. Recorded to price what the
                # suppressed branch would have commanded on the same tick.
                deform_x=deformed[0],
                deform_y=deformed[1],
                steering=d.commanded_steering_norm,
                maneuver=str(d.active_maneuver_type) if d.active_maneuver_type is not None else None,
                man_steering=d.maneuver_steering,
                speed=d.commanded_speed_mps,
                crosstrack=d.crosstrack_error_m,
                lap=d.laps_completed,
            )
        )
        if rng < p.best_rng:
            p.best_rng = rng
            p.sign_x, p.sign_y = committed.x, committed.y
            p.corridor = d.current_corridor
            p.lane_specs = [(spec.color, spec.x, spec.y, corr) for spec, corr in router.lane_specs]
            p.colour = next(
                (s.color for s in router.signs if abs(s.x - committed.x) < 1e-9 and abs(s.y - committed.y) < 1e-9),
                SignColor.UNKNOWN,
            )
    for p in pillars.values():
        p.ticks.sort(key=lambda t: t.rel)
        if not p.ticks:
            continue
        closest = min(p.ticks, key=lambda t: t.rng)
        lo, hi = p.ticks[0].rel, closest.rel
        p.ticks.extend(t for t in orphans if lo <= t.rel <= hi)
        p.ticks.sort(key=lambda t: t.rel)
    return list(pillars.values()), direction


@dataclass
class Row:
    """One pillar, classified and traced."""

    run: str
    verdict: str
    crossing: bool
    commit_rng: float
    commit_off: float
    n_approach: int
    duration_s: float
    path_m: float
    lane_ticks: int
    lane_max: float
    lane_at_commit: float
    asked_ticks: int
    """Ticks where the LANE asked for a crossing the chassis had not made: steer
    target on the legal side by > thresh while the pose was still wrong-side.
    The literal answer to 'did the planner ask'."""
    deform_ticks: int
    deform_max: float
    steer_toward: int
    steer_run: int
    steer_integral: float
    steer_sat: int
    """Approach ticks at |steering| > 0.9 -- the actuator at its stop."""
    steer_late: float
    """Toward-legal fraction over the LAST 20% of the approach."""
    man_ticks: int
    man_hidden: int
    """Approach ticks the escape layer owned OUTRIGHT (no steer target at all)."""
    man_types: dict[str, int]
    man_opposes: int
    reverse_ticks: int
    pose_gain: float
    final_off: float
    mean_speed: float
    lap: int
    lane_block: str
    """Which gate inside apply_sign_lanes would have refused this sign's
    corridor, evaluated on the router state at closest approach."""
    asked_first_rng: float | None
    """Range at which the plan FIRST asked for the cross -- the runway it left."""


def _classify(p: Pillar, direction: Direction, lane_thresh: float) -> Row | None:
    rule = pass_side_lateral_axis(p.corridor, p.colour, direction)
    committed = [t for t in p.ticks if t.committed]
    if rule is None or not committed:
        return None
    axis, want = rule
    idx = 0 if axis is Axis.X else 1
    sign_lat = p.sign_x if idx == 0 else p.sign_y

    def lat(x: float, y: float) -> float:
        return ((x if idx == 0 else y) - sign_lat) * want

    closest = min(committed, key=lambda t: t.rng)
    first = committed[0]
    achieved = lat(closest.pose_x, closest.pose_y)
    commanded = lat(closest.deform_x, closest.deform_y)
    verdict = "routing" if commanded < 0 else ("execution" if achieved < 0 else "ok")

    approach = [t for t in p.ticks if first.rel <= t.rel <= closest.rel]
    n = len(approach)
    path_m = sum(
        math.hypot(b.pose_x - a.pose_x, b.pose_y - a.pose_y) for a, b in zip(approach, approach[1:], strict=False)
    )

    lane_offs = [lat(t.steer_x, t.steer_y) for t in approach if t.steer_x is not None]
    deform_offs = [lat(t.deform_x, t.deform_y) for t in approach if t.deform_x is not None]
    asked = sum(
        1
        for t in approach
        if t.steer_x is not None and lat(t.steer_x, t.steer_y) > lane_thresh and lat(t.pose_x, t.pose_y) < 0
    )

    def wants_left_of(t: Tick) -> float:
        """Legal-side world direction projected onto the chassis's own left."""
        left_x, left_y = -math.sin(t.yaw), math.cos(t.yaw)
        legal = (float(want), 0.0) if idx == 0 else (0.0, float(want))
        return legal[0] * left_x + legal[1] * left_y

    toward = 0
    best_run = run_len = 0
    integral = 0.0
    sat = 0
    late_toward = late_n = 0
    late_cut = int(n * 0.8)
    for i, t in enumerate(approach):
        if t.steering is None:
            run_len = 0
            continue
        if abs(t.steering) > 0.9:
            sat += 1
        wl = wants_left_of(t)
        # A zero steering command votes for neither -- it is not a side.
        if wl == 0.0 or t.steering == 0.0:
            run_len = 0
            continue
        integral += t.steering * (1.0 if wl > 0 else -1.0)
        hit = (t.steering > 0) == (wl > 0)
        if i >= late_cut:
            late_n += 1
            late_toward += int(hit)
        if hit:
            toward += 1
            run_len += 1
            best_run = max(best_run, run_len)
        else:
            run_len = 0

    man_types: dict[str, int] = {}
    man_opposes = 0
    for t in approach:
        if t.maneuver is None:
            continue
        man_types[t.maneuver] = man_types.get(t.maneuver, 0) + 1
        if t.man_steering:
            wl = wants_left_of(t)
            if wl != 0.0 and (t.man_steering > 0) != (wl > 0):
                man_opposes += 1

    speeds = [abs(t.speed) for t in approach if t.speed is not None]
    return Row(
        run=p.run,
        verdict=verdict,
        crossing=lat(first.pose_x, first.pose_y) < 0,
        commit_rng=first.rng,
        commit_off=lat(first.pose_x, first.pose_y),
        n_approach=n,
        duration_s=closest.rel - first.rel,
        path_m=path_m,
        lane_ticks=sum(1 for v in lane_offs if v > lane_thresh),
        lane_max=max(lane_offs) if lane_offs else 0.0,
        lane_at_commit=lane_offs[0] if lane_offs else 0.0,
        asked_ticks=asked,
        deform_ticks=sum(1 for v in deform_offs if v > lane_thresh),
        deform_max=max(deform_offs) if deform_offs else 0.0,
        steer_toward=toward,
        steer_run=best_run,
        steer_integral=integral,
        steer_sat=sat,
        steer_late=(late_toward / late_n) if late_n else 0.0,
        man_ticks=sum(1 for t in approach if t.maneuver is not None),
        man_hidden=sum(1 for t in approach if not t.committed),
        man_types=man_types,
        man_opposes=man_opposes,
        reverse_ticks=sum(1 for t in approach if t.speed is not None and t.speed < 0),
        pose_gain=achieved - lat(first.pose_x, first.pose_y),
        final_off=achieved,
        mean_speed=sum(speeds) / len(speeds) if speeds else 0.0,
        lap=first.lap,
        lane_block=_lane_block(p, direction),
        asked_first_rng=next(
            (
                t.rng
                for t in approach
                if t.steer_x is not None and lat(t.steer_x, t.steer_y) > lane_thresh and lat(t.pose_x, t.pose_y) < 0
            ),
            None,
        ),
    )


def _lane_block(p: Pillar, direction: Direction) -> str:
    """Re-run apply_sign_lanes' own gates for this pillar and name the blocker.

    Only the gates that can be evaluated from router state are checked; the
    waypoint-level ones (indices empty, shifted == lateral) need the
    planned path, which the bag does not carry. So a verdict of built means
    'no ROUTER-side gate refused it', not 'the lane definitely moved' -- which
    is exactly why it is reported beside the measured lane offset rather than
    instead of it.
    """
    same = [e for e in p.lane_specs if e[3] == p.corridor]
    if not same:
        return "sign not in lane_specs"
    # apply_sign_lanes takes the AXIS from a colour-keyed lookup on an
    # ARBITRARY member of the corridor (corridor_signs[0]), even though the
    # axis itself depends only on the corridor. An UNKNOWN first member
    # therefore drops the whole corridor's lane, including identified signs.
    if pass_side_lateral_axis(p.corridor, SignColor(same[0][0]), direction) is None:
        return "corridor axis lookup refused (first sign UNKNOWN)"
    own = next((e for e in same if abs(e[1] - p.sign_x) < 1e-9 and abs(e[2] - p.sign_y) < 1e-9), None)
    if own is None:
        return "sign not in its own corridor group"
    rule = pass_side_lateral_axis(p.corridor, SignColor(own[0]), direction)
    if rule is None:
        return "own colour UNKNOWN (no plateau)"
    axis, mult = rule
    sign_lat = own[1] if axis is Axis.X else own[2]
    target = clamp_lateral(sign_lat + mult * 0.28, p.corridor)
    if (target - sign_lat) * mult <= 0.0:
        return "clamp put the target on the forbidden side"
    return "built"


def _summarise(label: str, rows: list[Row], lane_thresh: float) -> None:
    n = len(rows)
    if n == 0:
        print(f"  {label:<30} n=0")
        return

    def frac(pred) -> float:  # noqa: ANN001
        return 100.0 * sum(1 for r in rows if pred(r)) / n

    def mean(fn) -> float:  # noqa: ANN001
        return sum(fn(r) for r in rows) / n

    pct_lane = mean(lambda r: r.lane_ticks / max(r.n_approach, 1)) * 100
    print(f"  {label:<30} n={n:4d}")
    print(
        f"      1 LANE built (>{lane_thresh:.2f} m legal, any tick) {frac(lambda r: r.lane_ticks > 0):5.1f}%"
        f"   best {mean(lambda r: r.lane_max):+.3f} m   lane ticks {pct_lane:5.1f}%"
        f"   at commit {frac(lambda r: r.lane_at_commit > lane_thresh):5.1f}%"
    )
    print(
        f"      1 PLANNER ASKED FOR THE CROSS  {frac(lambda r: r.asked_ticks > 0):5.1f}% of passes,"
        f" {mean(lambda r: r.asked_ticks / max(r.n_approach, 1)) * 100:5.1f}% of ticks,"
        f" first asked at range {(sum(r.asked_first_rng for r in rows if r.asked_first_rng is not None) / max(sum(1 for r in rows if r.asked_first_rng is not None), 1)):.2f} m"
    )
    print(
        f"      1 DEFORM (discarded) legal side {frac(lambda r: r.deform_ticks > 0):5.1f}%"
        f"   best {mean(lambda r: r.deform_max):+.3f} m"
    )
    print(
        f"      2 STEER toward legal {mean(lambda r: r.steer_toward / max(r.n_approach, 1)) * 100:5.1f}% of ticks"
        f"   longest run {mean(lambda r: r.steer_run):5.1f}"
        f"   integral {mean(lambda r: r.steer_integral):+7.2f}"
        f"   never {frac(lambda r: r.steer_toward == 0):5.1f}%"
    )
    print(
        f"      2 STEER saturated |s|>0.9 {mean(lambda r: r.steer_sat / max(r.n_approach, 1)) * 100:5.1f}%"
        f"   toward-legal in LAST 20% of approach {mean(lambda r: r.steer_late) * 100:5.1f}%"
    )
    print(
        f"      3 MANOEUVRE any {frac(lambda r: r.man_ticks > 0 or r.man_hidden > 0):5.1f}%"
        f"   owned outright {mean(lambda r: r.man_hidden / max(r.n_approach, 1)) * 100:5.1f}%"
        f"   blended {mean(lambda r: r.man_ticks / max(r.n_approach, 1)) * 100:5.1f}%"
        f"   opposing {mean(lambda r: r.man_opposes):4.1f} ticks"
        f"   reverse {mean(lambda r: r.reverse_ticks / max(r.n_approach, 1)) * 100:5.1f}%"
    )
    types: dict[str, int] = {}
    for r in rows:
        for k, v in r.man_types.items():
            types[k] = types.get(k, 0) + v
    if types:
        print(
            "      3 MANOEUVRE types: "
            + ", ".join(f"{k}={v}" for k, v in sorted(types.items(), key=lambda kv: -kv[1]))
        )
    print(
        f"      $ APPROACH {mean(lambda r: r.n_approach):5.1f} ticks / {mean(lambda r: r.duration_s):5.2f} s"
        f" / {mean(lambda r: r.path_m):.2f} m driven"
        f"   speed {mean(lambda r: r.mean_speed):.3f} m/s   commit range {mean(lambda r: r.commit_rng):.2f} m"
    )
    print(
        f"      $ LATERAL {mean(lambda r: r.commit_off):+.3f} -> {mean(lambda r: r.final_off):+.3f} m"
        f"   gain {mean(lambda r: r.pose_gain):+.3f} m"
        f"   per metre driven {mean(lambda r: r.pose_gain / max(r.path_m, 0.01)):+.3f}"
    )


def _xtab(label: str, rows: list[Row], pred) -> None:  # noqa: ANN001
    """Failure rate of rows split on pred, with both arms counted.

    Printed as a pair so a null reads as a null: an effect only exists if the
    two arms differ, and both denominators have to be visible to see that.
    """
    yes = [r for r in rows if pred(r)]
    no = [r for r in rows if not pred(r)]

    def rate(sub: list[Row]) -> str:
        if not sub:
            return "    n=0     "
        f = sum(1 for r in sub if r.verdict == "execution")
        return f"{f:4d}/{len(sub):4d} = {100 * f / len(sub):5.1f}%"

    print(f"   {label:<38} YES {rate(yes)}   NO {rate(no)}")


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--lane-thresh", type=float, default=0.15)
    parser.add_argument("--per-lap", action="store_true", help="judge each lap's pass separately")
    args = parser.parse_args()
    tuning = get_tuning(None)
    sr = tuning.sign_router
    print(
        f"== shipped: SIGN_LANE_PLANNER={sr.SIGN_LANE_PLANNER} "
        f"SUPPRESS_DEFORM={sr.SIGN_LANE_SUPPRESS_DEFORM} "
        f"RAMP_M={sr.SIGN_LANE_RAMP_M} CORNER_ENTRY_M={sr.SIGN_LANE_CORNER_ENTRY_M} "
        f"SKIP_UNSAT={sr.SIGN_LANE_SKIP_UNSATISFIABLE} RELABEL_UNSAT={sr.SIGN_LANE_RELABEL_UNSATISFIABLE}"
    )
    print()

    rows: list[Row] = []
    skipped = 0
    read = 0
    for bag in args.bag_dirs:
        try:
            data, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        read += 1
        pillars, direction = _replay(
            Path(bag).name.replace("run_", ""), data, frames, scans, tuning, args.per_lap
        )
        for p in pillars:
            row = _classify(p, direction, args.lane_thresh)
            if row is not None:
                rows.append(row)

    print(f"== {read} bags read, {skipped} unreadable, {len(rows)} pillars classified")
    tally = {"routing": 0, "execution": 0, "ok": 0}
    for r in rows:
        tally[r.verdict] += 1
    print(f"   routing={tally['routing']}  execution={tally['execution']}  ok={tally['ok']}")
    correct = [r for r in rows if r.verdict != "routing"]
    cross = [r for r in correct if r.crossing]
    hold = [r for r in correct if not r.crossing]
    n_cf = sum(1 for r in cross if r.verdict == "execution")
    n_hf = sum(1 for r in hold if r.verdict == "execution")
    print(
        f"   of correctly-commanded: crossing {len(cross)} (fail {n_cf}, "
        f"{100 * n_cf / max(len(cross), 1):.1f}%), "
        f"already-legal {len(hold)} (fail {n_hf}, {100 * n_hf / max(len(hold), 1):.1f}%)"
    )
    print()
    print("== CROSSING PASSES: the failures against their own control")
    _summarise("crossing FAIL", [r for r in cross if r.verdict == "execution"], args.lane_thresh)
    print()
    _summarise("crossing SUCCESS (control)", [r for r in cross if r.verdict == "ok"], args.lane_thresh)
    print()
    _summarise("already-legal SUCCESS (control)", [r for r in hold if r.verdict == "ok"], args.lane_thresh)
    print()
    _summarise("already-legal FAIL", [r for r in hold if r.verdict == "execution"], args.lane_thresh)
    print()
    print("== WHY THE LANE DID NOT MATERIALISE (crossing passes whose lane never went legal-side)")
    for pop, label in ((cross, "crossing"),):
        missing = [r for r in pop if r.lane_ticks == 0]
        reasons: dict[str, int] = {}
        for r in missing:
            reasons[r.lane_block] = reasons.get(r.lane_block, 0) + 1
        print(f"   {label}: {len(missing)} of {len(pop)} passes had no legal-side lane")
        for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"      {k:<48} {v:4d}  ({100 * v / max(len(missing), 1):5.1f}%)")
    print()
    print("== CROSSING failure rate CONDITIONED (the effect size of each candidate)")
    _xtab("lane built (>thresh legal side)", cross, lambda r: r.lane_ticks > 0)
    _xtab("plan asked for the cross", cross, lambda r: r.asked_ticks > 0)
    _xtab("discarded deform was legal side", cross, lambda r: r.deform_ticks > 0)
    _xtab("any manoeuvre in the approach", cross, lambda r: r.man_ticks > 0 or r.man_hidden > 0)
    _xtab("any reverse tick in the approach", cross, lambda r: r.reverse_ticks > 0)
    _xtab("drove >= 0.70 m during approach", cross, lambda r: r.path_m >= 0.70)
    _xtab("steered toward legal on >50% ticks", cross, lambda r: r.steer_toward > r.n_approach * 0.5)
    print()
    print("== CROSSING failure rate by DISTANCE DRIVEN during the approach")
    for lo, hi in ((0.0, 0.25), (0.25, 0.5), (0.5, 0.75), (0.75, 1.0), (1.0, 1.5), (1.5, 99.0)):
        sub = [r for r in cross if lo <= r.path_m < hi]
        if not sub:
            continue
        f = sum(1 for r in sub if r.verdict == "execution")
        print(f"   {lo:.2f}-{hi:.2f} m: {f:4d}/{len(sub):4d} = {100 * f / len(sub):5.1f}% fail")
    print()
    # Signs discovered on lap 1 stay in SignRouter._signs for later laps
    # (reset_for_new_lap only re-arms _passed), so from lap 2 the lane is
    # PRE-BUILT with its full 0.9 m ramp instead of appearing under the
    # chassis. If runway is the mechanism, the crossing failure rate must fall
    # across this split; if it does not, runway is refuted.
    print("== CROSSING failure rate by LAP (lane pre-built from lap 2 on)")
    for lap in sorted({r.lap for r in cross}):
        sub = [r for r in cross if r.lap == lap]
        f = sum(1 for r in sub if r.verdict == "execution")
        print(f"   lap {lap}: {f}/{len(sub)} = {100 * f / len(sub):5.1f}% fail")
    print()
    print("== ALREADY-LEGAL failure rate by LAP (control: should stay low)")
    for lap in sorted({r.lap for r in hold}):
        sub = [r for r in hold if r.lap == lap]
        f = sum(1 for r in sub if r.verdict == "execution")
        print(f"   lap {lap}: {f}/{len(sub)} = {100 * f / len(sub):5.1f}% fail")


if __name__ == "__main__":
    main()
