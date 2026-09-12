r"""Price the UNBUILT "reverse to buy road" manoeuvre on bags, before it is built.

Context. Over 129 bags / 1113 passes, sign passes that must CROSS from the wrong
side to the legal one fail 67.8% against 6.9% for passes already legal, and
crossings are 215 of the 248 EXECUTION failures. The separator is ROAD, not
speed and not commit range: a failed crossing drives 0.41 m, a successful one
0.92 m, and under 0.25 m of road 89.5% fail.

The arithmetic that motivates a reverse. The chassis' minimum turn radius is a
SPEED CURVE, R(v) = 0.053 + 1.86 v (see
``pocket_turn_radius_is_0075_not_029_2026_09_10``). An S-curve buys lateral
about L^2 / 4R, so clearing ``y`` of lateral needs

    L_min = sqrt(4 * R(v) * y)

of road. When ``L_min`` exceeds the road actually remaining at commit, no
steering policy can make the pass -- only MORE ROAD can, and the only source of
more road on a 1 m corridor is to reverse and re-drive the approach.

This script prices exactly that, on bags only, and reports four things:

1. SHORTFALL. Per crossing pass, ``L_min - road`` from the values the navigator
   already holds at commit (speed, range to the committed sign, own lateral
   offset). How many passes are short, by how much, and what fraction of the
   crossing FAILURES they cover.
2. REAR CLEARANCE. A reverse is only legal if the ground behind is. The shipped
   gate is ``escape_recovery._trail_confirms_reverse``, which asks
   ``trail_clearance_behind(...) >= d + CONTACT_DIST`` over the pose trail. This
   script rebuilds that exact trail from the bag (same ``POSE_TRAIL_MIN_STEP_M``
   decimation, same ``POSE_TRAIL_LEN`` cap) and evaluates the same predicate at
   the same instants, beside the measured rear LIDAR sector. If the trail does
   not vouch for the required distance on most firings, the manoeuvre is NOT
   AVAILABLE and that is the answer.
3. TIME COST. Firings per run over its laps, times ~2 legs per firing at the
   reverse speed, against the 180 s limit.
4. THE CHEAPER ALTERNATIVE, which is the falsifier. ``R`` is a speed curve, so
   simply approaching at 0.15 m/s shrinks ``L_min`` by sqrt(0.332/R(v)). The
   script reports how many shortfalls that alone erases. If slowing down clears
   most of them, the reverse is not worth building.

CONTROL, mandatory here: every number is reported for crossing FAIL beside
crossing SUCCESS -- same geometry, opposite outcome -- and beside the
already-legal population whose value is known. A metric that reads the same on
all three is a null.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_reverse_budget.py \
        data/live/runs/run_2026090* data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Axis  # noqa: E402

from scripts.bag.diag_bag_cross_attempt import _classify, _replay  # noqa: E402
from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import create_bags_parser, read_vision_rows_and_scans  # noqa: E402
from scripts.common.stats import nearest_by_time  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402
from src.navigation.utils import _rear_clearance, trail_clearance_behind  # noqa: E402

# R(v) = R0 + K v, measured on hardware 2026-09-10 (pocket turn radius note).
R0 = 0.053
RK = 1.86

REVERSE_SPEED = 0.15  # m/s, the speed the bay's mirrored reverse actually runs


def radius_at(v: float) -> float:
    return R0 + RK * abs(v)


@dataclass
class Budget:
    """One crossing pass, priced."""

    run: str
    verdict: str
    crossing: bool
    v_commit: float
    y_need: float
    radius: float
    l_min: float
    road: float
    shortfall: float
    shortfall_path: float
    l_min_slow: float
    shortfall_slow: float
    trail_m: float | None
    rear_m: float | None
    trail_ok: bool
    path_m: float
    duration_s: float


def _trail_points(rows, min_step: float):  # noqa: ANN001,ANN201
    """Rebuild the navigator's own decimated pose trail from the bag.

    Mirrors ``CoreNavigator._update_state``: a breadcrumb is appended only when
    it is at least ``POSE_TRAIL_MIN_STEP_M`` from the last one. Returns
    ``[(rel, x, y, yaw)]``; the ``POSE_TRAIL_LEN`` cap is applied at read time.
    """
    out: list[tuple[float, float, float, float]] = []
    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        if out:
            _, px, py, _ = out[-1]
            if math.hypot(d.pose_x - px, d.pose_y - py) < min_step:
                continue
        out.append((rel, d.pose_x, d.pose_y, d.pose_yaw))
    return out


def _commit_state(p, direction):  # noqa: ANN001,ANN201
    """First committed tick of a pass, plus the pass-side lateral projection."""
    rule = pass_side_lateral_axis(p.corridor, p.colour, direction)
    committed = [t for t in p.ticks if t.committed]
    if rule is None or not committed:
        return None
    axis, want = rule
    idx = 0 if axis is Axis.X else 1
    sign_lat = p.sign_x if idx == 0 else p.sign_y
    first = committed[0]
    off = ((first.pose_x if idx == 0 else first.pose_y) - sign_lat) * want
    return first, off


def _price(p, direction, rows_trail, scans, scan_times, tuning, margin, trail_len) -> Budget | None:  # noqa: ANN001,PLR0913
    row = _classify(p, direction, 0.15)
    state = _commit_state(p, direction)
    if row is None or state is None:
        return None
    first, off = state

    v = abs(first.speed) if first.speed else row.mean_speed
    if v <= 0.0:
        v = row.mean_speed or 0.20
    # Lateral the chassis still has to buy: the distance from its own offset up
    # to the legal side, plus a margin for the half-chassis that must clear.
    y_need = max(0.0, -off) + margin
    radius = radius_at(v)
    l_min = math.sqrt(4.0 * radius * y_need)
    radius_slow = radius_at(REVERSE_SPEED)
    l_min_slow = math.sqrt(4.0 * radius_slow * y_need)
    # Road still available at commit: the straight-line range to the committed
    # sign, which is what the navigator itself holds on that tick.
    road = first.rng

    # -- the shipped reverse gate, re-evaluated at this exact instant --------
    window = [t for t in rows_trail if t[0] <= first.rel][-trail_len:]
    trail_m = None
    if window:
        _, tx, ty, tyaw = window[-1]
        trail_m = trail_clearance_behind([(x, y, yaw) for _, x, y, yaw in window], tx, ty, tyaw)
    rear_m = None
    if scan_times:
        ranges, angles = _scan_to_ranges_angles(
            deserialize_message(nearest_by_time(scans, scan_times, first.rel), LaserScan)
        )
        rear_m = _rear_clearance(ranges, angles, tuning)

    shortfall = l_min - road
    need_rev = max(0.0, shortfall)
    # Obstacles-resolved, exactly as CoreNavigator resolves them (the bags are
    # all Obstacles rounds, where OBSTACLES_* overrides are live).
    clearance = tuning.clearance.for_obstacles_challenge()
    contact = clearance.CONTACT_DIST
    trail_ok = trail_m is not None and trail_m >= need_rev + contact

    return Budget(
        run=p.run,
        verdict=row.verdict,
        crossing=row.crossing,
        v_commit=v,
        y_need=y_need,
        radius=radius,
        l_min=l_min,
        road=road,
        shortfall=shortfall,
        shortfall_path=l_min - row.path_m,
        l_min_slow=l_min_slow,
        shortfall_slow=l_min_slow - road,
        trail_m=trail_m,
        rear_m=rear_m,
        trail_ok=trail_ok,
        path_m=row.path_m,
        duration_s=row.duration_s,
    )


def _pct(num: int, den: int) -> str:
    return f"{num:4d}/{den:4d} = {100.0 * num / den:5.1f}%" if den else "   n=0     "


def _dist(label: str, vals: list[float]) -> None:
    if not vals:
        print(f"   {label:<34} n=0")
        return
    s = sorted(vals)

    def q(f: float) -> float:
        return s[min(len(s) - 1, int(f * len(s)))]

    print(
        f"   {label:<34} n={len(s):4d}  mean {statistics.fmean(s):+.3f}  "
        f"p10 {q(0.10):+.3f}  p50 {q(0.50):+.3f}  p90 {q(0.90):+.3f}  max {s[-1]:+.3f}"
    )


def _report(label: str, rows: list[Budget], contact: float) -> None:
    n = len(rows)
    if not n:
        print(f"  {label:<28} n=0")
        return
    short = [r for r in rows if r.shortfall > 0]
    print(f"  {label:<28} n={n:4d}")
    print(
        f"      speed {statistics.fmean([r.v_commit for r in rows]):.3f} m/s"
        f"   R {statistics.fmean([r.radius for r in rows]):.3f} m"
        f"   y_need {statistics.fmean([r.y_need for r in rows]):.3f} m"
        f"   L_min {statistics.fmean([r.l_min for r in rows]):.3f} m"
        f"   road {statistics.fmean([r.road for r in rows]):.3f} m"
    )
    print(f"      SHORTFALL > 0                {_pct(len(short), n)}")
    _dist("      shortfall (all passes)", [r.shortfall for r in rows])
    _dist("      shortfall (short passes only)", [r.shortfall for r in short])
    slow_fixed = sum(1 for r in short if r.shortfall_slow <= 0)
    print(f"      of those, ERASED by approaching at {REVERSE_SPEED} m/s  {_pct(slow_fixed, len(short))}")
    ok = sum(1 for r in short if r.trail_ok)
    have_trail = sum(1 for r in short if r.trail_m is not None)
    print(
        f"      REAR: trail vouches for shortfall+{contact:.2f} m  {_pct(ok, len(short))}"
        f"   (trail present {_pct(have_trail, len(short))})"
    )
    tv = [r.trail_m for r in short if r.trail_m is not None]
    rv = [r.rear_m for r in short if r.rear_m is not None]
    _dist("      trail_clearance_behind (m)", tv)
    _dist("      rear LIDAR sector (m)", rv)
    print(f"      rear sector readable on {_pct(len(rv), len(short))} of short passes")


def main() -> None:  # noqa: PLR0915
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--margin",
        type=float,
        default=0.10,
        help="lateral margin past the sign line the chassis must reach (m); ~half chassis width",
    )
    parser.add_argument("--csv", default=None, help="dump every priced pass for offline slicing")
    parser.add_argument("--per-lap", action="store_true", help="judge each lap's pass separately")
    args = parser.parse_args()
    tuning = get_tuning(None)
    esc = tuning.escape.for_obstacles_challenge()
    # Obstacles-resolved, exactly as CoreNavigator resolves them (the bags are
    # all Obstacles rounds, where OBSTACLES_* overrides are live).
    clearance = tuning.clearance.for_obstacles_challenge()
    contact = clearance.CONTACT_DIST
    print(
        f"== shipped gate: POSE_TRAIL_LEN={esc.POSE_TRAIL_LEN} "
        f"MIN_STEP={esc.POSE_TRAIL_MIN_STEP_M} CONTACT_DIST={contact} "
        f"| R(v)={R0}+{RK}v  margin={args.margin}  reverse speed={REVERSE_SPEED}"
    )
    print()

    budgets: list[Budget] = []
    per_run: dict[str, list[Budget]] = {}
    run_span: dict[str, float] = {}
    read = skipped = 0
    for bag in args.bag_dirs:
        try:
            data, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        read += 1
        run = Path(bag).name.replace("run_", "")
        pillars, direction = _replay(run, data, frames, scans, tuning, args.per_lap)
        trail = _trail_points(data, esc.POSE_TRAIL_MIN_STEP_M)
        scan_times = [t for t, _ in scans]
        run_span[run] = (data[-1][0] - data[0][0]) if data else 0.0
        for p in pillars:
            b = _price(p, direction, trail, scans, scan_times, tuning, args.margin, esc.POSE_TRAIL_LEN)
            if b is not None:
                budgets.append(b)
                per_run.setdefault(run, []).append(b)

    print(f"== {read} bags read, {skipped} unreadable, {len(budgets)} pillars priced")
    correct = [b for b in budgets if b.verdict != "routing"]
    cross = [b for b in correct if b.crossing]
    hold = [b for b in correct if not b.crossing]
    cf = [b for b in cross if b.verdict == "execution"]
    cs = [b for b in cross if b.verdict == "ok"]
    print(f"   crossing {len(cross)} (fail {len(cf)}), already-legal {len(hold)} (fail {sum(1 for b in hold if b.verdict == 'execution')})")
    print()
    print("== 1+2  THE BUDGET, AND THE ROAD BEHIND")
    _report("crossing FAIL", cf, contact)
    print()
    _report("crossing SUCCESS (control)", cs, contact)
    print()
    _report("already-legal (control)", hold, contact)
    print()

    print("== 1  COVERAGE: what fraction of the crossing FAILURES a road-shortfall explains")
    print(f"   crossing failures with shortfall > 0           {_pct(sum(1 for b in cf if b.shortfall > 0), len(cf))}")
    print(f"   crossing SUCCESSES with shortfall > 0 (control) {_pct(sum(1 for b in cs if b.shortfall > 0), len(cs))}")
    for lo in (0.0, 0.1, 0.2, 0.3, 0.5):
        print(
            f"   shortfall > {lo:.2f} m: fail {_pct(sum(1 for b in cf if b.shortfall > lo), len(cf))}"
            f"   success {_pct(sum(1 for b in cs if b.shortfall > lo), len(cs))}"
        )
    print()

    print("== 1  FAILURE RATE conditioned on a positive shortfall (is the predictor any good?)")
    for label, pred in (
        ("shortfall > 0", lambda b: b.shortfall > 0),
        ("shortfall > 0.20 m", lambda b: b.shortfall > 0.20),
        ("road < L_min/2", lambda b: b.road < b.l_min / 2),
    ):
        yes = [b for b in cross if pred(b)]
        no = [b for b in cross if not pred(b)]
        print(
            f"   {label:<24} YES {_pct(sum(1 for b in yes if b.verdict == 'execution'), len(yes))}"
            f"   NO {_pct(sum(1 for b in no if b.verdict == 'execution'), len(no))}"
        )
    print()

    print("== 2  REAR AVAILABILITY over every short crossing pass (fail + success)")
    short = [b for b in cross if b.shortfall > 0]
    for need in (0.0, 0.10, 0.20, 0.30):
        ok = sum(
            1
            for b in short
            if b.trail_m is not None and b.trail_m >= max(b.shortfall, need) + contact
        )
        print(f"   trail covers max(shortfall, {need:.2f}) + {contact:.2f} m   {_pct(ok, len(short))}")
    rear_vals = [b.rear_m for b in short if b.rear_m is not None]
    print(f"   rear LIDAR sector readable at all       {_pct(len(rear_vals), len(short))}")
    if rear_vals:
        _dist("   rear sector when readable", rear_vals)
    print()

    print("== 3  TIME COST, per run, at 2 legs per firing")
    rates = []
    costs = []
    for run, bs in sorted(per_run.items()):
        crossings = [b for b in bs if b.verdict != "routing" and b.crossing]
        fires = [b for b in crossings if b.shortfall > 0 and b.trail_ok]
        would = [b for b in crossings if b.shortfall > 0]
        cost = sum(2.0 * max(b.shortfall, 0.10) / REVERSE_SPEED + 1.0 for b in fires)
        rates.append(len(fires))
        costs.append(cost)
        if len(per_run) <= 20:
            print(
                f"   {run:<20} crossings {len(crossings):3d}  want-reverse {len(would):3d}"
                f"  gate-allows {len(fires):3d}  cost {cost:6.1f} s  (run {run_span[run]:.0f} s)"
            )
    if rates:
        print(
            f"   ACROSS {len(rates)} runs: firings/run mean {statistics.fmean(rates):.2f}"
            f"  median {statistics.median(rates):.1f}  max {max(rates)}"
        )
        print(
            f"   added time/run: mean {statistics.fmean(costs):.1f} s"
            f"  median {statistics.median(costs):.1f} s  max {max(costs):.1f} s"
            f"   (round limit 180 s)"
        )
    print()

    print("== 1b ROAD MEASURED AS PATH ACTUALLY DRIVEN (commit range under-reads a curved approach)")
    print(f"   mean path driven: fail {statistics.fmean([b.path_m for b in cf]):.3f} m   success {statistics.fmean([b.path_m for b in cs]):.3f} m")
    print(f"   L_min > path driven:  fail {_pct(sum(1 for b in cf if b.shortfall_path > 0), len(cf))}"
          f"   success {_pct(sum(1 for b in cs if b.shortfall_path > 0), len(cs))}")
    _dist("   shortfall_path FAIL", [b.shortfall_path for b in cf])
    _dist("   shortfall_path SUCCESS", [b.shortfall_path for b in cs])
    for lo in (0.0, 0.2, 0.4):
        yes = [b for b in cross if b.shortfall_path > lo]
        no = [b for b in cross if b.shortfall_path <= lo]
        print(
            f"   fail rate | shortfall_path > {lo:.2f}   YES {_pct(sum(1 for b in yes if b.verdict == 'execution'), len(yes))}"
            f"   NO {_pct(sum(1 for b in no if b.verdict == 'execution'), len(no))}"
        )
    print()
    if args.csv:
        import csv as _csv
        from dataclasses import asdict as _asdict
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = _csv.DictWriter(fh, fieldnames=list(_asdict(budgets[0]).keys()))
            w.writeheader()
            for b in budgets:
                w.writerow(_asdict(b))
        print(f"== wrote {len(budgets)} rows to {args.csv}")
        print()

    print("== 4  THE CHEAPER ALTERNATIVE: approach the sign at the reverse speed instead")
    s_now = [b for b in cross if b.shortfall > 0]
    s_slow = [b for b in cross if b.shortfall_slow > 0]
    print(f"   crossing passes short at the measured speed   {_pct(len(s_now), len(cross))}")
    print(f"   crossing passes still short at {REVERSE_SPEED} m/s      {_pct(len(s_slow), len(cross))}")
    print(
        f"   erased by slowing alone                        "
        f"{_pct(sum(1 for b in s_now if b.shortfall_slow <= 0), len(s_now))}"
    )


if __name__ == "__main__":
    main()
