r"""The router commanded the CORRECT side and the chassis did not get there.

``diag_bag_pass_side.py`` splits sign passes three ways -- routing error,
execution error, correct -- and on the 78-bag corpus (2026-09-06 to 09-10) the
execution bucket is **141 of 654 passes, 21.6%**, the same order as the routing
bucket that has had every session's attention. Nothing had ever been said about
what is inside it, so this script opens it.

The question is NOT "did the chassis miss", which is already the verdict. It is
**whether the pass was still winnable at the moment the router committed to it**.
Three answers want three different fixes and they are indistinguishable in the
outcome:

* **MANOEUVRE** -- an escape or stuck recovery was latched while the sign was
  committed. The steering was not the router's to give, so the sign lane cannot
  be tuned out of this one; it is the escape's cost, billed to the wrong lane.
* **NO ROOM** -- at commit the chassis was already on the wrong side and the
  remaining distance to the pillar cannot buy the lateral displacement needed,
  at the turn radius that speed actually permits. The outcome was decided
  BEFORE the router engaged, and only earlier commitment can move it. This is
  the 2026-09-07 run_234457 trace generalised: commanded correct on every tick,
  wrong side throughout, committed at 0.57 m.
* **HAD ROOM** -- the geometry allowed it and the chassis still did not go.
  Only this residue is a tracking/authority problem, and only this residue is
  what the known 35-40% lateral execution shortfall can be about.

THE CONTROL MATTERS MORE THAN THE BUCKETS. "No room" is only an explanation if
the passes that SUCCEEDED had room, so every bucket is also reported over the
``ok`` passes. A geometry test that fires just as often on the successes
explains nothing, and would mean the model below is too strict rather than the
commitment too late.

The feasibility bound is deliberately GENEROUS, so that a "no room" verdict is
hard to earn:

* the whole euclidean range to the pillar is treated as usable along-track
  distance, which over-counts it;
* a SINGLE arc is allowed, not the S-curve a real pass needs, which roughly
  doubles the reachable offset;
* the radius is the speed-dependent one (``R = intercept + slope*|v|``, capped),
  measured 2026-09-10 over 33 bags, rather than the 0.29 m constant -- and at
  the speeds a sign pass runs, that curve is SMALLER than the constant, so it
  grants more room, not less.

Anything the model still calls impossible is impossible with room to spare.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_exec_failures.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.bag.diag_bag_pass_side import Pass, _load, _passes  # noqa: E402
from scripts.common.bag_io import create_bags_parser  # noqa: E402
from scripts.common.stats import fmt_p50_p90, percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402


def turn_radius_at(speed_mps: float | None) -> float:
    """The radius this chassis actually achieves at ``speed_mps``.

    ``RobotSpecs.MIN_TURN_RADIUS_M`` (0.29) is this curve's value at ONE speed,
    about 0.118 m/s. Using the curve rather than the constant is what keeps the
    bound honest at the speeds a sign pass runs.
    """
    if speed_mps is None:
        return RobotSpecs.MIN_TURN_RADIUS_M
    return min(
        RobotSpecs.MIN_TURN_RADIUS_CAP_M,
        RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M + RobotSpecs.MIN_TURN_RADIUS_SLOPE_S * abs(speed_mps),
    )


def reachable_lateral_m(along_m: float, radius_m: float) -> float:
    """Lateral offset a single arc of ``radius_m`` buys over ``along_m`` ahead.

    Past a quarter turn the chassis is no longer closing on the pillar, so the
    bound saturates at the radius itself.
    """
    if along_m >= radius_m:
        return radius_m
    return radius_m - math.sqrt(max(0.0, radius_m * radius_m - along_m * along_m))


def classify(p: Pass) -> tuple[str, float, float]:
    """Bucket one pass, and return the lateral it NEEDED vs what it could have.

    ``commit_lateral_m`` is signed with positive meaning the legal side, so a
    negative value is the distance the chassis had to cross from scratch.
    """
    needed = max(0.0, -p.commit_lateral_m)
    radius = turn_radius_at(p.commit_speed_mps)
    available = reachable_lateral_m(p.commit_range_m, radius)
    if p.maneuver_during_pass:
        return "manoeuvre", needed, available
    if needed > available:
        return "no room", needed, available
    return "had room", needed, available


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="override one sign_discovery field for the replay, e.g. --set LIDAR_RANGE_FUSION=false",
    )
    args = parser.parse_args()

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)
    if overrides:
        print(f"== sign_discovery overrides: {overrides}")

    skipped: list[str] = []
    passes: list[Pass] = []
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            skipped.append(f"{Path(bag).name} ({type(exc).__name__})")
            continue
        run = Path(bag).name.replace("run_", "")
        found, _peak = _passes(run, rows, frames, scans, tuning)
        passes.extend(found)

    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(skipped[:6])}")
        print()
    if not passes:
        print("No sign passes reconstructed from these bags.")
        return

    # The same three-way split diag_bag_pass_side.py reports, so the execution
    # count here is checkable against that script rather than a second opinion.
    groups: dict[str, list[Pass]] = {"execution": [], "ok": [], "routing": []}
    for p in passes:
        groups["routing" if p.commanded < 0 else "execution" if p.achieved < 0 else "ok"].append(p)

    print(f"== {len(passes)} passes: {len(groups['routing'])} routing, "
          f"{len(groups['execution'])} execution, {len(groups['ok'])} ok")
    print()

    print("== WHY THE EXECUTION FAILURES FAILED -- and the same test on the passes that WORKED")
    print("   ('ok' is the control: a bucket that fires as often there explains nothing)")
    rows_out = []
    for bucket in ("manoeuvre", "no room", "had room"):
        cells = []
        for group in ("execution", "ok"):
            members = [p for p in groups[group] if classify(p)[0] == bucket]
            share = 100 * len(members) / len(groups[group]) if groups[group] else 0.0
            cells.append(f"{len(members):4d}  {share:5.1f}%")
        rows_out.append([bucket, *cells])
    print_table(rows_out, ["bucket", "execution (n / %)", "ok (n / %)"])
    print()

    for group in ("execution", "ok"):
        members = groups[group]
        if not members:
            continue
        needed = [classify(p)[1] for p in members]
        available = [classify(p)[2] for p in members]
        print(f"  {group}:")
        print(f"    commit range           {fmt_p50_p90([p.commit_range_m for p in members])}")
        speeds = [p.commit_speed_mps for p in members if p.commit_speed_mps is not None]
        print(f"    commit speed           {fmt_p50_p90(speeds, unit='m/s') if speeds else 'unpublished'}")
        print(f"    radius that speed buys {fmt_p50_p90([turn_radius_at(p.commit_speed_mps) for p in members])}")
        print(f"    lateral NEEDED         {fmt_p50_p90(needed)}")
        print(f"    lateral AVAILABLE      {fmt_p50_p90(available)}")
        print(f"    already on the legal side at commit: "
              f"{sum(1 for p in members if p.commit_lateral_m >= 0)}/{len(members)}")
        print()

    # The bucket table above answers "was it geometrically possible". This
    # answers the prior question -- was the chassis being asked to CROSS at all
    # -- and it is the one that separates, because a pass that starts on the
    # legal side only has to stay there.
    print("== DID THE PASS REQUIRE A CROSSING?  (side of the chassis at commit)")
    rows_cross = []
    for label, wanted in (("started WRONG side", False), ("started legal side", True)):
        cells = []
        for group in ("execution", "ok"):
            # bool(): the comparison can yield a numpy bool, and ``is`` against
            # a Python bool is then silently False for BOTH branches.
            members = [p for p in groups[group] if bool(p.commit_lateral_m >= 0) is wanted]
            share = 100 * len(members) / len(groups[group]) if groups[group] else 0.0
            cells.append(f"{len(members):4d}  {share:5.1f}%")
        rows_cross.append([label, *cells])
    print_table(rows_cross, ["at commit", "execution (n / %)", "ok (n / %)"])
    print()
    # Read the other way: of every pass that had to cross, how many made it?
    crossers = [p for p in passes if p.commanded >= 0 and p.commit_lateral_m < 0]
    holders = [p for p in passes if p.commanded >= 0 and p.commit_lateral_m >= 0]
    for label, members in (("had to CROSS", crossers), ("already legal", holders)):
        if not members:
            continue
        won = sum(1 for p in members if p.achieved >= 0)
        print(f"    {label:>14}: {won}/{len(members)} ended on the legal side "
              f"({100 * won / len(members):.1f}%)")
    print()

    # If the failures commit systematically later than the ones that worked,
    # the fix is upstream of the sign lane entirely: the detector's range, not
    # the chassis.
    print("== IS IT LATE COMMITMENT?  commit range by outcome")
    for label, members in (("execution", groups["execution"]), ("ok", groups["ok"])):
        ranges = [p.commit_range_m for p in members]
        if not ranges:
            continue
        print(f"    {label:>9}  p10 {percentile(ranges, 0.1):.3f}  p50 {percentile(ranges, 0.5):.3f}"
              f"  p90 {percentile(ranges, 0.9):.3f}   n={len(ranges)}")


if __name__ == "__main__":
    main()
