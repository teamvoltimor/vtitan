r"""In a CROSSING pass, does the planner ever ask for the crossing?

Background. ``diag_bag_pass_side.py`` splits sign passes into ROUTING (commanded
the wrong side), EXECUTION (commanded right, chassis went wrong) and ok. The
EXECUTION bucket is dominated by passes that had to CROSS from the wrong side to
the legal one, and those crossings fail regardless of speed, commit range or the
geometrically available arc -- that margin test is FLAT. So the failure is not
geometry. See ``adr:0051-sign-lane-planner`` for the measured verdict.

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

import sys
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)

from scripts.common.bag_io import create_bags_parser, read_vision_rows_and_scans  # noqa: E402
from scripts.common.cross_attempt import classify, replay  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402

if TYPE_CHECKING:
    from scripts.common.cross_attempt import Row


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


def main() -> int:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--lane-thresh", type=float, default=0.15)
    parser.add_argument("--per-lap", action="store_true", help="judge each lap's pass separately")
    args = parser.parse_args()
    tuning = get_tuning(None)
    sr = tuning.sign_router
    print(
        f"== shipped: sign_lane_planner={sr.sign_lane_planner} "
        f"SUPPRESS_DEFORM={sr.sign_lane_suppress_deform} "
        f"RAMP_M={sr.sign_lane_ramp_m} CORNER_ENTRY_M={sr.sign_lane_corner_entry_m} "
        f"SKIP_UNSAT={sr.sign_lane_skip_unsatisfiable} RELABEL_UNSAT={sr.sign_lane_relabel_unsatisfiable}"
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
        pillars, direction = replay(
            Path(bag).name.replace("run_", ""), data, frames, scans, tuning, args.per_lap
        )
        for p in pillars:
            row = classify(p, direction, args.lane_thresh)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
