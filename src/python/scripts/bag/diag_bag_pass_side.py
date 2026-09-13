r"""Which side did the robot pass each sign, and which side was it TOLD to?

Reported from the track on 2026-09-07: green signs passed on the RIGHT when the
rule requires the LEFT, repeatedly, across laps and across runs. Three causes
produce that same symptom and they need completely different fixes, so the point
of this script is to tell them apart rather than to confirm the symptom:

* **DIRECTION** -- the pass-side rule is travel-relative, and ``ROUTING_TABLE``'s
  clockwise rows are the exact negation of its counterclockwise ones. A router
  running the wrong direction does not degrade the lane, it MIRRORS it: every
  colour passes on the wrong side, consistently, all race. Tell: commanded side
  is wrong for EVERY sign, and flipping the direction makes them all correct.
* **COLOUR** -- a green seen as red (or the reverse) flips that sign's side and
  no other. Tell: commanded side is wrong only where the believed colour
  disagrees with the camera's own votes.
* **EXECUTION** -- the router commanded the correct side and the chassis did not
  get there. Tell: commanded side is RIGHT and achieved side is wrong, with a
  small lateral magnitude.

The believed sign colour is not in ``/nav_debug``, so the sign map is rebuilt by
replaying the real router over the recorded detections -- the same approach as
``diag_bag_sign_target_churn.py``, which reproduces the bag's own commitment
churn (7.6x against 7.4x measured independently).

MEASURED 2026-09-07 on run_233653 (clockwise) and run_234457 (counterclockwise),
47 pillars:

    commanded the WRONG side (routing):          11
    commanded right, chassis went wrong (exec):  10
    correct:                                     26

DIRECTION IS NOT THE CAUSE -- a mirrored table would flip reds and greens alike,
and reds are mostly legal. Both remaining causes are present in similar numbers.

TRAP, and it moves the answer: judge with the SIGN's corridor, not the robot's.
The router keys the deformation off the candidate sign's own corridor, and the
two disagree on 22 of these 47 pillars (they legitimately differ at corners).
Judging by the robot's corridor instead reports 17 routing / 6 execution -- a
different conclusion from the same data.

HONOURED SINCE 2026-09-13, and it had NOT been until then: the verdict line read
``d.current_corridor`` while this docstring said not to. Every "routing" count
this script printed before that date is INFLATED, including the ones quoted
above. Re-measured on the 2026-09-12 bags, the robot's corridor gives 21 routing
errors in competition and 22 in practice where the sign's corridor gives 1 and
0 -- the router commands the legal side 196 times out of 197.

WHAT THE TRACE FOUND, which the counts do not show. On the wrong-side pass
reported from the track in run_234457, the router commanded the CORRECT side on
every tick (+0.25 to +0.29) and the chassis was on the wrong side throughout
(-0.32 closing to -0.16). It never crossed, because the sign was not committed
until 0.57 m -- and crossing sides inside 0.57 m with the measured 0.29 m turn
radius is geometrically impossible. The outcome was decided before the router
ever engaged, by the detector's range (p50 0.70 m).

And the colour was wrong for a structural reason: DUPLICATE TRACKS SPLIT THE
COLOUR EVIDENCE. That pillar carried nine tracks within half a metre; the router
committed to one with 4 hits and 2.29 of RED weight while its siblings held
20-30 hits and 18-24 of GREEN. Pooling votes within 0.30 m calls it green 40.8
to 11.2. Colour is resolved per FRAGMENT, but a fragment is not a physical
object.

TRAP: do NOT read ``wrong_side_pass_count`` for this. It judges from the
router's own believed layout, which on hardware includes phantom pillars and
positions off by half a metre; it has read 24 where the truth was 7.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_side.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)

from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.pass_side import collect_passes  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="override one sign_discovery field for the replay, e.g. --set SNAP_TO_LATTICE_M=0.40",
    )
    args = parser.parse_args()

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)
    if overrides:
        print(f"== sign_discovery overrides: {overrides}")

    rows_out = []
    ask_rows: list[tuple[str, str, str, float, float, float, float, str, bool]] = []
    peaks: list[tuple[str, int]] = []
    skipped: list[tuple[str, str]] = []
    tally = {"routing": 0, "execution": 0, "ok": 0}
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            # Older bags in the archive carry a metadata version this rosbag2
            # cannot open. A corpus sweep must not die on one of them, so the
            # skip is reported and counted rather than raised.
            skipped.append((Path(bag).name, type(exc).__name__))
            continue
        direction = settled_direction(rows)
        run = Path(bag).name.replace("run_", "")
        passes, peak = collect_passes(run, rows, frames, scans, tuning)
        peaks.append((run, peak))
        for p in passes:
            # commanded/achieved are already expressed as +1 when they match the
            # rule's required side, so the reading needs no second convention.
            if p.commanded < 0:
                verdict = "ROUTING: commanded wrong side"
                tally["routing"] += 1
            elif p.achieved < 0:
                verdict = "EXECUTION: commanded right, went wrong"
                tally["execution"] += 1
            else:
                verdict = "ok"
                tally["ok"] += 1
            rows_out.append(
                [run, str(direction), p.corridor, str(p.colour), round(p.lateral_m, 3), verdict]
            )
            # Signed against the rule's legal side, so a negative ask is the
            # router itself planning the wrong side and a negative achievement
            # with a positive ask is the chassis failing to follow it.
            ask_rows.append(
                (run, str(p.colour), p.corridor,
                 (p.commanded or 0) * p.commanded_m, p.achieved * p.lateral_m,
                 p.commit_commanded_m, p.commit_lateral_m, verdict,
                 p.maneuver_during_pass)
            )
    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(n for n, _ in skipped[:6])}")
        print()
    print("== PEAK BELIEVED SIGNS per run (the track holds at most 8)")
    for run, peak in peaks:
        print(f"  {run}  peak={peak:3d}{'   OVER THE PHYSICAL MAX' if peak > 8 else ''}")
    over = sum(1 for _, p in peaks if p > 8)
    print(f"  runs over the physical max: {over}/{len(peaks)}   worst={max((p for _, p in peaks), default=0)}")
    print()

    if not rows_out:
        print("No sign passes reconstructed from these bags.")
        return
    print("== PASS SIDE, required vs commanded vs achieved")
    print_table(rows_out, ["run", "direction", "corridor", "colour", "clearance m", "verdict"])
    print()
    by_colour: dict[str, dict[str, int]] = {}
    for _run, _direction, _corr, colour, _lat, verdict in rows_out:
        bucket = by_colour.setdefault(colour, {"routing": 0, "execution": 0, "ok": 0})
        bucket["routing" if verdict.startswith("ROUTING") else "execution" if verdict.startswith("EXEC") else "ok"] += 1
    print("  by colour (routing / exec / ok, and the routing rate):")
    for colour, b in sorted(by_colour.items()):
        n_total = b["routing"] + b["execution"] + b["ok"]
        print(
            f"    {colour:>16}  {b['routing']:4d} / {b['execution']:4d} / {b['ok']:4d}"
            f"   routing {100 * b['routing'] / n_total:5.1f}%  of {n_total}"
        )
    print()
    # ASKED vs ACHIEVED, both read at the closest tick, so the gap between the
    # two columns is tracking error and nothing else. This is the control that
    # decides whether a near-miss is a PLAN failure (the router asked for the
    # clearance it got) or a TRACKING failure (it asked for much more).
    print("== ASKED vs ACHIEVED at the closest tick (signed, + = legal side)")
    ask_table = [
        [run, colour, corr, round(ask, 3), round(got, 3), round(got - ask, 3),
         round(c_ask, 3), round(c_got, 3), verdict.split(":")[0]]
        for run, colour, corr, ask, got, c_ask, c_got, verdict, _man in ask_rows
    ]
    print_table(
        ask_table,
        ["run", "colour", "corridor", "ask m", "got m", "got-ask", "ask@commit", "pose@commit", "verdict"],
    )
    print()
    grazes = [r for r in ask_rows if abs(r[4]) < 0.030]
    print(f"  passes grazing under 30 mm: {len(grazes)} of {len(ask_rows)}")
    for label, subset in (("ALL passes", ask_rows), ("grazes <30mm", grazes)):
        if not subset:
            continue
        asks = [abs(r[3]) for r in subset]
        gots = [abs(r[4]) for r in subset]
        errs = [abs(r[4] - r[3]) for r in subset]
        n = len(subset)
        print(
            f"    {label:>14}  n={n:3d}  mean |ask|={sum(asks) / n:.3f}"
            f"  mean |got|={sum(gots) / n:.3f}  mean |got-ask|={sum(errs) / n:.3f}"
            f"  max |got-ask|={max(errs):.3f}"
        )
    # The two worlds the measurement has to separate, counted on the grazes.
    plan_fail = sum(1 for r in grazes if abs(r[3]) < 0.030)
    track_fail = sum(1 for r in grazes if abs(r[3]) >= 0.030)
    print(f"    of the grazes: PLAN asked <30mm too: {plan_fail}   TRACKING asked >=30mm: {track_fail}")
    # A tracking failure has to come from somewhere. The open chain says the
    # planner is silent during manoeuvres, so split the SAME error by whether a
    # manoeuvre was latched: if the error lives in the manoeuvre ticks, the
    # open-loop arcs are the mechanism; if it does not, they are exonerated.
    print()
    print("  tracking error split by whether a manoeuvre was latched while committed:")
    for label, subset in (
        ("manoeuvred", [r for r in ask_rows if r[8]]),
        ("clean", [r for r in ask_rows if not r[8]]),
    ):
        if not subset:
            print(f"    {label:>12}  n=  0")
            continue
        errs = [abs(r[4] - r[3]) for r in subset]
        graze = sum(1 for r in subset if abs(r[4]) < 0.030)
        n = len(subset)
        print(
            f"    {label:>12}  n={n:3d}  mean |got-ask|={sum(errs) / n:.3f}"
            f"  grazes {graze}/{n} = {100 * graze / n:5.1f}%"
        )
    print()
    print(f"  commanded the WRONG side (routing):        {tally['routing']}")
    print(f"  commanded right, chassis went wrong (exec): {tally['execution']}")
    print(f"  correct:                                    {tally['ok']}")


if __name__ == "__main__":
    main()
