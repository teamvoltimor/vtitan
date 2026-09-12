"""Does ``side_correction`` actually buy the pass it fires for?

Reported from the track on 2026-09-12, watching three rounds: "many of the
times it makes those corrections it still fails to pass on the correct side, or
still fails to avoid a collision." That is a causal claim about ONE manoeuvre
type, and the bag can test it -- but only against a control, because
side_correction fires precisely where the pass is already hard. A high failure
rate DURING correction proves nothing on its own; what matters is the rate
against passes where it never fired.

So every reconstructed pass is split on whether side_correction held the wheel
while the sign was committed, and each half is scored on the two outcomes the
operator named:

  * WRONG SIDE  -- the chassis ended on the illegal side (routing or execution).
  * GRAZE       -- it cleared the pillar by less than ``GRAZE_M``, which on a
                   real track is where the pillar gets nudged.

Two readings are possible and they demand opposite fixes, which is the point of
splitting rather than confirming:

  * side_correction is a SYMPTOM -- it fires on the hard passes and those fail
    anyway. Removing it changes nothing; fix what makes the pass hard.
  * side_correction is a CAUSE -- passes it touches fail at a rate the
    difficulty alone does not explain, and it steers AGAINST the router's
    requested side while doing it. Then the reactive layer is the lever.

The ``agrees``/``opposes`` columns separate those: they count manoeuvre ticks
steering toward vs away from the side the router asked for. A symptom does not
systematically oppose the router; a cause does.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_side_correction_outcome.py BAG [BAG ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.bag.diag_bag_pass_side import _load, _passes  # noqa: E402
from scripts.common.bag_io import settled_direction  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402

GRAZE_M = 0.030
"""Below this the pillar is being touched, not passed. Chosen from the track
report of pillars being MOVED, not from a tuning constant: the 2026-09-12 runs
recorded passes at 5, 6 and 9 mm, which is contact."""

SIDE_CORRECTION = "side_correction"


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    tuning = get_tuning(None)
    rows_out = []
    # [n, wrong_side, graze, agrees, opposes] per arm.
    arms = {"with side_correction": [0, 0, 0, 0, 0], "WITHOUT (control)": [0, 0, 0, 0, 0]}
    skipped = []

    for bag in bags:
        try:
            rows, frames, scans = _load(bag)
        except (RuntimeError, OSError, ValueError) as exc:
            skipped.append((bag.name, type(exc).__name__))
            continue
        direction = settled_direction(rows)
        run = bag.name.replace("run_", "")
        passes, _peak = _passes(run, rows, frames, scans, tuning)

        for p in passes:
            sc_ticks = p.maneuver_ticks.get(SIDE_CORRECTION, 0)
            arm = "with side_correction" if sc_ticks else "WITHOUT (control)"
            wrong = p.commanded < 0 or p.achieved < 0
            graze = p.lateral_m < GRAZE_M

            a = arms[arm]
            a[0] += 1
            a[1] += int(wrong)
            a[2] += int(graze)
            a[3] += p.manoeuvre_agrees
            a[4] += p.manoeuvre_opposes

            rows_out.append(
                [
                    run,
                    str(direction).replace("Direction.", "")[:4],
                    p.corridor,
                    str(p.colour).replace("SignColor.", ""),
                    sc_ticks,
                    p.maneuver_ticks.get("k_turn", 0),
                    round(p.lateral_m, 3),
                    "WRONG" if wrong else "ok",
                    "GRAZE" if graze else "",
                ]
            )

    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(n for n, _ in skipped[:6])}\n")
    if not rows_out:
        print("No sign passes reconstructed from these bags.")
        return

    print("== EVERY PASS, split by whether side_correction held the wheel")
    print_table(
        rows_out,
        ["run", "dir", "corridor", "colour", "sc ticks", "k_turn", "clearance m", "side", "contact"],
    )
    print()

    print("== OUTCOME BY ARM  (the control is the whole point)")
    hdr = f"  {'arm':>22} {'n':>4} {'wrong side':>18} {'graze <30mm':>18} {'steer agrees/opposes':>22}"
    print(hdr)
    for name, (n, wrong, graze, agrees, opposes) in arms.items():
        if not n:
            print(f"  {name:>22} {0:4d}   (no passes in this arm)")
            continue
        print(
            f"  {name:>22} {n:4d} {wrong:6d} ({100 * wrong / n:5.1f}%) {graze:6d} ({100 * graze / n:5.1f}%)"
            f" {agrees:10d} /{opposes:6d}"
        )

    a_n, a_wrong, a_graze, a_agr, a_opp = arms["with side_correction"]
    c_n, c_wrong, c_graze, _, _ = arms["WITHOUT (control)"]
    print()
    if a_n and c_n:
        print(
            f"  wrong-side rate: {100 * a_wrong / a_n:.1f}% with vs {100 * c_wrong / c_n:.1f}% without"
            f"   ({'WORSE' if a_wrong / a_n > c_wrong / c_n else 'no worse'} under correction)"
        )
        print(
            f"  graze rate:      {100 * a_graze / a_n:.1f}% with vs {100 * c_graze / c_n:.1f}% without"
            f"   ({'WORSE' if a_graze / a_n > c_graze / c_n else 'no worse'} under correction)"
        )
    # The arms above only see ticks where a sign was COMMITTED, and the
    # per-pass tick counts came out tiny. That is either (a) side_correction
    # avoiding sign passes, or (b) commitment covering so little of the run
    # that a small count is proportionate. Those demand opposite conclusions,
    # so the denominator is not optional.
    print()
    print("== DENOMINATOR: is the committed window just short?")
    tot_committed = tot_ticks = 0
    sc_in = sc_out = kt_in = kt_out = 0
    for bag in bags:
        try:
            rows, _frames, _scans = _load(bag)
        except (RuntimeError, OSError, ValueError):
            continue
        for _t, s in rows:
            tot_ticks += 1
            # committed_sign_x_m is the node's own record of commitment, so
            # this needs no replay and cannot drift from the arms above by
            # more than the replay itself does.
            committed = s.committed_sign_x_m is not None
            tot_committed += int(committed)
            if s.active_maneuver_type is None:
                continue
            name = str(s.active_maneuver_type)
            if committed:
                sc_in += int(name == SIDE_CORRECTION)
                kt_in += int(name == "k_turn")
            else:
                sc_out += int(name == SIDE_CORRECTION)
                kt_out += int(name == "k_turn")
    if tot_ticks:
        print(f"  ticks with a sign COMMITTED: {tot_committed} of {tot_ticks} ({100 * tot_committed / tot_ticks:.1f}%)")
        if tot_committed and tot_ticks - tot_committed:
            out_n = tot_ticks - tot_committed
            print(
                f"  side_correction rate:  {100 * sc_in / tot_committed:5.1f}% while committed"
                f"  vs {100 * sc_out / out_n:5.1f}% while not"
            )
            print(
                f"  k_turn rate:           {100 * kt_in / tot_committed:5.1f}% while committed"
                f"  vs {100 * kt_out / out_n:5.1f}% while not"
            )

    print()
    if a_agr + a_opp:
        print(
            f"  while correcting, the wheel steered AGAINST the router's side on"
            f" {100 * a_opp / (a_agr + a_opp):.1f}% of manoeuvre ticks ({a_opp} of {a_agr + a_opp})"
        )


if __name__ == "__main__":
    main()
