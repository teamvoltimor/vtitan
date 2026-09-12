"""Would ``SIDE_CORRECTION_BLENDS`` fire on HARDWARE, or is it inert there too?

The sim corpus cannot answer it: over 23,290 sighted ticks side_correction ran
1.09% of the time and the blend gate was satisfied on ZERO of them, because
every sim side_correction is in reverse. That made a 256-run A/B void.

The gate is: manoeuvre type is SIDE_CORRECTION **and speed >= 0**. So before
shipping the flag to the robot, the same question has to be asked of the bags,
which is the only instrument that has seen the forward regime. A flag that
gates on forward side_correction is worth deploying only if forward
side_correction is what the robot actually does.

Reports, over every side_correction tick in each bag, the split by commanded
manoeuvre speed. The control is the sim number above: if hardware also comes
back ~0% forward, the flag is inert on both and the whole lever is a dead end,
which is a finding rather than a failure.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_blend_reachability.py BAG [BAG ...]
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows  # noqa: E402

SIDE_CORRECTION = "side_correction"
SIM_FORWARD_SHARE = "0 of 254 side_correction ticks (sighted corpus, 2026-09-12)"


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    grand: Counter[str] = Counter()
    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        per: Counter[str] = Counter()
        for _t, s in rows:
            per["ticks"] += 1
            if s.active_maneuver_type is None:
                continue
            per["maneuver"] += 1
            if str(s.active_maneuver_type) != SIDE_CORRECTION:
                continue
            per["side_correction"] += 1
            spd = s.maneuver_speed_mps
            if spd is None:
                per["speed_missing"] += 1
            elif spd > 0:
                per["forward"] += 1
            elif spd < 0:
                per["reverse"] += 1
            else:
                per["zero"] += 1
        sc = per["side_correction"]
        # A zero-speed side correction still satisfies `speed >= 0.0`, so it
        # counts toward the gate even though it commands no motion.
        gate = per["forward"] + per["zero"]
        print(
            f"  {bag.name}: ticks {per['ticks']:5d}  side_correction {sc:5d}"
            f" ({100 * sc / per['ticks']:5.1f}%)  fwd {per['forward']:4d}  zero {per['zero']:4d}"
            f"  rev {per['reverse']:4d}  -> GATE OPEN on {gate:4d}"
            f" ({100 * gate / sc:5.1f}% of them)" if sc else f"  {bag.name}: no side_correction"
        )
        grand.update(per)

    sc = grand["side_correction"]
    print(f"\n== {len(bags)} bag(s), {grand['ticks']} ticks")
    if not sc:
        print("  no side_correction at all -- the flag cannot fire here either")
        return
    gate = grand["forward"] + grand["zero"]
    print(f"  side_correction ticks: {sc} ({100 * sc / grand['ticks']:.1f}% of all ticks)")
    print(f"    forward  {grand['forward']:5d} ({100 * grand['forward'] / sc:5.1f}%)")
    print(f"    zero     {grand['zero']:5d} ({100 * grand['zero'] / sc:5.1f}%)")
    print(f"    reverse  {grand['reverse']:5d} ({100 * grand['reverse'] / sc:5.1f}%)")
    if grand["speed_missing"]:
        print(f"    speed not recorded {grand['speed_missing']}")
    print(f"\n  GATE OPEN (speed >= 0) on {gate} of {sc} ticks ({100 * gate / sc:.1f}%)")
    print(f"  CONTROL, same gate in the sim corpus: {SIM_FORWARD_SHARE}")
    # A non-zero gate is NOT reachability. The question is whether flipping the
    # flag moves enough ticks to change a race, so the threshold is a share of
    # the manoeuvre it gates -- not "more than zero", which the first version of
    # this script used and which reported 8 ticks in three rounds as REACHABLE.
    share = gate / sc
    print(
        "\n  VERDICT: "
        + (
            f"REACHABLE -- the gate opens on {100 * share:.1f}% of side_correction"
            if share >= 0.10
            else f"EFFECTIVELY INERT -- the gate opens on only {100 * share:.1f}% of side_correction "
            f"({gate} ticks in {grand['ticks']}). Flipping the flag cannot move a race."
        )
    )
    if grand["reverse"] > 0.9 * sc:
        print(
            f"\n  AND THE REASON: {100 * grand['reverse'] / sc:.1f}% of side_correction is REVERSE."
            "\n  Per escape_recovery._side_correction_blends, a correction that switched to"
            "\n  reverse is `already_touching` -- the chassis is against something. So the"
            "\n  robot is not mis-steering around pillars, it is backing off contact it has"
            "\n  ALREADY made, and the blend flag was never the lever."
        )


if __name__ == "__main__":
    main()
