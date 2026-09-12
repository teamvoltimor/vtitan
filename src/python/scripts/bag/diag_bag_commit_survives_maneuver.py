"""When a manoeuvre starts, does the committed sign SURVIVE it?

Measured on 2026-09-12: across three hardware rounds a sign is committed on
54.9% of ticks, yet side_correction runs on 0.8% of committed ticks against
22.6% of uncommitted ones (k_turn: 0.7% vs 24.3%). Manoeuvres and commitment
are almost mutually exclusive, and the 28-35x separation has two readings that
demand OPPOSITE fixes:

  (a) SUPPRESSION -- the reactive layer is held off while a sign is being
      pursued. Healthy. The separation is the guard working.
  (b) EVICTION -- the manoeuvre CLEARS the commitment, so the robot forgets
      the pillar it was routing around. Then the separation is an artefact of
      wiping the numerator, and every "the escape avoids sign passes" reading
      built on it is backwards.

The two are told apart at the ONSET tick, not in aggregate. For each manoeuvre
onset this walks backwards over the preceding ticks and asks whether a sign was
committed just before, then forwards to ask whether the SAME sign is still
committed after. Eviction looks like: committed before, gone at or just after
onset. Suppression looks like: already uncommitted well before onset, because
the manoeuvre was never allowed to start while committed.

The control for both is the base rate: if commitment is absent before 45% of
ONSETS and absent on 45% of ALL ticks, onsets are telling us nothing.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_commit_survives_maneuver.py BAG [BAG ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows  # noqa: E402

LOOKBACK = 3
"""Ticks before onset to read commitment from. At ~20 Hz this is ~0.15 s -- long
enough to skip a single-tick dropout, short enough that the robot has not
driven past the pillar."""

LOOKAHEAD = 5
"""Ticks after onset to give commitment a chance to come back."""


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    grand = {"onsets": 0, "committed_before": 0, "survived": 0, "evicted": 0, "returned": 0}
    base_committed = base_ticks = 0

    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        n = len(rows)
        committed = [r[1].committed_sign_x_m is not None for r in rows]
        man = [str(r[1].active_maneuver_type) if r[1].active_maneuver_type is not None else None for r in rows]

        base_ticks += n
        base_committed += sum(committed)

        per = {"onsets": 0, "committed_before": 0, "survived": 0, "evicted": 0, "returned": 0}
        for i in range(1, n):
            if man[i] is None or man[i - 1] is not None:
                continue  # not an onset
            per["onsets"] += 1
            was = any(committed[max(0, i - LOOKBACK) : i])
            if not was:
                continue
            per["committed_before"] += 1
            # Held through the onset itself?
            if committed[i]:
                per["survived"] += 1
            else:
                per["evicted"] += 1
                if any(committed[i : min(n, i + LOOKAHEAD)]):
                    per["returned"] += 1

        for k in grand:
            grand[k] += per[k]
        cb = per["committed_before"]
        print(
            f"  {bag.name}: onsets {per['onsets']:4d}  committed just before {cb:4d}"
            f"  -> survived {per['survived']:4d}  EVICTED {per['evicted']:4d}"
            f"  (of which came back within {LOOKAHEAD} ticks: {per['returned']})"
        )

    print()
    o, cb = grand["onsets"], grand["committed_before"]
    print(f"== {o} manoeuvre onsets across {len(bags)} bag(s)")
    if not o:
        print("  no onsets -- nothing to conclude")
        return
    print(
        f"  a sign was committed in the {LOOKBACK} ticks before onset: {cb} ({100 * cb / o:.1f}% of onsets)"
    )
    print(
        f"  CONTROL, base rate of commitment over all ticks:           "
        f"{base_committed} of {base_ticks} ({100 * base_committed / base_ticks:.1f}%)"
    )
    if not cb:
        print("\n  VERDICT: SUPPRESSION -- a manoeuvre never starts while a sign is committed.")
        return
    ev = grand["evicted"]
    print(f"\n  of those {cb}: survived the onset {grand['survived']}, EVICTED {ev} ({100 * ev / cb:.1f}%)")
    print(f"  of the evicted, commitment returned within {LOOKAHEAD} ticks: {grand['returned']}")
    print(
        "\n  ONSET VERDICT: "
        + (
            "EVICTION -- the manoeuvre clears the commitment"
            if ev > grand["survived"]
            else "commitment mostly SURVIVES the onset"
        )
    )

    # Onsets counted per EPISODE cannot explain a per-TICK rate. If manoeuvres
    # start while committed (60%) and commitment survives the onset (87%), the
    # only way side_correction can still be 0.8% of committed ticks is that the
    # episodes starting committed are SHORT and the long ones start free. That
    # is a claim about episode LENGTH, so measure length.
    print("\n== EPISODE LENGTH, split by whether it started committed")
    buckets: dict[str, list[int]] = {"started committed": [], "started free": []}
    committed_share: dict[str, list[float]] = {"started committed": [], "started free": []}
    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        n = len(rows)
        committed = [r[1].committed_sign_x_m is not None for r in rows]
        man = [r[1].active_maneuver_type is not None for r in rows]
        i = 0
        while i < n:
            if not man[i]:
                i += 1
                continue
            j = i
            while j < n and man[j]:
                j += 1
            arm = "started committed" if any(committed[max(0, i - LOOKBACK) : i]) else "started free"
            buckets[arm].append(j - i)
            committed_share[arm].append(sum(committed[i:j]) / (j - i))
            i = j
    for arm, lens in buckets.items():
        if not lens:
            print(f"  {arm:>18}: none")
            continue
        share = sum(committed_share[arm]) / len(committed_share[arm])
        print(
            f"  {arm:>18}: {len(lens):4d} episodes  mean {sum(lens) / len(lens):6.1f} ticks"
            f"  median {sorted(lens)[len(lens) // 2]:4d}  total {sum(lens):5d} ticks"
            f"  |  committed for {100 * share:5.1f}% of the episode"
        )


if __name__ == "__main__":
    main()
