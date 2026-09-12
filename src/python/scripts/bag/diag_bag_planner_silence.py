"""How long does the PLANNER go unrun while a manoeuvre holds the wheel?

Supersedes the "commitment eviction" reading of
``diag_bag_commit_survives_maneuver.py``, which was an ARTEFACT:
``committed_sign_x_m`` is populated only on the normal-drive path, and
``_drive_active_maneuver`` builds its snapshot from ``_base_debug``, which
omits it. The one exception is the ESCAPE_TRIGGERED tick, which is handed the
fully populated snapshot -- so a manoeuvre episode shows exactly one committed
tick and then none, and that reads as eviction when it is only reporting. The
router's ``_committed`` index persists in memory throughout; it is frozen, not
lost.

What is genuinely at stake is different and has to be measured on a field the
manoeuvre path does not suppress. ``steer_target_x`` is set in the same
normal-drive block, so it is subject to the same artefact and cannot be used
either. The honest measure is the one thing the early return provably changes:
the navigator publishes the manoeuvre's own steering, so the PLANNER's steering
is not merely unreported, it was never computed -- and the way to see that
without relying on a suppressed field is to measure how long the chassis goes
steering on a latched value.

So this reports, per manoeuvre episode:

  * length in ticks and seconds,
  * whether the latched steering is CONSTANT across the episode (a latched arc
    that no planner is correcting) or varies (retrace re-aims every tick),
  * the fraction of the whole run spent inside one.

A latched, constant-steering episode is time during which no sign, waypoint or
corridor input can influence the wheel, whatever the router remembers.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_planner_silence.py BAG [BAG ...]
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows  # noqa: E402


def main() -> None:
    bags = [Path(a) for a in sys.argv[1:]]
    if not bags:
        print(__doc__)
        raise SystemExit(2)

    grand: Counter[str] = Counter()
    all_lengths: list[int] = []
    constant_episodes = 0
    total_episodes = 0

    for bag in bags:
        rows, _ = load_nav_debug_rows(bag)
        n = len(rows)
        if not n:
            continue
        man = [r[1].active_maneuver_type is not None for r in rows]
        steer = [r[1].maneuver_steering for r in rows]
        times = [r[0] for r in rows]

        lengths: list[int] = []
        secs = 0.0
        const = 0
        i = 0
        while i < n:
            if not man[i]:
                i += 1
                continue
            j = i
            while j < n and man[j]:
                j += 1
            lengths.append(j - i)
            secs += times[min(j, n - 1)] - times[i]
            vals = {s for s in steer[i:j] if s is not None}
            const += int(len(vals) <= 1)
            i = j

        span = times[-1] - times[0]
        in_man = sum(man)
        print(
            f"  {bag.name}: {len(lengths):3d} episodes  {in_man}/{n} ticks"
            f" ({100 * in_man / n:4.1f}%)  {secs:5.1f}s of {span:5.1f}s"
            f" ({100 * secs / span:4.1f}%)  constant-steering episodes {const}/{len(lengths)}"
        )
        all_lengths += lengths
        constant_episodes += const
        total_episodes += len(lengths)
        grand["ticks"] += n
        grand["man_ticks"] += in_man

    if not all_lengths:
        print("\nno manoeuvre episodes")
        return
    s = sorted(all_lengths)
    print(f"\n== {total_episodes} episodes over {len(bags)} bag(s)")
    print(f"  episode length ticks: min {s[0]}  p50 {s[len(s) // 2]}  p90 {s[int(0.9 * len(s))]}  max {s[-1]}")
    print(f"  time inside a manoeuvre: {grand['man_ticks']} of {grand['ticks']} ticks "
          f"({100 * grand['man_ticks'] / grand['ticks']:.1f}%)")
    print(
        f"  episodes whose steering never changed: {constant_episodes} of {total_episodes}"
        f" ({100 * constant_episodes / total_episodes:.1f}%)"
    )
    print(
        "\n  Reading: a constant-steering episode is an open-loop arc. For its whole"
        "\n  length the wheel cannot respond to a sign, a waypoint or a corridor edge,"
        "\n  regardless of what the router still has committed in memory."
    )


if __name__ == "__main__":
    main()
