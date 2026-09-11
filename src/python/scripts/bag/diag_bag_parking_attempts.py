"""Count what the HARDWARE bags actually show about parking, with controls.

Parking is recorded as geometrically blocked in simulation (0/240). This asks a
different question: did the real robot ever even TRY? It scans every bag under a
root and reports, per bag, whether any /nav_debug snapshot reported
``phase == parking``, a non-null ``park_phase``, or ``parking_engaged``.

A null here is only meaningful next to a control, so the same pass also counts
two phases known to occur (``bay_exit``, ``finished_hold``) and the laps reached:
if the controls are zero too the reader is broken, not the robot.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_parking_attempts.py data/live/runs
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows  # noqa: E402


def main() -> None:
    root = Path(sys.argv[1])
    prefix = sys.argv[2] if len(sys.argv) > 2 else "run_"
    bags = sorted(d for d in root.iterdir() if d.is_dir() and d.name.startswith(prefix))

    phase_totals: Counter[str] = Counter()
    park_phase_totals: Counter[str] = Counter()
    engaged_bags = 0
    park_phase_bags = 0
    parking_phase_bags = 0
    read_ok = 0
    read_fail = 0
    lap_hist: Counter[int] = Counter()
    hits: list[str] = []
    controller_bags: Counter[bool] = Counter()

    for bag in bags:
        try:
            rows, _ = load_nav_debug_rows(bag)
        except Exception as exc:  # noqa: BLE001
            read_fail += 1
            print(f"READ FAIL {bag.name}: {type(exc).__name__}", flush=True)
            continue
        if not rows:
            read_fail += 1
            print(f"NO NAV_DEBUG {bag.name}", flush=True)
            continue
        read_ok += 1

        phases = Counter(str(s.phase) for _, s in rows if s.phase is not None)
        phase_totals.update(phases)
        pp = Counter(str(s.park_phase) for _, s in rows if s.park_phase is not None)
        park_phase_totals.update(pp)
        engaged = sum(1 for _, s in rows if s.parking_engaged)
        not_none = sum(1 for _, s in rows if s.parking_engaged is not None)
        controller_bags[bool(not_none)] += 1
        laps = max((s.laps_completed or 0) for _, s in rows)
        lap_hist[laps] += 1

        if engaged:
            engaged_bags += 1
        if pp:
            park_phase_bags += 1
        if any("parking" in p for p in phases):
            parking_phase_bags += 1
        if engaged or pp or any("parking" in p for p in phases):
            hits.append(f"{bag.name}: laps={laps} engaged_ticks={engaged} park_phase={dict(pp)}")

    print(f"\nbags scanned: {len(bags)}  readable: {read_ok}  unreadable: {read_fail}")
    print(f"bags with ANY parking evidence: {len(hits)}")
    for h in hits:
        print("  " + h)
    print(f"\n  parking_engaged bags: {engaged_bags}")
    print(f"  park_phase bags:      {park_phase_bags}")
    print(f"  phase==parking bags:  {parking_phase_bags}")
    print(f"  bags where a ParkController EXISTS (parking_engaged non-null): {dict(controller_bags)}")
    print("\nCONTROL -- phase tick totals across all readable bags:")
    for p, c in phase_totals.most_common():
        print(f"  {p:26s} {c}")
    print("\nCONTROL -- park_phase tick totals:", dict(park_phase_totals) or "EMPTY")
    print("\nCONTROL -- max laps_completed per bag:")
    for laps, c in sorted(lap_hist.items()):
        print(f"  laps={laps}: {c} bags")


if __name__ == "__main__":
    main()
