"""Probe: what actually stops the run once mapped signs are masked from escape.

The mapped/unmapped split withholds a routed sign's LIDAR returns from the
CRITICAL escape trigger, which was measured to be the gate. If the offset knob
is still inert afterwards, something else is holding the run — this reports the
candidates per tick so the answer is measured rather than guessed:

* ``raw`` / ``masked``: risk from the unmasked and masked scans. Ticks where
  they differ are the ones the split actually changed.
* ``k_turn``: CRITICAL escapes begun (the path the split suppresses).
* ``stuck``: escapes begun by ``StuckDetector`` instead — a completely separate
  trigger the split does not touch, and the obvious suspect if a masked robot
  merely creeps into the sign rather than reversing off it.
* ``creep`` / ``slow``: ticks where raw forward clearance or raw risk capped the
  speed. Under ``lidar_blind`` a sign affects neither; under the split it still
  affects both, which is the remaining behavioural difference between them.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/diag_escape_mask.py            # all 16 fixtures, summary
    python scripts/diag_escape_mask.py 0 --ticks  # per-tick detail for one
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import print_table
from shared.domain.enums import RiskLevel

from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

MAX_STEPS = 6000

_EXPECTED_ASSESS_CALLS = 2
"""``CoreNavigator.step`` assesses risk twice per tick with a sign router
present: once on the raw scan (speed) and once on the masked one (escape)."""


def probe(index: int, show_ticks: bool = False) -> Counter:
    """Run one fixture with the navigator instrumented; return event counts."""
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
    nav = sim._navigator  # noqa: SLF001 - a probe, by design
    counts: Counter = Counter()

    original_assess = nav._collision_controller.assess_risk  # noqa: SLF001 - a probe, by design
    original_begin = nav._begin_maneuver  # noqa: SLF001
    pending: list[RiskLevel] = []

    def assess(ranges: Any, angles: Any = None) -> RiskLevel:
        risk = original_assess(ranges, angles)
        pending.append(risk)
        return risk

    def begin(maneuver: Any) -> None:
        counts[f"begin_{maneuver.maneuver_type}"] += 1
        return original_begin(maneuver)

    nav._collision_controller.assess_risk = assess  # type: ignore[method-assign]  # noqa: SLF001
    nav._begin_maneuver = begin  # type: ignore[method-assign]  # noqa: SLF001

    def on_step(state: Any, _scan: Any) -> None:
        # step() calls assess_risk once on the raw scan and, when a sign router
        # is present, once more on the masked scan. Anything else means the
        # call sites changed and this probe is reading the wrong thing.
        if len(pending) == _EXPECTED_ASSESS_CALLS:
            raw, masked = pending
            counts["ticks"] += 1
            if raw != masked:
                counts[f"split_{raw}_to_{masked}"] += 1
            if raw == RiskLevel.CRITICAL:
                counts["raw_critical"] += 1
            if masked == RiskLevel.CRITICAL:
                counts["masked_critical"] += 1
            if show_ticks and raw != masked:
                print(f"  t={counts['ticks']:>4} ({state.x:.2f},{state.y:.2f}) raw={raw} masked={masked}")
        elif pending:
            counts[f"unexpected_assess_calls_{len(pending)}"] += 1
        pending.clear()

    result = sim.run(max_steps=MAX_STEPS, on_step=on_step)
    counts["laps"] = result.laps_completed
    counts["collided"] = int(result.collided)
    return counts


def main() -> None:
    """Probe one fixture or all of them."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("index", nargs="?", type=int)
    parser.add_argument("--ticks", action="store_true", help="print each tick the split changed")
    args = parser.parse_args()

    indices = [args.index] if args.index is not None else range(len(all_obstacles_demo_scenarios()))
    total: Counter = Counter()
    rows = []
    for i in indices:
        counts = probe(i, show_ticks=args.ticks)
        row = [f"FIXTURE {i:>2}"] + [counts.get(k, 0) for k in sorted(counts.keys())]
        rows.append(row)
        total.update(counts)
        print(f"FIXTURE {i:>2} " + "  ".join(f"{k}={v}" for k, v in sorted(counts.items())), flush=True)
    if len(list(indices)) > 1:
        print("TOTAL " + "  ".join(f"{k}={v}" for k, v in sorted(total.items())))


if __name__ == "__main__":
    main()
