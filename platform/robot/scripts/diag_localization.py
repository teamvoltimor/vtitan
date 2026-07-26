"""What does real state estimation cost the navigator?

Every closed-loop test drives the navigator on ground-truth pose, so
``LidarLocalizer`` — the position source the real robot actually navigates on —
is exercised by no closed-loop test at all. This runs the same scenarios both
ways and reports the difference, which is the honest measure of how much of the
current pass rate depends on perfect odometry.

Reports, per configuration:

* Open Challenge 3-lap solvability across all 8 narrow starts and 8 wide starts.
* Obstacles fixtures (collisions / laps).
* Position error of the estimate against ground truth.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_localization.py open
    python scripts/diag_localization.py obstacles
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.constants import CompetitionSpecs, CorridorDimensions
from shared.config.enums import Direction, Section

from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.gateway import ScenarioSimulator
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios, all_test_scenarios

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_STARTS = list(product(Section, Direction))
_OBSTACLES_MAX_STEPS = 6000
_WIDTH_MATCH_TOLERANCE_M = 1e-6


def _run_open(args: tuple[int, int, bool]) -> tuple[bool, int, bool, float]:
    index, width_mm, localize = args
    section, direction = _STARTS[index]
    meta = build_open_metadata(uniform_widths(width_mm), section, direction)
    sim = ScenarioSimulator(meta, num_laps=_N_LAPS, use_lidar_localization=localize)
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return within, result.laps_completed, result.collided, peak[0]


def _run_obstacles(args: tuple[int, bool]) -> tuple[bool, int, float]:
    index, localize = args
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        use_lidar_localization=localize,
    )
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(max_steps=_OBSTACLES_MAX_STEPS, on_step=on_step)
    return result.collided, result.laps_completed, peak[0]


def report_open(workers: int) -> None:
    """Open Challenge 3-lap solvability, perfect pose vs LIDAR-estimated pose."""
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for width_mm, name in ((_NARROW_MM, "narrow"), (_WIDE_MM, "wide")):
            for localize in (False, True):
                jobs = [(i, width_mm, localize) for i in range(len(_STARTS))]
                results = list(pool.map(_run_open, jobs))
                passed = sum(1 for within, _, _, _ in results if within)
                collided = sum(1 for _, _, c, _ in results if c)
                worst_laps = min(laps for _, laps, _, _ in results)
                peak = max(p for _, _, _, p in results)
                pose = "LIDAR-estimated" if localize else "ground truth   "
                print(
                    f"OPEN {name:<6} pose={pose}  pass {passed}/{len(_STARTS)}  "
                    f"collided {collided}/{len(_STARTS)}  min_laps {worst_laps}  "
                    f"peak_pos_err {peak * 100:5.1f}cm",
                    flush=True,
                )


def report_obstacles(workers: int) -> None:
    """Obstacles fixtures, perfect pose vs LIDAR-estimated pose."""
    count = len(all_obstacles_demo_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for localize in (False, True):
            results = list(pool.map(_run_obstacles, [(i, localize) for i in range(count)]))
            collisions = sum(1 for c, _, _ in results if c)
            laps1 = sum(1 for _, laps, _ in results if laps >= 1)
            laps3 = sum(1 for _, laps, _ in results if laps >= _N_LAPS)
            peak = max(p for _, _, p in results)
            pose = "LIDAR-estimated" if localize else "ground truth   "
            print(
                f"OBSTACLES pose={pose}  collisions {collisions}/{count}  "
                f"laps>=1 {laps1}/{count}  laps>=3 {laps3}/{count}  "
                f"peak_pos_err {peak * 100:5.1f}cm",
                flush=True,
            )


def _run_blind(args: tuple[int]) -> tuple[str, bool, int, bool, bool, float]:
    """One fixture with the layout withheld — LIDAR + IMU only."""
    (index,) = args
    scenario = all_test_scenarios()[index]
    true_widths = corridor_widths_from_metadata(scenario.metadata)
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True)
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    believed = sim.believed_widths or {}
    layout_ok = all(abs(believed.get(s, -1) - w) < _WIDTH_MATCH_TOLERANCE_M for s, w in true_widths.items())
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return scenario.label, within, result.laps_completed, result.collided, layout_ok, peak[0]


def report_blind(workers: int) -> None:
    """The 28 fixtures with no layout knowledge at all."""
    count = len(all_test_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(_run_blind, [(i,) for i in range(count)]))
    passed = sum(1 for _, w, _, _, _, _ in results if w)
    collided = sum(1 for _, _, _, c, _, _ in results if c)
    layout = sum(1 for _, _, _, _, ok, _ in results if ok)
    peak = max(p for _, _, _, _, _, p in results)
    print(
        f"BLIND  pass {passed}/{count}  collided {collided}/{count}  "
        f"layout_learned {layout}/{count}  peak_pos_err {peak * 100:5.1f}cm",
        flush=True,
    )
    for label, within, laps, coll, ok, err in results:
        if not (within and ok):
            print(
                f"    {label:<34} pass={within} laps={laps}/3 collided={coll} layout_ok={ok} pos_err={err * 100:.1f}cm",
                flush=True,
            )


def _run_open_fixture(args: tuple[int, bool]) -> tuple[str, bool, int, bool, float]:
    index, localize = args
    scenario = all_test_scenarios()[index]
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        use_lidar_localization=localize,
    )
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return scenario.label, within, result.laps_completed, result.collided, peak[0]


def report_open_fixtures(workers: int) -> None:
    """The 28 Go-generated Open Challenge fixtures — the centre-wall variations.

    These are the layouts ``sim:navigate:visualize --challenge open`` steps
    through, and no test drives them: ``test_scenario_catalog.py`` only asserts
    there are 28 and that label lookup works, while the closed-loop Open tests
    build their own symmetric/mixed layouts instead. So this is the only place
    the generated wall variations are actually run.
    """
    count = len(all_test_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for localize in (False, True):
            results = list(pool.map(_run_open_fixture, [(i, localize) for i in range(count)]))
            passed = sum(1 for _, w, _, _, _ in results if w)
            collided = sum(1 for _, _, _, c, _ in results if c)
            peak = max(p for _, _, _, _, p in results)
            pose = "LIDAR-estimated" if localize else "ground truth   "
            print(
                f"OPEN-FIXTURES pose={pose}  pass {passed}/{count}  "
                f"collided {collided}/{count}  peak_pos_err {peak * 100:5.1f}cm",
                flush=True,
            )
            for label, within, laps, coll, err in results:
                if not within:
                    print(
                        f"    FAIL {label:<34} laps={laps}/3 collided={coll} pos_err={err * 100:.1f}cm",
                        flush=True,
                    )


def main() -> None:
    """Run the requested comparison."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["open", "obstacles", "open-fixtures", "blind"])
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    if args.mode == "open":
        report_open(args.workers)
    elif args.mode == "open-fixtures":
        report_open_fixtures(args.workers)
    elif args.mode == "blind":
        report_blind(args.workers)
    else:
        report_obstacles(args.workers)


if __name__ == "__main__":
    main()
