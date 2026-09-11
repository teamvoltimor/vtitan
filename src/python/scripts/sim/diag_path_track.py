"""Run a blind Open Challenge scenario and record path-tracking error.

Compares the robot's ground-truth pose against both:

- the planned path as the navigator believes it (the RViz-displayed path)
- the planned path against the true track geometry

Outputs one CSV per scenario with per-tick cross-track errors and the current
belief/true corridor widths.

Also instruments WAYPOINT INDEX ADVANCEMENT, added 2026-08-30 to chase a stall
seen on hardware: at the first corner of ``run_20260830_013702`` indices 8 and 9
held 2.10 s and 2.95 s against a 0.4-1.3 s norm, immediately after a 1.3 m pose
jump. A frozen index means a stale lookahead target, which is a candidate cause
of both the Obstacles sign cross-track error and the unexplained escape rate.

``navigator.py`` has two ways past a waypoint and this script records whether
each one is available on every tick:

``next_closer``
    The next waypoint is strictly closer than the current one. Always active.
``wp_behind``
    The current waypoint reads as behind the chassis in its own local frame.
    This is the ``STALE_TARGET_RESCUE`` path, and it is **doubly disabled in the
    shipped configuration** -- the flag defaults to ``False``, and the branch is
    additionally gated on ``self._sign_router is not None``, which is ``None``
    for the Open Challenge. So a stall where ``wp_behind`` is true but
    ``next_closer`` is false is a stall the existing rescue was written for and
    is not permitted to fix.

Both tests are computed here in the navigator's own BELIEVED frame, read off
``navigator._debug``, not from the ground-truth state -- the advance loop runs
on believed pose, so testing against truth would measure a different thing.

Usage (from ``src``, with PYTHONPATH=".")::

    python scripts/sim/diag_path_track.py --scenario 450
"""

from __future__ import annotations

import os

os.environ.setdefault("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from multiprocessing import Pool
from pathlib import Path
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from typing import TYPE_CHECKING

from shared.domain.enums import Section
from shared.domain.models import ScenarioMetadata, Waypoint

from scripts.common.diag_base import add_tuning_arg, load_tuning
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.track_geometry import project_onto_path
from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from src.simulation.scenario_catalog import (
    _OPEN_CHALLENGE_SPACE,
    all_obstacles_demo_scenarios,
    open_scenario_by_index,
)
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState
    from src.simulation.scenario_catalog import NamedScenario
    from src.simulation.scenario_result import SimResult


def _transform_point(
    x_b: float,
    y_b: float,
    believed: tuple[float, float, float],
    true: tuple[float, float, float],
) -> tuple[float, float]:
    """Map a point from the believed frame to the true/map frame."""
    bx, by, byaw = believed
    tx, ty, tyaw = true
    delta_yaw = (tyaw - byaw + math.pi) % (2 * math.pi) - math.pi
    cos_d, sin_d = math.cos(delta_yaw), math.sin(delta_yaw)
    offset_x = tx - (bx * cos_d - by * sin_d)
    offset_y = ty - (bx * sin_d + by * cos_d)
    x_m = x_b * cos_d - y_b * sin_d + offset_x
    y_m = x_b * sin_d + y_b * cos_d + offset_y
    return x_m, y_m


def _displayed_path(
    waypoints: list[Waypoint],
    believed: tuple[float, float, float],
    true: tuple[float, float, float],
) -> list[Waypoint]:
    """Return the believed waypoints transformed into the map frame."""
    return [Waypoint(*_transform_point(wp.x, wp.y, believed, true)) for wp in waypoints]


_DEFAULT_OUTPUT_DIR = Path("scripts/sim/output")

_SIM_DT_S = 0.05
"""Matches ``ScenarioSimulator.run``'s default ``dt``. Only used to report stall
durations in seconds so they are comparable with the hardware bag figures."""


@dataclass(frozen=True, slots=True)
class WaypointProbeRow:
    """One tick's waypoint index and both advance tests, read off the navigator."""

    waypoint_index: int
    dist_to_wp_m: float | None
    wp_behind: bool | None
    next_closer: bool | None
    phase: str | None


class DwellRun(NamedTuple):
    """Consecutive ticks collapsed into a single waypoint-index dwell."""

    index: int
    start_step: int
    rows: list[WaypointProbeRow]


def _index_probe(navigator) -> WaypointProbeRow:  # noqa: ANN001 - diagnostic peek at private state
    """Read the waypoint index and both advance tests from the navigator.

    Everything here is taken from ``_debug``'s pose rather than the simulator's
    ground truth, because the advance loop in ``navigator.step`` runs on
    believed pose. Returns empty-valued fields on ticks where pose is not yet
    known (startup), so the CSV keeps one row per tick.
    """
    debug = getattr(navigator, "_debug", None)  # noqa: SLF001
    waypoints = navigator._waypoints  # noqa: SLF001
    index = navigator._waypoint_index  # noqa: SLF001
    phase = getattr(getattr(debug, "phase", None), "name", None)
    if debug is None or debug.pose_x is None or not waypoints:
        return WaypointProbeRow(
            waypoint_index=index,
            dist_to_wp_m=None,
            wp_behind=None,
            next_closer=None,
            phase=phase,
        )

    count = len(waypoints)
    wp = waypoints[index % count]
    nxt = waypoints[(index + 1) % count]
    dx, dy = wp.x - debug.pose_x, wp.y - debug.pose_y
    cos_yaw, sin_yaw = math.cos(debug.pose_yaw), math.sin(debug.pose_yaw)
    return WaypointProbeRow(
        waypoint_index=index,
        dist_to_wp_m=math.hypot(dx, dy),
        # The same dot product navigator.py uses for `raw_behind`.
        wp_behind=(dx * cos_yaw + dy * sin_yaw) <= 0,
        next_closer=nxt.distance_to_xy(debug.pose_x, debug.pose_y) < math.hypot(dx, dy),
        phase=phase,
    )


def _dwell_runs(rows: list[WaypointProbeRow]) -> list[DwellRun]:
    """Collapse per-tick rows into (index, start_step, ticks-at-that-index) runs."""
    runs: list[DwellRun] = []
    for step, row in enumerate(rows):
        if runs and runs[-1].index == row.waypoint_index:
            runs[-1].rows.append(row)
        else:
            runs.append(DwellRun(index=row.waypoint_index, start_step=step, rows=[row]))
    return runs


def open128_scenarios() -> list:
    """The same 128-scenario corpus the Open A/B harness reports against.

    ``open_scenario_by_index(0..127)`` is NOT this set -- it is the first 128
    entries of the enumeration, which is one corner of the space (all clockwise,
    three sections). Matching ``diag_open_direction_gates``' ``open128`` filter
    instead keeps stall counts comparable with the 126/128 pass figures rather
    than describing a slice nothing else is measured on.
    """
    return [p.to_named_scenario() for p in _OPEN_CHALLENGE_SPACE.all_params() if p.start_cell == 0]


def batch_scenarios(corpus: str) -> list:
    """The scenario set a ``--batch`` run measures over.

    ``open``
        The 128-scenario Open corpus (see :func:`open128_scenarios`). Failures
        here are dominated by ``over_time``, so this corpus answers "do stalls
        cost lap time".
    ``obstacles``
        The 256-scenario Obstacles corpus. Failures here are dominated by SIGN
        COLLISIONS, so this is the corpus that tests whether an index stall
        produces the stale lookahead target behind the sign-pass cross-track
        error -- which was the reason to investigate stalls at all.

    Both exist because the two challenges fail in different ways and a stall
    that costs time is not evidence of a stall that costs clearance.
    """
    if corpus == "obstacles":
        return all_obstacles_demo_scenarios(CORPUS_DIR)
    return open128_scenarios()


def _batch_one(args: tuple[int, float, str]) -> dict:
    """Run one scenario and reduce it to stall counters plus the verdict.

    Module level and taking a single picklable argument on purpose: the pool
    SPAWNS on Windows, so a closure or a locally-defined worker cannot be
    unpickled in the child and the whole batch dies with an AttributeError on
    ``__main__``. That is exactly how the first attempt at this failed.
    """
    scenario_index, stall_multiple, corpus = args
    scenario = batch_scenarios(corpus)[scenario_index]
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=getattr(scenario, "laps", 3),
        seed=getattr(scenario, "seed", None),
        blind=True,
        use_lidar_localization=True,
        tuning=load_tuning(None),
    )
    probes: list[WaypointProbeRow] = []

    def on_step(state, scan) -> None:  # noqa: ANN001, ARG001 - callback signature
        probes.append(_index_probe(sim.navigator))

    result = sim.run(on_step=on_step, max_steps=OBSTACLES_MAX_STEPS)

    runs = _dwell_runs(probes)
    dwells = sorted(len(run.rows) for run in runs)
    # RELATIVE, not an absolute tick count. Runs differ two-fold in how long
    # they dwell per waypoint -- a narrow-corridor scenario creeps and spends
    # ~26 ticks on an ordinary index where a wide one spends ~9 -- so a fixed
    # threshold labels every index of a slow run a "stall" and then reports
    # that stalls are everywhere. Measured against the run's OWN median, a
    # stall means "this index held far longer than this run's normal", which is
    # the thing worth comparing across the corpus. The first cut of this used
    # 30 ticks flat and reported 57% of ticks stalled in PASSING runs, which is
    # the signature of a threshold measuring speed rather than stalling.
    median = dwells[len(dwells) // 2] if dwells else 0
    floor = max(median * stall_multiple, 1)
    stalls = [run for run in runs if len(run.rows) >= floor]
    behind = [p.wp_behind for run in stalls for p in run.rows if p.wp_behind is not None]
    return {
        "idx": scenario_index,
        "label": scenario.label,
        "ok": bool(result.success),
        "laps": result.laps_completed,
        "collided": bool(result.collided),
        "timed_out": bool(result.timed_out),
        # `success` is a FOUR-term conjunction and the other two terms are not
        # implied by the first pair: a run can drive its three laps, never touch
        # a wall, never hit the simulator's step budget, and still fail on
        # `over_time` (the 180 s competition round limit) or on `parked`.
        # Recorded separately because a failure bucket that cannot say WHICH
        # term failed cannot be correlated with anything.
        "over_time": bool(result.over_time),
        "parked": result.parked,
        "pass_side": bool(result.pass_side_violation),
        "sim_time_s": result.sim_time_s,
        "ticks": len(probes),
        "indices": len(runs),
        "median_dwell": median,
        "stall_floor": floor,
        "stalls": len(stalls),
        "stall_ticks": sum(len(run.rows) for run in stalls),
        "max_dwell": max(dwells, default=0),
        "dwell_ratio": (max(dwells) / median) if median else None,
        "behind_frac": (sum(behind) / len(behind)) if behind else None,
    }


def report_batch(results: list[dict], *, stall_multiple: float) -> None:
    """Split the corpus by verdict and compare stall exposure across the split.

    The question this exists to answer is whether index stalls COST anything.
    A stall that shows up equally in passing and failing runs is waypoint
    spacing, not a defect, and tuning the reach threshold to remove it would be
    tuning noise.
    """
    passed = [r for r in results if r["ok"]]
    failed = [r for r in results if not r["ok"]]

    def summarise(label: str, group: list[dict]) -> None:
        if not group:
            print(f"{label:>18}  (none)")
            return
        stalled = [r for r in group if r["stalls"]]
        frac_ticks = [r["stall_ticks"] / r["ticks"] for r in group if r["ticks"]]
        behind = [r["behind_frac"] for r in group if r["behind_frac"] is not None]
        max_dwell = max(r["max_dwell"] for r in group)
        ratios = [r["dwell_ratio"] for r in group if r["dwell_ratio"] is not None]
        behind_col = f"{sum(behind) / len(behind):.0%}" if behind else "-"
        print(
            f"{label:>18}  n={len(group):>3}  "
            f"runs-with-stall={len(stalled) / len(group):>5.0%}  "
            f"stalls/run={sum(r['stalls'] for r in group) / len(group):>5.2f}  "
            f"ticks-stalled={(sum(frac_ticks) / len(frac_ticks) if frac_ticks else 0):>5.1%}  "
            f"worst-dwell={max_dwell:>4} ({max_dwell * _SIM_DT_S:>5.2f} s)  "
            f"worst/median={(sum(ratios) / len(ratios) if ratios else 0):>5.1f}x  "
            f"behind={behind_col:>4}",
        )

    print(f"\nStall exposure by verdict (stall = index held >= {stall_multiple}x that run's median dwell):")
    summarise("passed", passed)
    summarise("failed", failed)
    summarise("  - collided", [r for r in failed if r["collided"]])
    summarise("  - over time", [r for r in failed if r["over_time"]])
    summarise("  - short laps", [r for r in failed if r["laps"] < 3])
    summarise("  - not parked", [r for r in failed if r["parked"] is False])
    print(
        "\nIf the two rows match, stalls are waypoint spacing and not a defect worth "
        "tuning\nthe reach threshold for. `behind` is the share of stalled ticks "
        "STALE_TARGET_RESCUE\nwould have advanced past -- near 0% means that rescue is "
        "not the fix.",
    )


def report_index_stalls(rows: list[dict], *, min_ticks: int, reached_dist_m: float) -> None:
    """Print every waypoint index held for at least ``min_ticks`` ticks.

    The interesting column is ``rescuable``: the share of stalled ticks where
    the waypoint was already BEHIND the chassis. A stall that is mostly
    rescuable is the documented ``STALE_TARGET_RESCUE`` case arriving in a
    configuration that has that rescue switched off; a stall that is not is a
    different bug and needs a different fix.
    """
    runs: list[tuple[int, int, int]] = []  # (index, start_step, length)
    for row in rows:
        if runs and runs[-1][0] == row["waypoint_index"]:
            idx, start, length = runs[-1]
            runs[-1] = (idx, start, length + 1)
        else:
            runs.append((row["waypoint_index"], row["step"], 1))

    lengths = [length for _, _, length in runs]
    if not lengths:
        print("No waypoint indices recorded.")
        return
    median = sorted(lengths)[len(lengths) // 2]
    print(
        f"\nWaypoint index dwell: {len(runs)} indices, median {median} ticks "
        f"({median * _SIM_DT_S:.2f} s), max {max(lengths)} ticks "
        f"({max(lengths) * _SIM_DT_S:.2f} s)",
    )

    stalls = [r for r in runs if r[2] >= min_ticks]
    if not stalls:
        print(f"No index held >= {min_ticks} ticks ({min_ticks * _SIM_DT_S:.2f} s).")
        return

    by_step = {row["step"]: row for row in rows}
    print(f"\n{'index':>6} {'start_s':>8} {'held_s':>7} {'rescuable':>10} {'next_closer':>12} {'dist_m':>8}")
    for idx, start, length in stalls:
        window = [by_step[s] for s in range(start, start + length) if s in by_step]
        behind = [r["wp_behind"] for r in window if r["wp_behind"] is not None]
        closer = [r["next_closer"] for r in window if r["next_closer"] is not None]
        dists = [r["dist_to_wp_m"] for r in window if r["dist_to_wp_m"] is not None]
        rescuable = f"{sum(behind) / len(behind):.0%}" if behind else "-"
        closer_pct = f"{sum(closer) / len(closer):.0%}" if closer else "-"
        dist = f"{min(dists):.3f}" if dists else "-"
        print(
            f"{idx:>6} {start * _SIM_DT_S:>8.2f} {length * _SIM_DT_S:>7.2f} "
            f"{rescuable:>10} {closer_pct:>12} {dist:>8}",
        )
    print(
        "\nrescuable = share of stalled ticks where the waypoint was already behind "
        "the chassis,\ni.e. ticks STALE_TARGET_RESCUE would have advanced past had it "
        "been enabled.\ndist_m = closest the chassis came to the waypoint during the "
        f"stall; it is reached at {reached_dist_m:.3f} m.",
    )


def _write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def _safe_width(widths: dict[Section, float] | None, section: Section) -> float | None:
    if widths is None:
        return None
    return widths.get(section)


def _report_scenario_header(
    scenario_index: int,
    scenario: NamedScenario,
    sim: ScenarioSimulator,
    true_path: list[Waypoint],
) -> None:
    """Print the scenario/start/belief/waypoint-count report before the sim loop runs."""
    print(f"Scenario {scenario_index}: {scenario.label}")
    print(f"True start: {scenario.metadata['starting_conditions']}")
    print(f"Initial belief offset: {sim.belief_offset_poses}")
    print(f"True path has {len(true_path)} waypoints")


def _report_scenario_result(result: SimResult, ctes: list[float]) -> None:
    """Print the result/CTE summary report after the sim loop finishes."""
    print(
        f"Result: success={result.success} laps={result.laps_completed} "
        f"collided={result.collided} timed_out={result.timed_out}",
    )

    if ctes:
        print(
            f"CTE vs displayed path: min={min(ctes):.3f} max={max(ctes):.3f} "
            f"mean={sum(ctes) / len(ctes):.3f} m",
        )


def run_scenario(
    scenario_index: int,
    *,
    laps: int,
    blind: bool,
    use_lidar_localization: bool,
    tuning_path: str | None,
    max_steps: int,
    output_dir: Path,
    stall_ticks: int = 30,
) -> tuple[Path, list[dict]]:
    """Simulate one scenario and write a CSV.

    Returns the CSV path and the collected rows.
    """
    scenario = open_scenario_by_index(scenario_index)

    tuning = load_tuning(tuning_path)
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=laps,
        blind=blind,
        use_lidar_localization=use_lidar_localization,
        tuning=tuning,
    )

    meta = ScenarioMetadata.model_validate(scenario.metadata)
    true_path = calculate_waypoints(meta, num_laps=laps, tuning=tuning)

    _report_scenario_header(scenario_index, scenario, sim, true_path)

    rows: list[dict] = []

    def on_step(state: AckermannState, scan: LidarScan) -> None:  # noqa: ARG001 - scan required by callback signature
        believed, true = sim.belief_offset_poses
        # The navigator's current tracked path is private; this is a diagnostic
        # peek, not production code.
        path_map = _displayed_path(sim.navigator._waypoints, believed, true)  # noqa: SLF001
        proj_displayed = project_onto_path(path_map, state.x, state.y)
        proj_true = project_onto_path(true_path, state.x, state.y)
        widths = sim.believed_widths
        rows.append({
            "step": len(rows),
            "x": state.x,
            "y": state.y,
            # The ONLY per-tick attitude in this file. The believed_*/true_*
            # columns below are NOT poses -- see the comment on them -- so
            # without this a reader has no way to tell a corner from a straight,
            # and a 2026-09-03 attempt to split cross-track error that way
            # silently measured zero cornering ticks in every run.
            "yaw": state.yaw,
            # CONSTANT for the whole run: `belief_offset_poses` is the fixed
            # anchor PAIR defining the believed->true frame offset (the one
            # printed as "Initial belief offset" in the header), not the robot's
            # pose. Named for the frames they define, not for the chassis.
            "believed_x": believed[0],
            "believed_y": believed[1],
            "believed_yaw": believed[2],
            "true_x": true[0],
            "true_y": true[1],
            "true_yaw": true[2],
            "path_x": proj_displayed.x,
            "path_y": proj_displayed.y,
            "cte_displayed_m": proj_displayed.distance_m,
            "signed_offset_displayed_m": proj_displayed.signed_offset_m,
            "cte_true_m": proj_true.distance_m,
            "signed_offset_true_m": proj_true.signed_offset_m,
            "south_m": _safe_width(widths, Section.SOUTH),
            "north_m": _safe_width(widths, Section.NORTH),
            "east_m": _safe_width(widths, Section.EAST),
            "west_m": _safe_width(widths, Section.WEST),
            # The knob-reached-the-navigator check. A --tuning arm that moves
            # LOOKAHEAD_* but leaves this column identical did not take effect,
            # and its flat CTE result means nothing -- three separate constants
            # have now been A/B'd in this repo while silently never reaching the
            # navigator at all. Read off the published debug snapshot, so it is
            # the value pure pursuit actually used, not the one asked for.
            "lookahead_m": getattr(sim.navigator._debug, "lookahead_distance_m", None),  # noqa: SLF001
            **asdict(_index_probe(sim.navigator)),
        })

    result = sim.run(on_step=on_step, max_steps=max_steps)

    ctes = [r["cte_displayed_m"] for r in rows if r["cte_displayed_m"] is not None]
    _report_scenario_result(result, ctes)

    report_index_stalls(
        rows,
        min_ticks=stall_ticks,
        reached_dist_m=tuning.waypoints.MAIN_LOOP_REACHED_DISTANCE_M,
    )

    csv_path = output_dir / f"diag_path_track_{scenario_index}.csv"
    _write_csv(rows, csv_path)
    print(f"Wrote {csv_path}\n")
    return csv_path, rows


def main() -> None:
    """Parse arguments and run the requested scenario."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=int, default=450, help="Scenario index to run.")
    parser.add_argument("--laps", type=int, default=3, help="Number of laps to simulate.")
    parser.add_argument("--blind", action="store_true", default=True, help="Run blind (default).")
    parser.add_argument("--sighted", action="store_true", help="Override --blind and run sighted.")
    parser.add_argument("--no-lidar", action="store_true", help="Disable LIDAR localisation.")
    parser.add_argument("--max-steps", type=int, default=3000, help="Simulation step budget.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_DEFAULT_OUTPUT_DIR,
        help="Directory for the output CSV.",
    )
    parser.add_argument(
        "--stall-ticks",
        type=int,
        default=30,
        help="report waypoint indices held at least this many ticks (30 = 1.5 s; "
        "the hardware norm is 8-26 ticks and the stall under investigation was ~101)",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=None,
        metavar="N",
        help="run scenarios 0..N-1 blind and report stall exposure split by verdict, "
        "instead of writing a per-tick CSV for one scenario",
    )
    parser.add_argument("--jobs", type=int, default=12, help="worker processes for --batch")
    parser.add_argument(
        "--corpus",
        choices=("open", "obstacles"),
        default="open",
        help="which corpus --batch measures over; 'obstacles' is the one whose failures "
        "are sign collisions, so it is the corpus that tests the cross-track hypothesis",
    )
    parser.add_argument(
        "--stall-multiple",
        type=float,
        default=4.0,
        help="in --batch, an index counts as stalled at this multiple of the run's own "
        "median dwell (default 4x); relative because runs differ two-fold in base dwell",
    )
    add_tuning_arg(parser)
    args = parser.parse_args()

    if args.batch is not None:
        count = min(args.batch, len(batch_scenarios(args.corpus)))
        with Pool(args.jobs) as pool:
            results = pool.map(
                _batch_one,
                [(i, args.stall_multiple, args.corpus) for i in range(count)],
            )
        out = args.output_dir / f"diag_path_track_batch_{args.corpus}_{count}.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(results, indent=1))
        print(f"Corpus: {args.corpus}, {count} scenarios, blind")
        report_batch(results, stall_multiple=args.stall_multiple)
        print(f"\nWrote {out}")
        return

    run_scenario(
        args.scenario,
        laps=args.laps,
        blind=not args.sighted,
        use_lidar_localization=not args.no_lidar,
        tuning_path=args.tuning,
        max_steps=args.max_steps,
        output_dir=args.output_dir,
        stall_ticks=args.stall_ticks,
    )


if __name__ == "__main__":
    main()
