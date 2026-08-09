"""Reconstruct the planned waypoint path and replay the real target-selection logic against a bag.

Builds the same path ``calculate_waypoints`` would have built for the race
(from a corridor-width belief, direction, and starting section), then
replays the real ``WaypointController.select_target_point`` against each
bag tick's logged pose/waypoint_index and compares the result against what
the robot actually logged as its steering target. Consolidates four one-off
tmp scripts written during the 2026-08-08 path/index-mapping investigation:

  * default mode         -- tmp_replay_sel.py: build one path, replay
    ``select_target_point`` tick-by-tick, report match/mismatch.
  * ``--candidate-scan``  -- tmp_probe_tick.py: per-tick SKIP/RETURN
    reasoning trace through the forward candidate search.
  * ``--solve-rotation``  -- tmp_solve_idx.py: brute-force which
    (width, arc, waypoint-index rotation) combination reproduces the bag.
  * ``--try-reversed``    -- tmp_reversed.py: test the CW/CCW/reversed-path
    hypotheses against the bag's logged selections.

The brute-force grid/rotation searches in ``--solve-rotation`` and the
3-way path-variant search in ``--try-reversed`` are parallelized with
``ProcessPoolExecutor``, following the pattern in
``scripts/sim/diag_open_parallel.py``.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_path_replay.py RUN_DIR
    pixi run -e dev python scripts/bag/diag_bag_path_replay.py RUN_DIR --candidate-scan --tick 0
    pixi run -e dev python scripts/bag/diag_bag_path_replay.py RUN_DIR --solve-rotation \
        --widths 0.60,0.70,0.80,0.90,1.00 --arcs 0.25,0.45
    pixi run -e dev python scripts/bag/diag_bag_path_replay.py RUN_DIR --try-reversed
"""

from __future__ import annotations

import math
import os
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Section
from shared.domain.models import CorridorWidthEntry, CorridorWidths, ScenarioMetadata

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import print_table
from src.navigation.control.controllers.waypoint_controller import WaypointController
from src.navigation.planning.waypoints import calculate_waypoints

if TYPE_CHECKING:
    from shared.domain.models import NavigatorDebugSnapshot

_SPARE_CORES = 2
"""Cores left idle for the rest of the machine, matching diag_open_parallel.py."""
_MATCH_TOLERANCE_M = 0.03
"""How close a replayed target has to land to the logged one to count as a match."""
_MAX_MISMATCH_EXAMPLES = 6
"""How many mismatch examples to print in the default replay-and-compare mode."""


def _jobs(requested: int) -> int:
    return requested or max(1, (os.cpu_count() or 4) - _SPARE_CORES)


def _build_path(width_m: float, direction: str, section: str, laps: int, arc_radius: float | None) -> list[tuple[float, float]]:
    """Build the full multi-lap waypoint path for a uniform corridor-width belief."""
    tuning = NavigationTuning.load_default()
    widths = CorridorWidths(**{s.value: CorridorWidthEntry(width_mm=round(width_m * 1000)) for s in Section})
    meta = ScenarioMetadata.model_validate(
        {"corridor_widths": widths, "starting_conditions": {"direction": direction, "section": section}},
    )
    return calculate_waypoints(meta, num_laps=laps, arc_radius=arc_radius, tuning=tuning)


def _usable_rows(bag_dir: Path) -> list[tuple[float, NavigatorDebugSnapshot]]:
    """normal_drive ticks carrying everything select_target_point needs."""
    rows, _topics = load_nav_debug_rows(bag_dir)
    return [
        (t, s)
        for t, s in rows
        if s.phase.value == "normal_drive"
        and None not in (s.pose_x, s.pose_y, s.pose_yaw, s.waypoint_index, s.steer_target_x, s.steer_target_y)
    ]


def _idx_of(path: list[tuple[float, float]]) -> dict[tuple[float, float], int]:
    return {(round(x, 2), round(y, 2)): i for i, (x, y) in enumerate(path)}


def _replay_and_compare(bag_dir: Path, width_m: float, arc: float | None, direction: str, section: str, laps: int) -> None:
    """Default mode: build one path, replay select_target_point, report match/mismatch (tmp_replay_sel.py)."""
    path = _build_path(width_m, direction, section, laps, arc)
    ctrl = WaypointController.from_tuning(NavigationTuning.load_default())
    idx_of = _idx_of(path)

    nd = _usable_rows(bag_dir)
    print(f"path len {len(path)}   normal_drive ticks usable {len(nd)}")
    if not nd:
        print("no usable normal_drive ticks -- nothing to replay")
        return

    match = mismatch = 0
    examples = []
    logged_idx_seen: Counter[int] = Counter()
    for t, s in nd:
        logged_idx_seen[s.waypoint_index] += 1
        got = ctrl.select_target_point((s.pose_x, s.pose_y), s.pose_yaw, path, s.waypoint_index, s.lookahead_distance_m)
        logged = (s.steer_target_x, s.steer_target_y)
        if math.hypot(got[0] - logged[0], got[1] - logged[1]) < _MATCH_TOLERANCE_M:
            match += 1
        else:
            mismatch += 1
            if len(examples) < _MAX_MISMATCH_EXAMPLES:
                examples.append((
                    t,
                    s.waypoint_index,
                    (round(s.pose_x, 2), round(s.pose_y, 2), round(s.pose_yaw, 2)),
                    s.lookahead_distance_m,
                    f"logged {logged[0]:.2f},{logged[1]:.2f} (idx {idx_of.get((round(logged[0], 2), round(logged[1], 2)), '?')})",
                    f"replay {got[0]:.2f},{got[1]:.2f} (idx {idx_of.get((round(got[0], 2), round(got[1], 2)), '?')})",
                ))

    print(f"selection replay: match {match}  mismatch {mismatch}")
    print(f"logged waypoint_index values: {logged_idx_seen.most_common(6)}")
    for e in examples:
        print(" ", e)


def _candidate_scan(bag_dir: Path, width_m: float, arc: float | None, direction: str, section: str, laps: int, tick: int) -> None:
    """Per-tick SKIP/RETURN reasoning trace through the forward candidate search (tmp_probe_tick.py)."""
    path = _build_path(width_m, direction, section, laps, arc)
    n = len(path)
    idx_of = _idx_of(path)

    nd = _usable_rows(bag_dir)
    if not nd:
        print("no usable normal_drive ticks")
        return

    print("logged-target index vs logged waypoint_index, and implied offset:")
    seen: dict[tuple[int, int | None], tuple[float, NavigatorDebugSnapshot]] = {}
    for t, s in nd:
        li = idx_of.get((round(s.steer_target_x, 2), round(s.steer_target_y, 2)))
        key = (s.waypoint_index, li)
        if key not in seen:
            seen[key] = (t, s)
    rows = []
    for (wi, li), (t, s) in sorted(seen.items(), key=lambda kv: kv[1][0]):
        off = (li - wi) % n if li is not None else None
        d = math.hypot(s.steer_target_x - s.pose_x, s.steer_target_y - s.pose_y)
        rows.append((f"{t:.2f}", wi, li, off, f"{d:.2f}", s.lookahead_distance_m))
    print_table(rows, ["t", "logged_idx", "target_idx", "offset", "dist", "logged_look"])

    if not (0 <= tick < len(nd)):
        print(f"--tick {tick} out of range (0..{len(nd) - 1})")
        return
    t, s = nd[tick]
    print(f"\n--- candidate scan at t={t:.2f}, index={s.waypoint_index}, look={s.lookahead_distance_m} ---")
    cx, cy, yaw = s.pose_x, s.pose_y, s.pose_yaw
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    scan_rows = []
    for offset in range(16):
        i = (s.waypoint_index + offset) % n
        wx, wy = path[i]
        dx, dy = wx - cx, wy - cy
        dist = math.hypot(dx, dy)
        x_local = dx * cos_yaw + dy * sin_yaw
        verdict = "SKIP(behind)" if x_local <= 0 else ("RETURN" if dist >= s.lookahead_distance_m else "too close")
        scan_rows.append((offset, i, f"{wx:.2f}", f"{wy:.2f}", f"{dist:.2f}", f"{x_local:+.3f}", verdict))
        if x_local > 0 and dist >= s.lookahead_distance_m:
            break
    print_table(scan_rows, ["offset", "idx", "x", "y", "dist", "x_local", "verdict"])
    target_idx = idx_of.get((round(s.steer_target_x, 2), round(s.steer_target_y, 2)))
    print(f"LOGGED target: ({s.steer_target_x:.2f},{s.steer_target_y:.2f}) idx={target_idx}")


# --- --solve-rotation: parallel width x arc grid, then parallel rotation-index search. ---


def _grid_worker(payload: tuple[float, float, str, str, int, list[tuple[float, float]]]) -> tuple[float, float, int, int, list[tuple[float, float]]]:
    """Build one (width, arc) path and count how many logged targets it covers. Module-level so it can be pickled."""
    width, arc, direction, section, laps, logged_points = payload
    path = _build_path(width, direction, section, laps, arc)
    pts = {(round(x, 2), round(y, 2)) for x, y in path}
    covered = sum(1 for p in logged_points if p in pts)
    return width, arc, covered, len(path), path


def _rotation_worker(payload: tuple[int, list[tuple[float, float]], list[tuple[float, float, float, int, float, float, float]]]) -> tuple[int, int]:
    """Count matches for one rotation index k. Module-level so it can be pickled."""
    k, path, nd_primitive = payload
    ctrl = WaypointController.from_tuning(NavigationTuning.load_default())
    n = len(path)
    match = 0
    for x, y, yaw, wi, look, tx, ty in nd_primitive:
        got = ctrl.select_target_point((x, y), yaw, path, (wi + k) % n, look)
        if math.hypot(got[0] - tx, got[1] - ty) < _MATCH_TOLERANCE_M:
            match += 1
    return k, match


def _solve_rotation(bag_dir: Path, direction: str, section: str, laps: int, widths: list[float], arcs: list[float], jobs: int) -> None:
    """Brute-force (width, arc, rotation) calibration against the bag (tmp_solve_idx.py), parallelized."""
    nd = _usable_rows(bag_dir)
    print(f"normal_drive ticks usable: {len(nd)}")
    if not nd:
        print("no usable normal_drive ticks")
        return
    logged_points = [(round(s.steer_target_x, 2), round(s.steer_target_y, 2)) for _, s in nd]
    workers = _jobs(jobs)

    grid_payloads = [(w, a, direction, section, laps, logged_points) for w in widths for a in arcs]
    grid_results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(_grid_worker, p): p for p in grid_payloads}
        for done in as_completed(futures):
            grid_results.append(done.result())

    grid_results.sort(key=lambda r: (-r[2] / max(len(logged_points), 1), r[0], r[1]))
    print_table(
        [(w, a, n, f"{covered}/{len(logged_points)}") for w, a, covered, n, _path in grid_results],
        ["width", "arc", "path_len", "logged_targets_covered"],
    )

    best_width, best_arc, _covered, _n, best_path = grid_results[0]
    print(f"\nbest path: width={best_width} arc={best_arc} len={len(best_path)}")

    n = len(best_path)
    nd_primitive = [(s.pose_x, s.pose_y, s.pose_yaw, s.waypoint_index, s.lookahead_distance_m, s.steer_target_x, s.steer_target_y) for _, s in nd]
    rotation_payloads = [(k, best_path, nd_primitive) for k in range(n)]
    rotation_results: list[tuple[int, int]] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(_rotation_worker, p): p[0] for p in rotation_payloads}
        for done in as_completed(futures):
            rotation_results.append(done.result())
    rotation_results.sort(key=lambda r: -r[1])

    print("\nbest rotations (match / ticks):")
    print_table(
        [(k, m, f"{100 * m / len(nd):.1f}%") for m, k in [(m, k) for k, m in rotation_results[:5]]],
        ["k", "match", "pct"],
    )


# --- --try-reversed: parallel 3-way path-variant search. ---


def _variant_worker(payload: tuple[str, list[tuple[float, float]], int, list[tuple[float, float, float, int, float, float, float]]]) -> tuple[str, int, int]:
    """Best rotation match for one path variant. Module-level so it can be pickled."""
    name, path, k, nd_primitive = payload
    ctrl = WaypointController.from_tuning(NavigationTuning.load_default())
    n = len(path)
    match = 0
    for x, y, yaw, wi, look, tx, ty in nd_primitive:
        got = ctrl.select_target_point((x, y), yaw, path, (wi + k) % n, look)
        if math.hypot(got[0] - tx, got[1] - ty) < _MATCH_TOLERANCE_M:
            match += 1
    return name, k, match


def _try_reversed(bag_dir: Path, width_m: float, arc: float | None, section: str, laps: int, jobs: int) -> None:
    """Test CW / CCW / reversed(CW) path hypotheses against the bag (tmp_reversed.py), parallelized."""
    cw = _build_path(width_m, "clockwise", section, laps, arc)
    ccw = _build_path(width_m, "counterclockwise", section, laps, arc)
    rev = list(reversed(cw))

    nd = _usable_rows(bag_dir)
    if not nd:
        print("no usable normal_drive ticks")
        return
    nd_primitive = [(s.pose_x, s.pose_y, s.pose_yaw, s.waypoint_index, s.lookahead_distance_m, s.steer_target_x, s.steer_target_y) for _, s in nd]

    print(f"cw[0..3]  = {[(round(x, 2), round(y, 2)) for x, y in cw[:4]]}")
    print(f"ccw[0..3] = {[(round(x, 2), round(y, 2)) for x, y in ccw[:4]]}")
    print(f"cw == ccw? {cw == ccw}   cw == reversed(ccw)? {cw == list(reversed(ccw))}")

    variants = (
        ("CW (as planned for 'clockwise')", cw),
        ("CCW path", ccw),
        ("reversed(CW)", rev),
    )
    payloads = [(name, path, k, nd_primitive) for name, path in variants for k in range(len(path))]
    results: dict[str, tuple[int, int]] = {}
    with ProcessPoolExecutor(max_workers=_jobs(jobs)) as pool:
        futures = {pool.submit(_variant_worker, p): p for p in payloads}
        for done in as_completed(futures):
            name, k, match = done.result()
            best = results.get(name, (-1, -1))
            if match > best[0]:
                results[name] = (match, k)

    for name, _path in variants:
        match, k = results[name]
        print(f"{name:34s}: best {match}/{len(nd)} ({100 * match / len(nd):.1f}%) at rotation k={k}")


def main() -> None:
    """Parse CLI args and dispatch to the selected mode (replay/candidate-scan/solve-rotation/try-reversed)."""
    parser = create_bag_parser(
        "Reconstruct the planned waypoint path and replay select_target_point against a bag's logged pose/waypoint_index.",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=CorridorDimensions.NARROW,
        help=f"uniform corridor width (m) for the reconstructed path (default: track.toml narrow width, {CorridorDimensions.NARROW} m)",
    )
    parser.add_argument("--arc", type=float, default=None, help="corner arc radius ceiling (m); default is NavigationTuning's own default")
    parser.add_argument("--direction", choices=("clockwise", "counterclockwise"), default="clockwise")
    parser.add_argument("--section", choices=("north", "south", "east", "west"), default="south")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--candidate-scan", action="store_true", help="per-tick SKIP/RETURN reasoning trace (tmp_probe_tick.py)")
    parser.add_argument("--tick", type=int, default=0, help="which usable tick to trace in detail under --candidate-scan")
    parser.add_argument("--solve-rotation", action="store_true", help="brute-force width/arc/rotation calibration (tmp_solve_idx.py)")
    parser.add_argument("--widths", type=str, default="0.60,0.70,0.80,0.90,1.00", help="comma-separated width grid for --solve-rotation")
    parser.add_argument("--arcs", type=str, default="0.25,0.45", help="comma-separated arc grid for --solve-rotation")
    parser.add_argument("--try-reversed", action="store_true", help="test CW/CCW/reversed-path hypotheses (tmp_reversed.py)")
    parser.add_argument("--jobs", type=int, default=0, help="worker processes for the brute-force searches; 0 picks cores minus a couple")
    args = parser.parse_args()

    if args.candidate_scan:
        _candidate_scan(args.bag_dir, args.width, args.arc, args.direction, args.section, args.laps, args.tick)
    elif args.solve_rotation:
        widths = [float(w) for w in args.widths.split(",")]
        arcs = [float(a) for a in args.arcs.split(",")]
        _solve_rotation(args.bag_dir, args.direction, args.section, args.laps, widths, arcs, args.jobs)
    elif args.try_reversed:
        _try_reversed(args.bag_dir, args.width, args.arc, args.section, args.laps, args.jobs)
    else:
        _replay_and_compare(args.bag_dir, args.width, args.arc, args.direction, args.section, args.laps)


if __name__ == "__main__":
    main()
