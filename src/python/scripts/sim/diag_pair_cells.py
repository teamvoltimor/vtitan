"""Classify every consecutive sign pair in a corpus into its band-change cell.

A lane is planned per CORRIDOR, but what the chassis actually has to do is set by
consecutive PAIRS of signs along the driven path: whether the pass side stays on
the same band or crosses to the other, and whether the two signs sit in one
section or on opposite sides of a corner. That is 8 cells, and they are not
interchangeable -- a crossing on a straight has a straight base path and over a
metre of runway, while a crossing through a corner inherits an arc that is
already at the chassis radius floor.

Why this script exists: a single global constant (``sign_lane_hold_m``,
``sign_lane_ramp_m``) is applied to all 8 cells at once, and the aggregate sweep
columns cannot say which cell a collision came from. This emits the per-scenario
cell composition so it can be JOINED against a sweep's ``DETAIL`` rows, turning
"25 collisions" into "collisions per cell".

Measured ON THE DRIVEN LANE, not the centreline. The centreline frame reports
every across-corner gap as 0.62 m; on the lane the same pairs run 0.54 m
(IN->IN, which cuts the corner) to 1.14 m (OUT->OUT, which swings wide). Reading
the centreline once produced a wrong conclusion about which cell was tightest.

Usage:
    pixi run -e dev python scripts/sim/diag_pair_cells.py [--corpus] [--csv OUT]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.sim_defaults import CORPUS_DIR  # noqa: E402
from shared.config.constants import TrafficSignSpecs  # noqa: E402
from shared.config.navigation_tuning import NavigationTuning  # noqa: E402
from shared.domain.enums import Direction, Section  # noqa: E402
from shared.domain.models import SignColor  # noqa: E402
from src.navigation.geometry import chassis_half_diagonal_m  # noqa: E402
from src.navigation.planning.sign_lane import SignLaneParams, apply_sign_lanes  # noqa: E402
from src.navigation.planning.sign_router import signs_from_metadata  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402
from src.navigation.planning.waypoints import calculate_waypoints, corridor_for_position  # noqa: E402

# SOUTH and WEST border the inner square on their HIGH side, NORTH and EAST on
# their LOW side -- the same asymmetry clamp_lateral encodes. So the sign of the
# pass-side multiplier means opposite bands depending on the corridor, and it has
# to be normalised before two corridors can be compared.
_LOW_SIDE_CORRIDORS = (Section.SOUTH, Section.WEST)

# The method reads radii low: it takes the Menger radius of consecutive waypoint
# triples, and chord sampling of an arc always understates its radius. Calibrated
# against the configured corner arc (waypoints.arc_radius), a base corner reads
# 0.397 m where the config says 0.45 -- so divide by this to recover a true one.
_MENGER_CALIBRATION = 0.397 / 0.45


def _band(corridor: Section, mult: int) -> str:
    """Which band the pass side puts the lane on, comparable across corridors."""
    inner = (mult > 0) if corridor in _LOW_SIDE_CORRIDORS else (mult < 0)
    return "IN" if inner else "OUT"


def _cumulative(points: list[tuple[float, float]]) -> list[float]:
    """Arc length at each point along a polyline."""
    out = [0.0]
    for first, second in zip(points, points[1:], strict=False):
        out.append(out[-1] + math.dist(first, second))
    return out


def _min_radius(points: list[tuple[float, float]]) -> float:
    """Tightest Menger radius over consecutive triples; inf when collinear.

    Uncalibrated -- divide by ``_MENGER_CALIBRATION`` for a true radius.
    """
    tightest = float("inf")
    for a, b, c in zip(points, points[1:], points[2:], strict=False):
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        twice_area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if twice_area < 1e-12 or ab * bc * ca == 0.0:
            continue
        tightest = min(tightest, ab * bc * ca / (2.0 * twice_area))
    return tightest


def _lane_params(tuning: NavigationTuning) -> SignLaneParams:
    """The lane geometry the navigator would build, from the shipped tuning."""
    sr = tuning.sign_router
    offset = chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2 + sr.SIGN_CLEARANCE_MARGIN_M
    return SignLaneParams(
        lateral_offset=offset * sr.SIGN_LANE_OFFSET_FRAC,
        ramp_m=sr.SIGN_LANE_RAMP_M,
        hold_m=sr.SIGN_LANE_HOLD_M,
        corner_entry_m=sr.SIGN_LANE_CORNER_ENTRY_M,
        gap_centre_frac=sr.SIGN_LANE_GAP_CENTRE_FRAC,
    )


def _pairs_for(
    metadata: dict, direction: Direction, params: SignLaneParams
) -> list[tuple[str, float, float]]:
    """Every consecutive pair as ``(cell, lane gap m, uncalibrated radius m)``."""
    base = calculate_waypoints(metadata, 1)
    specs = signs_from_metadata(metadata)
    lane = apply_sign_lanes(base, [(s, corridor_for_position(s.x, s.y)) for s in specs], params, direction)
    base_xy = [(w.x, w.y) for w in base]
    lane_xy = [(w.x, w.y) for w in lane]
    lane_cum = _cumulative(lane_xy)

    placed: list[tuple[int, Section, str]] = []
    for spec in specs:
        corridor = corridor_for_position(spec.x, spec.y)
        rule = pass_side_lateral_axis(corridor, SignColor(spec.color), direction)
        if rule is None:
            continue
        index = min(range(len(base_xy)), key=lambda k: math.dist(base_xy[k], (spec.x, spec.y)))
        placed.append((index, corridor, _band(corridor, rule[1])))
    placed.sort()

    out: list[tuple[str, float, float]] = []
    for (i1, c1, b1), (i2, c2, b2) in zip(placed, placed[1:], strict=False):
        if i2 - i1 < 2:
            continue
        where = "corner" if c1 != c2 else "section"
        out.append((f"{b1}->{b2}/{where}", lane_cum[i2] - lane_cum[i1], _min_radius(lane_xy[i1 : i2 + 1])))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios-dir", default=None)
    parser.add_argument("--corpus", action="store_true", help=f"shorthand for --scenarios-dir {CORPUS_DIR}")
    parser.add_argument("--csv", default=None, help="write a per-scenario cell count table here")
    args = parser.parse_args()

    directory = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else None)
    if directory is None:
        print("Pass --corpus or --scenarios-dir; the committed fixtures are too few to read a cell from.")
        return
    files = sorted(Path(directory).glob("*_metadata.json"))
    if not files:
        print(f"NO SCENARIOS under {directory} -- the corpus is gitignored, run `task gen:corpus`.")
        return

    tuning = NavigationTuning.load_default()
    params = _lane_params(tuning)
    print(f"scenarios {len(files)}  from {directory}")
    print(
        f"lane: offset {params.lateral_offset:.4f}  ramp {params.ramp_m}  hold {params.hold_m}  "
        f"corner_entry {params.corner_entry_m}  gap_centre_frac {params.gap_centre_frac}"
    )

    rows: list[tuple[str, str, Counter[str]]] = []
    for direction in (Direction.COUNTERCLOCKWISE, Direction.CLOCKWISE):
        gaps: defaultdict[str, list[float]] = defaultdict(list)
        radii: defaultdict[str, list[float]] = defaultdict(list)
        for path in files:
            metadata = json.loads(path.read_text())
            counts: Counter[str] = Counter()
            for cell, gap, radius in _pairs_for(metadata, direction, params):
                counts[cell] += 1
                gaps[cell].append(gap)
                radii[cell].append(radius)
            rows.append((path.stem, direction.name, counts))

        total = sum(len(v) for v in gaps.values())
        crossings = sum(len(v) for k, v in gaps.items() if k.split("/")[0].split("->")[0] != k.split("/")[0].split("->")[1])
        print(f"\n=== {direction.name}   pairs {total}   crossings {crossings} ({100 * crossings / max(total, 1):.0f}%)")
        print(f"  {'cell':20} {'n':>5}  {'lane gap':>9}  {'radius':>8}  {'true R':>8}  kind")
        for cell in sorted(gaps, key=lambda k: -len(gaps[k])):
            g, r = sorted(gaps[cell]), sorted(radii[cell])
            n = len(g)
            median_r = r[n // 2]
            true_r = median_r / _MENGER_CALIBRATION
            left, right = cell.split("/")[0].split("->")
            shown = "inf" if math.isinf(median_r) else f"{median_r:.3f}"
            shown_true = "inf" if math.isinf(true_r) else f"{true_r:.3f}"
            print(
                f"  {cell:20} {n:5}  {g[n // 2]:8.2f} m  {shown:>8}  {shown_true:>8}  "
                f"{'CROSS' if left != right else 'hold'}"
            )

    if args.csv:
        cells = sorted({c for _, _, counts in rows for c in counts})
        out = Path(args.csv)
        with out.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write("scenario,direction," + ",".join(cells) + "\n")
            for scenario, direction, counts in rows:
                handle.write(f"{scenario},{direction}," + ",".join(str(counts[c]) for c in cells) + "\n")
        print(f"\nwrote {out} ({len(rows)} rows, {len(cells)} cells) -- join this on a sweep's DETAIL labels")


if __name__ == "__main__":
    main()
