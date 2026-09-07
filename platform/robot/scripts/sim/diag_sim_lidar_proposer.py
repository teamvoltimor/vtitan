"""Does the LIDAR proposer find the SIGNS, scored against a layout that is KNOWN?

The hardware version of this question (`scripts/bag/diag_bag_lidar_proposer.py`)
cannot be answered, and it says so: its only anchor is the camera, and on
hardware the camera confirms wall-shaped objects about as readily as pillars, so
a candidate's distance-to-nearer-wall comes out at p50 0.27 m for "confirmed"
tracks and 0.25 m for unconfirmed -- the same distribution. Nothing scored
against that anchor can separate a precision gain from noise.

Here the layout IS the ground truth. `sign_positions` in the scenario metadata
gives every sign's exact world position, so a proposed track is right or wrong
with no appeal to the camera. That decides the two things the bag run left open:

1. **What the cluster detector's real precision is**, rather than an upper bound.
2. **Whether the LATTICE PRIOR earns its recall cost.** Signs stand on a 6-point
   lattice per section -- 0.4/0.6 m lateral, 1.0/1.5/2.0 m depth, at most 2
   occupied -- so a candidate 0.4 m from its nearer lateral border is plausible
   and one at ~0 is a wall corner. Against the camera anchor that filter kept
   33% of tracks to move precision 47% -> 52%, which was unreadable.

**How far this transfers to hardware -- MEASURED, not assumed.** Running this
exact detector over both, with neither side needing ground truth:

    clusters per scan        hardware 3.10   sim 2.95    -- the SAME detector
    persistent tracks        hardware  204   sim  198
    first-see range p50      hardware 1.33 m sim 1.77 m  -- sim sees EARLIER
    share in the sign band   hardware  23%   sim  35%    -- (0.375-0.5 m)

Cluster YIELD matches, so the sim is a fair proxy for the detector itself. Two
gaps are real and both make the 84% an UPPER BOUND: the sim spots a candidate
~0.44 m earlier, and hardware's lattice-consistent population is a third
thinner, so there is more clutter (or more position smear) to reject there.

NOT a reason, though the obvious guess: side-ray dropout. Measured on the 09-07
runs, the -90 deg window has no valid return on 12-19% of ticks for the SINGLE
nearest ray, but only 0.0-1.7% across the +/-20 deg window this uses. The 66%
figure recorded on 09-03 is a single-ray number and does not apply here. The
windowed median already absorbs it.

The vision emulator still has PERFECT RANGE, so the camera-vs-LIDAR range
comparison that motivated the idea cannot be reproduced in sim at all.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        pixi run -e dev python scripts/sim/diag_sim_lidar_proposer.py --limit 8
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported FIRST: models <-> enums is a cycle that only resolves from the enums side)
from shared.config.constants import CorridorDimensions
from shared.domain.models import ScenarioMetadata

from scripts.common.lidar_clusters import (
    Track,
    associate,
    corridor_walls,
    find_clusters,
    to_world,
    wall_distance,
    width_of,
)
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from scripts.common.stats import percentile
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Sequence

_FIXTURES = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"


def collect(metadata: ScenarioMetadata, args: argparse.Namespace) -> list[tuple[float, float, float, float, float, float | None, float | None]]:
    """Run one scenario, returning a cluster observation per detection per tick.

    Pose comes from the gateway's kinematic state -- GROUND TRUTH, not the
    localizer estimate the navigator drives on. That is deliberate: this
    measures whether the LIDAR can find a sign, and pairing it with an
    estimated pose would fold localizer error into the answer.
    """
    observations: list[tuple[float, float, float, float, float, float | None, float | None]] = []

    def on_step(state, scan) -> None:  # noqa: ANN001
        pose = (state.x, state.y, state.yaw)
        walls = corridor_walls(
            scan,
            math.radians(args.wall_window_deg),
            args.max_wall_m,
            expected_width_m=args.corridor_width_m,
            width_tol_m=args.width_tol_m,
        )
        for c in find_clusters(
            scan,
            min_m=args.min_range,
            max_m=args.max_range,
            depth_m=args.depth,
            isolation_m=args.isolation,
        ):
            if not args.min_chord <= c.chord_m <= args.max_chord:
                continue
            x, y = to_world(pose, c.range_m, c.bearing_rad)
            observations.append((0.0, x, y, c.chord_m, c.range_m, wall_distance(c, walls), width_of(walls)))

    ScenarioSimulator(
        metadata,
        num_laps=args.laps,
        seed=args.seed,
        blind=args.blind,
        park=False,
    ).run(max_steps=args.max_steps, on_step=on_step)
    return observations


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--limit", type=int, default=8, help="Scenarios to run; 0 means all.")
    parser.add_argument("--laps", type=int, default=1, help="Laps per scenario; 1 is enough to see every sign.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    parser.add_argument("--sighted", dest="blind", action="store_false", default=True)
    parser.add_argument("--min-range", type=float, default=0.30)
    parser.add_argument("--max-range", type=float, default=2.50)
    parser.add_argument("--depth", type=float, default=0.08)
    parser.add_argument("--isolation", type=float, default=0.20)
    parser.add_argument("--min-chord", type=float, default=0.02)
    parser.add_argument("--max-chord", type=float, default=0.18)
    parser.add_argument("--assoc-radius", type=float, default=0.15)
    parser.add_argument("--min-hits", type=int, default=5)
    parser.add_argument("--match-radius", type=float, default=0.25, help="Distance within which a track IS a given sign.")
    parser.add_argument("--wall-window-deg", type=float, default=20.0)
    parser.add_argument(
        "--corridor-width-m",
        type=float,
        default=CorridorDimensions.OBSTACLES_WIDTH,
        help="Known corridor width; a measured pair that disagrees is rejected. 0 disables the gate.",
    )
    parser.add_argument("--width-tol-m", type=float, default=0.15, help="How far the measured width may differ.")
    parser.add_argument("--max-wall-m", type=float, default=1.50)
    parser.add_argument("--lattice-offset-m", type=float, default=0.40)
    parser.add_argument("--lattice-tol-m", type=float, default=0.12)
    args = parser.parse_args()

    paths = sorted(_FIXTURES.glob("*_metadata.json"))
    if args.limit:
        paths = paths[: args.limit]

    totals = {"tracks": 0, "hits": 0, "signs": 0, "found": 0, "lat_tracks": 0, "lat_hits": 0, "lat_found": 0}
    first_ranges: list[float] = []
    wall_true: list[float] = []
    wall_false: list[float] = []
    widths: list[float] = []

    print(f"{'scenario':<26} {'signs':>5} {'trk':>4} {'TP':>4} {'prec':>5} {'rec':>5}   {'lattice trk/TP/prec/rec':>24}")
    for path in paths:
        metadata = ScenarioMetadata.model_validate(json.loads(path.read_text()))
        signs = [(s.x, s.y) for s in metadata.sign_positions]
        if not signs:
            continue
        tracks = [t for t in associate(collect(metadata, args), args.assoc_radius) if t.hits >= args.min_hits]

        def is_sign(t: Track, signs: list[tuple[float, float]] = signs) -> bool:
            return any(math.hypot(t.x - sx, t.y - sy) <= args.match_radius for sx, sy in signs)

        def found(pool: list[Track], signs: list[tuple[float, float]] = signs) -> int:
            return sum(
                1 for sx, sy in signs if any(math.hypot(t.x - sx, t.y - sy) <= args.match_radius for t in pool)
            )

        def on_lattice(t: Track) -> bool:
            d = t.wall_dist_m
            return d is not None and abs(d - args.lattice_offset_m) <= args.lattice_tol_m

        tp = [t for t in tracks if is_sign(t)]
        lat = [t for t in tracks if on_lattice(t)]
        lat_tp = [t for t in lat if is_sign(t)]

        totals["tracks"] += len(tracks)
        totals["hits"] += len(tp)
        totals["signs"] += len(signs)
        totals["found"] += found(tracks)
        totals["lat_tracks"] += len(lat)
        totals["lat_hits"] += len(lat_tp)
        totals["lat_found"] += found(lat)
        first_ranges.extend(t.first_range_m for t in tp)
        wall_true.extend(t.wall_dist_m for t in tp if t.wall_dist_m is not None)
        wall_false.extend(t.wall_dist_m for t in tracks if not is_sign(t) and t.wall_dist_m is not None)
        widths.extend(w for w in (t.width_m for t in tracks) if w is not None)

        print(
            f"{path.stem.replace('_metadata',''):<26} {len(signs):>5} {len(tracks):>4} {len(tp):>4} "
            f"{_pct(len(tp), len(tracks)):>5} {_pct(found(tracks), len(signs)):>5}   "
            f"{len(lat):>6} {len(lat_tp):>4} {_pct(len(lat_tp), len(lat)):>5} {_pct(found(lat), len(signs)):>5}"
        )

    print()
    print("Scored against the TRUE sign layout, not a camera anchor:")
    print(f"  tracks:                  {totals['tracks']}")
    print(f"  precision:               {_pct(totals['hits'], totals['tracks'])}   ({totals['hits']} on a real sign)")
    print(f"  recall:                  {_pct(totals['found'], totals['signs'])}   ({totals['found']}/{totals['signs']} signs proposed)")
    print(f"  first-see range of a TP: {_fmt(first_ranges)}")
    print()
    print(f"LATTICE filter ({args.lattice_offset_m:.2f} +/- {args.lattice_tol_m:.2f} m from the nearer wall):")
    print(f"  tracks kept:             {totals['lat_tracks']}   ({_pct(totals['lat_tracks'], totals['tracks'])} of all)")
    print(f"  precision:               {_pct(totals['lat_hits'], totals['lat_tracks'])}   (was {_pct(totals['hits'], totals['tracks'])})")
    print(f"  recall:                  {_pct(totals['lat_found'], totals['signs'])}   (was {_pct(totals['found'], totals['signs'])})")
    print()
    print("SEPARATION -- the thing the hardware anchor could not show:")
    print(f"  measured corridor width: {_fmt(widths)}")
    print(f"  wall dist, TRUE signs:   {_fmt(wall_true)}   (lattice predicts ~{args.lattice_offset_m:.2f} m)")
    print(f"  wall dist, FALSE tracks: {_fmt(wall_false)}")


def _fmt(values: Sequence[float], unit: str = "m") -> str:
    if not values:
        return "   --    "
    return f"{percentile(values, 0.5):.2f} / {percentile(values, 0.9):.2f} {unit}"


def _pct(n: int, d: int) -> str:
    return f"{100.0 * n / d:.0f}%" if d else "n/a"


if __name__ == "__main__":
    main()
