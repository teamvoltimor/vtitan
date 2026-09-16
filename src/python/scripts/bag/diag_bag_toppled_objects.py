r"""Which track objects did the car KNOCK DOWN, and on which lap?

STATUS: passes its control on the one round with an independent eyewitness
account. The objects the operator described as knocked down are exactly the ones
that stop returning, and the round described as merely nudging a pillar out of
place shows none (adr:0062-sim-contact-model-and-parking).

A knocked-over block LEAVES THE LIDAR'S HORIZONTAL PLANE. That makes destruction
directly observable without a clearance threshold, a contact model or a sensor
error budget: the object itself is the witness, and it is either there or it is
not. Every other contact instrument in this repo infers a touch from a distance
that is smaller than some constant, and every one of those constants has been
argued about.

This measures two different things and does not confuse them:

* **TOPPLED** -- the returns go to zero and stay there while the robot still
  drives past. The object is on its side.
* **PUSHED** -- the returns continue but the cluster's centroid MARCHES, lap
  over lap, in one direction. The object is standing but displaced.
  ``diag_bag_pass_side_truth.py`` judges against a whole-bag average position,
  so a large march is a warning that its verdicts for that object are smeared.

Both matter for scoring and neither is visible in the lap counter.

METHOD

Accumulate every off-wall LIDAR return in world coordinates PER LAP rather than
over the whole bag, then, for each object the whole-bag map found, read its
per-lap return count normalised by EXPOSURE -- the ticks the robot spent within
``--exposure-radius`` of it. Normalising is what makes the comparison legitimate:
raw counts fall every lap simply because a lap that ends early gives the LIDAR
fewer chances to look.

CONTROLS, because a crashed diagnostic here exits 0:

* A lap with fewer than ``--min-exposure`` ticks near the object is NOT allowed
  to declare it gone; the final partial lap routinely has two or three ticks and
  would otherwise report every object on the mat as destroyed.
* The NEIGHBOURS are the control for occlusion. A toppled call is only
  interesting when other objects keep returning over the same laps, and the
  table prints all of them so that is checkable by eye: in the validated round
  the surviving parking wall keeps returning across the very laps its partner
  reads zero.
* Per-lap centroids are printed for every object, so a "topple" that is really a
  slide shows up as the cluster reappearing a few centimetres away.

TRAP: an object toppled BEFORE the bag starts never appears at all, and this
script cannot see it. A round that begins with fewer objects than the layout
states is evidence of that, not of a detection failure -- check the count first.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_toppled_objects.py RUN_DIR...
"""

from __future__ import annotations

import contextlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

# isort: off
from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_nav_debug,
    open_reader,
    scan_to_ranges_angles,
)
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants import RobotSpecs  # noqa: E402
from shared.domain.models import Pose  # noqa: E402

# isort: on

_SELF_RETURN_M = 0.15
_PEAK_CLAIM_M = 0.30
_OBJECT_R = 0.18  # returns this close to a peak belong to that object
_LOT_BAND_M = 0.30  # nearer than this to the OUTER wall is parking furniture, not a sign
_TRACK_MIN, _TRACK_MAX = 0.0, 3.0
_INNER_MIN, _INNER_MAX = 1.0, 2.0


def _off_wall(px: np.ndarray, py: np.ndarray, margin: float) -> np.ndarray:
    """Returns far enough from every track wall to be an object rather than the mat's edge."""
    outer = (
        (px > _TRACK_MIN + margin) & (px < _TRACK_MAX - margin) & (py > _TRACK_MIN + margin) & (py < _TRACK_MAX - margin)
    )
    inner = (
        (px > _INNER_MIN - margin) & (px < _INNER_MAX + margin) & (py > _INNER_MIN - margin) & (py < _INNER_MAX + margin)
    )
    return outer & ~inner


def _from_outer_wall(x: float, y: float) -> float:
    """Distance from the nearest OUTER wall face.

    Deliberately NOT "distance to the nearest wall". The parking lot stands
    against the OUTER wall only, so a general wall distance calls every object
    hugging the inner block parking furniture as well, which mislabels legal
    signs as walls. A column whose name
    promises one quantity and delivers another is the failure mode this repo
    keeps paying for; this one measures exactly what its name says.
    """
    return min(x - _TRACK_MIN, _TRACK_MAX - x, y - _TRACK_MIN, _TRACK_MAX - y)


def _read_per_lap(
    bag_dir: Path, wall_margin: float, cell: float
) -> tuple[list[tuple[Pose, int]], object, dict[int, dict[tuple[int, int], int]]]:
    """One pass over the bag, accumulating the LIDAR into a SEPARATE grid per lap."""
    reader = open_reader(bag_dir)
    pose: Pose | None = None
    lap = 0
    track: list[tuple[Pose, int]] = []
    direction = None
    per_lap: dict[int, dict[tuple[int, int], int]] = {}
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(snap.pose_x, (int, float)) and isinstance(snap.pose_y, (int, float)):
                pose = Pose(x=float(snap.pose_x), y=float(snap.pose_y), yaw=float(snap.pose_yaw or 0.0))
                lap = int(snap.laps_completed or 0)
                track.append((pose, lap))
            if snap.direction is not None:
                direction = snap.direction
            continue
        if topic != Topics.SCAN or pose is None:
            continue
        ranges = angles = None
        with contextlib.suppress(Exception):
            ranges, angles = scan_to_ranges_angles(deserialize_message(data, LaserScan))
        if ranges is None:
            continue
        keep = (ranges > _SELF_RETURN_M) & (ranges < RobotSpecs.LIDAR_MAX_RANGE * 0.99)
        r, a = ranges[keep], angles[keep]
        if r.size == 0:
            continue
        w = pose.yaw + a
        px, py = pose.x + r * np.cos(w), pose.y + r * np.sin(w)
        sel = _off_wall(px, py, wall_margin)
        grid = per_lap.setdefault(lap, {})
        for gx, gy in zip(np.floor(px[sel] / cell).astype(int), np.floor(py[sel] / cell).astype(int), strict=True):
            grid[(gx, gy)] = grid.get((gx, gy), 0) + 1
    return track, direction, per_lap


def _peaks(
    counts: dict[tuple[int, int], int], cell: float, min_returns: int, max_objects: int
) -> list[tuple[float, float, int]]:
    """Greedy peak picking: each peak claims a neighbourhood before the next is taken."""
    out: list[tuple[float, float, int]] = []
    for (gx, gy), n in sorted(counts.items(), key=lambda kv: -kv[1]):
        if n < min_returns:
            break
        x, y = (gx + 0.5) * cell, (gy + 0.5) * cell
        if any(math.hypot(x - a, y - b) < _PEAK_CLAIM_M for a, b, _ in out):
            continue
        out.append((x, y, n))
        if len(out) >= max_objects:
            break
    return out


def _lap_stats(
    grid: dict[tuple[int, int], int], cx: float, cy: float, cell: float
) -> tuple[int, tuple[float, float] | None]:
    """Returns from this object on this lap, and where its centroid sat."""
    total = 0
    sx = sy = 0.0
    for (gx, gy), n in grid.items():
        x, y = (gx + 0.5) * cell, (gy + 0.5) * cell
        if math.hypot(x - cx, y - cy) <= _OBJECT_R:
            total += n
            sx += x * n
            sy += y * n
    return total, ((sx / total, sy / total) if total else None)


def main() -> int:
    """Report the objects that left the LIDAR plane, and the ones that marched."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--wall-margin", type=float, default=0.25)
    parser.add_argument("--cell", type=float, default=0.05)
    parser.add_argument("--min-returns", type=int, default=120, help="returns before a cluster counts as an object")
    parser.add_argument("--max-objects", type=int, default=16)
    parser.add_argument("--exposure-radius", type=float, default=1.5, help="a tick this close to an object sees it")
    parser.add_argument(
        "--min-exposure", type=int, default=40, help="ticks a lap needs before its silence means anything"
    )
    parser.add_argument("--alive", type=float, default=0.30, help="returns/tick that counts as still standing")
    parser.add_argument("--dead", type=float, default=0.05, help="returns/tick that counts as gone from the plane")
    parser.add_argument("--march", type=float, default=0.08, help="centroid travel (m) that counts as PUSHED")
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        track, direction, per_lap = _read_per_lap(bag_dir, args.wall_margin, args.cell)
        if not per_lap:
            print(f"\n=== {bag_dir.name}: no scans with a pose, nothing to report")
            continue
        merged: dict[tuple[int, int], int] = {}
        for grid in per_lap.values():
            for k, n in grid.items():
                merged[k] = merged.get(k, 0) + n
        objects = _peaks(merged, args.cell, args.min_returns, args.max_objects)
        laps = sorted(per_lap)

        rows = []
        toppled = pushed = 0
        for cx, cy, n in objects:
            rates: list[float | None] = []
            exps: list[int] = []
            cents: list[tuple[float, float] | None] = []
            for lap in laps:
                total, cent = _lap_stats(per_lap[lap], cx, cy, args.cell)
                ex = sum(
                    1 for p, l in track if l == lap and math.hypot(p.x - cx, p.y - cy) < args.exposure_radius
                )
                exps.append(ex)
                rates.append(total / ex if ex else None)
                cents.append(cent if ex >= args.min_exposure else None)

            verdict = "standing"
            for i in range(len(laps) - 1):
                rate = rates[i]
                if rate is None or rate < args.alive:
                    continue
                later = [(rates[j], exps[j]) for j in range(i + 1, len(laps)) if exps[j] >= args.min_exposure]
                if later and all(r is not None and r <= args.dead for r, _ in later):
                    verdict = f"TOPPLED after L{laps[i]}"
                    toppled += 1
                    break

            seen = [c for c in cents if c is not None]
            march = max((math.hypot(a[0] - b[0], a[1] - b[1]) for a in seen for b in seen), default=0.0)
            if verdict == "standing" and march >= args.march:
                verdict = "PUSHED"
                pushed += 1

            rows.append(
                [
                    f"({cx:.2f},{cy:.2f})",
                    "lot?" if _from_outer_wall(cx, cy) < _LOT_BAND_M else "sign",
                    n,
                    *[("None" if r is None else f"{r:.2f}") for r in rates],
                    f"{march * 100:.1f}",
                    verdict,
                ]
            )

        print(f"\n=== {bag_dir.name}  direction={direction}  laps={laps}")
        print_table(
            rows,
            ["object", "kind", "returns", *[f"L{lap} /tick" for lap in laps], "march cm", "verdict"],
        )
        print(f"exposure ticks per lap: {dict(zip(laps, [sum(1 for _p, l in track if l == lap) for lap in laps], strict=True))}")
        print(f"TOPPLED: {toppled}    PUSHED: {pushed}    of {len(objects)} objects")
    print(
        "\nA toppled object is gone from the LIDAR's horizontal plane; the neighbours in the\n"
        "same table are the control for occlusion. Objects destroyed BEFORE the bag opens\n"
        "cannot appear here -- check the object count against the stated layout first."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
