r"""Where does the sign lane lose its 1.4 m of anticipation?

``activation_dist_m`` is 1.40: the router is built to start deforming a metre
and a half before the pillar. It does not get that. Measured 2026-09-11 over 78
hardware bags, the commitment lands at p50 **0.43 m** on the passes that fail
and 0.52 m on the ones that work, and a pass that must CROSS from the wrong
side wins only 34% of the time -- the room to cross is what the missing metre
would have bought.

"Commits late" is the symptom. This script asks WHERE the metre goes, because
each stage loses it for a different reason and wants a different fix:

1. **SEEN** -- the first frame the detector emits a RED/GREEN box for the
   pillar at all. Bounded by the camera and the model, and nothing downstream
   can recover distance that was never perceived.
2. **INGESTED** -- the first observation the map actually accepts. Loses
   distance to ``MAX_INGEST_RANGE_M`` (1.5) and to the aspect/confidence gates,
   so a pillar can be visible for many frames before it counts.
3. **PUBLISHED** -- the track reaching ``MIN_HITS`` (3) and becoming a sign the
   router can see. Pure latency: three confirmations at the frame rate, while
   the robot keeps closing.
4. **COMMITTED** -- the router engaging it, which is what ``activation_dist_m``
   governs and the only stage anyone has been tuning.

Reported as the range at each step, so the drops between them are the budget.
If most of the metre is gone before step 4, raising ``activation_dist_m`` is
inert -- the router cannot engage a sign that does not exist yet, which is the
class of bug ``SNAP_TO_LATTICE_M`` and the tuning-threading fix both turned out
to be.

Ranges are robot-to-BELIEVED-pillar at the moment of each event. The believed
position is the honest input: it is what every stage downstream actually used.

Usage::

    pixi run -e dev test   # unrelated; this script is run directly:
    pixi run -e dev python scripts/bag/diag_bag_commit_chain.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from scripts.bag.diag_bag_pass_side import _load, _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import create_bags_parser, decode_detections, settled_direction  # noqa: E402
from scripts.common.stats import nearest_by_time, percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Direction  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402

CELL_M = 0.35
"""Positions within this radius are treated as the same physical pillar.

The same constant ``diag_bag_pass_side.py`` uses to fold a commitment's
anchors, and comfortably below the 0.20 m spacing of the two division lines
once the map's own error is allowed for.
"""


def _key(x: float, y: float) -> tuple[int, int]:
    return round(x / CELL_M), round(y / CELL_M)


def chain_for_run(rows, frames, scans, tuning) -> list[dict[str, float]]:  # noqa: ANN001
    """First range at each stage, per pillar, replaying the shipped router."""
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    scan_times = [t for t, _ in scans]
    frame_i = 0
    # stage -> {cell: range}. First write wins, which is the earliest event.
    seen: dict[tuple[int, int], float] = {}
    ingested: dict[tuple[int, int], float] = {}
    published: dict[tuple[int, int], float] = {}
    committed: dict[tuple[int, int], float] = {}

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )

        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            for det in decode_detections(frames[frame_i][1]):
                if det.class_name not in (SignColor.RED, SignColor.GREEN):
                    continue
                # SEEN is the raw box, before any gate. Located through the
                # same projection the map uses, so the four stages are measured
                # on one ruler rather than three.
                o = detection_to_observation(det, pose, tuning, ranges, angles)
                if o is None:
                    continue
                cell = _key(o.world_x_m, o.world_y_m)
                rng = math.hypot(o.world_x_m - d.pose_x, o.world_y_m - d.pose_y)
                seen.setdefault(cell, rng)
                obs.append(o)
            frame_i += 1

        if d.steer_target_x is None:
            continue
        router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )

        # INGESTED: a track exists for this cell at all. PUBLISHED: the router
        # can see it, i.e. it survived MIN_HITS.
        for track in getattr(router._sign_map, "_tracks", []):  # noqa: SLF001
            cell = _key(track.x, track.y)
            ingested.setdefault(cell, math.hypot(track.x - d.pose_x, track.y - d.pose_y))
        for sign in router.signs:
            cell = _key(sign.x, sign.y)
            published.setdefault(cell, math.hypot(sign.x - d.pose_x, sign.y - d.pose_y))

        anchor = router.committed_sign_position
        if anchor is not None:
            cell = _key(anchor.x, anchor.y)
            committed.setdefault(cell, math.hypot(anchor.x - d.pose_x, anchor.y - d.pose_y))

    out = []
    # Only pillars that made it all the way: a cell that never committed has no
    # budget to account for, and including it would flatter the early stages.
    for cell, commit_rng in committed.items():
        if cell not in seen:
            continue
        out.append(
            {
                "seen": seen[cell],
                "ingested": ingested.get(cell, float("nan")),
                "published": published.get(cell, float("nan")),
                "committed": commit_rng,
            }
        )
    return out


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    args = parser.parse_args()
    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)

    chains: list[dict[str, float]] = []
    skipped = 0
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        chains.extend(chain_for_run(rows, frames, scans, tuning))

    if skipped:
        print(f"== SKIPPED {skipped} unreadable bag(s)")
    if not chains:
        print("No committed pillars reconstructed from these bags.")
        return

    print(f"== {len(chains)} committed pillars. Range at each stage, and what the stage COST.")
    stages = ("seen", "ingested", "published", "committed")
    rows_out = []
    previous_p50 = None
    for stage in stages:
        vals = [c[stage] for c in chains if not math.isnan(c[stage])]
        if not vals:
            rows_out.append([stage, "--", "--", "--", "--"])
            continue
        p50 = percentile(vals, 0.5)
        drop = "--" if previous_p50 is None else f"-{previous_p50 - p50:.3f} m"
        rows_out.append([
            stage,
            f"{percentile(vals, 0.1):.3f}",
            f"{p50:.3f}",
            f"{percentile(vals, 0.9):.3f}",
            drop,
        ])
        previous_p50 = p50
    print_table(rows_out, ["stage", "p10", "p50 range m", "p90", "p50 lost here"])
    print()

    tuned = get_tuning(None)
    activation = tuned.sign_router.ACTIVATION_DIST_M if hasattr(tuned, "sign_router") else None
    if activation:
        # The headroom question: could the router have engaged earlier at all?
        could = [c for c in chains if not math.isnan(c["published"]) and c["published"] > activation]
        print(f"== activation_dist_m = {activation:.2f}")
        print(f"   pillars PUBLISHED further out than that: {len(could)}/{len(chains)}"
              f"  ({100 * len(could) / len(chains):.1f}%)")
        print("   Those are the only ones where activation_dist_m is the binding constraint.")
        print("   For the rest the router engaged as soon as a sign existed, and raising it is inert.")


if __name__ == "__main__":
    main()
