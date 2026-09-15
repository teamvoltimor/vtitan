r"""What would the MAGENTA barrier belief have suppressed, on the real rounds?

The simulator never emits a magenta detection and never mislabels a colour
(``vision_color_flip_rate`` ships at 0.0, unmeasured), so the barrier-as-red
failure is structurally unreachable in the corpus. A bag is therefore the ONLY
instrument that can score
:mod:`src.navigation.planning.barrier_belief`, and this is that instrument.

It replays each bag's recorded detections in order against the recorded pose,
builds the belief out of the magenta boxes exactly as the gateway now does, and
asks of every RED box: would it have been refused?

Two numbers decide whether this is worth shipping, and they pull opposite ways:

* WALL-SHAPED reds suppressed -- the barrier being kept out of the sign map,
  which is the point.
* PILLAR-SHAPED reds suppressed -- real signs wrongly refused, which is the
  cost. A pillar standing legitimately close to the lot is the failure case,
  and it is why the suppression radius is not simply made large.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_barrier_belief.py RUN_DIR [RUN_DIR ...]
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
# `ruff check --fix` will re-sort these into the cycle if the noqa markers are
# dropped -- they are load-bearing, not decoration.
from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_detections,
    decode_nav_debug,
    open_reader,
    scan_to_ranges_angles,
)
from shared.domain.models import Detection, Pose, SignColor  # noqa: E402

from scripts.bag.diag_bag_barrier_gate import MAX_PILLAR_ASPECT  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.barrier_belief import BarrierBelief  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_world_point  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Sequence


def _pose_of(snap: object) -> Pose | None:
    """The snapshot's pose, or None for a tick recorded before the navigator had a fix."""
    if not isinstance(snap.pose_x, (int, float)) or not isinstance(snap.pose_y, (int, float)):
        return None
    yaw = snap.pose_yaw if isinstance(snap.pose_yaw, (int, float)) else 0.0
    return Pose(x=float(snap.pose_x), y=float(snap.pose_y), yaw=float(yaw))


def _seed_from_start(belief: BarrierBelief, start: Pose) -> None:
    """Assert the lot at the start pose, bypassing the evidence threshold.

    One sighting would never qualify. The start pose is not evidence to
    accumulate but a fact the rulebook supplies, so it is asserted
    ``min_sightings`` times rather than observed once.
    """
    for _ in range(max(belief.min_sightings, 1)):
        belief.observe(start.x, start.y)


def _pct(part: int, whole: int) -> str:
    """``part`` as a percentage of ``whole``, or ``-`` when nothing was counted."""
    return f"{100.0 * part / whole:.1f}%" if whole else "-"


@dataclass
class _Tally:
    """Benefit and cost, counted separately for wall-shaped and pillar-shaped reds."""

    magenta: int = 0
    reds: int = 0
    wall_supp: int = 0
    wall_total: int = 0
    pillar_supp: int = 0
    pillar_total: int = 0

    def score(
        self,
        det: Detection,
        pose: Pose,
        belief: BarrierBelief,
        ranges: Sequence[float] | None,
        angles: Sequence[float] | None,
        *,
        feed_belief: bool,
    ) -> None:
        """Fold one detection in: magenta builds the belief, red is scored against it."""
        if det.color is SignColor.MAGENTA:
            point = detection_to_world_point(det, pose, lidar_ranges_m=ranges, lidar_angles_rad=angles)
            if point is not None:
                if feed_belief:
                    belief.observe(*point)
                self.magenta += 1
            return
        if det.color is not SignColor.RED:
            return
        self.reds += 1
        point = detection_to_world_point(det, pose, lidar_ranges_m=ranges, lidar_angles_rad=angles)
        if point is None:
            return
        bbox = det.as_bbox()
        height = bbox.y_max - bbox.y_min
        if height <= 0:
            return
        suppressed = int(belief.suppresses(*point))
        if (bbox.x_max - bbox.x_min) / height > MAX_PILLAR_ASPECT:
            self.wall_total += 1
            self.wall_supp += suppressed
        else:
            self.pillar_total += 1
            self.pillar_supp += suppressed


def replay(bag_dir: Path, *, fuse_lidar: bool = True, lot_source: str = "magenta") -> dict[str, float | int | str]:
    """Rebuild the belief from one bag and score it against that bag's reds.

    ``fuse_lidar`` selects which projection is replayed. True mirrors
    production (`sign_discovery.lidar_range_fusion` ships true, so the gateway
    hands the latest sweep to every projection); False is the raw pinhole this
    script replayed until 2026-09-15, kept only so the two can be compared.

    ``lot_source`` selects where the lot comes from. ``magenta`` is the shipped
    belief, built from detections. ``start-pose`` ignores every detection and
    puts the lot at the run's OWN START POSE, on the reasoning
    :func:`parking_lot_from_in_bay_start` already states: in the Obstacles
    Challenge the robot starts INSIDE the lot, so the start pose IS the lot and
    no sensing is required. It is here as the control the detection-built
    belief has to beat.
    """
    sd = get_tuning(None).sign_discovery
    belief = BarrierBelief(
        min_sightings=sd.barrier_belief_min_sightings,
        merge_radius_m=sd.barrier_merge_radius_m,
        suppression_radius_m=sd.barrier_suppression_radius_m,
    )

    reader = open_reader(bag_dir)
    pose: Pose | None = None
    start_pose: Pose | None = None
    # The gateway fuses the LATEST scan into every projection
    # (`get_vision_detections` -> `get_lidar_scan`). Replaying without it
    # measured the raw pinhole, which production has not run since
    # `lidar_range_fusion` shipped -- and the pinhole under-reads far enough to
    # put the believed lot off the mat, so the tool invented its own verdict.
    ranges = angles = None
    tally = _Tally()

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            fixed = _pose_of(snap)
            if fixed is not None:
                pose = fixed
                if start_pose is None:
                    start_pose = pose
                    if lot_source == "start-pose":
                        _seed_from_start(belief, pose)
            continue
        if topic == Topics.SCAN:
            if fuse_lidar:
                with contextlib.suppress(Exception):
                    ranges, angles = scan_to_ranges_angles(deserialize_message(data, LaserScan))
            continue
        if topic != Topics.VISION_DETECTIONS or pose is None:
            continue
        try:
            payload = json.loads(deserialize_message(data, String).data)
        except Exception:  # noqa: BLE001
            continue
        for det in decode_detections(payload):
            tally.score(det, pose, belief, ranges, angles, feed_belief=lot_source == "magenta")

    believed = belief.believed()
    return {
        "run": bag_dir.name.replace("run_", ""),
        "lot from": lot_source,
        "proj": "lidar" if fuse_lidar else "pinhole",
        "magenta": tally.magenta,
        "reds": tally.reds,
        "believed lots": len(believed),
        # WHERE the belief put the lot is the whole verdict: a lot placed off
        # the mat suppresses nothing, and one placed on a traffic lane
        # suppresses real pillars. The benefit/cost pair alone cannot tell
        # those two failures apart.
        "lot (x,y)": ", ".join(f"({s.x_m:.2f},{s.y_m:.2f})" for s in believed) or "-",
        "lot hits": ", ".join(str(s.sightings) for s in believed) or "-",
        "wall supp": f"{tally.wall_supp}/{tally.wall_total}",
        "wall %": _pct(tally.wall_supp, tally.wall_total),
        "PILLAR supp": f"{tally.pillar_supp}/{tally.pillar_total}",
        "PILLAR %": _pct(tally.pillar_supp, tally.pillar_total),
    }


def main() -> int:
    parser = create_bags_parser(__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--lot",
        choices=("magenta", "start-pose", "both"),
        default="magenta",
        help="where the lot comes from; 'start-pose' is the no-sensing control",
    )
    parser.add_argument(
        "--projection",
        choices=("lidar", "pinhole", "both"),
        default="lidar",
        help="which projection to replay; 'lidar' mirrors production, 'both' compares the arms",
    )
    args = parser.parse_args()

    sd = get_tuning(None).sign_discovery
    print(
        f"belief: min_sightings={sd.barrier_belief_min_sightings} "
        f"merge={sd.barrier_merge_radius_m} suppress={sd.barrier_suppression_radius_m}"
    )
    arms = (True, False) if args.projection == "both" else (args.projection == "lidar",)
    lots = ("magenta", "start-pose") if args.lot == "both" else (args.lot,)
    rows = [
        replay(bag_dir, fuse_lidar=arm, lot_source=lot) for bag_dir in args.bag_dirs for lot in lots for arm in arms
    ]
    headers = list(rows[0].keys())
    print_table([[r[h] for h in headers] for r in rows], headers)
    print(
        "\nwall supp is the BENEFIT (barrier kept out of the sign map);\n"
        "PILLAR supp is the COST (a real sign refused). Read both."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
