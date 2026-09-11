r"""How many wall-shaped RED detections still reach the sign map, and through which hole?

The operator's report from the 2026-09-11 15:2x rounds: the robot approaches the
parking lot "as if it thought it were a red block". ``MAX_PILLAR_ASPECT`` exists
for exactly that -- the magenta barrier reads as RED under motion blur at p50
confidence 0.79, so colour and confidence cannot reject it and shape must.

But the gate has two deliberate exemptions, and either one lets the barrier
through:

* CLIPPED. A box touching the frame edge skips the shape test, because a pillar
  the robot is closing on grows out of frame and its w/h crosses 1.0 with
  nothing about the pillar having changed. Measured previously: 61% of the reds
  this gate rejects are clipped.
* NOT BARRIER_POSSIBLE. The test only runs where the lot can BE -- the parking
  corridor or an unknown one. A wide red box seen from any other corridor is
  assumed to be a pillar. The camera's 102 deg HFOV does not respect corridor
  boundaries, so this assumption is worth checking rather than trusting.

So this replays every red detection through the gate and reports which ones are
ADMITTED despite being wall-shaped, split by the exemption that admitted them.
A leak concentrated in one exemption names the fix; one spread evenly says the
shape test itself is the wrong instrument.

The parking corridor is DERIVED from the magenta detections rather than read
from metadata, which the bag does not carry -- the barrier is the only magenta
object on the track. Derived ONCE over every bag given, never per run: the lot
does not move between rounds, while a single round's magenta lands in the
neighbouring corridor often enough to name the wrong one. Measured 2026-09-11,
three rounds of one session split 97% south, 67% south and 60% WEST.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_barrier_gate.py \
        data/live/runs/run_20260911_1523* data/live/runs/run_20260911_1528*
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

from rclpy.serialization import deserialize_message
from shared.config.constants import RobotSpecs
from shared.domain.enums import Section
from shared.domain.models import SignColor
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import (
    Topics,
    create_bags_parser,
    decode_detections,
    decode_nav_debug,
    open_reader,
)
from scripts.common.tables import print_table

MAX_PILLAR_ASPECT = 1.0
"""``SignDiscoveryParams.MAX_PILLAR_ASPECT`` as shipped."""

FRAME_EDGE_TOLERANCE_PX = 2.0
"""``SignDiscoveryParams.FRAME_EDGE_TOLERANCE_PX`` as shipped."""

MIN_MAGENTA_FOR_A_VERDICT = 20
"""Below this the parking corridor cannot be derived from the runs themselves,
and a guess would decide the whole report."""

RedBox = tuple[tuple[float, float, float, float], str | None]


def _frame_size(boxes: list[RedBox]) -> tuple[float, float]:
    """Capture size measured from the runs' own boxes, never from config.

    ``camera/config.toml`` carries the MODEL INPUT size while the detections are
    in CAPTURE coordinates, and the clipping test is an edge comparison -- a
    frame width wrong by a factor of 2.4 makes every box look unclipped.
    """
    return (
        max((b[2] for b, _ in boxes), default=float(RobotSpecs.CAMERA_WIDTH)),
        max((b[3] for b, _ in boxes), default=float(RobotSpecs.CAMERA_HEIGHT)),
    )


def collect(bag_dir: Path) -> tuple[list[RedBox], Counter]:
    """Every red box with the corridor the robot was in, and magenta by corridor."""
    reader = open_reader(bag_dir)
    corridor: str | None = None
    reds: list[RedBox] = []
    magenta_sections: Counter[str] = Counter()

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            try:
                snap = decode_nav_debug(data)
            except Exception:  # noqa: BLE001
                continue
            corridor = str(snap.current_corridor) if snap.current_corridor else None
            continue
        if topic != Topics.VISION_DETECTIONS:
            continue
        try:
            payload = json.loads(deserialize_message(data, String).data)
        except Exception:  # noqa: BLE001
            continue
        for det in decode_detections(payload):
            bbox = det.as_bbox()
            box = (bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max)
            if det.class_name == SignColor.MAGENTA:
                magenta_sections[corridor or "None"] += 1
            elif det.class_name == SignColor.RED:
                reds.append((box, corridor))

    return reds, magenta_sections


def report(
    name: str,
    reds: list[RedBox],
    parking_corridor: str,
    frame: tuple[float, float],
    *,
    neighbours_count: bool,
) -> None:
    frame_w, frame_h = frame
    adjacent = {s.value for s in Section(parking_corridor).neighbours} if neighbours_count else set()
    buckets: Counter[str] = Counter()
    for (x_min, y_min, x_max, y_max), sect in reds:
        height = y_max - y_min
        width = x_max - x_min
        wall_shaped = height > 0 and width / height > MAX_PILLAR_ASPECT
        clipped = (
            x_min <= FRAME_EDGE_TOLERANCE_PX
            or y_min <= FRAME_EDGE_TOLERANCE_PX
            or x_max >= frame_w - FRAME_EDGE_TOLERANCE_PX
            or y_max >= frame_h - FRAME_EDGE_TOLERANCE_PX
        )
        barrier_possible = sect is None or sect == parking_corridor or sect in adjacent
        if not wall_shaped:
            buckets["pillar-shaped (gate not concerned)"] += 1
        elif barrier_possible and not clipped:
            buckets["wall-shaped, REJECTED by the gate"] += 1
        elif not barrier_possible:
            buckets["wall-shaped, ADMITTED: corridor exempt"] += 1
        else:
            buckets["wall-shaped, ADMITTED: box clipped"] += 1

    total = len(reds)
    if not total:
        print(f"\n== {name}: no red detections")
        return
    rows = [
        [label, count, f"{100.0 * count / total:.1f}%"]
        for label, count in sorted(buckets.items(), key=lambda kv: -kv[1])
    ]
    print(f"\n== {name}: {total} red detections")
    print_table(rows, ["outcome", "n", "share"])

    admitted = sum(v for k, v in buckets.items() if "ADMITTED" in k)
    wall = admitted + buckets["wall-shaped, REJECTED by the gate"]
    if wall:
        print(f"   of {wall} wall-shaped reds the gate LET THROUGH {admitted} ({100.0 * admitted / wall:.1f}%)")


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()

    per_bag: dict[str, list[RedBox]] = {}
    pooled: Counter[str] = Counter()
    for bag_dir in args.bag_dirs:
        try:
            reds, magenta = collect(Path(bag_dir))
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:90]}")
            continue
        per_bag[Path(bag_dir).name] = reds
        pooled += magenta

    labelled = Counter({k: v for k, v in pooled.items() if k != "None"})
    if sum(pooled.values()) < MIN_MAGENTA_FOR_A_VERDICT or not labelled:
        print("too few corridor-labelled magenta detections to derive the parking corridor")
        return 0
    parking_corridor, n = labelled.most_common(1)[0]
    print(
        f"parking corridor derived as {parking_corridor!r} over all bags "
        f"({n}/{sum(labelled.values())} corridor-labelled magenta detections)"
    )

    frame = _frame_size([b for reds in per_bag.values() for b in reds])
    all_reds: list[RedBox] = [b for reds in per_bag.values() for b in reds]
    # Both rules, side by side, because the question is what the widening BUYS.
    # The lot's corridor alone is what shipped before 2026-09-11; adding its
    # neighbours is the repair, and the residue under the second rule is what
    # neither reaches.
    for neighbours_count in (False, True):
        rule = "lot's corridor + NEIGHBOURS" if neighbours_count else "lot's corridor only (was shipped)"
        print("\n" + "=" * 70)
        print(f"RULE: {rule}")
        for name, reds in per_bag.items():
            report(name, reds, parking_corridor, frame, neighbours_count=neighbours_count)
        if len(per_bag) > 1:
            report("ALL RUNS", all_reds, parking_corridor, frame, neighbours_count=neighbours_count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
