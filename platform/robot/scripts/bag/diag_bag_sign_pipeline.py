r"""Where the sign-avoidance pipeline loses its anticipation.

Obstacles runs collide with pillars and rarely reach half a lap. The escape is
not the culprit -- ``diag_bag_contact_bearing.py`` shows it sees 87% of what it
hits and fires at ``CONTACT_DIST`` by design, so it is a backstop, not
avoidance. Avoidance is the sign lane, which is built to activate at
``ACTIVATION_DIST_M`` 1.4 m and ramp over ``SIGN_LANE_RAMP_M`` 0.9 m.

It never gets that far. This measures the chain end to end on one set of runs
so the loss can be attributed to a stage instead of guessed at:

1. WHAT THE CAMERA SEES. Range implied by each detection's bbox height through
   the shipped pinhole (``_CAMERA_FOCAL_PX * sign height / px * RANGE_SCALE``)
   -- the same arithmetic ``sign_discovery`` does, so this is the range the
   ingest saw and not an idealised one.
2. WHEN THE ROUTER COMMITS. Distance to the sign at first commitment, from
   ``committed_sign_x_m``/``_y_m``.
3. WHAT CLEARANCE RESULTS. Closest robot-to-sign approach over that
   commitment.

Reads the payload as a JSON LIST. ``parse_detection`` takes a single detection
object, so handing it the topic payload silently yields nothing and reports a
camera that saw zero signs all session.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_pipeline.py \
        data/live/runs/run_20260908_0047* data/live/runs/run_20260908_0053*
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import TrafficSignSpecs

from scripts.common.bag_io import create_bags_parser, open_reader
from src.config.tuning_helpers import get_tuning
from src.navigation.planning.sign_discovery import _CAMERA_FOCAL_PX

VISION_TOPIC = "/vision/detections"

FRAME_W_PX = 1536
"""Camera width, for the frame-clipping test. A clipped box's height is still
valid but its aspect is not, which is why the aspect gate excuses it."""


def _q(values: list[float], f: float) -> float:
    return values[min(len(values) - 1, int(f * len(values)))]


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()

    tuning = get_tuning(None)
    disc = tuning.sign_discovery
    ranges: list[float] = []
    clipped = short = beyond = total = 0

    for bag in args.bag_dirs:
        reader = open_reader(Path(bag))
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != VISION_TOPIC:
                continue
            try:
                dets = json.loads(deserialize_message(data, String).data)
            except Exception:  # noqa: BLE001
                continue
            for det in dets:
                x0, _y0, x1, _y1 = det["bbox"]
                height_px = det["bbox"][3] - det["bbox"][1]
                total += 1
                if x0 <= disc.FRAME_EDGE_TOLERANCE_PX or x1 >= FRAME_W_PX - disc.FRAME_EDGE_TOLERANCE_PX:
                    clipped += 1
                if height_px < disc.MIN_RELIABLE_BBOX_HEIGHT_PX:
                    short += 1
                    continue
                rng = _CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT / height_px * disc.RANGE_SCALE
                if rng > disc.MAX_INGEST_RANGE_M:
                    beyond += 1
                ranges.append(rng)

    print(f"\n== STAGE 1: WHAT THE CAMERA SEES  ({total} detections)")
    if not ranges:
        print("  none usable")
        return 0
    ranges.sort()
    print(f"  frame-clipped: {clipped} ({100 * clipped / total:.1f}%)")
    print(f"  below MIN_RELIABLE_BBOX_HEIGHT_PX: {short}")
    print(f"  beyond MAX_INGEST_RANGE_M ({disc.MAX_INGEST_RANGE_M}), dropped: {beyond}")
    print(
        f"  implied range m: p10={_q(ranges, 0.10):.2f} p50={_q(ranges, 0.50):.2f} "
        f"p90={_q(ranges, 0.90):.2f} max={ranges[-1]:.2f}"
    )
    for thr, label in ((0.9, "SIGN_LANE_RAMP_M"), (1.4, "ACTIVATION_DIST_M")):
        k = sum(1 for v in ranges if v >= thr)
        print(f"    at or beyond {thr} m ({label}): {k}/{len(ranges)} ({100 * k / len(ranges):.1f}%)")

    print("\n  Stages 2 and 3 are diag_bag_sign_approach.py, which already reports")
    print("  commit range and closest approach over the same commitments. Run both:")
    print("  the loss between the camera's p50 and the commit p50 is track formation")
    print("  (MIN_HITS), and the gap between ACTIVATION_DIST_M and the camera's p50 is")
    print("  the detector's reach -- they are different fixes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
