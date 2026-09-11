r"""How far does the detector actually reach, and what throws its boxes away?

62% of the sign lane's missing anticipation is spent before the lane exists:
`diag_bag_commit_chain.py` puts the first usable sighting at p50 **0.824 m**
against an `activation_dist_m` of 1.40. Nothing downstream can recover distance
that was never perceived, so this asks whether the detector is the limit or
whether a gate is throwing away boxes it did emit.

The arithmetic that makes this worth checking. The pinhole is
``d = f * H / h * RANGE_SCALE`` with f = 621.9 px, H = 0.10 m and RANGE_SCALE
1.95, so d = 121.3 / h:

    box   5 px -> 24.3 m       box  50 px -> 2.43 m
    box  20 px ->  6.1 m       box 147 px -> 0.82 m

`MIN_RELIABLE_BBOX_HEIGHT_PX` is 5, i.e. 24 m on a 3 m track -- it cannot be
binding. And a first sighting at 0.824 m means the box was already ~147 px
tall, 17% of the frame height. A detector that only fires on an object that
large is leaving most of the track unused; a gate that discards the smaller
boxes is a cheap fix. The two look identical from downstream, so count them.

Reports, over every RED/GREEN box in the bags:

* the height distribution of the RAW boxes, which is what the model emits;
* what each gate in ``detection_to_observation`` rejects, and at what implied
  range, so a gate throwing away long-range boxes is visible as a rejection
  population further out than the accepted one;
* the CONFIDENCE of the raw boxes against range, since a confidence floor
  applied inside the detector would show up as boxes simply not existing beyond
  some distance rather than being rejected here.

The last bullet is the one that separates the two answers, and it is the reason
this reads raw payloads rather than replaying the map: if the far boxes are not
in the payload at all, no gate in this repo is responsible.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_detection_reach.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.common.bag_io import create_bags_parser, decode_detections, read_vision_rows_and_scans  # noqa: E402
from scripts.common.stats import percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from shared.config.constants.track import TrafficSignSpecs  # noqa: E402
from shared.domain.models import SignColor  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.planning.sign_discovery import _CAMERA_FOCAL_PX  # noqa: E402


def implied_range(height_px: float, range_scale: float) -> float:
    """Pinhole range for a box of this height, or inf for a degenerate box."""
    if height_px <= 0:
        return float("inf")
    return _CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT / height_px * range_scale


def main() -> None:
    args = create_bags_parser(__doc__).parse_args()
    tuning = get_tuning(None).sign_discovery

    heights: list[float] = []
    confidences: list[float] = []
    # range bucket -> [count, mean confidence sum]
    by_range: dict[str, list[float]] = {}
    rejected_small: list[float] = []
    rejected_aspect: list[float] = []
    accepted: list[float] = []
    clipped_count = 0
    skipped = 0

    for bag in args.bag_dirs:
        try:
            _rows, frames, _scans = read_vision_rows_and_scans(Path(bag), with_scans=False)
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        for _rel, payload in frames:
            for det in decode_detections(payload):
                if det.class_name not in (SignColor.RED, SignColor.GREEN):
                    continue
                bbox = det.as_bbox()
                h = bbox.y_max - bbox.y_min
                w = bbox.x_max - bbox.x_min
                heights.append(h)
                confidences.append(det.confidence)
                rng = implied_range(h, tuning.RANGE_SCALE)

                bucket = (
                    "0.0-0.5 m" if rng < 0.5
                    else "0.5-1.0 m" if rng < 1.0
                    else "1.0-1.5 m" if rng < 1.5
                    else "1.5-2.0 m" if rng < 2.0
                    else "2.0+ m"
                )
                entry = by_range.setdefault(bucket, [0.0, 0.0])
                entry[0] += 1
                entry[1] += det.confidence

                # The same order detection_to_observation applies them.
                if h < tuning.MIN_RELIABLE_BBOX_HEIGHT_PX:
                    rejected_small.append(rng)
                    continue
                edge = tuning.FRAME_EDGE_TOLERANCE_PX
                clipped = (
                    bbox.x_min <= edge
                    or bbox.y_min <= edge
                    or bbox.x_max >= RobotSpecs.CAMERA_WIDTH - edge
                    or bbox.y_max >= RobotSpecs.CAMERA_HEIGHT - edge
                )
                if clipped:
                    clipped_count += 1
                if tuning.MAX_PILLAR_ASPECT > 0.0 and not clipped and h > 0 and w / h > tuning.MAX_PILLAR_ASPECT:
                    rejected_aspect.append(rng)
                    continue
                accepted.append(rng)

    if skipped:
        print(f"== SKIPPED {skipped} unreadable bag(s)")
    if not heights:
        print("No RED/GREEN detections in these bags.")
        return

    print(f"== {len(heights)} RED/GREEN boxes. Pinhole: d = {_CAMERA_FOCAL_PX:.1f} * "
          f"{TrafficSignSpecs.HEIGHT} / h * {tuning.RANGE_SCALE}")
    print()
    print("== WHAT THE MODEL EMITS -- raw box height, and the range it implies")
    rows_h = []
    for q in (0.01, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99):
        px = percentile(heights, q)
        rows_h.append([f"p{int(q * 100)}", f"{px:7.1f} px", f"{implied_range(px, tuning.RANGE_SCALE):6.2f} m"])
    print_table(rows_h, ["quantile", "box height", "implied range"])
    print()

    print("== BOXES BY IMPLIED RANGE (is the model even looking out there?)")
    rows_r = []
    for bucket in ("0.0-0.5 m", "0.5-1.0 m", "1.0-1.5 m", "1.5-2.0 m", "2.0+ m"):
        count, conf_sum = by_range.get(bucket, [0.0, 0.0])
        rows_r.append([
            bucket,
            f"{int(count):7d}",
            f"{100 * count / len(heights):5.1f}%",
            f"{conf_sum / count:.3f}" if count else "--",
        ])
    print_table(rows_r, ["implied range", "boxes", "share", "mean confidence"])
    print()

    print("== WHAT THE GATES THROW AWAY")
    total = len(heights)
    rows_g = [
        ["accepted as an observation", f"{len(accepted):7d}", f"{100 * len(accepted) / total:5.1f}%"],
        [f"rejected: height < {tuning.MIN_RELIABLE_BBOX_HEIGHT_PX:g} px",
         f"{len(rejected_small):7d}", f"{100 * len(rejected_small) / total:5.1f}%"],
        [f"rejected: aspect > {tuning.MAX_PILLAR_ASPECT:g}",
         f"{len(rejected_aspect):7d}", f"{100 * len(rejected_aspect) / total:5.1f}%"],
        ["(of all boxes, frame-clipped)", f"{clipped_count:7d}", f"{100 * clipped_count / total:5.1f}%"],
    ]
    print_table(rows_g, ["outcome", "boxes", "share"])
    print()
    for label, vals in (("accepted", accepted), ("rejected by aspect", rejected_aspect)):
        if vals:
            print(f"  implied range, {label:<20} p10 {percentile(vals, 0.1):.2f}  "
                  f"p50 {percentile(vals, 0.5):.2f}  p90 {percentile(vals, 0.9):.2f} m")
    print()
    print(f"  confidence over all boxes: p10 {percentile(confidences, 0.1):.3f}  "
          f"p50 {percentile(confidences, 0.5):.3f}  p90 {percentile(confidences, 0.9):.3f}")


if __name__ == "__main__":
    main()
