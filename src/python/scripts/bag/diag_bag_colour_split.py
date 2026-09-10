"""Does the detector emit BOTH colours for one physical pillar?

`diag_bag_vision.py` measured the symptom on 2026-09-10: greens exit the frame
on the wrong side 9 of 13 times at median x_norm -0.43, deep on the RED side,
while reds fail 8/17 but in the correct direction. A green routed as RED is
exactly what produces that, and the documented mechanism is duplicate tracks
splitting the colour vote (a 4-hit red fragment committed over a 40.8-hit green
pillar, 2026-09-07).

That mechanism has never been measured directly. This does, from the frames
alone, without replaying the router -- so it is independent of the routing
logic it is meant to indict.

Two questions, and they fail differently:

1. **WITHIN a frame**: two detections of DIFFERENT class whose boxes overlap.
   One object cannot be both colours, so an overlapping pair is the detector
   contradicting itself in a single image. No tracking, no association, no
   pose -- the weakest possible assumption.
2. **ACROSS frames**: a run of detections at a stable image position whose
   class CHANGES. This needs an association rule (below), so it is the softer
   of the two: read the within-frame number first.

Association across frames is deliberately crude -- nearest previous detection
within ``--assoc-x`` of normalised x, within ``--assoc-gap`` seconds. It is not
a tracker and does not need to be: a false association inflates flips, so a LOW
flip count is trustworthy and a high one is a prompt to look, not a verdict.

The control that makes a null readable: ``same-class overlapping pairs`` is
printed alongside. Two boxes of the SAME colour overlapping is ordinary (a
pillar and a wall segment behind it), so if that number is also ~0 the frames
simply contain no overlaps and the cross-colour zero means nothing.

RESULT 2026-09-10, AND IT IS ABOUT THIS INSTRUMENT, NOT ABOUT THE HYPOTHESIS.
Both tests came back VOID on run_20260910_135651 + _140059 (5551 frames, 1124
with pillars), and their own controls are what said so:

* Test 1 is dead: **2** same-colour overlapping pairs in 1124 pillar frames. The
  detector emits at most one box per object and the objects are spatially
  separate, so there are no overlaps of ANY kind and the cross-colour 0 measures
  nothing.
* Test 2 is dead: **5 associations against 1293 new tracks** at the defaults,
  and still only **22 against 1276** at ``--assoc-gap 1.5 --assoc-x 0.30``. The
  flip count is 0 BY CONSTRUCTION. Pillar frames arrive at ~2.7 Hz and a close
  pillar's normalised x moves further than any usable gate between samples, so
  naive frame-to-frame association cannot work on this data at all.

**Do not widen the gates and re-run -- the approach is wrong, not the numbers.**
The colour question needs the association the ROUTER already does with pose:
extend the ``SignRouter`` replay in ``diag_bag_pass_side.py`` (which computes
the believed colour) to also dump each committed track's per-frame class tally,
and compare the committed colour against that tally. Kept here so nobody
rebuilds the frame-level version, and because the two CONTROL lines are the
only reason this was caught instead of reported as "colour split refuted".

Usage::

    pixi run -e dev python scripts/bag/diag_bag_colour_split.py \
        ../../data/live/runs/run_20260910_135651
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import Counter
from pathlib import Path

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import create_bags_parser, elapsed_seconds, open_reader

_PILLAR_CLASSES = ("red", "green")


def _boxes_overlap(a: list[float], b: list[float]) -> float:
    """Intersection over the SMALLER box, so a big wall box containing a small
    pillar box scores 1.0 rather than being diluted by its own area."""
    ax0, ay0, ax1, ay1 = a[0], a[1], a[2], a[3]
    bx0, by0, bx1, by1 = b[0], b[1], b[2], b[3]
    ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    iy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ix * iy
    if inter <= 0:
        return 0.0
    smaller = min(max(1e-9, (ax1 - ax0) * (ay1 - ay0)), max(1e-9, (bx1 - bx0) * (by1 - by0)))
    return inter / smaller


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--overlap", type=float, default=0.30, help="min intersection-over-smaller to call two boxes the same object")
    parser.add_argument("--assoc-x", type=float, default=0.12, help="max normalised-x gap to associate across frames")
    parser.add_argument("--assoc-gap", type=float, default=0.40, help="max seconds between frames to associate")
    args = parser.parse_args()

    frames = 0
    frames_with_pillars = 0
    cross_pairs = 0
    same_pairs = 0
    frames_with_cross = 0
    cross_overlaps: list[float] = []
    conf_minority: list[float] = []
    conf_majority: list[float] = []
    cross_by_pair: Counter[str] = Counter()

    # crude cross-frame association
    open_tracks: list[dict] = []
    flips = 0
    flip_detail: Counter[str] = Counter()
    tracks_closed = 0
    associated = 0
    new_tracks = 0

    for bag_dir in args.bag_dirs:
        reader = open_reader(Path(bag_dir))
        t0 = None
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != "/vision/detections":
                continue
            if t0 is None:
                t0 = stamp
            t = elapsed_seconds(stamp, t0)
            frames += 1
            dets = [d for d in json.loads(deserialize_message(data, String).data) if str(d["class_name"]) in _PILLAR_CLASSES]
            if not dets:
                continue
            frames_with_pillars += 1

            # 1. within-frame contradiction
            saw_cross_here = False
            for i in range(len(dets)):
                for j in range(i + 1, len(dets)):
                    a, b = dets[i], dets[j]
                    ov = _boxes_overlap(a["bbox"], b["bbox"])
                    if ov < args.overlap:
                        continue
                    if str(a["class_name"]) == str(b["class_name"]):
                        same_pairs += 1
                        continue
                    cross_pairs += 1
                    saw_cross_here = True
                    cross_overlaps.append(ov)
                    lo, hi = sorted((a, b), key=lambda d: float(d["confidence"]))
                    conf_minority.append(float(lo["confidence"]))
                    conf_majority.append(float(hi["confidence"]))
                    cross_by_pair["+".join(sorted((str(a["class_name"]), str(b["class_name"]))))] += 1
            if saw_cross_here:
                frames_with_cross += 1

            # 2. cross-frame class flip
            for d in dets:
                x = float(d["x"])
                cls = str(d["class_name"])
                best = None
                for tr in open_tracks:
                    if t - tr["t"] > args.assoc_gap:
                        continue
                    dx = abs(tr["x"] - x)
                    if dx <= args.assoc_x and (best is None or dx < best[0]):
                        best = (dx, tr)
                if best is None:
                    new_tracks += 1
                    open_tracks.append({"x": x, "t": t, "cls": cls, "n": 1, "counts": Counter([cls])})
                    continue
                associated += 1
                tr = best[1]
                if tr["cls"] != cls:
                    flips += 1
                    flip_detail[f"{tr['cls']}->{cls}"] += 1
                tr.update(x=x, t=t, cls=cls, n=tr["n"] + 1)
                tr["counts"][cls] += 1
            keep = [tr for tr in open_tracks if t - tr["t"] <= args.assoc_gap]
            tracks_closed += len(open_tracks) - len(keep)
            open_tracks = keep

    if not frames:
        print("no /vision/detections frames -- wrong bag, or the topic was not recorded")
        sys.exit(1)

    print(f"frames {frames}, with red/green pillars {frames_with_pillars}")
    print()
    print("== 1. WITHIN A FRAME: two classes on ONE object (no association assumed)")
    print(f"  cross-colour overlapping pairs : {cross_pairs}")
    print(f"  frames containing one          : {frames_with_cross}"
          f"  ({frames_with_cross / max(1, frames_with_pillars):.1%} of pillar frames)")
    print(f"  CONTROL same-colour pairs      : {same_pairs}"
          "   <- if this is ~0 too, the frames just have no overlaps and the line above means nothing")
    if cross_overlaps:
        print(f"  overlap (int/smaller) p50      : {statistics.median(cross_overlaps):.2f}")
        print(f"  confidence, lower of the pair  : p50 {statistics.median(conf_minority):.2f}")
        print(f"  confidence, higher of the pair : p50 {statistics.median(conf_majority):.2f}")
        for pair, n in cross_by_pair.most_common():
            print(f"    {pair}: {n}")
    print()
    print("== 2. ACROSS FRAMES: a stable image position whose class CHANGES")
    print("   (crude association; a false one INFLATES flips, so a low number is the trustworthy direction)")
    print(f"  CONTROL associations made : {associated}"
          f"   (vs {new_tracks} detections that opened a NEW track)"
          "\n     <- if associations are ~0 the flip count below is 0 BY CONSTRUCTION and says nothing")
    print(f"  class flips : {flips}")
    for k, n in flip_detail.most_common():
        print(f"    {k}: {n}")


if __name__ == "__main__":
    main()
