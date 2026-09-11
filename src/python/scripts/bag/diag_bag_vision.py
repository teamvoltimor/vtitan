"""What the camera actually reported, and whether the robot could act on it.

Written for four hardware observations from the 2026-09-05 runs that the sim
cannot reproduce, because the simulated sign detector is exact and never blurs,
mislabels, or arrives late:

* red pillars COLLIDED with rather than avoided;
* green pillars passed on their RIGHT, which rule 9.24 scores as a round-ender;
* magenta WALL segments classified as red pillars in corners;
* the camera visibly out of focus while the chassis is moving.

Those are three different failures and one candidate cause, so the report is
built to separate them rather than to score a run:

* BLUR -- if detection quality is speed-dependent, confidence falls as
  commanded speed rises, and the fix is exposure or a settle-gate, not the
  model.
* CONFUSION -- if magenta walls read as red pillars, red detections cluster
  where a wall fills the frame, at large box area and a WIDE aspect ratio,
  while a genuine pillar is tall and narrow.
* LATENCY -- if the robot sees the pillar but too late to steer around it, the
  first confident frame of a track leaves too little time before the pillar
  leaves view.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_vision.py data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import create_bag_parser, decode_nav_debug, elapsed_seconds, open_reader
from scripts.common.stats import percentile
from scripts.common.tables import print_table

_TRACK_GAP_S = 0.5
"""Silence that ends a detection track.

A track is one ENCOUNTER with one pillar, which is the unit the pass-side rule
scores. Per-frame counts answer a different question and are dominated by how
long the robot happened to be facing the sign."""

_FALLBACK_FRAME_W = 1536.0
"""Capture width assumed only when a run yields no boxes to measure one from.

DERIVED PER RUN, never taken from config: ``camera/config.toml`` carries
``width = 640``, which is the MODEL INPUT size, while ``/vision/detections``
reports boxes in CAPTURE coordinates -- 1536x864 on the 2026-09-05 runs. Using
the config number puts the image centre at 320 instead of 768, and since every
pass-side verdict here is the SIGN of ``x - centre``, that silently inverts the
verdict for any pillar between those two columns. It did: red read 1/2 wrong
against a 640 frame and 2/2 correct against the real one."""

_PILLAR_MAX_ASPECT = 1.0
"""Width/height above which a box is not pillar-shaped.

Calibrated against the run's own GREEN detections, which are the class with no
magenta wall to be confused with: their w/h p90 is 0.85 and only 0.6% exceed
1.0. A pillar is taller than it is wide by construction, so a box wider than
tall is either two pillars merged or a stretch of wall."""

_MIN_TRACK_FRAMES = 3
"""Frames a track needs before its exit side is read.

A one- or two-frame track has no exit: the pillar was seen once and lost, so
the last sample is where it was DETECTED, not the side it was passed on."""

_CONF_FLOOR = 0.5
"""Confidence a detection needs before it is treated as actionable here.

Reporting-only: this script does not know the node's own threshold, so this is
a fixed reference line for comparing runs, not a claim about what the navigator
acted on."""


@dataclass
class Track:
    """One contiguous run of detections of a single class."""

    cls: str
    t_start: float
    t_end: float
    n: int = 0
    conf: list[float] = field(default_factory=list)
    area: list[float] = field(default_factory=list)
    aspect: list[float] = field(default_factory=list)
    x_norm: list[float] = field(default_factory=list)
    speeds: list[float] = field(default_factory=list)
    sections: list[str] = field(default_factory=list)



def _report_census(by_cls: dict[str, list]) -> None:
    """Per-class detection counts, confidence and box shape."""
    print("detections by class:")
    print_table(
        [
            [
                cls,
                len(group),
                f"{statistics.median([g[2] for g in group]):.2f}",
                f"{percentile([g[2] for g in group], 0.1):.2f}",
                f"{statistics.median([g[3] for g in group]):.0f}",
                f"{statistics.median([g[4] for g in group]):.2f}",
            ]
            for cls, group in sorted(by_cls.items(), key=lambda kv: -len(kv[1]))
        ],
        ["class", "n", "conf p50", "conf p10", "area p50", "w/h p50"],
    )


def _build_tracks(per_frame: list[tuple], centre_x: float) -> list[Track]:
    """Group per-frame detections into one track per pillar ENCOUNTER."""
    tracks: list[Track] = []
    last_by_cls: dict[str, Track] = {}
    for t, cls, conf, area, aspect, x, speed, section in per_frame:
        current = last_by_cls.get(cls)
        if current is None or t - current.t_end > _TRACK_GAP_S:
            current = Track(cls=cls, t_start=t, t_end=t)
            tracks.append(current)
            last_by_cls[cls] = current
        current.t_end = t
        current.n += 1
        current.conf.append(conf)
        current.area.append(area)
        current.aspect.append(aspect)
        current.x_norm.append((x - centre_x) / centre_x)
        current.speeds.append(speed)
        current.sections.append(section)
    return tracks


def _report_tracks(tracks: list[Track]) -> None:
    """One row per class: how many encounters, and how long each lasted."""
    print(f"\ntracks (one encounter = one pillar, split on a gap > {_TRACK_GAP_S}s):")
    print_table(
        [
            [
                cls,
                len(group),
                f"{statistics.median([tr.t_end - tr.t_start for tr in group]):.2f}s",
                f"{statistics.median([tr.n for tr in group]):.0f}",
                f"{statistics.median([max(tr.area) for tr in group]):.0f}",
                f"{statistics.median([statistics.median(tr.x_norm) for tr in group]):+.2f}",
            ]
            for cls, group in (
                (cls, [tr for tr in tracks if tr.cls == cls]) for cls in sorted({tr.cls for tr in tracks})
            )
        ],
        ["class", "tracks", "dur p50", "frames p50", "peak area p50", "x_norm p50"],
    )


def _report_blur(per_frame: list[tuple]) -> None:
    """Detection confidence against the speed commanded at that frame."""
    print("\nBLUR -- detection quality by commanded speed at the frame:")
    speed_bins = [
        (0.0, 0.01, "stopped"),
        (0.01, 0.20, "0.01-0.20"),
        (0.20, 0.35, "0.20-0.35"),
        (0.35, 0.45, "0.35-0.45"),
        (0.45, 9.9, ">=0.45"),
    ]
    rows = []
    for low, high, label in speed_bins:
        group = [r for r in per_frame if low <= abs(r[6]) < high]
        if not group:
            rows.append([label, 0, "-", "-", "-"])
            continue
        confs = [g[2] for g in group]
        rows.append([
            label,
            len(group),
            f"{statistics.median(confs):.2f}",
            f"{percentile(confs, 0.1):.2f}",
            f"{sum(1 for c in confs if c < _CONF_FLOOR) / len(confs):.1%}",
        ])
    print_table(rows, ["speed m/s", "detections", "conf p50", "conf p10", f"share conf<{_CONF_FLOOR}"])


def _report_sections(per_frame: list[tuple]) -> None:
    """Where each class is seen, by corridor section."""
    print("\nCONFUSION -- detections by corridor section:")
    counts: Counter[tuple[str, str]] = Counter()
    for row in per_frame:
        counts[(row[1], row[7])] += 1
    sections = sorted({key[1] for key in counts})
    print_table(
        [
            [cls, sum(v for k, v in counts.items() if k[0] == cls)]
            + [
                f"{counts.get((cls, s), 0) / sum(v for k, v in counts.items() if k[0] == cls):.0%}"
                for s in sections
            ]
            for cls in sorted({key[0] for key in counts})
        ],
        ["class", "n", *sections],
    )


def _report_box_shape(by_cls: dict[str, list]) -> None:
    """Aspect-ratio split: a pillar is tall and narrow, a wall segment is wide."""
    print("\nCONFUSION -- box shape. A PILLAR is tall and narrow; a WALL segment is wide:")
    print_table(
        [
            [
                cls,
                len(group),
                f"{percentile([g[4] for g in group], 0.5):.2f}",
                f"{percentile([g[4] for g in group], 0.9):.2f}",
                f"{sum(1 for g in group if g[4] > 1.0) / len(group):.1%}",
            ]
            for cls, group in sorted(by_cls.items())
        ],
        ["class", "n", "w/h p50", "w/h p90", "share WIDER than tall"],
    )


def _report_shape_split(by_cls: dict[str, list], centre_x: float) -> None:
    """Each class partitioned into pillar- and wall-shaped boxes."""
    print("\nSHAPE SPLIT -- red detections partitioned by the aspect ratio green never shows:")
    print(
        f"   (green's w/h p90 is {percentile([g[4] for g in by_cls.get('green', [])], 0.9):.2f}, so"
        f" anything above {_PILLAR_MAX_ASPECT} is a shape no genuine pillar produced in this run)"
    )
    rows = []
    for cls in sorted(by_cls):
        for label, keep in (("pillar-shaped", True), ("wall-shaped", False)):
            group = [g for g in by_cls[cls] if (g[4] <= _PILLAR_MAX_ASPECT) is keep]
            if not group:
                continue
            rows.append([
                cls,
                label,
                len(group),
                f"{statistics.median([g[3] for g in group]):.0f}",
                f"{statistics.median([(g[5] - centre_x) / centre_x for g in group]):+.2f}",
                ", ".join(
                    f"{s} {n / len(group):.0%}"
                    for s, n in Counter(g[7] for g in group).most_common(2)
                ),
            ])
    print_table(rows, ["class", "shape", "n", "area p50", "x_norm p50", "top sections"])

    # PASS SIDE. Which side the chassis actually went past on, read at the END
    # of a track: a pillar leaves the frame on the side the robot passed it.
    # Travel-relative rule 9.19 -- the car passes to the RIGHT of a red pillar,
    # so a correctly-passed red exits on the LEFT of the image (x_norm < 0),
    # and a correctly-passed green exits on the RIGHT (x_norm > 0).


def _report_pass_side(tracks: list[Track]) -> None:
    """Which image side each pillar left the frame on, against rule 9.19."""
    print("\nPASS SIDE -- image side each pillar EXITED on (pillar-shaped tracks only):")
    print("   rule 9.19 travel-relative: red must exit LEFT (x_norm<0), green must exit RIGHT (x_norm>0)")
    want_positive_exit = {"green": True, "red": False}
    rows = []
    for cls in ("red", "green"):
        group = [
            tr
            for tr in tracks
            if tr.cls == cls and statistics.median(tr.aspect) <= _PILLAR_MAX_ASPECT and tr.n >= _MIN_TRACK_FRAMES
        ]
        if not group:
            rows.append([cls, 0, "-", "-", "-"])
            continue
        exits = [statistics.median(tr.x_norm[-3:]) for tr in group]
        correct = sum(1 for e in exits if (e > 0) is want_positive_exit[cls])
        rows.append([
            cls,
            len(group),
            "RIGHT (x>0)" if want_positive_exit[cls] else "LEFT (x<0)",
            f"{statistics.median(exits):+.2f}",
            f"{correct}/{len(group)}",
        ])
    print_table(rows, ["class", "tracks", "required exit", "actual exit p50", "CORRECT"])


def _report_latency(tracks: list[Track]) -> None:
    """How long a track stayed confident before it ended."""
    print("\nLATENCY -- from the first confident frame of a track to the end of the track:")
    rows = []
    for cls in sorted({tr.cls for tr in tracks}):
        group = [tr for tr in tracks if tr.cls == cls]
        windows = []
        for tr in group:
            first_ok = next((i for i, c in enumerate(tr.conf) if c >= _CONF_FLOOR), None)
            if first_ok is None:
                continue
            span = tr.t_end - tr.t_start
            windows.append(span - first_ok * span / max(1, tr.n - 1))
        if not windows:
            rows.append([cls, len(group), f"never conf>={_CONF_FLOOR}", "-"])
            continue
        rows.append([
            cls,
            len(group),
            f"{statistics.median(windows):.2f}s",
            f"{percentile(windows, 0.1):.2f}s",
        ])
    print_table(rows, ["class", "tracks", "confident window p50", "p10"])



def _collect(reader: object) -> tuple[list[tuple], list[float], int, int, int]:
    """Read the bag once, carrying speed and section forward onto each detection.

    Detections are the only topic sampled; ``/ackermann_cmd`` and ``/nav_debug``
    are held as the most recent value so every detection row knows the speed the
    chassis was commanded at and the corridor it was in when the frame arrived.
    """
    t0: float | None = None
    speed_now = 0.0
    section_now = "?"
    laps_now = 0

    per_frame: list[tuple[float, str, float, float, float, float, float, str]] = []
    box_right: list[float] = []
    frames_with_any = 0
    frames_total = 0

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if t0 is None:
            t0 = ts
        t = elapsed_seconds(ts, t0)

        if topic == "/ackermann_cmd":
            speed_now = float(deserialize_message(data, AckermannDriveStamped).drive.speed)
            continue

        if topic == "/nav_debug":
            snap = decode_nav_debug(data)
            if snap is not None:
                section_now = str(getattr(snap.current_corridor, "value", snap.current_corridor))
                laps_now = int(snap.laps_completed)
            continue

        if topic != "/vision/detections":
            continue

        frames_total += 1
        dets = json.loads(deserialize_message(data, String).data)
        if dets:
            frames_with_any += 1
        for det in dets:
            width = float(det["width"])
            height = float(det["height"])
            box_right.append(float(det["bbox"][2]))
            per_frame.append((
                t,
                str(det["class_name"]),
                float(det["confidence"]),
                float(det["area"]),
                (width / height) if height else float("nan"),
                float(det["x"]),
                speed_now,
                section_now,
            ))

    return per_frame, box_right, frames_total, frames_with_any, laps_now


def main() -> None:
    """Print the census, then the blur / confusion / latency splits."""
    parser = create_bag_parser("Vision detection census for a recorded run.")
    args = parser.parse_args()
    per_frame, box_right, frames_total, frames_with_any, laps_now = _collect(open_reader(args.bag_dir))

    if not frames_total:
        print("no /vision/detections in this bag")
        return

    # Widest box edge the run ever produced IS the frame edge, because boxes
    # clip there: 22 of 800 detections in run_20260905_214920 land exactly on
    # 1536.0. Rounded up to a multiple of 32 so a run that never quite touches
    # the edge still lands on the real width rather than a few pixels short.
    frame_w = 32.0 * math.ceil(max(box_right) / 32.0) if box_right else _FALLBACK_FRAME_W
    centre_x = frame_w / 2.0
    print(f"\nframe width derived from box extents: {frame_w:.0f}px (centre {centre_x:.0f})")

    print(
        f"\nvision frames: {frames_total}   with >=1 detection: {frames_with_any} "
        f"({frames_with_any / frames_total:.1%})"
    )
    print(f"laps completed: {laps_now}\n")

    by_cls: dict[str, list] = defaultdict(list)
    for row in per_frame:
        by_cls[row[1]].append(row)

    tracks = _build_tracks(per_frame, centre_x)

    _report_census(by_cls)
    _report_tracks(tracks)
    _report_blur(per_frame)
    _report_sections(per_frame)
    _report_box_shape(by_cls)
    _report_shape_split(by_cls, centre_x)
    _report_pass_side(tracks)
    _report_latency(tracks)


if __name__ == "__main__":
    main()
