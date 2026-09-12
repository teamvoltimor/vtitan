"""Why GREEN detections collapsed between two hardware rounds ninety minutes apart.

The operator watched the last round pass the first three RED pillars correctly
on every lap and almost never pass a GREEN one correctly, and counting
``/vision/detections`` found the green share had fallen from 55.7% to 33.4%
between 21:18 and 22:56. That topic is camera + Hailo, upstream of the sign map
and the router, so nothing downstream can explain it.

This script separates the four candidate causes that the raw counts cannot:

* COUNT vs CONFIDENCE -- the detector filters below ``detector.toml``'s
  ``min_confidence`` BEFORE publishing, so a green that fell under the floor is
  INVISIBLE on this topic. What IS observable is whether the surviving greens
  piled up against the floor (censoring) or kept a healthy distribution
  (genuinely fewer pillars in view).
* RANGE -- range is derived from box HEIGHT through the same pinhole
  ``sign_discovery._detection_to_world`` uses, never from width, which carries
  motion smear. A lens refocus moves the in-focus band, so a loss concentrated
  in one range band points at optics while a flat loss points at lighting.
* MISCLASSIFICATION -- if greens are read as RED or MAGENTA the total holds
  while the share moves. Per-run class totals and per-frame co-occurrence say
  which.
* EXPOSURE -- detections per second normalises out round length, which raw
  counts do not.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_green_loss.py data/live/runs/run_A data/live/runs/run_B ...
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

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import open_reader
from scripts.common.stats import percentile
from scripts.common.tables import print_table

_VISION_TOPIC = "/vision/detections"

_SIGN_HEIGHT_M = 0.10
"""Real pillar height, the numerator of the pinhole range.

Matches ``TrafficSignSpecs.HEIGHT``; imported lazily below so the script still
runs if the specs module moves."""

_RANGE_BANDS = ((0.0, 0.3), (0.3, 0.5), (0.5, 0.8), (0.8, 99.0))
"""Range bands chosen around the DECISION window, not around the median box.

``camera/config.toml`` records first-usable-detection at 0.824 m and router
commitment at 0.469 m, so 0.5 and 0.8 are the two edges that matter; 0.3 splits
the already-passing frames off the bottom."""

_CAPTURE_WIDTHS = (640.0, 1280.0, 1536.0, 1920.0)
"""Known capture widths a run's boxes are snapped to.

DERIVED PER RUN from the widest box column seen, never read from config:
``camera/config.toml`` carries both a 640 MODEL INPUT size and a 1536 CAPTURE
size, and picking the wrong one rescales every pinhole range by 2.4x."""


def _frame_width(max_x: float) -> float:
    """The capture width whose frame contains every observed box column."""
    for w in _CAPTURE_WIDTHS:
        if max_x <= w + 1.0:
            return w
    return max_x


@dataclass
class ClassStats:
    """Every per-detection quantity this script reports, for one colour."""

    conf: list[float] = field(default_factory=list)
    height: list[float] = field(default_factory=list)
    aspect: list[float] = field(default_factory=list)
    ranges: list[float] = field(default_factory=list)


def _read_frames(bag_dir: Path) -> tuple[list[tuple[float, list[dict]]], float]:
    """Every ``/vision/detections`` frame with its elapsed seconds, plus duration.

    Reads ONLY the vision topic: the scans dominate a hardware bag and pulling
    them makes a 16-bag sweep take an hour. Duration is the full bag span
    (first to last message of ANY topic), because detections-per-second must be
    normalised by how long the round ran, not by how long the camera happened
    to publish.
    """
    reader = open_reader(bag_dir)
    t0: int | None = None
    t_last = 0
    frames: list[tuple[float, list[dict]]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        t_last = t
        if topic != _VISION_TOPIC:
            continue
        payload = json.loads(deserialize_message(data, String).data) or []
        frames.append(((t - t0) / 1e9, payload))
    return frames, (t_last - (t0 or t_last)) / 1e9


def _colour(det: dict) -> str | None:
    """The detection's class, from either publisher generation's key."""
    raw = det.get("class_name", det.get("class"))
    return str(raw).lower() if raw is not None else None


def _analyse(bag_dir: Path) -> dict:
    """One run reduced to the per-class quantities the report compares."""
    frames, duration = _read_frames(bag_dir)

    max_x = 0.0
    for _, payload in frames:
        for det in payload:
            bbox = det.get("bbox") or ()
            if len(bbox) == 4:
                max_x = max(max_x, float(bbox[2]))
    frame_w = _frame_width(max_x) if max_x else 1536.0
    focal_px = (frame_w / 2) / math.tan(_hfov() / 2)

    per_class: dict[str, ClassStats] = defaultdict(ClassStats)
    frames_with: Counter[str] = Counter()
    cooccur = 0
    for _, payload in frames:
        seen: set[str] = set()
        for det in payload:
            cls = _colour(det)
            bbox = det.get("bbox") or ()
            if cls is None or len(bbox) != 4:
                continue
            x_min, y_min, x_max, y_max = (float(v) for v in bbox)
            h = max(y_max - y_min, 1e-6)
            w = max(x_max - x_min, 0.0)
            st = per_class[cls]
            st.conf.append(float(det.get("confidence", 0.0)))
            st.height.append(h)
            st.aspect.append(w / h)
            st.ranges.append(focal_px * _SIGN_HEIGHT_M / h)
            seen.add(cls)
        for cls in seen:
            frames_with[cls] += 1
        if {"red", "green"} <= seen:
            cooccur += 1

    tracks = _tracks(frames, focal_px)

    return {
        "name": bag_dir.name,
        "tracks": tracks,
        "duration_s": duration,
        "n_frames": len(frames),
        "frame_w": frame_w,
        "per_class": dict(per_class),
        "frames_with": frames_with,
        "cooccur": cooccur,
    }


_TRACK_GAP_S = 0.5
"""Silence that ends a detection track.

A track is one ENCOUNTER with one pillar, which is the unit the pass-side rule
scores and the unit the router publishes. Per-frame counts answer a different
question -- they are dominated by how long the robot happened to be facing the
sign, which a run full of escapes changes on its own."""


def _tracks(frames: list[tuple[float, list[dict]]], focal_px: float) -> dict[str, list[dict]]:
    """Contiguous per-colour detection runs -- one ENCOUNTER each.

    Tracks are what the router can act on: a 217-detection run and a
    306-detection run can hold the same number of encounters, and the number of
    encounters is what the pass-side verdict count has to be compared against."""
    open_t: dict[str, dict] = {}
    out: dict[str, list[dict]] = defaultdict(list)
    for t, payload in frames:
        seen: dict[str, list[tuple[float, float]]] = defaultdict(list)
        for det in payload:
            cls = _colour(det)
            bbox = det.get("bbox") or ()
            if cls is None or len(bbox) != 4:
                continue
            h = max(float(bbox[3]) - float(bbox[1]), 1e-6)
            seen[cls].append((float(det.get("confidence", 0.0)), focal_px * _SIGN_HEIGHT_M / h))
        for cls, obs in seen.items():
            cur = open_t.get(cls)
            if cur is None or t - cur["t_end"] > _TRACK_GAP_S:
                if cur is not None:
                    out[cls].append(cur)
                cur = {"t_start": t, "n": 0, "conf": [], "ranges": []}
                open_t[cls] = cur
            cur["t_end"] = t
            cur["n"] += len(obs)
            cur["conf"].extend(c for c, _ in obs)
            cur["ranges"].extend(d for _, d in obs)
    for cls, cur in open_t.items():
        out[cls].append(cur)
    return dict(out)


def _report_tracks(runs: list[dict]) -> None:
    """Encounters, not frames -- the unit the router publishes a sign for."""
    print("\n=== ENCOUNTERS (tracks, 0.5 s gap). 'usable' = >=3 frames AND ever inside 0.8 m ===")
    rows = []
    for r in runs:
        row = [r["name"].replace("run_20260911_", "")]
        for cls in ("red", "green"):
            tr = r["tracks"].get(cls, [])
            usable = [t for t in tr if t["n"] >= 3 and t["ranges"] and min(t["ranges"]) < 0.8]
            dur = [t["t_end"] - t["t_start"] for t in usable]
            row.append(len(tr))
            row.append(len(usable))
            row.append(f"{statistics.median(dur):.2f}" if dur else "-")
        rows.append(row)
    print_table(rows, headers=["run", "red trk", "red usable", "red s", "grn trk", "grn usable", "grn s"])


def _hfov() -> float:
    """Camera horizontal FOV in radians, from the robot specs."""
    from shared.config.constants import RobotSpecs  # noqa: PLC0415

    return float(RobotSpecs.CAMERA_HFOV)


def _q(values: list[float], p: float) -> str:
    return f"{percentile(values, p):.2f}" if values else "-"


def _report_census(runs: list[dict]) -> None:
    """Counts, share and exposure-normalised rate per run."""
    print("\n=== CENSUS: counts, share, detections/second ===")
    rows = []
    for r in runs:
        pc = r["per_class"]
        n = {c: len(s.conf) for c, s in pc.items()}
        total = sum(n.values())
        rg = n.get("red", 0) + n.get("green", 0)
        rows.append(
            [
                r["name"].replace("run_20260911_", ""),
                f"{r['duration_s']:.0f}",
                total,
                n.get("red", 0),
                n.get("green", 0),
                n.get("magenta", 0),
                f"{100 * n.get('green', 0) / rg:.1f}%" if rg else "-",
                f"{n.get('red', 0) / r['duration_s']:.2f}" if r["duration_s"] else "-",
                f"{n.get('green', 0) / r['duration_s']:.2f}" if r["duration_s"] else "-",
                r["frames_with"].get("red", 0),
                r["frames_with"].get("green", 0),
                r["cooccur"],
            ]
        )
    print_table(
        rows,
        headers=["run", "dur", "all", "red", "green", "magenta", "grn%", "red/s", "grn/s", "fR", "fG", "both"],
    )


def _report_confidence(runs: list[dict]) -> None:
    """Confidence distribution per colour.

    A green LOST to a threshold never reaches this topic, so the signature to
    look for is a surviving distribution pressed against the floor -- p10 and
    p50 falling together toward ``min_confidence`` -- not a missing tail."""
    print("\n=== CONFIDENCE by colour (p10/p50/p90, and share under 0.5/0.6) ===")
    rows = []
    for r in runs:
        for cls in ("red", "green"):
            st = r["per_class"].get(cls)
            if not st or not st.conf:
                rows.append([r["name"].replace("run_20260911_", ""), cls, 0, "-", "-", "-", "-", "-"])
                continue
            c = st.conf
            rows.append(
                [
                    r["name"].replace("run_20260911_", ""),
                    cls,
                    len(c),
                    _q(c, 0.1),
                    _q(c, 0.5),
                    _q(c, 0.9),
                    f"{100 * sum(1 for v in c if v < 0.5) / len(c):.0f}%",
                    f"{100 * sum(1 for v in c if v < 0.6) / len(c):.0f}%",
                ]
            )
    print_table(rows, headers=["run", "cls", "n", "p10", "p50", "p90", "<0.5", "<0.6"])


def _report_range(runs: list[dict]) -> None:
    """Detection count per range band, from box HEIGHT through the pinhole."""
    print("\n=== RANGE bands (pinhole from box HEIGHT; counts, and green share of red+green) ===")
    rows = []
    for r in runs:
        red = r["per_class"].get("red", ClassStats()).ranges
        grn = r["per_class"].get("green", ClassStats()).ranges
        row = [r["name"].replace("run_20260911_", "")]
        for lo, hi in _RANGE_BANDS:
            nr = sum(1 for v in red if lo <= v < hi)
            ng = sum(1 for v in grn if lo <= v < hi)
            row.append(f"{nr}/{ng}" + (f" {100 * ng / (nr + ng):.0f}%" if nr + ng else ""))
        row.append(_q(grn, 0.5))
        row.append(_q(red, 0.5))
        rows.append(row)
    print_table(
        rows,
        headers=["run", *[f"{lo:.1f}-{hi:.1f} R/G" for lo, hi in _RANGE_BANDS], "grn p50 m", "red p50 m"],
    )


def _report_confidence_by_band(runs: list[dict]) -> None:
    """Median confidence inside each range band, per colour.

    Stratifying by range is what separates a LENS change (which moves the
    in-focus band, so confidence should move in SOME bands and not others) from
    a LIGHTING change (which moves every band together)."""
    print("\n=== CONFIDENCE p50 within each range band (red | green) ===")
    rows = []
    for r in runs:
        row = [r["name"].replace("run_20260911_", "")]
        for lo, hi in _RANGE_BANDS:
            cell = []
            for cls in ("red", "green"):
                st = r["per_class"].get(cls, ClassStats())
                vals = [c for c, d in zip(st.conf, st.ranges, strict=False) if lo <= d < hi]
                cell.append(f"{statistics.median(vals):.3f}" if len(vals) >= 10 else "-")
            row.append(" | ".join(cell))
        rows.append(row)
    print_table(rows, headers=["run", *[f"{lo:.1f}-{hi:.1f}" for lo, hi in _RANGE_BANDS]])


def _report_shape(runs: list[dict]) -> None:
    """Box aspect (w/h) per colour -- a wall read as a pillar is WIDE."""
    print("\n=== BOX SHAPE, aspect w/h p50 and share wider than tall ===")
    rows = []
    for r in runs:
        row = [r["name"].replace("run_20260911_", "")]
        for cls in ("red", "green", "magenta"):
            st = r["per_class"].get(cls, ClassStats())
            if not st.aspect:
                row.append("-")
                continue
            wide = 100 * sum(1 for a in st.aspect if a > 1.0) / len(st.aspect)
            row.append(f"{statistics.median(st.aspect):.2f} ({wide:.0f}% wide)")
        rows.append(row)
    print_table(rows, headers=["run", "red", "green", "magenta"])


def main() -> None:
    bag_dirs = [Path(a) for a in sys.argv[1:]]
    if not bag_dirs:
        print("usage: diag_bag_green_loss.py <bag_dir> [<bag_dir> ...]")
        raise SystemExit(2)

    runs = []
    for d in sorted(bag_dirs):
        if not d.exists():
            print(f"MISSING {d}")
            continue
        try:
            runs.append(_analyse(d))
        except Exception as exc:  # noqa: BLE001 - one bad bag must not kill the sweep
            print(f"FAILED {d.name}: {type(exc).__name__}: {exc}")
    if not runs:
        print("no runs analysed")
        raise SystemExit(1)

    print(f"frame widths derived per run: {sorted({r['frame_w'] for r in runs})}")
    _report_census(runs)
    _report_tracks(runs)
    _report_confidence(runs)
    _report_range(runs)
    _report_confidence_by_band(runs)
    _report_shape(runs)


if __name__ == "__main__":
    main()
