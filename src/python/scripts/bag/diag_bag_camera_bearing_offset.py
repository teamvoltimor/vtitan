r"""Is the camera's BEARING off by a constant, and if so by how much?

Range is not what limits a projected sign position. Cluster-gated LIDAR range
fusion ships on (``sign_discovery.lidar_range_fusion``), and feeding it in
against the raw pinhole barely moves the believed lot against how far the lot
itself wanders across rounds of ONE physical object. What is left is BEARING:
``_detection_to_world`` records a residual, and at range that is a lateral miss
of the observed size.

That matters because of WHICH fixes it admits. The camera bearing is

    theta_h = (0.5 - cx / W) * HFOV

so exactly two one-constant errors are possible:

* a **mounting yaw** error, which adds a constant offset ``delta``, and
* an **HFOV** error, which scales the whole fan by ``s``.

Either is a single number. Anything left over is lens distortion or noise and is
a far bigger job, so this separates them before anyone tunes.

THE ASSOCIATION PROBLEM, and why this does not gate on bearing. Asking "which
pillar is this box?" by picking the nearest one IN BEARING assumes the answer:
a symmetric window around a biased estimate pulls the measured bias toward
zero. So association is by RANGE instead, which is independent of the quantity
being measured, and only detections with EXACTLY ONE pillar in view within
``--range-tol`` of their pinhole range are counted. Ambiguous ones never vote.

Ground truth is the LIDAR rather than the sign map, because the sign map is
built from these same bearings and would agree with any error they carry. Every
return more than ``--wall-margin`` from a known wall is accumulated in world
coordinates over the whole bag; a pillar is a physical object and piles up,
while an occlusion edge smears.

TWO CONTROLS, because a crashed or mis-wired diagnostic exits 0.

* The pillar map is printed FIRST. If it is empty or its clusters sit on walls,
  everything below it is meaningless.
* Every run is repeated with each detection paired to a DIFFERENT tick's pose.
  That destroys the correspondence while leaving every distribution intact, so
  the shuffled row is what this estimator reports on noise. Read the measured
  row only against it.

WHAT IT REPORTS. Read the slope FIRST: it behaves as association quality,
tending to 0 when association is perfect and to 1 when it is random, so it says
whether a row is worth reading at all. The measured residual, its IQR and the
slope, measured against the shuffled control, are recorded in
``adr:0058-sign-discovery-range-and-barrier-belief``.

THE CONSEQUENCE, and it is the point of the script. The residual in
``_detection_to_world`` is not an offset. It is zero-mean scatter, so **neither
one-constant fix exists**: no mounting-yaw constant and no HFOV scale can
remove a zero-mean spread. Anything that wants a better sign position has to
average sightings (tracking) or carry the uncertainty into the planner.

CAVEAT: the residual also absorbs pose error and any surviving mis-association,
so the measured scatter is an UPPER bound on the camera's own bearing scatter,
not a clean measurement of it.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_camera_bearing_offset.py \
        RUN_DIR [RUN_DIR ...]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
# `isort: off` is what holds that order; without it `ruff check --fix` re-sorts
# these back into the cycle and the script dies on import.
# isort: off
from scripts.common.bag_io import (
    create_bags_parser,
    decode_detections,
    posed_rows,
    read_vision_rows_and_scans,
    scan_to_ranges_angles,
)
from shared.config.constants import RobotSpecs
from shared.domain.models import SignColor

from scripts.common.tables import print_table

# isort: on

# The mat is 0..3 m with a 1x1 inner block at (1,1)-(2,2) (`track.toml`:
# min_coord 0, max_coord 3, corner_min 1.0, corner_max 2.0).
_TRACK_MIN, _TRACK_MAX = 0.0, 3.0
_INNER_MIN, _INNER_MAX = 1.0, 2.0
_FOCAL_PX = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)
# Below this the return is the chassis itself, not the track.
_SELF_RETURN_M = 0.15
# A pillar is ~5 cm across but smears over several cells, so each peak claims
# this much before the next is taken.
_PEAK_CLAIM_M = 0.20
# Fewer points than this and a fitted slope is noise, so it is not reported.
_MIN_FOR_SLOPE = 20


def _off_wall(px: np.ndarray, py: np.ndarray, margin: float) -> np.ndarray:
    """True for points clear of every wall face, outer ring and inner block alike."""
    near_outer = (
        (np.abs(px - _TRACK_MIN) < margin)
        | (np.abs(px - _TRACK_MAX) < margin)
        | (np.abs(py - _TRACK_MIN) < margin)
        | (np.abs(py - _TRACK_MAX) < margin)
    )
    # Distance to the inner block's boundary: outside it, the nearest face;
    # inside it, negative. Chebyshev distance to the square is enough here.
    dx = np.maximum(_INNER_MIN - px, px - _INNER_MAX)
    dy = np.maximum(_INNER_MIN - py, py - _INNER_MAX)
    outside = np.maximum(dx, dy)
    near_inner = np.abs(outside) < margin
    on_mat = (
        (px > _TRACK_MIN - margin)
        & (px < _TRACK_MAX + margin)
        & (py > _TRACK_MIN - margin)
        & (py < _TRACK_MAX + margin)
    )
    return on_mat & ~near_outer & ~near_inner


def _pose_at(times: np.ndarray, poses: np.ndarray, t: float) -> np.ndarray | None:
    """The pose sampled nearest ``t``, or None when the bag carries none."""
    if times.size == 0:
        return None
    return poses[int(np.argmin(np.abs(times - t)))]


def _pillar_map(
    scans: list[tuple[float, object]],
    times: np.ndarray,
    poses: np.ndarray,
    *,
    wall_margin: float,
    cell_m: float,
    min_returns: int,
    max_pillars: int,
) -> list[tuple[float, float, int]]:
    """LIDAR-confirmed off-wall clusters, strongest first, as ``(x, y, returns)``."""
    counts: dict[tuple[int, int], int] = {}
    for t, raw in scans:
        pose = _pose_at(times, poses, t)
        if pose is None:
            continue
        ranges, angles = scan_to_ranges_angles(deserialize_message(raw, LaserScan))
        keep = (ranges > _SELF_RETURN_M) & (ranges < RobotSpecs.LIDAR_MAX_RANGE * 0.99)
        ranges, angles = ranges[keep], angles[keep]
        if ranges.size == 0:
            continue
        world = pose[2] + angles
        px = pose[0] + ranges * np.cos(world)
        py = pose[1] + ranges * np.sin(world)
        sel = _off_wall(px, py, wall_margin)
        for gx, gy in zip(np.floor(px[sel] / cell_m).astype(int), np.floor(py[sel] / cell_m).astype(int), strict=True):
            counts[(gx, gy)] = counts.get((gx, gy), 0) + 1

    # Greedy peak picking: a pillar is ~5 cm across but its returns smear over
    # several cells, so each peak claims a 0.20 m neighbourhood before the next
    # is taken. Averaging the whole grid instead would merge two pillars a lane
    # apart into one phantom between them.
    pillars: list[tuple[float, float, int]] = []
    for (gx, gy), n in sorted(counts.items(), key=lambda kv: -kv[1]):
        if n < min_returns:
            break
        x, y = (gx + 0.5) * cell_m, (gy + 0.5) * cell_m
        if any(math.hypot(x - px0, y - py0) < _PEAK_CLAIM_M for px0, py0, _ in pillars):
            continue
        pillars.append((x, y, n))
        if len(pillars) >= max_pillars:
            break
    return pillars


def _detection_bearings(
    frames: list[tuple[float, list[dict]]],
    times: np.ndarray,
    *,
    max_range_m: float,
    range_scale: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per detection: the raw fan angle, the pinhole range, and the pose index."""
    fan: list[float] = []
    rng: list[float] = []
    idx: list[int] = []
    for t, payload in frames:
        if times.size == 0:
            continue
        i = int(np.argmin(np.abs(times - t)))
        for det in decode_detections(payload):
            if det.color not in (SignColor.RED, SignColor.GREEN):
                continue
            bbox = det.as_bbox()
            height = bbox.y_max - bbox.y_min
            if height <= 0:
                continue
            # Only close signs: a far box is a few pixels wide, so its centre
            # column carries more quantisation than the offset being measured.
            distance = _FOCAL_PX * 0.10 / height * range_scale
            if distance > max_range_m:
                continue
            fan.append((0.5 - bbox.center.x / RobotSpecs.CAMERA_WIDTH) * RobotSpecs.CAMERA_HFOV)
            rng.append(distance)
            idx.append(i)
    return np.asarray(fan), np.asarray(rng), np.asarray(idx, dtype=int)


def _residuals(
    fan: np.ndarray,
    rng: np.ndarray,
    idx: np.ndarray,
    poses: np.ndarray,
    pillars: list[tuple[float, float, int]],
    *,
    range_tol_m: float,
    max_range_m: float,
    shuffle: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Bearing residuals for detections a pillar can be matched to UNAMBIGUOUSLY.

    Association is by RANGE, never by bearing, which is what keeps the estimate
    honest: gating on bearing would pull the measured offset toward zero by
    construction. A detection is kept only when exactly ONE pillar in view sits
    within ``range_tol_m`` of its pinhole range, so no ambiguous case votes.

    ``shuffle`` pairs each detection with a DIFFERENT tick's pose. That destroys
    the true correspondence while leaving every distribution intact, so whatever
    it reports is this estimator's chance rate rather than a measurement.

    Returns:
        ``(residual_rad, fan_rad)`` for the matched detections.
    """
    if fan.size == 0 or not pillars:
        return np.zeros(0), np.zeros(0)
    px = np.array([p[0] for p in pillars])
    py = np.array([p[1] for p in pillars])
    use = np.random.default_rng(0).permutation(idx) if shuffle else idx
    rx, ry, ryaw = poses[use, 0], poses[use, 1], poses[use, 2]

    dx = px[None, :] - rx[:, None]
    dy = py[None, :] - ry[:, None]
    dist = np.hypot(dx, dy)
    true_b = np.arctan2(dy, dx) - ryaw[:, None]
    true_b = np.arctan2(np.sin(true_b), np.cos(true_b))

    in_view = (dist < max_range_m) & (np.abs(true_b) < RobotSpecs.CAMERA_HFOV / 2)
    range_ok = in_view & (np.abs(dist - rng[:, None]) < range_tol_m)
    unique = range_ok.sum(axis=1) == 1
    if not unique.any():
        return np.zeros(0), np.zeros(0)

    which = np.argmax(range_ok[unique], axis=1)
    matched_b = true_b[unique][np.arange(which.size), which]
    resid = fan[unique] - matched_b
    return np.arctan2(np.sin(resid), np.cos(resid)), fan[unique]


def _summarise(resid: np.ndarray, fan: np.ndarray, label: str) -> list:
    """One table row: how big the residual is, and whether it scales with the fan angle."""
    if resid.size == 0:
        return [label, 0, "-", "-", "-", "-"]
    deg = np.degrees(resid)
    p25, p50, p75 = np.percentile(deg, [25, 50, 75])
    # A constant offset is a MOUNTING YAW error; one proportional to the fan
    # angle is an HFOV error. The slope of residual against fan angle tells
    # them apart, and 1 + slope is the HFOV scale the data wants.
    slope = float(np.polyfit(np.degrees(fan), deg, 1)[0]) if resid.size > _MIN_FOR_SLOPE else float("nan")
    return [label, resid.size, f"{p50:+.1f}", f"{p25:+.1f}..{p75:+.1f}", f"{slope:+.3f}", f"{1.0 + slope:.3f}"]


def main() -> int:
    """Vote for the camera's bearing offset against a LIDAR-built pillar map."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--wall-margin", type=float, default=0.25, help="clearance a return needs from any wall")
    parser.add_argument("--cell", type=float, default=0.04, help="occupancy cell size (m)")
    parser.add_argument("--min-returns", type=int, default=200, help="returns a cell needs to be a pillar")
    parser.add_argument("--max-pillars", type=int, default=14)
    parser.add_argument("--max-range", type=float, default=2.0, help="ignore signs beyond this (m)")
    parser.add_argument(
        "--range-tol", type=float, default=0.35, help="how close the pinhole range must be to claim a pillar (m)"
    )
    args = parser.parse_args()

    from src.config.tuning_helpers import get_tuning  # noqa: PLC0415

    range_scale = get_tuning(None).sign_discovery.range_scale

    for bag_dir in args.bag_dirs:
        rows, frames, scans = read_vision_rows_and_scans(bag_dir)
        posed = posed_rows(rows)
        times = np.array([t for t, _ in posed])
        poses = np.array([[s.pose_x, s.pose_y, s.pose_yaw or 0.0] for _, s in posed], dtype=float)

        pillars = _pillar_map(
            scans,
            times,
            poses,
            wall_margin=args.wall_margin,
            cell_m=args.cell,
            min_returns=args.min_returns,
            max_pillars=args.max_pillars,
        )
        print(f"\n=== {bag_dir.name}: {len(posed)} posed rows, {len(scans)} scans, {len(frames)} vision frames")
        print(f"CONTROL -- LIDAR pillar map ({len(pillars)} clusters, range_scale={range_scale}):")
        if not pillars:
            print("  NONE. The vote below is meaningless; lower --min-returns or widen --wall-margin.")
        else:
            print_table([[f"{x:.2f}", f"{y:.2f}", n] for x, y, n in pillars], ["x", "y", "returns"])

        fan, rng, idx = _detection_bearings(frames, times, max_range_m=args.max_range, range_scale=range_scale)
        print(f"detections within {args.max_range} m: {fan.size}")

        rows_out = []
        for label, shuffle in (("MEASURED", False), ("shuffled (chance)", True)):
            resid, matched_fan = _residuals(
                fan,
                rng,
                idx,
                poses,
                pillars,
                range_tol_m=args.range_tol,
                max_range_m=args.max_range,
                shuffle=shuffle,
            )
            rows_out.append(_summarise(resid, matched_fan, label))
        print_table(rows_out, ["", "matched", "p50 deg", "IQR deg", "slope", "HFOV scale"])
        print()
        print(
            "A non-zero p50 with a near-zero slope is a MOUNTING YAW error (one constant). A near-zero p50 with a non-zero slope is an HFOV error (one scale). Compare both against the shuffled row: that is what this estimator reports on noise."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
