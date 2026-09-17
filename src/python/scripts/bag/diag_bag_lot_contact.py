r"""Where, and with WHAT PART of the chassis, does the car touch the parking lot?

The operator's account of the 2026-09-15 afternoon rounds says the car brushed
the parking wall while CLOSING a lap, in both directions and on the layout with
no pillar in the finish section. The replayed pass verdicts already put two
rounds' south red at 0.000-0.003 m, but "close to a believed sign" cannot say
which side of the chassis arrived there, nor whether the lot or a pillar was the
thing hit.

METHOD, deliberately pose-free for the contact itself:

* every LIDAR tick, take the closest return in each of four chassis-frame
  sectors (front, rear, left, right) with the self-return floor applied, and
  convert it to a gap against the chassis FOOTPRINT rather than the LIDAR
  origin, so a 0.10 m return abeam and a 0.10 m return ahead are comparable;
* call a tick CONTACT when any sector's footprint gap is under ``--contact-m``,
  and report the sector, the lap, the phase and the encoder's travel direction;
* attribute the contact to the LOT only through the BELIEF (pose plus the
  finish-section geometry), and print the pose's own divergence alongside, so a
  7-15 cm frame excursion cannot be mistaken for a measurement.

The lap split is the point: contacts spread evenly over a round are ordinary
close driving, contacts bunched in the last metres of each lap are the lap-close
geometry the operator described.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_lot_contact.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
from collections import Counter

import numpy as np
from shared.config.constants import RobotSpecs

from scripts.common.bag_io import (
    LIDAR_YAW_OFFSET_RAD,
    create_bags_parser,
    open_reader,
    read_bag,
    read_motion_streams,
)

HALF_L = RobotSpecs.LENGTH / 2
HALF_W = RobotSpecs.WIDTH / 2
# The LIDAR is NOT at the chassis centre: it sits 0.1222 m forward of it, so the
# footprint reaches 0.028 m ahead of the sensor and 0.272 m behind it. Measuring
# the gap from the sensor origin understates every forward gap by 12 cm and
# reports the chassis's own tail as an obstacle; the first cut of this script
# did exactly that and called 70% of a round a contact.
AHEAD = HALF_L - RobotSpecs.LIDAR_MOUNT_X_OFFSET
BEHIND = HALF_L + RobotSpecs.LIDAR_MOUNT_X_OFFSET
SECTORS = (
    ("front", 0.0, math.pi / 4),
    ("left", math.pi / 2, math.pi / 4),
    ("rear", math.pi, math.pi / 4),
    ("right", -math.pi / 2, math.pi / 4),
)


def footprint_gap(
    ranges, angles, centre_rad: float, half_fov_rad: float, floor_m: float, mask=None, bins: int = 0
) -> float | None:
    """Closest return in a sector, measured from the chassis rectangle's edge.

    The chassis is the axis-aligned rectangle around its own centre, which the
    sensor is offset from: for a ray at bearing ``a`` the exit distance is the
    slab minimum of ``AHEAD/cos a`` or ``BEHIND/|cos a|`` and ``HALF_W/|sin a|``.
    A return INSIDE that rectangle is the chassis itself and is dropped, not
    reported as a negative gap.
    """
    if ranges is None or len(ranges) == 0:
        return None
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    delta = np.arctan2(np.sin(angles - centre_rad), np.cos(angles - centre_rad))
    keep = (np.abs(delta) <= half_fov_rad) & np.isfinite(ranges) & (ranges > floor_m)
    if mask is not None and bins:
        idx = ((angles + math.pi) / (2 * math.pi) * bins).astype(int) % bins
        keep &= ~mask[idx]
    if not bool(np.any(keep)):
        return None
    a = angles[keep]
    ca, sa = np.cos(a), np.sin(a)
    along = np.where(ca >= 0, AHEAD, BEHIND) / np.maximum(np.abs(ca), 1e-6)
    across = HALF_W / np.maximum(np.abs(sa), 1e-6)
    reach = np.minimum(along, across)
    gaps = ranges[keep] - reach
    gaps = gaps[gaps > 0.0]
    return float(np.min(gaps)) if gaps.size else None


def self_return_mask(scans, bins: int, floor_m: float, persistence: float) -> np.ndarray:
    """Angle bins whose return is a FIXTURE of the chassis, not the track.

    A ray that comes back short in nearly every tick of a round is not finding
    an obstacle in every tick of a round: it is finding the camera mast, a
    wheel, or the rear occluder. Measured here as the share of ticks in which
    the bin's closest return sits under ``floor_m``; above ``persistence`` the
    bin is masked out entirely. Necessary because the LIDAR's own occluded band
    (+-120-160 deg in robot frame) reports the chassis at a constant range, and
    an unmasked run calls 70% of a round a contact.
    """
    short = np.zeros(bins)
    seen = np.zeros(bins)
    for _ts, scan in scans:
        ranges = np.asarray(scan.ranges_m, dtype=float)
        angles = np.asarray(scan.angles_rad, dtype=float)
        ok = np.isfinite(ranges) & (ranges > 0.0)
        if not bool(np.any(ok)):
            continue
        idx = ((angles[ok] + math.pi) / (2 * math.pi) * bins).astype(int) % bins
        np.add.at(seen, idx, 1.0)
        np.add.at(short, idx[ranges[ok] < floor_m], 1.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        share = np.where(seen > 0, short / np.maximum(seen, 1.0), 0.0)
    return share >= persistence


def nearest(series, ts):
    if not series:
        return None
    return min(series, key=lambda tv: abs(tv[0] - ts))[1]


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--contact-m", type=float, default=0.03, help="Footprint gap under which a tick counts as contact.")
    parser.add_argument("--range-floor-m", type=float, default=0.05, help="Returns at or below this are self-returns from the chassis.")
    parser.add_argument("--rest-dps", type=float, default=20.0, help="Wheel speed under which the chassis counts as at rest.")
    parser.add_argument("--bins", type=int, default=72, help="Angle bins for the self-return mask.")
    parser.add_argument("--fixture-m", type=float, default=0.35, help="Range under which a bin's return counts as short, for the mask.")
    parser.add_argument("--fixture-share", type=float, default=0.80, help="Share of ticks a bin must be short in to be masked as a chassis fixture.")
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        scans, rows = read_bag(open_reader(bag_dir), LIDAR_YAW_OFFSET_RAD)
        wheel = read_motion_streams(bag_dir).drive_speed_dps
        if not rows:
            print(f"\n=== {bag_dir.name}: no nav_debug")
            continue

        mask = self_return_mask(scans, args.bins, args.fixture_m, args.fixture_share)
        by_sector: Counter[str] = Counter()
        by_lap: Counter[int] = Counter()
        by_phase: Counter[str] = Counter()
        sector_lap: Counter[tuple[str, int]] = Counter()
        events: list[tuple[float, str, float, int, str, float]] = []
        ticks = 0
        for ts, scan in scans:
            snap = min(rows, key=lambda r: abs(r[0] - ts))[1]
            phase = str(getattr(snap, "phase", "?")).split(".")[-1]
            lap = int(getattr(snap, "laps_completed", 0) or 0)
            speed = nearest(wheel, ts)
            ticks += 1
            worst_name, worst_gap = None, None
            for name, centre, half in SECTORS:
                gap = footprint_gap(
                    scan.ranges_m, scan.angles_rad, centre, half, args.range_floor_m, mask, args.bins
                )
                if gap is not None and (worst_gap is None or gap < worst_gap):
                    worst_name, worst_gap = name, gap
            if worst_gap is None or worst_gap >= args.contact_m:
                continue
            by_sector[worst_name] += 1
            by_lap[lap] += 1
            by_phase[phase] += 1
            sector_lap[(worst_name, lap)] += 1
            events.append((ts, worst_name, worst_gap, lap, phase, float(speed or 0.0)))

        masked_deg = sorted(round(math.degrees(-math.pi + (i + 0.5) * 2 * math.pi / args.bins)) for i in np.flatnonzero(mask))
        print(
            f"\n=== {bag_dir.name}   ticks={ticks}   contact ticks={len(events)}"
            f" (<{args.contact_m:.2f} m footprint gap)   masked bins {len(masked_deg)}/{args.bins}"
            f" at {masked_deg[:12]}{'...' if len(masked_deg) > 12 else ''}"
        )
        if not events:
            continue
        print("  by sector: " + ", ".join(f"{k} {v}" for k, v in by_sector.most_common()))
        print("  by lap:    " + ", ".join(f"L{k} {v}" for k, v in sorted(by_lap.items())))
        print("  by phase:  " + ", ".join(f"{k} {v}" for k, v in by_phase.most_common(5)))
        moving = [e for e in events if abs(e[5]) >= args.rest_dps]
        print(f"  moving during contact: {len(moving)} of {len(events)}")
        # WHERE, in believed coordinates, on a 0.5 m grid. The belief wanders up
        # to 15 cm within a round, so this is only good enough to name a
        # CORRIDOR and tell the lot's band from the far wall's.
        cells: Counter[tuple[float, float]] = Counter()
        for ts, _name, _gap, _lap, _phase, _speed in events:
            snap = min(rows, key=lambda r: abs(r[0] - ts))[1]
            px, py = getattr(snap, "pose_x", None), getattr(snap, "pose_y", None)
            if px is None or py is None:
                continue
            cells[(round(px * 2) / 2, round(py * 2) / 2)] += 1
        print("  believed cells: " + ", ".join(f"({x:.1f},{y:.1f}) {n}" for (x, y), n in cells.most_common(6)))
        # The tightest handful, with their believed pose, so the lot can be
        # recognised by position without trusting that position to find them.
        events.sort(key=lambda e: e[2])
        for ts, name, gap, lap, phase, speed in events[:8]:
            snap = min(rows, key=lambda r: abs(r[0] - ts))[1]
            px, py = getattr(snap, "pose_x", None), getattr(snap, "pose_y", None)
            pose = f"({px:.2f},{py:.2f})" if px is not None and py is not None else "(no pose)"
            print(f"    t={ts:7.1f}  {name:5s} gap {gap * 100:5.1f} cm  lap {lap}  {phase:16s} pose {pose}  wheel {speed:+.0f} dps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
