r"""Which bearings does the LIDAR actually return from, and IN WHICH FRAME?

The sensor frame is NOT the robot frame. The offset production applies is
``RobotSpecs.lidar_yaw_offset_rad()``: the mount trim PLUS 180 degrees whenever
``lidar.inverted = true``, which it is on this chassis. A census read in one
frame alone can mistake that rotation for a physical wedge, and a measurement
only correct relative to an unstated frame is how the mistake happened.

So this prints the census in BOTH frames, side by side, always. There is no flag
to print only one: showing both makes the 180-degree rotation visible as a shift
of the pattern rather than invisible as a plausible table. Compare the
ROBOT-frame row against ``lidar_sectors.toml``, which is stated in that frame.
See adr:0080-lidar-mount-and-scan-plane for the mount and scan-plane decision.

METHOD

Bin every ray of every sampled scan into 10-degree bearing bins over the full
circle, and report per bin:

* **finite%** -- the share of rays the driver returned a real number for. The
  Slamtec C1 emits NaN/inf for a no-return, and a wedge of the mount blocking
  the beam shows up here as a run of low bins.
* **sub%** -- the share that came back BELOW ``min_valid_range_m`` (0.044 m).
  Those are returns the sector filter throws away, so navigation is just as
  blind there as if they were dropouts, but a naive finite-only census scores
  them as healthy. On this mount most of the rear wedge is sub-floor rather
  than absent, and reading only finite% understates the blindness.

TRAP: do NOT reach for ``bag_io.scan_to_ranges_angles`` here. It reproduces the
production gateway faithfully, which means it SUBSTITUTES ``LIDAR_MAX_RANGE``
for every non-finite ray -- after which nothing in the array is non-finite and
this census reads 100 percent everywhere. This script decodes the raw
``LaserScan`` on purpose, and is the only diagnostic in the repo that should.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_lidar_frame_census.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from scripts.common.bag_io import LIDAR_YAW_OFFSET_RAD, Topics, create_bags_parser, open_reader
from scripts.common.fidelity_axes import SECTOR_BANDS, SUB_FLOOR_M, SectorCensus

_BIN_DEG = 10
_BINS = 360 // _BIN_DEG


def _census(scans: list[bytes], offset_rad: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-bin finite, sub-floor and total ray counts at a given frame offset."""
    finite = np.zeros(_BINS)
    sub = np.zeros(_BINS)
    total = np.zeros(_BINS)
    for data in scans:
        msg = deserialize_message(data, LaserScan)
        raw = np.asarray(msg.ranges, dtype=float)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)) + offset_rad
        deg = np.degrees((angles + math.pi) % (2 * math.pi) - math.pi)
        idx = np.clip(((deg + 180) // _BIN_DEG).astype(int), 0, _BINS - 1)
        ok = np.isfinite(raw)
        np.add.at(total, idx, 1)
        np.add.at(finite, idx, ok)
        np.add.at(sub, idx, ok & (raw < SUB_FLOOR_M))
    return finite, sub, total


def _print_bins(label: str, finite: np.ndarray, sub: np.ndarray, total: np.ndarray) -> None:
    print(f"  {label:14} " + " ".join(f"{-180 + _BIN_DEG * i:+4d}" for i in range(_BINS)))
    for name, values in (("finite%", finite), ("sub-floor%", sub)):
        cells = " ".join(f"{100 * values[i] / max(total[i], 1):4.0f}" for i in range(_BINS))
        print(f"  {name:>14} {cells}")


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stride", type=int, default=4, help="Use every Nth scan (default 4)")
    args = parser.parse_args()

    print(f"lidar_yaw_offset = {math.degrees(LIDAR_YAW_OFFSET_RAD):.1f} deg  (robot frame = sensor frame + offset)")
    if abs(LIDAR_YAW_OFFSET_RAD) < 1e-6:
        print("  NOTE: offset is zero, so the two tables below are identical by construction.")

    for bag_dir in args.bag_dirs:
        reader = open_reader(bag_dir)
        scans: list[bytes] = []
        seen = 0
        while reader.has_next():
            topic, data, _t = reader.read_next()
            if topic != Topics.SCAN:
                continue
            if seen % args.stride == 0:
                scans.append(data)
            seen += 1
        if not scans:
            print(f"\n=== {bag_dir.name}: no /scan messages")
            continue

        print(f"\n=== {bag_dir.name}  scans={seen} sampled={len(scans)}  bins of {_BIN_DEG} deg")
        for label, offset in (("SENSOR frame", 0.0), ("ROBOT frame", LIDAR_YAW_OFFSET_RAD)):
            finite, sub, total = _census(scans, offset)
            _print_bins(label, finite, sub, total)

        census = SectorCensus()
        for data in scans:
            msg = deserialize_message(data, LaserScan)
            raw = np.asarray(msg.ranges, dtype=float)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)) + LIDAR_YAW_OFFSET_RAD
            deg = np.degrees((angles + math.pi) % (2 * math.pi) - math.pi)
            census.add(deg, raw, finite=np.isfinite(raw))
        print(f"  ROBOT-frame bands: {census.format()}")

    print(
        f"\nBands are {', '.join(b[0] for b in SECTOR_BANDS)} in the ROBOT frame. A wedge that moves\n"
        f"between the two tables above has moved by the mount offset, not by anything physical --\n"
        f"compare the ROBOT-frame row against lidar_sectors.toml, which is stated in that frame.\n"
        f"Rays below {SUB_FLOOR_M} m are dropped by the sector filter, so sub-floor% is blindness too."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
