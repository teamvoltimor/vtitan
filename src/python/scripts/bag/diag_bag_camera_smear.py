"""Motion-blur (smear) budget for the sign camera, measured against yaw rate.

The operator reports the camera going "slightly out of focus while moving".
Autofocus is already ruled out -- ``rpi_camera_module_3.toml`` pins
``camera_af_mode = "manual"`` at 0.8 D -- so the remaining candidate is MOTION
BLUR, and blur is a budget that can be computed and then checked against the
bags.

Three things are printed:

* SMEAR BUDGET -- pure geometry, no bag. 1536 px across a 102 deg HFOV is
  15.06 px/deg, so a yaw rate of w rad/s drags the image w*57.3*15.06 px/s
  across the sensor. Multiplied by a candidate exposure that is the horizontal
  smear in pixels, and it is compared against the APPARENT WIDTH of a 50 mm
  sign at the ranges that matter. Smear comparable to the box width is the
  failure mode.

* MEASURED SMEAR -- the box ASPECT RATIO against yaw rate. A traffic sign is
  50x100 mm, so a clean box has w/h ~ 0.5. Horizontal smear widens the box
  without touching its height, so if blur is real the aspect must RISE with
  yaw rate. This is a direct physical measurement of smear and does not
  depend on the detector's confidence calibration at all.

* CONFIDENCE vs YAW RATE -- the question the operator actually asked, with two
  controls. Yaw rate is preferred over commanded speed (which is what
  ``diag_bag_vision.py``'s BLUR section uses) because the budget above says
  ROTATION dominates translation while cornering.

Two traps this script exists to avoid:

* ``/imu/data``'s ``angular_velocity`` is ALL ZEROS with covariance -1 on this
  robot -- BNO08x UART-RVC mode provides no gyro. The yaw rate must come from
  differentiating the orientation quaternion. The script PROVES this per-corpus
  rather than assuming it (see the IMU ANGULAR_VELOCITY CONTROL block).

* The detection message lands ~0.85 s after the shutter (measured camera
  pipeline lag). Blur is set by the yaw rate at CAPTURE, not at publish, so
  every detection is matched to the yaw rate at ``t_msg - lag``. ``--lag-s 0``
  runs the un-shifted version for comparison.

Controls carried, so a null is readable:

* RANGE control -- confidence must fall monotonically with implied range
  (known: 0.854 at <0.5 m down to 0.524 beyond 1.5 m). If this control does
  not reproduce, the confidence plumbing is wrong and no yaw verdict is valid.
* YAW SPREAD control -- the yaw-rate distribution is printed. If the bins are
  degenerate there is nothing to correlate against.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_camera_smear.py RUN_DIR [RUN_DIR ...]
    pixi run -e dev python scripts/bag/diag_bag_camera_smear.py --lag-s 0 $(cat corpus.txt)
"""

from __future__ import annotations

import bisect
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_detections,
    elapsed_seconds,
    open_reader,
    quaternion_yaw,
)
from scripts.common.stats import percentile  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from shared.config.constants.track import TrafficSignSpecs  # noqa: E402
from shared.domain.models import SignColor  # noqa: E402

_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)
"""Pinhole focal length in pixels, same expression sign_discovery uses."""

_PX_PER_DEG: float = RobotSpecs.CAMERA_WIDTH / math.degrees(RobotSpecs.CAMERA_HFOV)
"""Linearised image scale. The real tangent mapping compresses the edges, so this
is the CENTRE-of-frame rate -- the optimistic end. Signs are steered toward the
centre by the router, so it is also the relevant one."""

_DEFAULT_LAG_S = 0.85
"""Measured capture-to-use lag of the camera pipeline. Blur is fixed at capture."""

_YAW_HALF_WINDOW_S = 0.03
"""Half-width of the finite-difference window used to read a yaw rate. At the
~166 Hz IMU rate a single-sample difference is quantisation noise; 60 ms spans
~10 samples and is still short against the ~0.5 s duration of a corner."""

_CONF_FLOOR = 0.45
"""`min_confidence` the detector gate uses -- boxes below it never reach the router."""

_EXPOSURES_US = (2000, 3000, 4000, 6000, 8000, 12000, 16000, 20000, 33000)
_RANGES_M = (0.5, 0.75, 1.0, 1.25, 1.5)
_YAW_BINS = ((0.0, 0.10), (0.10, 0.25), (0.25, 0.50), (0.50, 0.80), (0.80, 9.9))
_RANGE_BINS = ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 99.0))


def _sign_width_px(range_m: float) -> float:
    """Apparent width in pixels of a 50 mm sign face at `range_m`."""
    return _FOCAL_PX * TrafficSignSpecs.WIDTH / range_m


def _implied_range(height_px: float) -> float:
    """Pinhole range from box HEIGHT -- the axis horizontal smear does not touch."""
    if height_px <= 0:
        return float("inf")
    return _FOCAL_PX * TrafficSignSpecs.HEIGHT / height_px


def _report_budget() -> None:
    """Smear in pixels per exposure, against sign width at the useful ranges."""
    print("\n=== SMEAR BUDGET (geometry only, no bag) ===")
    print(f"frame {RobotSpecs.CAMERA_WIDTH}px over {math.degrees(RobotSpecs.CAMERA_HFOV):.0f} deg "
          f"= {_PX_PER_DEG:.2f} px/deg   focal {_FOCAL_PX:.1f} px")
    print(f"sign face {TrafficSignSpecs.WIDTH * 1000:.0f} mm wide, {TrafficSignSpecs.HEIGHT * 1000:.0f} mm tall\n")

    rows = [[f"{r:.2f} m", f"{_sign_width_px(r):.1f} px", f"{_sign_width_px(r) * 2:.1f} px"] for r in _RANGES_M]
    print_table(rows, ["range", "sign width", "sign height"])

    for yaw in (0.25, 0.50, 0.80):
        print(f"\nyaw rate {yaw:.2f} rad/s = {math.degrees(yaw):.1f} deg/s "
              f"= {math.degrees(yaw) * _PX_PER_DEG:.0f} px/s of image motion")
        px_per_s = math.degrees(yaw) * _PX_PER_DEG
        rows = []
        for us in _EXPOSURES_US:
            smear = px_per_s * us / 1e6
            cells = [f"{us / 1000:.0f} ms", f"{smear:.1f} px"]
            cells += [f"{smear / _sign_width_px(r):.0%}" for r in _RANGES_M]
            rows.append(cells)
        print_table(rows, ["exposure", "smear"] + [f"vs {r:.2f}m" for r in _RANGES_M])


class _YawTrack:
    """Yaw samples from one bag, queryable as a rate at an arbitrary time."""

    def __init__(self) -> None:
        self.t: list[float] = []
        self.yaw: list[float] = []
        self.gyro_z: list[float] = []

    def add(self, t: float, yaw: float, gyro_z: float) -> None:
        self.t.append(t)
        self.yaw.append(yaw)
        self.gyro_z.append(gyro_z)

    def _yaw_at(self, t: float) -> float | None:
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.yaw[0] if abs(self.t[0] - t) < 0.2 else None
        if i >= len(self.t):
            return self.yaw[-1] if abs(self.t[-1] - t) < 0.2 else None
        lo, hi = i - 1, i
        pick = lo if (t - self.t[lo]) <= (self.t[hi] - t) else hi
        return self.yaw[pick] if abs(self.t[pick] - t) < 0.2 else None

    def rate_at(self, t: float) -> float | None:
        """Signed yaw rate rad/s across a +/-`_YAW_HALF_WINDOW_S` window centred on `t`."""
        a = self._yaw_at(t - _YAW_HALF_WINDOW_S)
        b = self._yaw_at(t + _YAW_HALF_WINDOW_S)
        if a is None or b is None:
            return None
        d = math.atan2(math.sin(b - a), math.cos(b - a))
        return d / (2.0 * _YAW_HALF_WINDOW_S)


def _collect(bag_dir: Path, lag_s: float) -> tuple[list[tuple], int, int, int]:
    """One pass over a bag: detections tagged with the yaw rate at CAPTURE time.

    Returns `(rows, imu_n, gyro_nonzero_n, unmatched_n)` where each row is
    `(abs_yaw_rate, conf, cls, width_px, height_px, range_m)`.
    """
    reader = open_reader(bag_dir)
    t0: int | None = None
    yaw_track = _YawTrack()
    frames: list[tuple[float, list[dict]]] = []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if t0 is None:
            t0 = ts
        rel = elapsed_seconds(ts, t0)
        if topic == Topics.IMU_DATA:
            msg = deserialize_message(data, Imu)
            yaw_track.add(rel, quaternion_yaw(msg.orientation), float(msg.angular_velocity.z))
        elif topic == Topics.VISION_DETECTIONS:
            frames.append((rel, json.loads(deserialize_message(data, String).data) or []))

    rows: list[tuple] = []
    unmatched = 0
    for rel, payload in frames:
        rate = yaw_track.rate_at(rel - lag_s)
        for det in decode_detections(payload):
            if det.class_name not in (SignColor.RED, SignColor.GREEN):
                continue
            bbox = det.as_bbox()
            h = bbox.y_max - bbox.y_min
            w = bbox.x_max - bbox.x_min
            if h <= 0 or w <= 0:
                continue
            if rate is None:
                unmatched += 1
                continue
            rows.append((abs(rate), float(det.confidence), str(det.class_name), w, h, _implied_range(h)))

    gyro_nonzero = sum(1 for g in yaw_track.gyro_z if g != 0.0)
    return rows, len(yaw_track.t), gyro_nonzero, unmatched


def _bin_rows(rows: list[tuple], lo: float, hi: float, key: int) -> list[tuple]:
    return [r for r in rows if lo <= r[key] < hi]


def _stats_line(group: list[tuple]) -> list[str]:
    confs = [r[1] for r in group]
    aspects = [r[3] / r[4] for r in group]
    widths = [r[3] for r in group]
    below = sum(1 for c in confs if c < _CONF_FLOOR) / len(confs)
    return [
        str(len(group)),
        f"{percentile(confs, 0.5):.3f}",
        f"{percentile(confs, 0.1):.3f}",
        f"{below:.1%}",
        f"{percentile(aspects, 0.5):.3f}",
        f"{percentile(widths, 0.5):.1f}",
        f"{percentile([r[5] for r in group], 0.5):.2f}",
    ]


_STAT_COLS = ["boxes", "conf p50", "conf p10", f"share<{_CONF_FLOOR}", "aspect w/h p50", "width px p50", "range p50"]


def main() -> None:
    """Print the smear budget, then the measured yaw-rate splits with controls."""
    parser = create_bags_parser(__doc__)
    parser.add_argument("--lag-s", type=float, default=_DEFAULT_LAG_S,
                        help="capture-to-publish lag subtracted before reading the yaw rate")
    args = parser.parse_args()

    _report_budget()

    rows: list[tuple] = []
    imu_n = gyro_nonzero = unmatched = 0
    bags_ok = bags_skipped = 0
    for bag in args.bag_dirs:
        try:
            r, n, g, u = _collect(Path(bag), args.lag_s)
        except (RuntimeError, OSError, ValueError) as exc:  # noqa: PERF203
            print(f"  SKIP {bag}: {exc}")
            bags_skipped += 1
            continue
        rows += r
        imu_n += n
        gyro_nonzero += g
        unmatched += u
        bags_ok += 1

    print(f"\n=== CORPUS: {bags_ok} bags read, {bags_skipped} skipped, lag {args.lag_s:.2f}s ===")
    print(f"IMU samples {imu_n}   red/green boxes matched to a yaw rate {len(rows)}   unmatched {unmatched}")

    print("\n=== IMU ANGULAR_VELOCITY CONTROL ===")
    print(f"/imu/data angular_velocity.z non-zero samples: {gyro_nonzero} of {imu_n}")
    print("  (expected 0 of N -- BNO08x UART-RVC publishes no gyro. A non-zero count here")
    print("   means the robot changed IMU node and the quaternion path should be re-checked.)")

    if not rows:
        print("\nno red/green boxes matched -- nothing to correlate")
        return

    yaws = [r[0] for r in rows]
    print("\n=== YAW SPREAD CONTROL (at capture, per detection) ===")
    print(f"|yaw rate| rad/s: p10 {percentile(yaws, 0.1):.3f}  p50 {percentile(yaws, 0.5):.3f}  "
          f"p90 {percentile(yaws, 0.9):.3f}  max {max(yaws):.3f}")
    print(f"implied smear at 8 ms: p50 {math.degrees(percentile(yaws, 0.5)) * _PX_PER_DEG * 0.008:.1f} px  "
          f"p90 {math.degrees(percentile(yaws, 0.9)) * _PX_PER_DEG * 0.008:.1f} px")

    print("\n=== RANGE CONTROL -- confidence must fall with range ===")
    table = []
    for lo, hi in _RANGE_BINS:
        g = _bin_rows(rows, lo, hi, 5)
        if g:
            table.append([f"{lo:.1f}-{hi:.1f} m", *_stats_line(g)])
    print_table(table, ["range", *_STAT_COLS])

    print("\n=== CONFIDENCE AND BOX SHAPE vs YAW RATE AT CAPTURE ===")
    table = []
    for lo, hi in _YAW_BINS:
        g = _bin_rows(rows, lo, hi, 0)
        if g:
            table.append([f"{lo:.2f}-{hi:.2f}", *_stats_line(g)])
    print_table(table, ["|yaw| rad/s", *_STAT_COLS])

    print("\n=== SAME SPLIT, STRATIFIED BY RANGE (yaw correlates with corners; range must be held) ===")
    for rlo, rhi in _RANGE_BINS[:3]:
        sub = _bin_rows(rows, rlo, rhi, 5)
        if len(sub) < 50:
            continue
        print(f"\nrange {rlo:.1f}-{rhi:.1f} m  ({len(sub)} boxes)")
        table = []
        for lo, hi in _YAW_BINS:
            g = _bin_rows(sub, lo, hi, 0)
            if len(g) >= 10:
                table.append([f"{lo:.2f}-{hi:.2f}", *_stats_line(g)])
        print_table(table, ["|yaw| rad/s", *_STAT_COLS])

    _report_effective_exposure(rows)


def _report_effective_exposure(rows: list[tuple]) -> None:
    """Invert the smear budget: how long must the shutter be to widen boxes this much?

    The box HEIGHT is smear-free, so `aspect * height` recovers the width in the
    same units for every range. Subtracting the near-stationary bin's aspect
    leaves the EXCESS width that only rotation can explain, and dividing it by
    that bin's own median yaw rate in px/s gives an exposure in seconds.

    This is an estimate, not a readout. It assumes all excess width is smear --
    a detector that simply localises a blurred edge less tightly would inflate
    it the same way -- so treat it as an upper bound on the shutter, and settle
    the question with a real ``libcamera`` metadata capture on the Pi.
    """
    print("\n=== EFFECTIVE EXPOSURE IMPLIED BY THE BOX WIDENING ===")
    print("(excess box width / image px-per-second at that yaw rate; upper bound, see docstring)")
    for rlo, rhi in _RANGE_BINS[:3]:
        sub = _bin_rows(rows, rlo, rhi, 5)
        base = _bin_rows(sub, *_YAW_BINS[0], 0)
        if len(base) < 50:
            continue
        base_aspect = percentile([r[3] / r[4] for r in base], 0.5)
        print(f"\nrange {rlo:.1f}-{rhi:.1f} m   baseline aspect (|yaw|<{_YAW_BINS[0][1]}) {base_aspect:.3f}")
        table = []
        for lo, hi in _YAW_BINS[1:]:
            g = _bin_rows(sub, lo, hi, 0)
            if len(g) < 30:
                continue
            yaw_med = percentile([r[0] for r in g], 0.5)
            h_med = percentile([r[4] for r in g], 0.5)
            excess_px = (percentile([r[3] / r[4] for r in g], 0.5) - base_aspect) * h_med
            px_per_s = math.degrees(yaw_med) * _PX_PER_DEG
            exp_ms = 1000.0 * excess_px / px_per_s if px_per_s > 0 else float("nan")
            table.append([
                f"{lo:.2f}-{hi:.2f}", str(len(g)), f"{yaw_med:.3f}",
                f"{h_med:.1f}", f"{excess_px:+.1f}", f"{px_per_s:.0f}", f"{exp_ms:.1f}",
            ])
        print_table(table, ["|yaw| bin", "boxes", "yaw p50", "height px", "excess width", "px/s", "implied exp ms"])


if __name__ == "__main__":
    main()
