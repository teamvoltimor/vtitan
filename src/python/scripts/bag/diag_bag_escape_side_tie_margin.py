r"""Is the K-turn's side a reading, or a coin flip on a near-tie?

``diag_bag_escape_sign_agreement.py`` measured that the escape's steering sign
matches the side the router wanted on only 56% of episodes, and 48-49% in a
corner. That script could not say WHY. This one can.

``CollisionAvoidanceController._k_turn_steer_sign`` decides the side with::

    if left_clear != right_clear:
        return -1.0 if left_clear > right_clear else 1.0
    # ... only here does it fall back to Direction

That is a STRICT float inequality. An exact tie between two independently
computed sector minima is close to impossible, so the ``Direction`` fallback the
docstring spends four paragraphs on is nearly unreachable, and every decision is
taken on whatever difference the two sectors happen to show -- including
differences of a millimetre, which carry no information about which side is
actually the safer one to swing into.

FALSIFIABLE PREDICTION under that hypothesis: episodes whose steering OPPOSES
the router must pile up at SMALL margins, while the ones that AGREE should be
spread across the range. If both groups show the same margin distribution the
hypothesis is REFUTED, and the fix is to force the router's side outright rather
than to add a tie band.

FIDELITY, because a reconstruction that does not reproduce the robot's own
decision proves nothing:

* The sectors are not re-implemented. The production ``_sector_to_model`` is
  called on a controller built by ``CollisionAvoidanceController.from_tuning``
  with the Obstacles-resolved clearances, so the half-FOV, the self-detection
  filter, ``min_valid_range_m``, the no-return upper bound and both rear blind
  wedges are the shipped ones by construction, not by transcription.
* ``_scan_to_ranges_angles`` mirrors the gateway's own preprocessing, including
  the LIDAR yaw offset.
* The reconstructed sign is compared against the ``maneuver_steering`` the bag
  RECORDED for every K-turn episode. That agreement rate is printed FIRST. If it
  is not high, every distribution below is describing a different function than
  the one that ran, and must be discarded.

KNOWN GAP, stated rather than hidden: production feeds ``_k_turn_steer_sign``
the MASKED scan (``escape_ranges``), and the bag does not record the full
believed sign map, only the committed one. So the exact mask cannot be rebuilt
offline. Two arms are printed: ``raw`` (no mask) and ``cmt-mask`` (returns
within ``ESCAPE_MASK_RADIUS_M`` of the COMMITTED pillar withheld, i.e. the one
piece of the mask the bag does support). The fidelity control above says which
arm is closer to what really ran.

Usage::

    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
    PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_escape_side_tie_margin.py \
        ../../data/live/runs/run_20260912_09*
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs

from scripts.bag.diag_bag_escape_sign_agreement import _collect, _sign
from scripts.common.bag_io import Topics, create_bags_parser, open_reader, scan_to_ranges_angles
from scripts.common.stats import fmean
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.control.controllers import CollisionAvoidanceController

_BANDS = (0.02, 0.05, 0.10, 0.20, 0.40)
"""Candidate tie-band widths, in metres, for the sensitivity table."""


def _scan_times(bag_dir: Path) -> tuple[list[float], list[bytes]]:
    """Every ``/scan`` message in one bag, as ``(elapsed_s, raw_bytes)``.

    Kept serialized: a bag carries ~800 sweeps and only the few dozen sitting at
    a latch tick are ever decoded.
    """
    reader = open_reader(bag_dir)
    t0: int | None = None
    times: list[float] = []
    blobs: list[bytes] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic == Topics.SCAN:
            times.append((t - t0) / 1e9)
            blobs.append(data)
    return times, blobs


def _nearest(times: list[float], target: float) -> int | None:
    """Index of the scan closest in time to ``target``, or None with no scans."""
    if not times:
        return None
    return min(range(len(times)), key=lambda i: abs(times[i] - target))


def _side_clearances(
    controller: CollisionAvoidanceController,
    ranges: np.ndarray,
    angles: np.ndarray,
) -> tuple[float, float, int, int]:
    """``(left_clear, right_clear, left_valid, right_valid)`` exactly as the k-turn sees them.

    Mirrors ``_k_turn_steer_sign``'s own two calls, argument for argument, by
    calling the production helper off the production controller rather than
    re-deriving the sector geometry here.
    """
    left, right = (
        controller._sector_to_model(  # noqa: SLF001 - the function under test
            ranges,
            angles,
            centre,
            controller.threat_half_fov_rad,
            filter_self_detection=True,
            self_detection_threshold_m=controller.self_detection_threshold_m,
            min_valid_range_m=controller.min_valid_range_m,
            no_data_range_m=controller.no_data_range_m,
            blind_wedge_left_min_rad=controller.blind_wedge_left_min_rad,
            blind_wedge_left_max_rad=controller.blind_wedge_left_max_rad,
            blind_wedge_right_min_rad=controller.blind_wedge_right_min_rad,
            blind_wedge_right_max_rad=controller.blind_wedge_right_max_rad,
        )
        for centre in (math.pi / 2, -math.pi / 2)
    )
    lc = left.min_range_m if left.valid_count > 0 else controller.no_data_range_m
    rc = right.min_range_m if right.valid_count > 0 else controller.no_data_range_m
    return lc, rc, left.valid_count, right.valid_count


def _mask_committed(
    ranges: np.ndarray,
    angles: np.ndarray,
    pose: tuple[float, float, float],
    sign_xy: tuple[float, float],
    radius_m: float,
) -> np.ndarray:
    """Withhold returns whose endpoint lands within ``radius_m`` of the committed pillar.

    The one piece of ``mask_mapped_obstacles`` the bag can support, since only
    the committed sign's believed position is recorded. Masked rays are pushed
    to the no-return sentinel the sector filter already rejects, which is the
    same substitution the production mask makes.
    """
    px, py, pyaw = pose
    bearings = angles + pyaw
    ex = px + ranges * np.cos(bearings)
    ey = py + ranges * np.sin(bearings)
    hit = np.hypot(ex - sign_xy[0], ey - sign_xy[1]) <= radius_m
    out = ranges.copy()
    out[hit] = RobotSpecs.LIDAR_MAX_RANGE
    return out


def _pcts(values: list[float]) -> str:
    """``p10/p25/p50/p75/p90`` of ``values`` in metres, or a dash when empty."""
    if not values:
        return "-"
    v = sorted(values)

    def q(f: float) -> float:
        return v[min(int(f * len(v)), len(v) - 1)]

    return f"{q(0.10):.3f} / {q(0.25):.3f} / {q(0.50):.3f} / {q(0.75):.3f} / {q(0.90):.3f}"


def main() -> int:  # noqa: C901, PLR0912, PLR0915
    """Reconstruct the k-turn side comparison at every latch tick and band it."""
    parser = create_bags_parser(__doc__)
    parser.add_argument("--commit-window", type=int, default=8)
    parser.add_argument("--settle", type=int, default=4)
    parser.add_argument("--kind", default="k_turn", help="manoeuvre kind whose side rule is under test")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    tuning = get_tuning(None)
    controller = CollisionAvoidanceController.from_tuning(
        tuning, clearance=tuning.clearance.for_obstacles_challenge()
    )
    mask_r = tuning.sign_router.escape_mask_radius_m

    tag = f"  [{args.label}]" if args.label else ""
    print(f"== K-TURN SIDE: READING OR NEAR-TIE?{tag}")
    print(f"   threat_half_fov_rad        = {controller.threat_half_fov_rad:.4f} "
          f"({math.degrees(controller.threat_half_fov_rad):.1f} deg)")
    print(f"   min_valid_range_m          = {controller.min_valid_range_m}")
    print(f"   self_detection_threshold_m = {controller.self_detection_threshold_m}")
    print(f"   no_data_range_m            = {controller.no_data_range_m}")
    print(f"   escape_mask_radius_m       = {mask_r}  (cmt-mask arm only)")

    rows: list[dict] = []
    for bag_dir in args.bag_dirs:
        b = Path(bag_dir)
        try:
            episodes, _ = _collect(b, args.commit_window, args.settle)
            times, blobs = _scan_times(b)
        except Exception as exc:  # noqa: BLE001 - one bad bag must not kill the corpus
            print(f"!! {b.name}: {type(exc).__name__}: {exc}")
            continue
        for ep in episodes:
            i = _nearest(times, ep.t_entry)
            if i is None:
                continue
            ranges, angles = scan_to_ranges_angles(deserialize_message(blobs[i], LaserScan))
            lc, rc, lv, rv = _side_clearances(controller, ranges, angles)
            rec = {
                "kind": ep.kind,
                "zone": ep.zone,
                "wanted": ep.wanted_side,
                "steer": _sign(ep.steer_mean),
                "raw_margin": abs(lc - rc),
                "raw_tie": lc == rc,
                "raw_blind": lv == 0 and rv == 0,
                "raw_pred": -1.0 if lc > rc else (1.0 if lc < rc else 0.0),
                "scan_lag": abs(times[i] - ep.t_entry),
                "cmt_margin": None,
                "cmt_pred": None,
            }
            if ep.sign_x_m is not None and ep.sign_y_m is not None and ep.pose_entry is not None:
                masked = _mask_committed(
                    ranges, angles, ep.pose_entry, (ep.sign_x_m, ep.sign_y_m), mask_r
                )
                mlc, mrc, _, _ = _side_clearances(controller, masked, angles)
                rec["cmt_margin"] = abs(mlc - mrc)
                rec["cmt_pred"] = -1.0 if mlc > mrc else (1.0 if mlc < mrc else 0.0)
            rows.append(rec)

    if not rows:
        print("no episodes with a scan -- nothing below means anything")
        return 0

    kt = [r for r in rows if r["kind"] == args.kind]
    print(f"\n   {len(rows)} latched episodes, {len(kt)} of kind '{args.kind}'")
    lags = sorted(r["scan_lag"] for r in rows)
    print(f"   scan-to-latch lag: p50={lags[len(lags) // 2] * 1000:.0f} ms"
          f"  p90={lags[int(0.9 * len(lags))] * 1000:.0f} ms")

    # ---------------- fidelity control ----------------
    print("\n== FIDELITY CONTROL: does the reconstruction reproduce the recorded side?")
    ft = []
    for arm, key in (("raw", "raw_pred"), ("cmt-mask", "cmt_pred")):
        ok = [r for r in kt if r[key] is not None and r["steer"] != 0]
        if not ok:
            continue
        hit = sum(1 for r in ok if _sign(r[key]) == r["steer"])
        ft.append([arm, len(ok), f"{hit}/{len(ok)}", f"{100 * hit / len(ok):.0f}%"])
    print_table(ft, ["arm", "n", "matches recorded sign", "rate"])
    print("   <- production feeds the MASKED scan, which the bag cannot fully rebuild.")
    print("      Read the distributions below through whichever arm scores higher, and")
    print("      treat the shortfall from 100% as the offline reconstruction's error bar.")

    # ---------------- is the Direction fallback reachable? ----------------
    ties = sum(1 for r in rows if r["raw_tie"])
    blind = sum(1 for r in rows if r["raw_blind"])
    print("\n== DOES THE `Direction` FALLBACK EVER RUN?")
    print(f"   exact float tie (left_clear == right_clear): {ties}/{len(rows)}"
          f" ({100 * ties / len(rows):.1f}%)")
    print(f"   no valid ray on EITHER side:                 {blind}/{len(rows)}"
          f" ({100 * blind / len(rows):.1f}%)")
    print("   <- both are the only paths to the Direction fallback. Near zero means the")
    print("      docstring's four paragraphs describe a branch that does not execute.")

    # ---------------- the margin distributions ----------------
    for arm, key in (("raw", "raw_margin"), ("cmt-mask", "cmt_margin")):
        usable = [r for r in kt if r[key] is not None and r["wanted"] != 0 and r["steer"] != 0]
        if not usable:
            continue
        print(f"\n== MARGIN |left_clear - right_clear| AT THE LATCH  [{arm}, kind={args.kind}]")
        print("   percentiles p10 / p25 / p50 / p75 / p90, metres")
        mt = []
        for zone in ("corner", "straight", "ALL"):
            zs = [r for r in usable if zone == "ALL" or r["zone"] == zone]
            if not zs:
                continue
            agree = [r[key] for r in zs if r["steer"] == r["wanted"]]
            oppose = [r[key] for r in zs if r["steer"] != r["wanted"]]
            mt.append([zone, "AGREES", len(agree), _pcts(agree),
                       f"{fmean(agree):.3f}" if agree else "-"])
            mt.append([zone, "OPPOSES", len(oppose), _pcts(oppose),
                       f"{fmean(oppose):.3f}" if oppose else "-"])
        print_table(mt, ["zone", "group", "n", "p10 / p25 / p50 / p75 / p90", "mean"])

        # ---------------- the sensitivity table ----------------
        print(f"\n   SENSITIVITY: episodes falling INSIDE a tie band  [{arm}]")
        print("   Split by zone as well as pooled: the corner is where agreement is worst,")
        print("   so a band that only worked pooled would be answering the wrong question.")
        st = []
        oppose_all = [r[key] for r in usable if r["steer"] != r["wanted"]]
        for zone in ("ALL", "corner", "straight"):
            zs = [r for r in usable if zone == "ALL" or r["zone"] == zone]
            agree_z = [r[key] for r in zs if r["steer"] == r["wanted"]]
            oppose_z = [r[key] for r in zs if r["steer"] != r["wanted"]]
            for band in _BANDS:
                a = sum(1 for v in agree_z if v < band)
                o = sum(1 for v in oppose_z if v < band)
                st.append([
                    zone,
                    f"{band:.2f}",
                    f"{o}/{len(oppose_z)} ({100 * o / max(len(oppose_z), 1):.0f}%)",
                    f"{a}/{len(agree_z)} ({100 * a / max(len(agree_z), 1):.0f}%)",
                    f"{o - a:+d}",
                    f"{o / max(a, 1):.2f}",
                ])
        print_table(st, ["zone", "band m", "OPPOSES captured", "AGREES disturbed", "net", "capture ratio"])
        print("   <- the band to ship maximises 'OPPOSES captured' while keeping")
        print("      'AGREES disturbed' low. A ratio near 1.0 means the band is blind to")
        print("      the distinction and cannot be the mechanism.")

        # The number that decides whether a tie band can carry the fix at all.
        # An episode that opposed the router on a LARGE, unambiguous clearance
        # difference is not a near-tie by any definition, so no band reaches it.
        for cut in (0.20, 0.40):
            confident = sum(1 for v in oppose_all if v >= cut)
            print(f"   OPPOSES taken on a CONFIDENT margin >= {cut:.2f} m: "
                  f"{confident}/{len(oppose_all)} ({100 * confident / max(len(oppose_all), 1):.0f}%)"
                  "   <- unreachable by ANY tie band")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
