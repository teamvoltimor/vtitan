r"""Does the locked K-turn back its TAIL into whatever sits beside the rear flank?

The Obstacles corpus (2026-09-16) lost five of six pillar scenarios this way: the wall
ahead fires a CRITICAL escape, the locked reverse curves the tail toward the steer side,
and the pillar the chassis was passing sits exactly there. ``k_turn_tail_clearance_m``
(``escape_recovery._decline_lock_into_tail``) now declines the lock when something is in
the strip the tail sweeps. This script asks the bags the two questions that decide whether
the simulator's failure is also the car's:

* At the latch of every LOCKED K-turn, was there anything in the swept strip, judged with
  production's own geometry (self-returns removed, rear bumper back by the reverse
  distance, tail-side flank out by the reach)? That is the gate's firing rate.
* During the manoeuvre, did the LIDAR see the tail-side rear flank CLOSE on something?
  Tracked as the minimum lateral gap between that flank and any return in the rear half
  on the tail side. Straight K-turns are the control: their tail curves nowhere.

If the strip predicts the closing (closing rate much higher with the strip occupied than
empty), the mechanism is real on hardware and the gate is aimed at it. If the tail closes
just as often with an empty strip, the closing comes from somewhere the strip does not
look. If the tail never closes at all, the mechanism is a simulator artefact.

Distances are from the chassis CENTRE and the flank; the sensor sits
``RobotSpecs.LIDAR_MOUNT_X_OFFSET`` ahead of centre, and the rear occlusion band means a
pillar beside the tail is often INVISIBLE to the raw scan -- the ``committed`` column
counts the router's committed sign as a mapped point for that reason, but the bag does not
carry the other routed positions, so the map source here is a floor.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_kturn_tail.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
import statistics

import numpy as np
from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import ManeuverType

from scripts.common.bag_io import LIDAR_YAW_OFFSET_RAD, create_bags_parser, open_reader, read_bag
from src.navigation.control.controllers import CollisionAvoidanceController, bumper_gap_behind
from src.navigation.control.controllers.collision_avoidance.sectors import ranges_beyond_chassis


def body_points(scan, margin_m: float, floor_m: float, no_data_m: float) -> np.ndarray:
    """Returns as chassis-centre-frame (px, py) with self-returns removed."""
    ranges = ranges_beyond_chassis(scan.ranges_m, scan.angles_rad, margin_m)
    angles = np.asarray(scan.angles_rad, dtype=float)
    ok = np.isfinite(ranges) & (ranges >= floor_m) & (ranges < no_data_m)
    r, a = ranges[ok], angles[ok]
    return np.column_stack((r * np.cos(a) + RobotSpecs.LIDAR_MOUNT_X_OFFSET, r * np.sin(a)))


def strip_nearest(points: np.ndarray, tail_side: float, reverse_m: float, reach_m: float) -> float | None:
    """Nearest point inside the strip the tail sweeps, as production defines it."""
    if points.size == 0:
        return None
    px, py = points[:, 0], points[:, 1]
    lateral = tail_side * py
    inside = (px >= -(RobotSpecs.LENGTH / 2 + reverse_m)) & (px <= 0.0) & (lateral >= 0.0) & (lateral <= RobotSpecs.WIDTH / 2 + reach_m)
    if not np.any(inside):
        return None
    return float(np.min(np.hypot(px[inside], py[inside])))


def tail_flank_gap(points: np.ndarray, tail_side: float) -> float | None:
    """Smallest lateral gap between the tail-side REAR flank and any return beside it."""
    if points.size == 0:
        return None
    px, py = points[:, 0], points[:, 1]
    lateral = tail_side * py - RobotSpecs.WIDTH / 2
    beside = (px >= -(RobotSpecs.LENGTH / 2 + 0.05)) & (px <= 0.0) & (lateral >= -0.02) & (lateral <= 0.30)
    if not np.any(beside):
        return None
    return float(np.min(lateral[beside]))


def rear_room(controller: CollisionAvoidanceController, scan, contact_dist: float) -> float | None:
    """Rear room beyond CONTACT_DIST as production's rear sector reads it; None when unmeasured.

    Production's ``rear_sector`` filters the chassis's own returns and reports
    ``measured=False`` when nothing valid is left in the rear arc -- the usual
    case on this mount, whose rear is the occlusion band. A crude range floor
    instead read the chassis itself as "no room" on every latch.
    """
    rear = controller.rear_sector(scan.ranges_m, scan.angles_rad)
    if not rear.measured:
        return None
    return bumper_gap_behind(rear.min_range_m) - contact_dist


def episodes(rows):
    """(t0, t1, steering) per contiguous K-turn latch, steering from its first tick."""
    out, start, last, steer = [], None, None, 0.0
    for ts, snap in rows:
        is_k = snap.active_maneuver_type == ManeuverType.K_TURN
        if is_k and start is None:
            start, steer = ts, float(snap.maneuver_steering or 0.0)
        elif not is_k and start is not None:
            out.append((start, last if last is not None else ts, steer))
            start = None
        if is_k:
            last = ts
    if start is not None:
        out.append((start, last if last is not None else start, steer))
    return out


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--reach-m", type=float, default=None, help="Lateral reach of the strip (default: production's Obstacles value).")
    parser.add_argument("--close-m", type=float, default=0.03, help="Flank gap under which the tail counts as having CLOSED on something.")
    parser.add_argument("--range-floor-m", type=float, default=0.05)
    args = parser.parse_args()

    tuning = NavigationTuning.load_default()
    escape = tuning.escape.for_obstacles_challenge()
    clearance = tuning.clearance.for_obstacles_challenge()
    reach = args.reach_m if args.reach_m is not None else escape.k_turn_tail_clearance_m
    reverse_m = abs(escape.rev_speed) * escape.k_turn_max_s
    straight_min = abs(escape.rev_speed) * escape.k_turn_min_s
    margin = tuning.sign_router.escape_mask_chassis_margin_m
    no_data = tuning.lidar_sectors.no_data_range_m
    half_sign = TrafficSignSpecs.WIDTH / 2
    controller = CollisionAvoidanceController.from_tuning(tuning, clearance=clearance, escape=escape)
    print(f"reach={reach:.2f} reverse_m={reverse_m:.3f} straight_min={straight_min:.3f} contact_dist={clearance.contact_dist} close_m={args.close_m}")

    pooled = []
    for bag_dir in args.bag_dirs:
        scans, rows = read_bag(open_reader(bag_dir), LIDAR_YAW_OFFSET_RAD)
        if not scans:
            print(f"\n=== {bag_dir.name}: no scans")
            continue
        scan_ts = np.array([t for t, _ in scans])
        pose = [(ts, snap) for ts, snap in rows if snap.pose_x is not None]

        eps = []
        for t0, t1, steer in episodes(rows):
            i0 = int(np.searchsorted(scan_ts, t0))
            if i0 >= len(scans):
                continue
            latch = scans[i0][1]
            locked = abs(steer) > 0.05
            tail_side = 1.0 if steer > 0 else -1.0
            pts = body_points(latch, margin, args.range_floor_m, no_data)
            nearest = strip_nearest(pts, tail_side, reverse_m, reach) if locked else None
            source = "scan" if nearest is not None else ""
            # The committed sign as a mapped point, in the chassis frame at the latch.
            snap = next((s for ts, s in pose if ts >= t0), None)
            if locked and snap is not None and snap.committed_sign_x_m is not None and snap.pose_yaw is not None:
                dx, dy = snap.committed_sign_x_m - snap.pose_x, snap.committed_sign_y_m - snap.pose_y
                c, s_ = math.cos(-snap.pose_yaw), math.sin(-snap.pose_yaw)
                px, py = dx * c - dy * s_, dx * s_ + dy * c
                lateral = tail_side * py - half_sign
                if -(RobotSpecs.LENGTH / 2 + reverse_m) <= px <= 0.0 and 0.0 <= lateral <= RobotSpecs.WIDTH / 2 + reach:
                    d = math.hypot(px, py)
                    if nearest is None or d < nearest:
                        nearest, source = d, "committed"
            room = rear_room(controller, latch, clearance.contact_dist)
            gaps = []
            for k in range(i0, len(scans)):
                if scan_ts[k] > t1 + 0.1:
                    break
                g = tail_flank_gap(body_points(scans[k][1], margin, args.range_floor_m, no_data), tail_side)
                if g is not None:
                    gaps.append(g)
            eps.append(
                {
                    "locked": locked,
                    "strip": nearest,
                    "source": source,
                    "room": room,
                    "min_gap": min(gaps) if gaps else None,
                    "closed": bool(gaps) and min(gaps) <= args.close_m,
                }
            )
        pooled.extend(eps)
        locked = [e for e in eps if e["locked"]]
        straight = [e for e in eps if not e["locked"]]
        hit = [e for e in locked if e["strip"] is not None]
        kept = [e for e in hit if e["room"] is not None and e["room"] < straight_min]
        print(
            f"\n=== {bag_dir.name}  k_turns={len(eps)} locked={len(locked)} straight={len(straight)}"
            f"\n  strip occupied at latch: {len(hit)}/{len(locked)}"
            f" (scan {sum(e['source'] == 'scan' for e in hit)}, committed {sum(e['source'] == 'committed' for e in hit)})"
            f" -> would decline {len(hit) - len(kept)}, lock kept for no rear room {len(kept)}"
            f" (rear unmeasured at latch: {sum(e['room'] is None for e in hit)})"
        )
        for label, group in (("locked, strip occupied", hit), ("locked, strip empty  ", [e for e in locked if e["strip"] is None]), ("straight (control)   ", straight)):
            seen = [e for e in group if e["min_gap"] is not None]
            if not seen:
                continue
            closed = sum(e["closed"] for e in seen)
            print(f"  {label}: n={len(seen):<3} tail closed <= {args.close_m:.2f} m: {closed} ({100 * closed / len(seen):.0f}%)  min flank gap p50 {statistics.median(e['min_gap'] for e in seen):.3f}")

    if not pooled:
        return 0
    locked = [e for e in pooled if e["locked"]]
    hit = [e for e in locked if e["strip"] is not None]
    kept = [e for e in hit if e["room"] is not None and e["room"] < straight_min]
    print(f"\n=== POOLED  k_turns={len(pooled)} locked={len(locked)}")
    print(f"  strip occupied at latch: {len(hit)}/{len(locked)} ({100 * len(hit) / max(1, len(locked)):.0f}%) -> decline {len(hit) - len(kept)}, kept {len(kept)}")
    for label, group in (("locked, strip occupied", hit), ("locked, strip empty  ", [e for e in locked if e["strip"] is None]), ("straight (control)   ", [e for e in pooled if not e["locked"]])):
        seen = [e for e in group if e["min_gap"] is not None]
        if not seen:
            continue
        closed = sum(e["closed"] for e in seen)
        print(f"  {label}: n={len(seen):<3} tail closed: {closed} ({100 * closed / len(seen):.0f}%)  min flank gap p50 {statistics.median(e['min_gap'] for e in seen):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
