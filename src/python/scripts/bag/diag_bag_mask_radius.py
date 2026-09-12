r"""How large may ``escape_mask_radius_m`` grow before it starts eating walls?

The radius ships at 0.12 m and decides which rays belong to the sign the router
is already passing. Measured on the 2026-09-11 rounds, the surviving escape
triggers sit p50 0.144 m from the committed belief with p10 0.120 -- truncated
EXACTLY at the radius, which says the pillar's own returns are falling just
outside it.

Since ``4fc0fbab`` the anchor is the MEASURED cluster, not the belief, so the
radius no longer has to cover map error -- only the pillar's extent plus range
noise. The cost is named in ``mask_mapped_obstacles``'s own docstring: a wall
behind a sign can be 0.15 m away, and masking a wall removes a guard nothing
else replaces.

So this sweeps the radius at the same contact-recovery engagements
``diag_bag_mask_floor`` scores, and reports for each value how many recoveries
the mask suppresses (the BENEFIT) and how many rays it withholds that land on a
TRACK WALL (the COST), measured two ways -- against the nominal wall map placed
by the believed pose, and against the scan itself (the nearest return that is
not the anchor pillar). The map-based cost inherits the pose error; the
scan-based one does not and is the one to trust when they disagree.

Same two replay assumptions as ``diag_bag_mask_floor``: only the COMMITTED
belief is masked, never the router's whole routed map, so suppression
under-claims in every arm equally; and the belief's corridor is taken as the
robot's own.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import TrackDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import ManeuverType, Section
from shared.domain.models import Pose, Waypoint

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, decode_scan, open_reader
from scripts.common.stats import percentile
from src.navigation.control.controllers.collision_avoidance import (
    CollisionAvoidanceController,
    mask_mapped_obstacles,
)
from src.navigation.control.controllers.collision_avoidance.sectors import chassis_exit_range_m
from src.navigation.planning.lidar_proposer import ProposerParams, find_clusters
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

ASSOC_M = 0.35
CHASSIS_MARGIN_M = 0.01
DROPPED_RANGE_M = 12.0
PILLAR_EXTENT_M = 0.06
"""A sign is 0.05 m across, so a return within this of the anchor is the sign
itself rather than something standing behind it."""
WALL_TOL_M = 0.05
"""How close a ray endpoint must land to a nominal wall segment to be called a
wall return. Absorbs range noise and the map's own placement tolerance."""
RADII = (0.12, 0.14, 0.16, 0.18, 0.20, 0.25, 0.30)
CONTACT_RECOVERIES = (ManeuverType.SIDE_CORRECTION,)


@dataclass
class RadiusResult:
    engagements: int = 0
    still_fires: int = 0
    masked_rays: int = 0
    masked_wall_rays: int = 0
    engagements_masking_a_wall: int = 0


def _wall_segments(walls: TrackWalls) -> np.ndarray:
    lo, hi = TrackDimensions.MIN_COORD, TrackDimensions.MAX_COORD
    inner = walls.inner_block
    segs = [
        (lo, lo, hi, lo),
        (lo, hi, hi, hi),
        (lo, lo, lo, hi),
        (hi, lo, hi, hi),
        (inner.x_min, inner.y_min, inner.x_max, inner.y_min),
        (inner.x_min, inner.y_max, inner.x_max, inner.y_max),
        (inner.x_min, inner.y_min, inner.x_min, inner.y_max),
        (inner.x_max, inner.y_min, inner.x_max, inner.y_max),
    ]
    return np.asarray(segs, dtype=float)


def _dist_to_walls(px: np.ndarray, py: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Perpendicular distance from each point to the nearest wall segment."""
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = bx - ax, by - ay
    len2 = ex * ex + ey * ey
    dx = px[:, None] - ax[None, :]
    dy = py[:, None] - ay[None, :]
    t = np.clip((dx * ex[None, :] + dy * ey[None, :]) / len2[None, :], 0.0, 1.0)
    cx = dx - t * ex[None, :]
    cy = dy - t * ey[None, :]
    return np.min(np.hypot(cx, cy), axis=1)


def _chassis_floor_scan(scan):
    angles = np.asarray(scan.angles_rad, dtype=float)
    ranges = np.asarray(scan.ranges_m, dtype=float)
    floor = chassis_exit_range_m(angles) + CHASSIS_MARGIN_M
    return replace(scan, ranges_m=tuple(np.where(ranges >= floor, ranges, DROPPED_RANGE_M).tolist()))


def _belief_in_robot_frame(snap):
    if snap.committed_sign_x_m is None or snap.committed_sign_y_m is None:
        return None
    if snap.pose_x is None or snap.pose_y is None or snap.pose_yaw is None:
        return None
    dx = snap.committed_sign_x_m - snap.pose_x
    dy = snap.committed_sign_y_m - snap.pose_y
    cos, sin = math.cos(-snap.pose_yaw), math.sin(-snap.pose_yaw)
    return dx * cos - dy * sin, dx * sin + dy * cos


def _final_walls(bag_dir: Path) -> TrackWalls | None:
    """Last fully-populated corridor belief of the run, as in diag_yaw_flip_replay."""
    reader = open_reader(bag_dir)
    last = None
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != Topics.NAV_DEBUG:
            continue
        try:
            snap = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            continue
        widths = {
            Section.NORTH: snap.belief_north_m,
            Section.SOUTH: snap.belief_south_m,
            Section.EAST: snap.belief_east_m,
            Section.WEST: snap.belief_west_m,
        }
        if all(w is not None for w in widths.values()):
            last = widths
    return TrackWalls(corridor_geometry_from_widths(last)) if last else None


def analyse(bag_dir: Path, controller: CollisionAvoidanceController):
    walls = _final_walls(bag_dir)
    segs = _wall_segments(walls) if walls is not None else None
    reader = open_reader(bag_dir)
    results = {r: RadiusResult() for r in RADII}
    scan = None
    prev_type = None
    anchored = 0
    unanchored = 0
    anchor_wall_map: list[float] = []
    anchor_wall_scan: list[float] = []
    trigger_to_anchor: list[float] = []

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.SCAN:
            scan = decode_scan(deserialize_message(data, LaserScan), _LIDAR_YAW_OFFSET_RAD)
            continue
        if topic != Topics.NAV_DEBUG:
            continue
        try:
            snap = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            continue
        now = snap.active_maneuver_type
        engaged = now in CONTACT_RECOVERIES and prev_type not in CONTACT_RECOVERIES
        prev_type = now
        if not engaged or scan is None:
            continue
        belief = _belief_in_robot_frame(snap)
        if belief is None:
            unanchored += 1
            continue
        bx, by = belief
        pose = Pose(snap.pose_x, snap.pose_y, snap.pose_yaw)

        clusters = find_clusters(_chassis_floor_scan(scan), ProposerParams(min_range_m=0.01))
        best = math.inf
        anchor = None
        for c in clusters:
            cx = c.range_m * math.cos(c.bearing_rad)
            cy = c.range_m * math.sin(c.bearing_rad)
            d = math.hypot(cx - bx, cy - by)
            if d < best:
                best, anchor = d, (c.range_m, c.bearing_rad)
        if anchor is None or best > ASSOC_M:
            unanchored += 1
            continue
        anchored += 1

        bearing_w = anchor[1] + pose.yaw
        ax = pose.x + anchor[0] * math.cos(bearing_w)
        ay = pose.y + anchor[0] * math.sin(bearing_w)
        anchor_xy = [Waypoint(ax, ay)]
        mapped = [
            (
                Waypoint(snap.committed_sign_x_m, snap.committed_sign_y_m),
                corridor_for_position(pose.x, pose.y),
            )
        ]

        ranges = np.asarray(scan.ranges_m, dtype=float)
        angles = np.asarray(scan.angles_rad, dtype=float)
        finite = np.isfinite(ranges) & (ranges < 11.0)
        exw = pose.x + ranges * np.cos(angles + pose.yaw)
        eyw = pose.y + ranges * np.sin(angles + pose.yaw)
        d_anchor = np.hypot(exw - ax, eyw - ay)
        self_ray = ranges <= chassis_exit_range_m(angles) + CHASSIS_MARGIN_M

        if segs is not None:
            d_wall = _dist_to_walls(exw, eyw, segs)
            is_wall = finite & ~self_ray & (d_wall <= WALL_TOL_M)
            anchor_wall_map.append(float(_dist_to_walls(np.array([ax]), np.array([ay]), segs)[0]))
        else:
            is_wall = np.zeros_like(finite)

        beyond = finite & ~self_ray & (d_anchor > PILLAR_EXTENT_M)
        if beyond.any():
            anchor_wall_scan.append(float(d_anchor[beyond].min()))

        for radius in RADII:
            masked = np.asarray(
                mask_mapped_obstacles(
                    scan.ranges_m,
                    scan.angles_rad,
                    pose,
                    mapped,
                    radius,
                    cluster_xy=anchor_xy,
                    assoc_m=ASSOC_M,
                ),
                dtype=float,
            )
            gone = ~np.isfinite(masked) & finite
            risk = controller.assess_risk(masked, scan.angles_rad)
            threat = controller.detect_threat_direction(masked, scan.angles_rad)
            fired = controller.compute_escape_maneuver(
                risk, threat, masked, scan.angles_rad, snap.direction
            )
            res = results[radius]
            res.engagements += 1
            res.still_fires += 1 if fired is not None and fired.maneuver_type in CONTACT_RECOVERIES else 0
            res.masked_rays += int(gone.sum())
            wall_gone = int((gone & is_wall).sum())
            res.masked_wall_rays += wall_gone
            res.engagements_masking_a_wall += 1 if wall_gone else 0

            if radius == RADII[0] and fired is not None and fired.maneuver_type in CONTACT_RECOVERIES:
                near = finite & ~self_ray & np.isfinite(masked) & (ranges < 0.5)
                if near.any():
                    trigger_to_anchor.append(float(d_anchor[near].min()))

    return results, anchored, unanchored, anchor_wall_map, anchor_wall_scan, trigger_to_anchor


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()
    tuning = NavigationTuning()
    controller = CollisionAvoidanceController.from_tuning(tuning)

    totals = {r: RadiusResult() for r in RADII}
    anchored = unanchored = 0
    wall_map: list[float] = []
    wall_scan: list[float] = []
    trig: list[float] = []

    for bag_dir in args.bag_dirs:
        try:
            per, a, u, wm, ws, tg = analyse(Path(bag_dir), controller)
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:90]}")
            continue
        anchored += a
        unanchored += u
        wall_map.extend(wm)
        wall_scan.extend(ws)
        trig.extend(tg)
        print(f"{Path(bag_dir).name:<26} anchored={a:<4} no anchor={u}")
        for r, res in per.items():
            t = totals[r]
            t.engagements += res.engagements
            t.still_fires += res.still_fires
            t.masked_rays += res.masked_rays
            t.masked_wall_rays += res.masked_wall_rays
            t.engagements_masking_a_wall += res.engagements_masking_a_wall

    print(
        f"\n== {anchored} engagements with an ANCHORED mask "
        f"({unanchored} with no anchor -- the radius is inert there)"
    )
    if not anchored:
        return 0

    def band(name: str, vals: list[float]) -> None:
        if not vals:
            print(f"   {name}: none")
            return
        print(
            f"   {name}: p10={percentile(vals, 0.10):.3f} p50={percentile(vals, 0.50):.3f} "
            f"p90={percentile(vals, 0.90):.3f} m (n={len(vals)})"
        )

    band("anchor to nearest NOMINAL wall (believed pose)", wall_map)
    band("anchor to nearest non-pillar RETURN (scan only) ", wall_scan)
    band("surviving near trigger ray to anchor at r=0.12 ", trig)

    print(
        f"\n   {'radius':>7} {'recovery STILL fires':>22} {'masked rays/engagement':>24} "
        f"{'wall rays masked':>18} {'engagements eating a wall':>27}"
    )
    for r, res in totals.items():
        n = res.engagements
        print(
            f"   {r:>7.2f} {res.still_fires:>4}/{n:<4} ({100.0 * res.still_fires / n:5.1f}%) "
            f"{res.masked_rays / n:>20.1f} {res.masked_wall_rays:>18} "
            f"{res.engagements_masking_a_wall:>12}/{n} ({100.0 * res.engagements_masking_a_wall / n:5.1f}%)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
