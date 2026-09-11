r"""Does the escape mask's range floor blind it at the moment of contact?

``ESCAPE_MASK_CLUSTER_MIN_RANGE_M`` ships at 0.15 m. It was chosen on
run_20260911_110734 by measuring the robot-to-sign range over every COMMITTED
tick of a wedge (p10 0.252, p50 0.363, p90 0.544 m) -- the wrong population.
The mask exists to stop a contact recovery firing on the pillar the router is
already passing, and those recoveries engage at a robot-to-pillar range of p50
0.09-0.14 m, BELOW the floor. A floor picked on approach ranges switches the
mask off exactly where the failures are.

The obvious repair -- drop the floor -- reintroduces what the floor was for:
the chassis's own returns become clusters, and a self-return taken for a pillar
would mask a REAL obstacle. But a scalar was never the right instrument for
that job either. ``collision_avoidance.sectors.chassis_exit_range_m`` already
answers it per bearing: nothing outside the robot can return closer than the
chassis boundary along that ray, which over the rear sector alone runs 0.137 to
0.272 m. That is a geometric fact, not a tuned number.

So this compares four floors at the ENGAGEMENT ticks -- the instants a contact
recovery latched -- and asks three things of each: does the committed belief
associate to a cluster within ``ESCAPE_MASK_CLUSTER_ASSOC_M``, how many of the
clusters the floor admits are the robot seeing itself, and -- the only column
that is an OUTCOME rather than an intermediate -- would the recovery still have
engaged with that mask in place?

The self-return column is the veto. A floor that associates more often by
manufacturing self-returns is worse than the one it replaces, because a mask
anchored on the chassis can be pointed at anything.

Two replay assumptions, both stated because they bound the last column:

* the mask is given ONLY the committed belief, not the router's whole map. The
  real mask withholds every routed sign, so a recovery counted as still firing
  here might not on the robot. It under-claims, never over-claims.
* the belief's corridor is taken as the robot's own, which is what
  ``mask_mapped_obstacles``'s gate compares against. The robot is committed to
  that sign, so on hardware the two agree whenever anything is close enough to
  mask.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_mask_floor.py \
        data/live/runs/run_20260911_131459 data/live/runs/run_20260911_132017
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
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import ManeuverType
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
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

ASSOC_M = 0.35
"""``ESCAPE_MASK_CLUSTER_ASSOC_M`` as shipped. Held fixed here: this asks about
the FLOOR, and moving both at once would make neither attributable."""

CHASSIS_MARGIN_M = 0.01
"""Grown off the nominal rectangle. The body is not a perfect box and the mount
has tolerance, so a return a few millimetres outside it is still the robot.
Matches ``diag_bag_contact_bearing``'s ``SELF_MARGIN_M``."""

DROPPED_RANGE_M = 12.0
"""Beyond ``ProposerParams.max_range_m``, so ``find_clusters`` discards the ray.
A finite value rather than inf because ``decode_scan`` already substitutes a
finite range for dropouts and this must not read as a different KIND of miss."""

NO_MASK = "none (control)"
"""The known-present control. Every tick counted here DID engage on the robot,
so an unmasked replay must reproduce ~100% of them; anything less is the replay
being wrong rather than a mask being good, and the other rows are unreadable
until this one is full."""

CONTACT_RECOVERIES = (ManeuverType.SIDE_CORRECTION,)
"""The engagement this is about. ``k_turn``/``stuck_reverse`` are separate
manoeuvres with their own triggers and are counted apart, never pooled."""


@dataclass(frozen=True, slots=True)
class FloorResult:
    associated: int = 0
    engagements: int = 0
    self_clusters: int = 0
    total_clusters: int = 0
    assoc_dists: tuple[float, ...] = ()
    still_fires: int = 0

    def plus(self, other: FloorResult) -> FloorResult:
        return FloorResult(
            associated=self.associated + other.associated,
            engagements=self.engagements + other.engagements,
            self_clusters=self.self_clusters + other.self_clusters,
            total_clusters=self.total_clusters + other.total_clusters,
            assoc_dists=self.assoc_dists + other.assoc_dists,
            still_fires=self.still_fires + other.still_fires,
        )


def _is_self_return(range_m: float, bearing_rad: float) -> bool:
    """Is this point inside the chassis rectangle, in the LIDAR's own frame?"""
    boundary = float(chassis_exit_range_m(np.asarray([bearing_rad]))[0])
    return range_m <= boundary + CHASSIS_MARGIN_M


def _floor_scan(scan, floor_m: float | None):
    """The scan with everything below the floor pushed out of cluster range.

    ``floor_m`` of ``None`` means the per-bearing chassis boundary instead of a
    scalar -- the candidate repair, emulated here by pre-filtering rather than
    by changing ``find_clusters``, so the production code stays untouched until
    the number says it should not.
    """
    angles = np.asarray(scan.angles_rad, dtype=float)
    ranges = np.asarray(scan.ranges_m, dtype=float)
    floor = (
        chassis_exit_range_m(angles) + CHASSIS_MARGIN_M
        if floor_m is None
        else np.full_like(ranges, floor_m)
    )
    kept = np.where(ranges >= floor, ranges, DROPPED_RANGE_M)
    return replace(scan, ranges_m=tuple(kept.tolist()))


def _belief_in_robot_frame(snap) -> tuple[float, float] | None:
    if snap.committed_sign_x_m is None or snap.committed_sign_y_m is None:
        return None
    if snap.pose_x is None or snap.pose_y is None or snap.pose_yaw is None:
        return None
    dx = snap.committed_sign_x_m - snap.pose_x
    dy = snap.committed_sign_y_m - snap.pose_y
    cos, sin = math.cos(-snap.pose_yaw), math.sin(-snap.pose_yaw)
    return dx * cos - dy * sin, dx * sin + dy * cos


def analyse(
    bag_dir: Path,
    floors: dict[str, float | None],
    controller: CollisionAvoidanceController,
    controller_radius_m: float,
) -> tuple[dict[str, FloorResult], int, list[float]]:
    reader = open_reader(bag_dir)
    results = {name: FloorResult() for name in floors}
    scan = None
    prev_type: ManeuverType | None = None
    uncommitted = 0
    belief_ranges: list[float] = []

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
            # No commitment at the engagement: the mask has nothing to anchor,
            # so this tick cannot separate the floors and is reported apart
            # rather than scored as a miss against every one of them.
            uncommitted += 1
            continue
        bx, by = belief
        belief_ranges.append(math.hypot(bx, by))

        pose = Pose(snap.pose_x, snap.pose_y, snap.pose_yaw)
        mapped = [(Waypoint(snap.committed_sign_x_m, snap.committed_sign_y_m), corridor_for_position(pose.x, pose.y))]

        for name, floor_m in floors.items():
            clusters = find_clusters(_floor_scan(scan, floor_m), ProposerParams(min_range_m=0.01))
            best = math.inf
            selfies = 0
            cluster_xy: list[Waypoint] = []
            for cluster in clusters:
                if _is_self_return(cluster.range_m, cluster.bearing_rad):
                    selfies += 1
                cx = cluster.range_m * math.cos(cluster.bearing_rad)
                cy = cluster.range_m * math.sin(cluster.bearing_rad)
                best = min(best, math.hypot(cx - bx, cy - by))
                bearing = cluster.bearing_rad + pose.yaw
                cluster_xy.append(
                    Waypoint(
                        pose.x + cluster.range_m * math.cos(bearing),
                        pose.y + cluster.range_m * math.sin(bearing),
                    )
                )

            masked = np.asarray(scan.ranges_m, dtype=float) if name == NO_MASK else mask_mapped_obstacles(
                scan.ranges_m,
                scan.angles_rad,
                pose,
                mapped,
                controller_radius_m,
                cluster_xy=cluster_xy,
                assoc_m=ASSOC_M,
            )
            risk = controller.assess_risk(masked, scan.angles_rad)
            threat = controller.detect_threat_direction(masked, scan.angles_rad)
            fired = controller.compute_escape_maneuver(risk, threat, masked, scan.angles_rad, snap.direction)

            results[name] = results[name].plus(
                FloorResult(
                    associated=1 if best <= ASSOC_M else 0,
                    engagements=1,
                    self_clusters=selfies,
                    total_clusters=len(clusters),
                    assoc_dists=(best,) if math.isfinite(best) else (),
                    still_fires=1 if fired is not None and fired.maneuver_type in CONTACT_RECOVERIES else 0,
                )
            )

    return results, uncommitted, belief_ranges


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()

    # None is the per-bearing chassis boundary; the rest are the scalars that
    # have actually been shipped or considered, so the table is a decision and
    # not a sweep.
    floors: dict[str, float | None] = {
        NO_MASK: 0.30,
        "0.30 proposer": 0.30,
        "0.15 shipped": 0.15,
        "0.08": 0.08,
        "chassis geom": None,
    }

    tuning = NavigationTuning()
    controller = CollisionAvoidanceController.from_tuning(tuning)

    totals = {name: FloorResult() for name in floors}
    total_uncommitted = 0
    all_belief_ranges: list[float] = []

    for bag_dir in args.bag_dirs:
        try:
            per_bag, uncommitted, belief_ranges = analyse(
                Path(bag_dir), floors, controller, tuning.sign_router.ESCAPE_MASK_RADIUS_M
            )
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:90]}")
            continue
        total_uncommitted += uncommitted
        all_belief_ranges.extend(belief_ranges)
        n = next(iter(per_bag.values())).engagements
        print(f"{Path(bag_dir).name:<26} engagements with a commitment={n:<4} without={uncommitted}")
        for name, res in per_bag.items():
            totals[name] = totals[name].plus(res)

    n = next(iter(totals.values())).engagements
    print(f"\n== ALL RUNS: {n} contact-recovery engagements with a committed belief")
    print(f"   ({total_uncommitted} more engaged with nothing committed -- no anchor either way)")
    if not n:
        return 0
    if all_belief_ranges:
        print(
            "   robot-to-belief range at engagement: "
            f"p10={percentile(all_belief_ranges, 0.10):.3f} "
            f"p50={percentile(all_belief_ranges, 0.50):.3f} "
            f"p90={percentile(all_belief_ranges, 0.90):.3f} m"
        )
    print(f"\n   {'floor':<14} {'associated':>13} {'self-returns':>16} {'recovery STILL fires':>22}")
    for name, res in totals.items():
        pct = 100.0 * res.associated / res.engagements if res.engagements else 0.0
        selfies = (
            f"{res.self_clusters}/{res.total_clusters} "
            f"({100.0 * res.self_clusters / res.total_clusters:.1f}%)"
            if res.total_clusters
            else "no clusters"
        )
        fires = f"{res.still_fires}/{res.engagements} ({100.0 * res.still_fires / res.engagements:.1f}%)"
        print(f"   {name:<14} {res.associated:>4}/{res.engagements:<4} {pct:5.1f}% {selfies:>16} {fires:>22}")

    print("\n   nearest-cluster distance at engagement (the association's margin):")
    for name, res in totals.items():
        if not res.assoc_dists:
            print(f"     {name:<14} no finite cluster on any engagement")
            continue
        d = list(res.assoc_dists)
        print(
            f"     {name:<14} p10={percentile(d, 0.10):.3f} p50={percentile(d, 0.50):.3f} "
            f"p90={percentile(d, 0.90):.3f} m   (n={len(d)})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
