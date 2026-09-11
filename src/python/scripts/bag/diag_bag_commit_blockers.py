r"""What holds the router back once a sign EXISTS but is not yet committed?

``diag_bag_commit_chain.py`` measured the anticipation budget over 78 hardware
bags, 486 committed pillars:

    seen       0.824 m
    ingested   0.791 m   -0.033
    published  0.748 m   -0.043
    committed  0.469 m   -0.279   <- 79% of everything lost inside the lane

and closed the obvious door: only 3.1% of pillars are published beyond
``activation_dist_m`` (1.40), so raising it is inert for the other 97%. The
router is not waiting to be allowed in -- it is allowed in and declines.

So this counts WHY it declines, over every tick where a published sign sits
inside ``activation_dist`` and nothing is committed. The gates, in the order
``_active_sign_candidates`` and ``deform_waypoint`` apply them:

* **passed** -- already retired; nothing to do.
* **behind** -- more than ``BEHIND_TOLERANCE`` behind the chassis along its own
  heading, so it is cleared rather than upcoming.
* **cross-corridor** -- a different corridor than the robot's label AND further
  than ``activation_dist``.
* **corner** -- ``is_squarely_in_corridor`` says the TARGET WAYPOINT is on a
  turning arc rather than the straight segment the deformation model assumes,
  so the candidate is skipped and the claim dropped.

The last one is the hypothesis this script exists to test, and it is not
obviously right: the gate is deliberate and well argued in the code, guarding a
deformation model that genuinely does not hold through a corner. If it turns
out to dominate, the finding is NOT "delete the gate" -- it is that the sign
lane has no anticipation wherever a pillar sits near a corner, which is where
the lattice puts a good many of them.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_commit_blockers.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from scripts.bag.diag_bag_pass_side import _load, _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import create_bags_parser, decode_detections, settled_direction  # noqa: E402
from scripts.common.stats import nearest_by_time  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Direction  # noqa: E402
from shared.domain.models import Pose, SignColor, Waypoint  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import (  # noqa: E402
    BEHIND_TOLERANCE,
    SignRouter,
    is_squarely_in_corridor,
)


def blockers_for_run(rows, frames, scans, tuning) -> dict[str, int]:  # noqa: ANN001
    """Count, per tick, why the nearest reachable published sign was not committed."""
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    scan_times = [t for t, _ in scans]
    frame_i = 0
    tally = {
        "nothing else in range": 0,
        "corner (waypoint not squarely in corridor)": 0,
        "behind the chassis": 0,
        "cross-corridor and far": 0,
        "passed": 0,
        "waiting: router still holds an earlier sign": 0,
        "reachable but declined (unmodelled)": 0,
    }

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            for det in decode_detections(frames[frame_i][1]):
                if det.class_name not in (SignColor.RED, SignColor.GREEN):
                    continue
                o = detection_to_observation(det, pose, tuning, ranges, angles)
                if o is not None:
                    obs.append(o)
            frame_i += 1

        if d.steer_target_x is None:
            continue
        waypoint = (d.steer_target_x, d.steer_target_y)
        router.deform_waypoint(
            waypoint, (d.pose_x, d.pose_y), d.pose_yaw, d.current_corridor, obs
        )
        # The question is NOT "is the router busy" -- it is busy 91% of ticks,
        # usually with the pillar it is already passing. The budget being
        # explained is the 0.28 m between a pillar being PUBLISHED and that
        # same pillar being committed, so the subject is the nearest published
        # sign that is NOT the current claim.
        committed_idx = router._committed  # noqa: SLF001
        activation = router._config.activation_dist  # noqa: SLF001
        best: tuple[float, str] | None = None
        for i, sign in enumerate(router._signs):  # noqa: SLF001
            if i == committed_idx:
                continue
            dist = math.hypot(sign.x - d.pose_x, sign.y - d.pose_y)
            if dist > activation:
                continue
            if i in router._passed:  # noqa: SLF001
                reason = "passed"
            else:
                dx, dy = sign.x - d.pose_x, sign.y - d.pose_y
                along = dx * math.cos(d.pose_yaw) + dy * math.sin(d.pose_yaw)
                sign_corridor = router._sign_corridors[i]  # noqa: SLF001
                if along < -BEHIND_TOLERANCE:
                    reason = "behind the chassis"
                elif sign_corridor != d.current_corridor and dist > activation:
                    reason = "cross-corridor and far"
                elif not is_squarely_in_corridor(
                    waypoint[0], waypoint[1], sign_corridor, router._context  # noqa: SLF001
                ):
                    reason = "corner (waypoint not squarely in corridor)"
                elif committed_idx is not None:
                    # Every gate passed, and the only thing in the way is that
                    # the router is still holding an earlier sign. THIS is the
                    # handover cost, and commit_hysteresis is what enforces it.
                    reason = "waiting: router still holds an earlier sign"
                else:
                    # Reachable by every gate this script models, yet nothing
                    # committed -- so the cause is downstream of them.
                    reason = "reachable but declined (unmodelled)"
            if best is None or dist < best[0]:
                best = (dist, reason)
        if best is None:
            tally["nothing else in range"] += 1
        else:
            tally[best[1]] += 1

    return tally


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    args = parser.parse_args()
    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)

    total: dict[str, int] = {}
    skipped = 0
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        for reason, count in blockers_for_run(rows, frames, scans, tuning).items():
            total[reason] = total.get(reason, 0) + count

    if skipped:
        print(f"== SKIPPED {skipped} unreadable bag(s)")
    ticks = sum(total.values())
    if not ticks:
        print("No ticks reconstructed from these bags.")
        return

    print(f"== {ticks} navigation ticks, by what blocked the NEXT (uncommitted) sign")
    rows_out = [
        [reason, f"{count:7d}", f"{100 * count / ticks:5.1f}%"]
        for reason, count in sorted(total.items(), key=lambda kv: -kv[1])
    ]
    print_table(rows_out, ["state", "ticks", "share"])
    print()

    waiting = ticks - total.get("nothing else in range", 0)
    if waiting:
        print(f"== {waiting} ticks had a published sign in range that was NOT the current claim.")
        for reason in (
            "corner (waypoint not squarely in corridor)",
            "waiting: router still holds an earlier sign",
        ):
            n = total.get(reason, 0)
            print(f"   {reason:<46} {n:6d}  {100 * n / waiting:5.1f}% of those")


if __name__ == "__main__":
    main()
