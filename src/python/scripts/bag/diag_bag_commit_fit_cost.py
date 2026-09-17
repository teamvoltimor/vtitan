r"""What would the commit-fit speed cap have DONE on the recorded rounds, tick by tick?

``sign_commit_fit_speed`` caps speed so the chassis turn radius (a speed
curve, ``R = intercept + slope * v``) can buy the lateral a committed pass
still needs within the run-up left to the pillar. ``diag_bag_pass_side_speed``
shows the geometry separates failed crossings at the COMMIT tick; this replays
the shipped router over every tick and applies the production formula
(``CoreNavigator._commit_fit_speed_cap``) to say:

* on what share of committed ticks the cap would BIND (fit below the
  commanded speed), and how hard -- the commanded speed against the capped one;
* the round-clock cost, as the extra seconds those ticks would take at the
  capped speed rather than the commanded one (``dt * (v_cmd / v_cap - 1)``),
  with the floor applied. This is the number that decides whether the cap is
  affordable at 180 s per round;
* how the cost splits between passes the chassis must CROSS and passes
  already on the legal side, since only the former is what the cap is for.

Replays the router the way ``diag_bag_commit_chain.py`` does, from the
recorded pose stream and camera frames, so the belief the cap reads is the
one the car would have had. Speeds are the recorded ``commanded_speed_mps``.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_commit_fit_cost.py RUN_DIR [RUN_DIR ...] \
        [--floor 0.10]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.domain.enums import Direction
from shared.domain.models import Pose, SignColor

from scripts.common.bag_io import (
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    scan_to_ranges_angles,
    settled_direction,
)
from scripts.common.stats import nearest_by_time, percentile
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.geometry import arc_fit_speed_mps, chassis_half_diagonal_m
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.planning.sign_router import SignRouter


def ticks_for_run(
    rows, frames, scans, tuning, floor: float, need_mode: str, run_up_mode: str
) -> tuple[list[dict], float]:
    """Per committed tick: commanded speed, fit, crossing flag; plus the run's driving time."""
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    scan_times = [t for t, _ in scans]
    frame_i = 0
    need_full = {
        "diag": chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2,
        "square": RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2,
        "cross": 0.0,
    }[need_mode]
    lead = RobotSpecs.LENGTH / 2 if run_up_mode == "nose" else 0.0
    out: list[dict] = []
    prev_rel = None
    driving_s = 0.0

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            for det in decode_detections(frames[frame_i][1]):
                if det.color not in (SignColor.RED, SignColor.GREEN):
                    continue
                o = detection_to_observation(det, pose, tuning, ranges, angles)
                if o is not None:
                    obs.append(o)
            frame_i += 1
        if d.steer_target_x is None:
            continue
        router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )
        dt = 0.0 if prev_rel is None else max(0.0, min(0.5, rel - prev_rel))
        prev_rel = rel
        v_cmd = d.commanded_speed_mps
        if v_cmd is None or v_cmd <= 0.0:
            continue
        driving_s += dt

        anchor = router.committed_sign_position
        if anchor is None:
            continue
        have = router.committed_pass_side_offset((d.pose_x, d.pose_y))
        if have is None:
            continue
        need = need_full - have
        dx, dy = anchor.x - d.pose_x, anchor.y - d.pose_y
        along = dx * math.cos(d.pose_yaw) + dy * math.sin(d.pose_yaw)
        fit = None
        if need > 0.0 and along > 0.0:
            fit = arc_fit_speed_mps(along - lead, need)
        cap = None if fit is None else max(fit, floor)
        out.append(
            {
                "dt": dt,
                "v_cmd": v_cmd,
                "cap": cap,
                "binds": cap is not None and cap < v_cmd,
                "crossing": have < 0.0,
                "along": along,
                "need": need,
            }
        )
    return out, driving_s


def main() -> int:
    """Replay the router over each bag and price the cap in round-clock seconds."""
    parser = create_bags_parser(__doc__)
    parser.add_argument("--floor", type=float, default=None, help="override sign_commit_fit_floor_mps")
    parser.add_argument(
        "--need",
        choices=("diag", "square", "cross"),
        default="diag",
        help="lateral the arc must buy: half-diagonal+sign (production), half-width+sign, or just the sign's line",
    )
    parser.add_argument(
        "--run-up",
        choices=("nose", "centre"),
        default="nose",
        help="measure the run-up from the nose (production) or the chassis centre (the bag instrument)",
    )
    args = parser.parse_args()
    tuning = get_tuning(None)
    floor = args.floor if args.floor is not None else tuning.sign_router.sign_commit_fit_floor_mps

    per_run = []
    all_ticks: list[dict] = []
    skipped = 0
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        ticks, driving_s = ticks_for_run(rows, frames, scans, tuning, floor, args.need, args.run_up)
        cost = sum(t["dt"] * (t["v_cmd"] / t["cap"] - 1.0) for t in ticks if t["binds"])
        per_run.append(
            (Path(bag).name.replace("run_", ""), driving_s, len(ticks), sum(1 for t in ticks if t["binds"]), cost)
        )
        all_ticks.extend(ticks)

    if skipped:
        print(f"== SKIPPED {skipped} unreadable bag(s)")
    if not all_ticks:
        print("No committed ticks reconstructed.")
        return 0

    print(
        f"== need={args.need} run-up={args.run_up} floor {floor:.3f} m/s; {len(all_ticks)} committed driving ticks over {len(per_run)} run(s)"
    )
    binds = [t for t in all_ticks if t["binds"]]
    print(f"   cap binds on {len(binds)} ({100 * len(binds) / len(all_ticks):.1f}% of committed ticks)")
    for label, pred in (("must CROSS", lambda t: t["crossing"]), ("already legal", lambda t: not t["crossing"])):
        grp = [t for t in all_ticks if pred(t)]
        b = [t for t in grp if t["binds"]]
        if not grp:
            continue
        ratio = sorted(t["v_cmd"] / t["cap"] for t in b)
        print(
            f"   {label:>14}: {len(grp):5d} ticks, binds {len(b):4d} ({100 * len(b) / len(grp):.1f}%)"
            + (f", v_cmd/cap p50 {percentile(ratio, 0.5):.2f} p90 {percentile(ratio, 0.9):.2f}" if ratio else "")
        )
    at_floor = sum(1 for t in binds if t["cap"] <= floor + 1e-9)
    print(
        f"   of the binding ticks, {at_floor} ({100 * at_floor / max(1, len(binds)):.0f}%) sit ON the floor"
        f" (the fit asked for less than {floor:.2f} m/s)"
    )
    print()

    print("== round-clock cost: extra seconds at the capped speed instead of the commanded one")
    rows_out = []
    total_cost = total_drive = 0.0
    for run, driving_s, n, nb, cost in per_run:
        rows_out.append(
            [
                run,
                f"{driving_s:.0f}",
                str(n),
                str(nb),
                f"{cost:+.1f}",
                f"{100 * cost / driving_s:.1f}%" if driving_s else "--",
            ]
        )
        total_cost += cost
        total_drive += driving_s
    rows_out.append(
        [
            "ALL",
            f"{total_drive:.0f}",
            str(len(all_ticks)),
            str(len(binds)),
            f"{total_cost:+.1f}",
            f"{100 * total_cost / total_drive:.1f}%" if total_drive else "--",
        ]
    )
    print_table(rows_out, ["run", "driving s", "committed", "binds", "cost s", "of driving"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
