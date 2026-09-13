r"""What obstacle map did the robot actually build, and is it plannable?

Operator question 2026-09-12: *"how many signs does it believe in, where does it
put them, how stable are they, and could better manoeuvres be planned off that
map?"*

Two halves, and they are deliberately separate because one of them cannot lie.

**RECORDED (authoritative).** ``active_sign_count`` and
``committed_sign_x_m``/``_y_m`` come straight off ``/nav_debug``. They are what
the robot believed on the day, at the revision that was on the Pi, with no
replay in the loop. Everything here is measured against two rulebook facts:

* the track physically holds AT MOST 8 pillars (2 per section x 4 sections), so
  any tick believing more is believing something impossible;
* a pillar may only stand on one of the 24 legal cells built by
  ``legal_sign_positions()`` (depths 1.0/1.5/2.0 m, division lines 0.4/0.6 m
  from each wall), so the distance from a believed position to its nearest legal
  cell is a pure error measurement with no model in it.

**REPLAYED (comparative).** The same recorded detections and poses are pushed
through a freshly built ``SignRouter`` twice: once with ``SLOT_SIGN_MAP`` as the
TOML pins it, and once with it forced the other way. That is the only way to say
how far this corpus sits from the 125-bag prediction in
``src.navigation.planning.sign_slot_map`` (routing error 23.3% -> 15.0%, worst
peak believed 24 -> 7, runs over the physical max 32/125 -> 0/125, position
changes 44.9 -> 3.1 per run).

CONTROLS:

* The recorded half is printed FIRST and never merged with the replayed half. A
  replay can be wrong; ``active_sign_count`` on the wire cannot.
* ``off-lattice`` share is reported for the recorded committed positions. Under
  a slot map it is 0.0% BY CONSTRUCTION -- a published position IS a legal cell
  -- so a non-zero reading is proof the slot map was NOT the map that ran, which
  is exactly the check that says which revision produced these bags.
* Jump sizes are reported as a distribution, not a mean. A map that re-points
  rarely but by 0.5 m is a different failure from one that jitters by 0.02 m,
  and a mean cannot tell them apart.
* Distinct believed positions per run is printed beside the peak count. A peak
  of 8 held by 8 positions is a full map; a peak of 8 reached by 40 different
  positions over the run is churn wearing a cardinality disguise.

Usage::

    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
    PYTHONPATH=. pixi run -e dev python scripts/bag/diag_bag_sign_map_reconstruct.py \
        ../../data/live/runs/run_20260912_09*
"""

from __future__ import annotations

import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.models import Pose

from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles
from scripts.common.bag_io import (
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.stats import nearest_by_time
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning, tuning_with_overrides
from src.navigation.planning.sign_discovery import detection_to_observation, legal_sign_positions
from src.navigation.planning.sign_router import SignRouter
from src.navigation.planning.waypoints import corridor_for_position

PHYSICAL_MAX_SIGNS = 8
"""Two per section over four sections. The rulebook cap, not a tuning value."""

_LEGAL = legal_sign_positions()

# Two believed positions within this are treated as the same pillar when
# counting DISTINCT positions. Half the closest legal pair (0.20 m across the
# two division lines), so it can never merge two genuinely different cells.
_SAME_POSITION_M = 0.10


def _nearest_legal(x: float, y: float) -> float:
    """Distance from ``(x, y)`` to the closest legal cell."""
    return min(math.dist((x, y), p) for p in _LEGAL)


@dataclass(slots=True)
class RecordedStats:
    """Everything read straight off ``/nav_debug`` -- no replay involved."""

    run: str
    ticks: int = 0
    duration_s: float = 0.0
    laps: int = 0
    count_ticks: int = 0
    peak: int = 0
    over_max: int = 0
    counts: Counter[int] = field(default_factory=Counter)
    committed_ticks: int = 0
    committed_positions: list[tuple[float, float]] = field(default_factory=list)
    committed_jumps: list[float] = field(default_factory=list)
    committed_offlattice: list[float] = field(default_factory=list)
    first_belief_s: float | None = None


def _recorded(rows, run: str) -> RecordedStats:  # noqa: ANN001
    st = RecordedStats(run=run)
    st.ticks = len(rows)
    st.duration_s = rows[-1][0] if rows else 0.0
    st.laps = max((s.laps_completed for _, s in rows), default=0)
    prev: tuple[float, float] | None = None
    for rel, s in rows:
        if s.active_sign_count is not None:
            st.count_ticks += 1
            st.counts[s.active_sign_count] += 1
            st.peak = max(st.peak, s.active_sign_count)
            if s.active_sign_count > PHYSICAL_MAX_SIGNS:
                st.over_max += 1
            if s.active_sign_count > 0 and st.first_belief_s is None:
                st.first_belief_s = rel
        if s.committed_sign_x_m is None or s.committed_sign_y_m is None:
            continue
        here = (s.committed_sign_x_m, s.committed_sign_y_m)
        st.committed_ticks += 1
        st.committed_positions.append(here)
        st.committed_offlattice.append(_nearest_legal(*here))
        if prev is not None and math.dist(prev, here) > 1e-9:
            st.committed_jumps.append(math.dist(prev, here))
        prev = here
    return st


@dataclass(slots=True)
class ReplayStats:
    """One arm of the replay over one bag."""

    run: str
    arm: str
    ticks: int = 0
    peak: int = 0
    over_max: int = 0
    position_changes: int = 0
    jumps: list[float] = field(default_factory=list)
    offlattice: list[float] = field(default_factory=list)
    published_total: int = 0
    first_publication_s: float | None = None
    empty_ticks: int = 0


def _pose_at(pose_times: list[float], poses: dict[float, Pose], target: float) -> Pose | None:
    """The recorded pose nearest ``target`` seconds, or None with no poses."""
    if not pose_times:
        return None
    best = min(pose_times, key=lambda t: abs(t - target))
    return poses[best]


def _replay(run: str, rows, frames, scans, tuning, arm: str, latency: float) -> ReplayStats:  # noqa: ANN001, C901, PLR0912, PLR0915
    """Push the recorded observations through a fresh router and watch its map."""
    st = ReplayStats(run=run, arm=arm)
    direction = settled_direction(rows) or None
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)

    poses = {
        rel: Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        for rel, d in rows
        if d.pose_x is not None and d.pose_y is not None and d.pose_yaw is not None
    }
    pose_times = sorted(poses)
    if not pose_times:
        return st
    parking_corridor = corridor_for_position(poses[pose_times[0]].x, poses[pose_times[0]].y)
    scan_times = [t for t, _ in scans]

    frame_i = 0
    latest: tuple[float, list[dict]] | None = None
    prev_positions: list[tuple[float, float]] = []

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        if d.steer_target_x is None or d.steer_target_y is None:
            continue
        st.ticks += 1
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)

        # PRODUCTION SHAPE: `_latest_detections` is never cleared, so the
        # navigator refolds the last frame on every tick until a new one
        # arrives. Replaying one-frame-per-tick instead would feed the map a
        # fraction of the evidence it really got.
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            latest = frames[frame_i]
            frame_i += 1

        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )

        obs = []
        if latest is not None:
            frame_rel, payload = latest
            seen_from = _pose_at(pose_times, poses, frame_rel - latency) or pose
            corridor_now = corridor_for_position(seen_from.x, seen_from.y)
            barrier_possible = (
                parking_corridor is None
                or corridor_now is None
                or corridor_now == parking_corridor
                or corridor_now in parking_corridor.neighbours
            )
            for det in decode_detections(payload):
                o = detection_to_observation(
                    det, seen_from, tuning, ranges, angles, barrier_possible=barrier_possible
                )
                if o is not None:
                    obs.append(o)

        router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )

        # Cardinality is read off the ROUTED list (what the router still
        # intends to steer around), identity off the FULL sign list. They are
        # not interchangeable: `routed_sign_positions` drops passed signs, so
        # its index i is not a stable identity and comparing it tick-to-tick
        # would score every pass as a position change.
        n = len(router.routed_sign_positions)
        st.peak = max(st.peak, n)
        if n > PHYSICAL_MAX_SIGNS:
            st.over_max += 1
        if n == 0:
            st.empty_ticks += 1
        elif st.first_publication_s is None:
            st.first_publication_s = rel
        positions = [(s.x, s.y) for s in router.signs]
        for i, p in enumerate(positions):
            if i < len(prev_positions):
                moved = math.dist(prev_positions[i], p)
                if moved > _SAME_POSITION_M:
                    st.position_changes += 1
                    st.jumps.append(moved)
        st.published_total = max(st.published_total, len(positions))
        prev_positions = positions

    st.offlattice = [_nearest_legal(x, y) for x, y in prev_positions]
    return st


def _pct(values: list[float], q: float) -> float:
    """The ``q`` quantile of ``values`` by nearest rank, NaN when empty."""
    if not values:
        return float("nan")
    values = sorted(values)
    return values[min(int(q * len(values)), len(values) - 1)]


def main() -> None:  # noqa: C901, PLR0912, PLR0915
    """Report the recorded belief first, then the two-arm replay beside it."""
    parser = create_bags_parser(__doc__)
    parser.add_argument("--latency", type=float, default=0.12, help="camera latency assumed when pairing a frame with its pose")
    parser.add_argument("--no-replay", action="store_true", help="recorded half only -- fast, and needs no vision topic")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    tuning = get_tuning(None)
    pinned_slot = tuning.sign_router.SLOT_SIGN_MAP
    tag = f"  [{args.label}]" if args.label else ""

    print(f"== TUNING IN FORCE{tag}")
    print(f"   sign_router.SLOT_SIGN_MAP            = {pinned_slot}   (pydantic default False; TOML pin decides)")
    print(f"   sign_router.SLOT_ACCEPT_RADIUS_M     = {tuning.sign_router.SLOT_ACCEPT_RADIUS_M}")
    print(f"   sign_router.SLOT_MIN_EVIDENCE        = {tuning.sign_router.SLOT_MIN_EVIDENCE}")
    print(f"   sign_router.SLOT_REPOINT_MARGIN      = {tuning.sign_router.SLOT_REPOINT_MARGIN}")
    print(f"   legal cells: {len(_LEGAL)}, physical max pillars: {PHYSICAL_MAX_SIGNS}")

    recorded: list[RecordedStats] = []
    bags: list[Path] = [Path(bag_dir) for bag_dir in args.bag_dirs]

    # ---------------- recorded half ----------------
    from scripts.common.bag_io import load_nav_debug_rows  # noqa: PLC0415 - ROS import kept local

    for b in bags:
        try:
            rows, _ = load_nav_debug_rows(b)
        except Exception as exc:  # noqa: BLE001
            print(f"!! {b.name}: {type(exc).__name__}: {exc}")
            continue
        if rows:
            recorded.append(_recorded(rows, b.name))

    if not recorded:
        print("no bags read -- nothing below means anything")
        return

    print(f"\n== RECORDED BELIEF (straight off /nav_debug, {len(recorded)} runs)")
    table = []
    for st in recorded:
        pos = st.committed_positions
        seen: list[tuple[float, float]] = []
        for p in pos:
            if all(math.dist(p, q) > _SAME_POSITION_M for q in seen):
                seen.append(p)
        distinct = len(seen)
        table.append([
            st.run.replace("run_", ""),
            f"{st.duration_s:.0f}",
            st.laps,
            st.peak,
            f"{st.over_max}/{st.count_ticks}",
            f"{st.first_belief_s:.1f}" if st.first_belief_s is not None else "-",
            f"{100 * st.committed_ticks / max(st.ticks, 1):.0f}%",
            distinct,
            len(st.committed_jumps),
            f"{statistics.fmean(st.committed_offlattice):.3f}" if st.committed_offlattice else "-",
        ])
    print_table(
        table,
        ["run", "dur s", "laps", "peak believed", "ticks > 8", "1st belief s",
         "committed ticks", "distinct cmt pos", "cmt jumps", "mean off-lattice m"],
    )

    peaks = sorted(st.peak for st in recorded)
    over_runs = sum(1 for st in recorded if st.over_max > 0)
    all_counts: Counter[int] = Counter()
    for st in recorded:
        all_counts.update(st.counts)
    total_count_ticks = sum(all_counts.values())
    all_off = [v for st in recorded for v in st.committed_offlattice]
    all_jumps = [v for st in recorded for v in st.committed_jumps]

    print(f"\n   peak believed per run: p50={peaks[len(peaks) // 2]} p90={_pct([float(p) for p in peaks], 0.9):.0f} max={peaks[-1]}"
          f"   (physical max {PHYSICAL_MAX_SIGNS})")
    print(f"   runs that ever believed more than {PHYSICAL_MAX_SIGNS}: {over_runs}/{len(recorded)}")
    print(f"   ticks believing more than {PHYSICAL_MAX_SIGNS}:         "
          f"{sum(st.over_max for st in recorded)}/{total_count_ticks} "
          f"({100 * sum(st.over_max for st in recorded) / max(total_count_ticks, 1):.1f}%)")
    print("   believed-count histogram: " + "  ".join(
        f"{k}:{100 * v / max(total_count_ticks, 1):.0f}%" for k, v in sorted(all_counts.items())
    ))
    if all_off:
        offl = sum(1 for v in all_off if v > 1e-6)
        print(f"\n   committed positions OFF the legal lattice: {offl}/{len(all_off)}"
              f" ({100 * offl / len(all_off):.1f}%)")
        print(f"   distance to nearest legal cell: p50={_pct(all_off, 0.5):.3f}  p90={_pct(all_off, 0.9):.3f}"
              f"  max={max(all_off):.3f} m")
        print("   <- EXACTLY 0.0% off-lattice is the slot map's signature: a published")
        print("      position IS a legal cell. Anything above 0% proves free clustering ran.")
    if all_jumps:
        print(f"\n   committed-position JUMPS: {len(all_jumps)} over {len(recorded)} runs"
              f"  ({len(all_jumps) / len(recorded):.1f} per run)")
        print(f"      size p50={_pct(all_jumps, 0.5):.3f}  p90={_pct(all_jumps, 0.9):.3f}  max={max(all_jumps):.3f} m")
        # A jump is only a MAP defect when it is too small to be a different
        # pillar. The legal lattice puts the lane partner 0.20 m away and the
        # next depth 0.50 m away, so anything past 0.30 m is the router handing
        # the commitment to the NEXT pillar -- correct behaviour, not churn.
        jitter = sum(1 for v in all_jumps if v <= 0.10)
        repoint = sum(1 for v in all_jumps if 0.10 < v <= 0.30)
        handoff = sum(1 for v in all_jumps if v > 0.30)
        print(f"      <=0.10 m  jitter (sub-lane)              : {jitter:4} ({100 * jitter / len(all_jumps):.0f}%)")
        print(f"      0.10-0.30 m  RE-POINT while committed    : {repoint:4} ({100 * repoint / len(all_jumps):.0f}%)"
              "   <- the spec predicts 0 of these")
        print(f"      >0.30 m  handoff to the NEXT pillar      : {handoff:4} ({100 * handoff / len(all_jumps):.0f}%)"
              "   <- correct behaviour, not churn")

    # A commitment that returns to a position it has already left is churn no
    # size threshold can catch: the router is oscillating between two beliefs.
    revisits = 0
    commit_runs = 0
    for st in recorded:
        seq: list[tuple[float, float]] = []
        for p in st.committed_positions:
            if not seq or math.dist(seq[-1], p) > _SAME_POSITION_M:
                seq.append(p)
        if len(seq) > 1:
            commit_runs += 1
        for i, p in enumerate(seq):
            if any(math.dist(p, q) <= _SAME_POSITION_M for q in seq[:max(i - 1, 0)]):
                revisits += 1
    print(f"\n   commitment RETURNED to a position it had already left: {revisits}"
          f" over {commit_runs} runs with more than one commitment")
    print("   <- an A -> B -> A commitment is oscillation, and no jump-size cut catches it")

    print("\n   TIME WITHOUT A MAP -- the window no plan can use")
    firsts = [st.first_belief_s for st in recorded if st.first_belief_s is not None]
    if firsts:
        print(f"      first tick believing any sign: p50={_pct(firsts, 0.5):.1f}  p90={_pct(firsts, 0.9):.1f}"
              f"  max={max(firsts):.1f} s")
        share = [
            st.first_belief_s / st.duration_s
            for st in recorded
            if st.first_belief_s is not None and st.duration_s > 0
        ]
        print(f"      as a share of the round: p50={100 * _pct(share, 0.5):.0f}%  p90={100 * _pct(share, 0.9):.0f}%")
        print("   <- a manoeuvre planned off the map cannot exist before this instant")

    if args.no_replay:
        return

    # ---------------- replayed half ----------------
    arms = {
        f"slot={pinned_slot} (SHIPPED)": tuning,
        f"slot={not pinned_slot} (control)": tuning_with_overrides(
            {"SLOT_SIGN_MAP": not pinned_slot}, group="sign_router", base=tuning
        ),
    }
    results: dict[str, list[ReplayStats]] = {k: [] for k in arms}
    for b in bags:
        try:
            rows, frames, scans = read_vision_rows_and_scans(b)
        except Exception as exc:  # noqa: BLE001
            print(f"!! replay {b.name}: {type(exc).__name__}: {exc}")
            continue
        if not frames:
            continue
        for name, t in arms.items():
            try:
                results[name].append(_replay(b.name, rows, frames, scans, t, name, args.latency))
            except Exception as exc:  # noqa: BLE001
                print(f"!! replay {b.name} [{name}]: {type(exc).__name__}: {exc}")

    usable = sum(len(v) for v in results.values())
    print(f"\n== REPLAYED MAP, same observations both arms   ({usable // max(len(arms), 1)} bags replayed)")
    print("   <- if the bag count is 0, the corpus carried no /vision/detections and")
    print("      nothing in this section means anything")
    if not usable:
        return
    rt = []
    for name, stats in results.items():
        if not stats:
            continue
        peaks_r = sorted(s.peak for s in stats)
        offs = [v for s in stats for v in s.offlattice]
        jumps = [v for s in stats for v in s.jumps]
        firsts = [s.first_publication_s for s in stats if s.first_publication_s is not None]
        empty = sum(s.empty_ticks for s in stats)
        ticks = sum(s.ticks for s in stats)
        rt.append([
            name,
            len(stats),
            peaks_r[len(peaks_r) // 2],
            peaks_r[-1],
            f"{sum(1 for s in stats if s.over_max > 0)}/{len(stats)}",
            f"{statistics.fmean([s.position_changes for s in stats]):.1f}",
            f"{_pct(jumps, 0.5):.3f}" if jumps else "-",
            f"{100 * sum(1 for v in offs if v > 1e-6) / max(len(offs), 1):.0f}%",
            f"{statistics.fmean(firsts):.1f}" if firsts else "-",
            f"{100 * empty / max(ticks, 1):.0f}%",
        ])
    print_table(
        rt,
        ["arm", "bags", "p50 peak", "max peak", "runs over 8", "pos changes/run",
         "p50 jump m", "final map off-lattice", "1st publication s", "empty-map ticks"],
    )
    print("\n   SPEC PREDICTION for the slot arm (src/navigation/planning/sign_slot_map.py,")
    print("   125 bags): worst peak 24 -> 7, runs over the physical max 32/125 -> 0/125,")
    print("   position changes per run 44.9 -> 3.1. Compare the two rows above against it.")


if __name__ == "__main__":
    main()
