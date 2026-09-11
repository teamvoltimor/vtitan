r"""WHERE is a duplicate sign track BORN, and is the corridor flap the cause?

The believed sign map invents pillars: 9-23 believed signs on a track holding at
most 8, and 56% of believed positions matching no legal lattice point. Four
attempts to deduplicate AT PUBLICATION have all been measured worse. The defect
is upstream, so this instruments the TRACK LIFECYCLE instead of the output.

With the shipped tuning (``snap_to_lattice_m = 0``, ``sign_lidar_propose =
false``) a track can be born in exactly one place: ``ObservedSignMap._fold``,
when ``_nearest_track`` returns None. That happens for exactly two reasons, and
they are separated here:

* **CORRIDOR-GATED SPLIT** -- a track DOES exist within ``association_dist_m``
  of the new observation, but it was created under a different settled ROBOT
  corridor, so ``_nearest_track`` refuses it. This is the mechanism the
  corridor flap would drive.
* **NO NEIGHBOUR** -- no track within the association radius at all. Sub-split
  by how far the nearest one is: 0.25-0.50 m is the believed position WANDERING
  out of its own association radius; beyond 0.50 m is a first sighting.

A COLOUR FLIP CANNOT BIRTH A TRACK -- association is position-and-corridor
only, colour is a vote folded after the association decision. It is reported as
a descriptor of the split (does the new track disagree with the neighbour it was
refused?) rather than as a cause, because the code cannot make it one.

THE BARRIER (fixed in b3d7a038) is accounted for, not re-found: the replay
reproduces the gateway's ``barrier_possible`` test from the run's own start
corridor, and every birth carries a flag saying whether the aspect gate was
DISABLED for the detection that created it.

FALSIFICATION CONTROL for the flap claim: a birth "coinciding" with a flap means
nothing unless flaps are rare. The base rate -- the share of ALL ticks that sit
within the same window of a flap -- is printed next to it, and the ratio of the
two is the lift. A lift near 1.0 refutes the flap hypothesis.

PATH CONTROLS, so a wrong path cannot masquerade as a null: the share of ticks
believing more than 8 signs (independently measured at 86%) and the share of
believed positions off the legal lattice (56.3%) are printed first.

FIDELITY. Unlike ``diag_bag_pass_side.py``, this pairs each detection frame with
the pose the CAMERA SAW FROM (``captured_at`` when the payload carries one,
else stepped back by ``VISION_LATENCY_S``), which is what
``ROS2HardwareGateway._pose_when_seen`` does on the robot. Pairing at receipt
moves the believed position by most of a sign's lateral offset and would
manufacture births that the robot never had.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_track_birth.py \
        data/live/runs/run_2026090[6-9]_* data/live/runs/run_2026091[01]_*
    ... --set ROBOT_CORRIDOR_FLIP_TICKS=20
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402

from scripts.bag.diag_bag_pass_geometry import classify_lattice  # noqa: E402
from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    decode_detections,
    load_nav_debug_rows,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.stats import nearest_by_time  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import detection_to_observation  # noqa: E402
from src.navigation.planning.sign_router import SignRouter  # noqa: E402
from src.navigation.planning.waypoints import corridor_for_position  # noqa: E402

PHYSICAL_MAX_SIGNS = 8
"""Rulebook maximum pillars on the whole track."""

WANDER_M = 0.50
"""Nearest neighbour inside this but outside association_dist_m: the believed
position moved further than its own association radius between observations."""

FLAP_WINDOWS = (1, 3, 5, 10)
"""Tick windows the birth/flap coincidence is reported over."""


@dataclass
class Birth:
    """One track, and what created it."""

    run: str
    tick: int
    cause: str
    nearest_m: float | None
    nearest_corridor: str
    corridor: str
    colour_disagrees: bool
    barrier_gate_off: bool
    ticks_since_flap: int | None
    published: bool = False
    excess: bool = False
    lattice: str | None = None


@dataclass
class RunStats:
    """Per-run counters the pooled tables are built from."""

    run: str
    ticks: int = 0
    map_ticks: int = 0
    """Ticks the map was actually FED. ``observe`` returns early on an empty
    frame, so the settle counter and every birth live on these ticks only, and a
    base rate taken over all ticks would understate the chance coincidence."""
    ticks_over_max: int = 0
    ingests: int = 0
    stale_ingests: int = 0
    """Ingests that re-folded a frame this run had already folded."""
    rec_ticks: int = 0
    rec_over_max: int = 0
    rec_peak: int = 0
    rec_committed: int = 0
    rec_committed_illegal: int = 0
    peak_signs: int = 0
    raw_flips: int = 0
    settled_flips: int = 0
    debug_corridor_flips: int = 0
    tracks: int = 0
    published: int = 0
    published_illegal: int = 0
    published_duplicate: int = 0
    distinct_cells: int = 0
    flap_ticks: set[int] = field(default_factory=set)
    births: list[Birth] = field(default_factory=list)


def _frame_lag(frames: list[tuple[float, list[dict]]], fallback: float) -> tuple[float, str]:
    """Capture-to-receipt lag for this bag, and where it came from.

    ``captured_at`` is a wall clock the bag's own relative times cannot be
    compared to directly, so the lag is taken as the MEDIAN difference between
    consecutive frames' receipt times and capture stamps, anchored on the first
    frame. When no payload carries the key, the shipped fallback stands -- the
    same order ``_pose_when_seen`` resolves it in.
    """
    stamps: list[float] = []
    anchor: float | None = None
    for rel, payload in frames:
        for d in payload:
            cap = d.get("captured_at")
            if cap is None:
                continue
            if anchor is None:
                anchor = float(cap) - rel
            stamps.append(rel - (float(cap) - anchor))
            break
    if len(stamps) < 10:
        return fallback, "VISION_LATENCY_S"
    stamps.sort()
    # The anchor forces the first sample to 0, so the median of the REST is the
    # drift of capture against receipt; the absolute lag is unrecoverable from a
    # bag that stamps receipt only. Fall back whenever that drift is degenerate.
    med = stamps[len(stamps) // 2]
    if not 0.0 < med < 3.0:
        return fallback, "VISION_LATENCY_S (captured_at unusable)"
    return med, "captured_at"


def _pose_at(pose_times: list[float], poses: dict[float, Pose], target: float) -> Pose | None:
    """Nearest recorded pose at or before ``target``; the earliest if none is."""
    if not pose_times:
        return None
    lo, hi = 0, len(pose_times) - 1
    if target <= pose_times[0]:
        return poses[pose_times[0]]
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if pose_times[mid] <= target:
            lo = mid
        else:
            hi = mid - 1
    return poses[pose_times[lo]]


def _replay(run: str, rows, frames, scans, tuning, latency: float, stale: bool, stamped: bool, legacy_gate: bool = False) -> RunStats:  # noqa: ANN001, C901
    """Replay the shipped router over one bag and record every track birth."""
    st = RunStats(run=run)
    direction = settled_direction(rows) or None
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    smap = router._sign_map  # noqa: SLF001  (the object under test)
    assert smap is not None, "discover=True must build an ObservedSignMap"

    poses: dict[float, Pose] = {}
    for rel, d in rows:
        if d.pose_x is not None and d.pose_y is not None and d.pose_yaw is not None:
            poses[rel] = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
    pose_times = sorted(poses)
    if not pose_times:
        return st

    # The lot sits in the corridor the robot STARTED in -- the same fact
    # `set_parking_corridor` is given on the robot.
    first = poses[pose_times[0]]
    parking_corridor = corridor_for_position(first.x, first.y)

    scan_times = [t for t, _ in scans]
    frame_i = 0
    latest_frame: list[tuple[float, list[dict]] | None] = [None]
    last_ingested: list[float] = [-1.0]
    prev_raw = prev_settled = prev_debug = object()
    last_flap_tick: int | None = None
    tick = 0
    mtick = 0

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        tick += 1
        st.ticks += 1

        # RECORDED controls, straight off the wire -- independent of this
        # replay, so a wrong replay path cannot hide behind them.
        if d.active_sign_count is not None:
            st.rec_ticks += 1
            st.rec_peak = max(st.rec_peak, d.active_sign_count)
            if d.active_sign_count > PHYSICAL_MAX_SIGNS:
                st.rec_over_max += 1
        if d.committed_sign_x_m is not None and d.committed_sign_y_m is not None:
            st.rec_committed += 1
            if classify_lattice(d.committed_sign_x_m, d.committed_sign_y_m) is None:
                st.rec_committed_illegal += 1
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)

        raw = corridor_for_position(pose.x, pose.y)
        if prev_raw is not object() and raw != prev_raw:
            st.raw_flips += 1
        prev_raw = raw
        if prev_debug is not object() and d.current_corridor != prev_debug:
            st.debug_corridor_flips += 1
        prev_debug = d.current_corridor

        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )

        obs = []
        gate_off = False
        pending: list[tuple[float, list[dict]]] = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            pending.append(frames[frame_i])
            frame_i += 1
        if stale:
            # PRODUCTION SHAPE. `_latest_detections` is never cleared once
            # consumed, so the navigator folds the SAME frame again on every
            # tick until a new one arrives -- and `_pose_when_seen` falls back
            # to ``now - VISION_LATENCY_S`` when the payload carries no
            # ``captured_at``, so each refold reprojects that frame from a pose
            # that has MOVED. Only a stamped payload pins it.
            if pending:
                latest_frame[0] = pending[-1]
            if latest_frame[0] is not None:
                frame_rel, payload = latest_frame[0]
                st.ingests += 1
                if frame_rel == last_ingested[0]:
                    st.stale_ingests += 1
                last_ingested[0] = frame_rel
                pending = [(frame_rel if stamped else rel, payload)]
            else:
                pending = []
        for frame_rel, payload in pending:
            seen_from = _pose_at(pose_times, poses, frame_rel - latency) or pose
            corridor_now = corridor_for_position(seen_from.x, seen_from.y)
            # ``legacy_gate`` restores the pre-b3d7a038 test, which treated a
            # corridor boundary as a sight line and so DISABLED the pillar-shape
            # gate everywhere but the lot's own corridor. Deployed on the Pi for
            # every bag in this corpus.
            barrier_possible = (
                parking_corridor is None
                or corridor_now is None
                or corridor_now == parking_corridor
                or (not legacy_gate and corridor_now in parking_corridor.neighbours)
            )
            gate_off = gate_off or not barrier_possible
            for det in decode_detections(payload):
                o = detection_to_observation(
                    det, seen_from, tuning, ranges, angles, barrier_possible=barrier_possible
                )
                if o is not None:
                    obs.append(o)

        if d.steer_target_x is None or d.steer_target_y is None:
            continue

        if obs:
            mtick += 1
            st.map_ticks += 1

        before = [(t.x, t.y, t.corridor, t.color) for t in smap._tracks]  # noqa: SLF001
        settled_before = smap._robot_corridor  # noqa: SLF001

        router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )

        settled_after = smap._robot_corridor  # noqa: SLF001
        if prev_settled is not object() and settled_after != settled_before:
            st.settled_flips += 1
            st.flap_ticks.add(mtick)
            last_flap_tick = mtick
        prev_settled = settled_after

        st.peak_signs = max(st.peak_signs, router.active_sign_count)
        if router.active_sign_count > PHYSICAL_MAX_SIGNS:
            st.ticks_over_max += 1

        for new in smap._tracks[len(before) :]:  # noqa: SLF001
            near_d: float | None = None
            near_same_corridor = False
            near_colour = SignColor.UNKNOWN
            for x, y, corridor, colour in before:
                dist = math.hypot(new.x - x, new.y - y)
                if near_d is None or dist < near_d:
                    near_d, near_same_corridor, near_colour = dist, corridor == new.corridor, colour
            if near_d is not None and near_d <= smap._association_dist_m and not near_same_corridor:  # noqa: SLF001
                cause = "corridor_gated_split"
            elif near_d is not None and near_d <= WANDER_M:
                cause = "position_wander" if near_same_corridor else "wander_cross_corridor"
            else:
                cause = "first_sighting"
            st.births.append(
                Birth(
                    run=run,
                    tick=mtick,
                    cause=cause,
                    nearest_m=near_d,
                    nearest_corridor="same" if near_same_corridor else "other",
                    corridor=str(new.corridor),
                    colour_disagrees=near_colour is not SignColor.UNKNOWN and near_colour is not new.color,
                    barrier_gate_off=gate_off,
                    ticks_since_flap=None if last_flap_tick is None else mtick - last_flap_tick,
                )
            )

    tracks = smap._tracks  # noqa: SLF001
    st.tracks = len(tracks)
    birth_by_index = {i: b for i, b in enumerate(st.births)}
    cells: dict[str, int] = {}
    for i, t in enumerate(tracks):
        if t.published_index is None:
            continue
        st.published += 1
        cell = classify_lattice(t.x, t.y)
        label = None if cell is None else f"{cell[0]}-{'inner' if cell[1] else 'outer'}-{round(t.x, 1)},{round(t.y, 1)}"
        birth = birth_by_index.get(i)
        if birth is not None:
            birth.published = True
            birth.lattice = label
        if label is None:
            st.published_illegal += 1
            if birth is not None:
                birth.excess = True
            continue
        cells[label] = cells.get(label, 0) + 1
        if cells[label] > 1:
            st.published_duplicate += 1
            if birth is not None:
                birth.excess = True
    st.distinct_cells = len(cells)
    return st




def _wire_publications(bag_dirs) -> None:  # noqa: ANN001, C901
    """The SAME question asked of the WIRE, with no replay in the loop.

    ``active_sign_count`` is ``len(signs) - len(passed)`` and ``passed`` only
    grows, so any INCREASE in it is a publication -- a new believed pillar,
    observable directly in the bag. ``current_corridor`` is on the wire too.
    So the flap hypothesis can be tested on the robot's OWN run rather than on
    a reconstruction of it, which matters because the replay of these bags is
    a tamer map than the robot had (see the controls).

    The base rate is again the share of ticks that merely SIT near a flip; the
    ratio of the two is the lift, and a lift near 1.0 refutes the flap.
    """
    tot_ticks = tot_pubs = 0
    flip_total = 0
    near_pub = {w: 0 for w in FLAP_WINDOWS}
    near_tick = {w: 0 for w in FLAP_WINDOWS}
    runs = 0
    peaks: list[int] = []
    finals: list[int] = []
    skipped: Counter[str] = Counter()
    for bag in bag_dirs:
        try:
            rows, _ = load_nav_debug_rows(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            skipped[type(exc).__name__] += 1
            continue
        counts = [(i, d) for i, (_rel, d) in enumerate(rows) if d.active_sign_count is not None]
        if len(counts) < 50:
            continue
        runs += 1
        n = len(counts)
        tot_ticks += n
        peaks.append(max(d.active_sign_count or 0 for _i, d in counts))
        finals.append(counts[-1][1].active_sign_count or 0)
        flips: list[int] = []
        pubs: list[int] = []
        prev_c = prev_corr = None
        for j, (_i, d) in enumerate(counts):
            if prev_c is not None and (d.active_sign_count or 0) > prev_c:
                pubs.append(j)
            prev_c = d.active_sign_count or 0
            if prev_corr is not None and d.current_corridor != prev_corr:
                flips.append(j)
            prev_corr = d.current_corridor
        flip_total += len(flips)
        tot_pubs += len(pubs)
        for w in FLAP_WINDOWS:
            covered: set[int] = set()
            for t in flips:
                covered.update(range(max(t - w, 0), min(t + w, n - 1) + 1))
            near_tick[w] += len(covered)
            near_pub[w] += sum(1 for pth in pubs if pth in covered)

    print("")
    print(f"== WIRE-ONLY: publications vs corridor flips  ({runs} runs, skipped={dict(skipped)})")
    if not runs:
        return
    peaks.sort()
    finals.sort()
    print(f"   peak active_sign_count per run: p50={peaks[len(peaks) // 2]} p90={peaks[int(0.9 * len(peaks))]} max={peaks[-1]}  (physical max {PHYSICAL_MAX_SIGNS})")
    print(f"   final active_sign_count per run: p50={finals[len(finals) // 2]} max={finals[-1]}")
    print(f"   publications (active_sign_count increases): {tot_pubs} over {tot_ticks} ticks")
    print(f"   corridor flips: {flip_total} ({flip_total / runs:.1f} per run)")
    for w in FLAP_WINDOWS:
        p_pub = near_pub[w] / max(tot_pubs, 1)
        p_base = near_tick[w] / max(tot_ticks, 1)
        lift = p_pub / p_base if p_base else float("nan")
        print(f"   within +/-{w:2d} ticks of a flip: publications {100 * p_pub:5.1f}%   base {100 * p_base:5.1f}%   lift {lift:.2f}x")


def main() -> None:  # noqa: C901
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    parser.add_argument(
        "--stale-reingest",
        action="store_true",
        help="Reproduce the SHIPPED gateway: refold the last detection frame on every tick.",
    )
    parser.add_argument(
        "--legacy-barrier-gate",
        action="store_true",
        help="Restore the pre-b3d7a038 barrier test, which is what every bag in the archive ran.",
    )
    parser.add_argument(
        "--wire-only",
        action="store_true",
        help="Skip the replay: test the flap against the publications RECORDED on /nav_debug.",
    )
    args = parser.parse_args()

    if args.wire_only:
        _wire_publications(args.bag_dirs)
        return

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)
    if overrides:
        print(f"== sign_discovery overrides: {overrides}")
    if args.legacy_barrier_gate:
        print("== barrier gate: LEGACY (pre-b3d7a038)")
    print(f"== detection ingest mode: {'STALE RE-INGEST (production shape)' if args.stale_reingest else 'each frame once'}")

    stats: list[RunStats] = []
    skipped: Counter[str] = Counter()
    lag_source: Counter[str] = Counter()
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError, PermissionError) as exc:
            skipped[type(exc).__name__] += 1
            continue
        if not frames:
            skipped["no_detections"] += 1
            continue
        latency, source = _frame_lag(frames, tuning.sign_discovery.VISION_LATENCY_S)
        lag_source[source] += 1
        try:
            stats.append(
                _replay(
                    Path(bag).name.replace("run_", ""),
                    rows, frames, scans, tuning, latency,
                    args.stale_reingest,
                    source == "captured_at",
                    args.legacy_barrier_gate,
                )
            )
        except Exception as exc:  # noqa: BLE001
            print(f"  !! {Path(bag).name}: replay failed {type(exc).__name__}: {exc}")
            skipped["replay_failed"] += 1

    print(f"\n== CORPUS  {len(stats)} bags replayed, {sum(skipped.values())} skipped {dict(skipped)}")
    print(f"   detection/pose pairing source: {dict(lag_source)}")
    if not stats:
        print("   nothing to report")
        return

    ticks = sum(s.ticks for s in stats)
    map_ticks = sum(s.map_ticks for s in stats)
    over = sum(s.ticks_over_max for s in stats)
    pub = sum(s.published for s in stats)
    illegal = sum(s.published_illegal for s in stats)
    print("\n== CONTROLS (a wrong path shows up here, not as a null)")
    print(f"   ticks believing more than {PHYSICAL_MAX_SIGNS} signs: {over}/{ticks} ({100 * over / max(ticks, 1):.1f}%)  [expect ~86%]")
    print(f"   published positions off the legal lattice:  {illegal}/{max(pub, 1)} ({100 * illegal / max(pub, 1):.1f}%)  [expect ~56%]")
    peaks = sorted(s.peak_signs for s in stats)
    print(f"   peak believed signs per run: p50={peaks[len(peaks) // 2]} max={peaks[-1]}  [expect 9-23]")
    rec_t = sum(s.rec_ticks for s in stats)
    rec_o = sum(s.rec_over_max for s in stats)
    rec_c = sum(s.rec_committed for s in stats)
    rec_ci = sum(s.rec_committed_illegal for s in stats)
    rpeaks = sorted(s.rec_peak for s in stats)
    print("   -- and the same three RECORDED on the wire, which this replay must reproduce:")
    print(f"      /nav_debug ticks with active_sign_count > {PHYSICAL_MAX_SIGNS}: {rec_o}/{rec_t} ({100 * rec_o / max(rec_t, 1):.1f}%)")
    print(f"      recorded peak active_sign_count per run: p50={rpeaks[len(rpeaks) // 2]} max={rpeaks[-1]}")
    print(f"      committed_sign positions off the lattice: {rec_ci}/{max(rec_c, 1)} ({100 * rec_ci / max(rec_c, 1):.1f}%)")
    print(f"   map-fed ticks (observe actually ran): {map_ticks}/{ticks} ({100 * map_ticks / max(ticks, 1):.1f}%)")
    ing = sum(s.ingests for s in stats)
    stale_ing = sum(s.stale_ingests for s in stats)
    if ing:
        print(f"   frame ingests: {ing}, of which RE-folds of an already-folded frame: {stale_ing} ({100 * stale_ing / ing:.1f}%)")

    print("\n== CORRIDOR FLAP per run (medians over runs)")
    def med(values: list[float]) -> float:
        vs = sorted(values)
        return vs[len(vs) // 2] if vs else 0.0

    print(f"   /nav_debug current_corridor changes: p50={med([s.debug_corridor_flips for s in stats]):.0f} max={max(s.debug_corridor_flips for s in stats)}")
    print(f"   raw corridor_for_position changes:   p50={med([s.raw_flips for s in stats]):.0f} max={max(s.raw_flips for s in stats)}")
    print(f"   SETTLED map corridor changes:        p50={med([s.settled_flips for s in stats]):.0f} max={max(s.settled_flips for s in stats)}")

    births = [b for s in stats for b in s.births]
    print(f"\n== TRACK BIRTHS  ({len(births)} tracks born, {pub} published, {sum(s.published_duplicate for s in stats)} duplicate + {illegal} illegal = excess)")
    table = []
    for cause in ("corridor_gated_split", "position_wander", "wander_cross_corridor", "first_sighting"):
        group = [b for b in births if b.cause == cause]
        if not group:
            continue
        published = [b for b in group if b.published]
        excess = [b for b in group if b.excess]
        table.append(
            [
                cause,
                len(group),
                f"{100 * len(group) / len(births):.1f}%",
                len(published),
                len(excess),
                f"{100 * len(excess) / max(len(group), 1):.1f}%",
                sum(1 for b in group if b.barrier_gate_off),
                sum(1 for b in group if b.colour_disagrees),
            ]
        )
    print_table(table, ["birth cause", "births", "share", "published", "EXCESS", "excess rate", "gate off", "colour differs"])

    print("   how far the refused/absent neighbour was (m), per cause:")
    for cause in ("corridor_gated_split", "position_wander", "wander_cross_corridor"):
        ds = sorted(b.nearest_m for b in births if b.cause == cause and b.nearest_m is not None)
        if not ds:
            continue
        print(
            f"     {cause:<22} p10={ds[len(ds) // 10]:.3f} p50={ds[len(ds) // 2]:.3f} "
            f"p90={ds[int(0.9 * len(ds))]:.3f}   under 0.10 m: {100 * sum(1 for v in ds if v < 0.10) / len(ds):.0f}%"
        )

    excess_all = [b for b in births if b.excess]
    if excess_all:
        print(f"\n   EXCESS pillars by birth cause ({len(excess_all)} total):")
        for cause, n in Counter(b.cause for b in excess_all).most_common():
            print(f"     {cause:<24} {n:5d}  ({100 * n / len(excess_all):.1f}%)")

    print("\n== FLAP COINCIDENCE  (birth rate near a flap vs the base rate of ticks near one)")
    for w in FLAP_WINDOWS:
        near_birth = sum(
            1 for b in births if b.ticks_since_flap is not None and 0 <= b.ticks_since_flap <= w
        )
        near_ticks = 0
        for s in stats:
            covered = set()
            for t in s.flap_ticks:
                covered.update(range(t, min(t + w, s.map_ticks) + 1))
            near_ticks += len(covered)
        p_birth = near_birth / max(len(births), 1)
        p_base = near_ticks / max(map_ticks, 1)
        lift = p_birth / p_base if p_base > 0 else float("nan")
        print(f"   within {w:2d} ticks: births {100 * p_birth:5.1f}%   base {100 * p_base:5.1f}%   lift {lift:.2f}x")
        # PER CAUSE, because a corridor change also means the robot has driven
        # into a part of the track it has not seen: a first sighting SHOULD
        # follow one. Only the gated split is evidence for the flap.
        for cause in ("corridor_gated_split", "position_wander", "first_sighting"):
            group = [b for b in births if b.cause == cause]
            if not group:
                continue
            k = sum(1 for b in group if b.ticks_since_flap is not None and 0 <= b.ticks_since_flap <= w)
            pc = k / len(group)
            print(f"        {cause:<22} {100 * pc:5.1f}%   lift {pc / p_base if p_base else float('nan'):.2f}x  (n={len(group)})")
    print("   lift ~1.0 REFUTES the flap; a lift well above 1 is the flap creating tracks.")

    print("\n== PER RUN")
    print_table(
        [
            [
                s.run,
                s.ticks,
                s.peak_signs,
                s.debug_corridor_flips,
                s.settled_flips,
                len(s.births),
                s.published,
                s.published_illegal,
                s.published_duplicate,
                s.distinct_cells,
            ]
            for s in stats
        ],
        ["run", "ticks", "peak", "dbg flips", "settled flips", "births", "pub", "illegal", "dup", "cells"],
    )


if __name__ == "__main__":
    main()
