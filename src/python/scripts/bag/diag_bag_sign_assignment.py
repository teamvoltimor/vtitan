r"""Can a RECOMPUTED assignment over the 24 legal cells replace the track map?

The shipped ``ObservedSignMap`` is free-form clustering: every observation either
joins a track or starts one, so the believed map holds 9-25 pillars on a track
that admits at most 8, and ~50% of believed positions match no legal lattice
point. Four dedup attempts AT PUBLICATION have been measured worse. This asks
the other question: what if the map never invents a position at all, and instead
ASSIGNS accumulated evidence to the 24 positions a pillar may legally stand at,
at most 2 per section, recomputed every tick rather than committed incrementally?

The prototype here is a SLOT map:

* Evidence is accumulated PER LEGAL CELL. Each observation snaps to its nearest
  legal cell; further than ``--accept-r`` from every one of them it makes no
  claim and is dropped (counted, since that drop rate is the whole question of
  whether the evidence in a bag can support an assignment at all).
* The assignment is the top ``--max-per-section`` cells of each section by
  accumulated confidence weight, above ``--min-evidence``. Recomputed from the
  totals every tick, so it is NOT first-past-the-post: a phantom that published
  early loses its cell the moment the real pillar out-evidences it. That is the
  specific failure of MAX_SIGNS_PER_SECTION (b9e5fffd), which was monotone.
* Publication is monotone in slot COUNT and non-monotone in slot CONTENT. The
  router keys ``_passed``/``_engaged`` by list index, so a slot is never removed;
  it is RE-POINTED at whichever cell now ranks there. A published sign therefore
  cannot vanish under a committed router -- but it can MOVE, and how often that
  happens is the churn number this script exists to produce.

ARMS. The bare assignment leaves two risks open, and each has a knob, so the
prototype runs SEVERAL assignment arms over ONE shared evidence accumulator in
one replay (the knobs change how evidence becomes slots, never the evidence):

* ``pool`` -- colour is decided over every cell within this radius of the
  assigned one, not by that cell alone. 0.0 is per-cell (the arm already
  measured); 0.25 pools the two LANES of one depth (0.20 m apart), which is
  exactly the pair a re-point swaps between; 0.55 also pools the neighbouring
  DEPTH row (0.50 m apart).
* ``margin`` -- hysteresis on the re-point. A challenger cell must out-weigh the
  incumbent by this FACTOR before the slot moves. 1.0 is pure ranking. The cost
  is measured directly as the DELAY between the tick the challenger first led
  and the tick the slot actually moved.
* ``laneflip`` -- an adversarial control, not a candidate. Every assigned cell
  is replaced by its lane partner (same section, same depth, other division
  line), i.e. lane is deliberately wrong 100% of the time. Since lane is the
  coin-flip axis AND the axis the pass-side rule keys on, the routing rate of
  this arm is what says whether the routing improvement is really about lane.

CONTROLS, because a churn number means nothing without a base rate:

* The SHIPPED map already rewrites a published sign position in place on every
  closer observation. That rate is measured in the same replay, on the same
  observations, and printed alongside. If the assignment churns less than what
  already ships, churn is not a reason to reject it.
* Cardinality: the assignment is bounded by 2*4 = 8 BY CONSTRUCTION. It is
  printed anyway, because a number that is not 8-bounded proves the path wrong.
* Routing error is judged exactly as ``diag_bag_pass_side.py`` judges it, with
  ``pass_side_lateral_axis``, for EVERY arm in the same pass over the same bag.
  Believed-count alone is not a verdict: a count that falls because real pillars
  merged is a regression wearing a disguise -- so the shipped map's passes are
  additionally split on whether the assignment covered them, and the UNCOVERED
  ones are decomposed into phantom / starved / outranked.

FIDELITY: detections are paired with the pose the CAMERA SAW FROM, as
``diag_bag_sign_track_birth.py`` does, not the pose at receipt.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_assignment.py \
        data/live/runs/run_2026090[6-9]_* data/live/runs/run_2026091[01]_*
"""

from __future__ import annotations

import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from shared.domain.enums import Axis, Direction  # noqa: E402
from shared.domain.models import Pose, SignColor  # noqa: E402

from scripts.bag.diag_bag_pass_geometry import classify_lattice  # noqa: E402
from scripts.bag.diag_bag_sign_track_birth import _frame_lag, _pose_at  # noqa: E402
from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles  # noqa: E402
from scripts.common.bag_io import (  # noqa: E402
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    settled_direction,
)
from scripts.common.stats import nearest_by_time  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import (  # noqa: E402
    detection_to_observation,
    legal_sign_positions,
)
from src.navigation.planning.sign_router import SignRouter, SignSpec  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402
from src.navigation.planning.waypoints import corridor_for_position  # noqa: E402

PHYSICAL_MAX_SIGNS = 8
COVER_M = 0.35
"""A shipped pass is COVERED when the assignment held a cell this close to it."""
CONVERGED_TICKS = 20
"""Map-ticks a slot must have held its cell before a re-point counts as LATE."""

_LEGAL = legal_sign_positions()
_CELL_SECTION: dict[tuple[float, float], str] = {}
_CELL_INNER: dict[tuple[float, float], bool] = {}
for _p in _LEGAL:
    _cls = classify_lattice(_p[0], _p[1])
    assert _cls is not None, f"legal lattice point {_p} does not classify"
    _CELL_SECTION[_p] = _cls[0]
    _CELL_INNER[_p] = _cls[1]

_LANE_PARTNER: dict[tuple[float, float], tuple[float, float]] = {}
for _p in _LEGAL:
    _mate = min(
        (
            q
            for q in _LEGAL
            if q != _p and _CELL_SECTION[q] == _CELL_SECTION[_p] and _CELL_INNER[q] != _CELL_INNER[_p]
        ),
        key=lambda q: math.dist(q, _p),
    )
    _LANE_PARTNER[_p] = _mate

_POOL_NEIGHBOURS: dict[tuple[float, tuple[float, float]], tuple[tuple[float, float], ...]] = {}


def _pool_cells(cell: tuple[float, float], pool_r: float) -> tuple[tuple[float, float], ...]:
    """Legal cells within ``pool_r`` of ``cell``, itself included. Memoised."""
    key = (pool_r, cell)
    hit = _POOL_NEIGHBOURS.get(key)
    if hit is None:
        hit = tuple(c for c in _LEGAL if math.dist(c, cell) <= pool_r)
        _POOL_NEIGHBOURS[key] = hit
    return hit


def _nearest_cell(x: float, y: float) -> tuple[tuple[float, float], float]:
    """Nearest legal cell and the distance to it. Never None; the caller gates."""
    best = min(_LEGAL, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
    return best, math.dist(best, (x, y))


@dataclass
class SlotMap:
    """Evidence per legal cell, and the assignment recomputed from it.

    Shared by every arm: the knobs change how evidence becomes slots, never the
    evidence, so one accumulator per bag keeps the arms exactly comparable.
    """

    accept_r: float
    min_evidence: float
    max_per_section: int
    weight: dict[tuple[float, float], float] = field(default_factory=lambda: defaultdict(float))
    votes: dict[tuple[float, float], dict[SignColor, float]] = field(
        default_factory=lambda: defaultdict(lambda: defaultdict(float))
    )
    claimed: int = 0
    dropped: int = 0
    drop_dists: list[float] = field(default_factory=list)

    def observe(self, observations, max_range_m: float, robot) -> None:  # noqa: ANN001
        for o in observations:
            if math.hypot(o.world_x_m - robot[0], o.world_y_m - robot[1]) > max_range_m:
                continue
            cell, dist = _nearest_cell(o.world_x_m, o.world_y_m)
            self.drop_dists.append(dist)
            if dist > self.accept_r:
                self.dropped += 1
                continue
            self.claimed += 1
            self.weight[cell] += o.confidence
            self.votes[cell][o.color] += o.confidence

    def ranked(self) -> dict[str, list[tuple[float, float]]]:
        """Qualifying cells per section, heaviest first. The arms share this."""
        by_section: dict[str, list[tuple[float, tuple[float, float]]]] = defaultdict(list)
        for cell, w in self.weight.items():
            if w >= self.min_evidence:
                by_section[_CELL_SECTION[cell]].append((w, cell))
        out: dict[str, list[tuple[float, float]]] = {}
        for section, cells in by_section.items():
            cells.sort(key=lambda t: (-t[0], t[1]))
            out[section] = [cell for _w, cell in cells]
        return out

    def colour_for(self, cell: tuple[float, float], pool_r: float) -> SignColor:
        """Winning colour vote at ``cell``, pooled over cells within ``pool_r``.

        ``pool_r`` 0.0 reads the cell alone -- the per-cell vote, which is
        already pooled over the WHOLE ROUND because the accumulator never
        forgets. A positive radius is the remedy for the one flip the per-cell
        vote cannot prevent: a slot RE-POINTED to a neighbour adopts that
        neighbour's vote, and a lane pair 0.20 m apart is one pillar's evidence
        split in two.
        """
        if pool_r <= 0.0:
            votes = self.votes[cell]
            return max(votes, key=lambda c: votes[c]) if votes else SignColor.UNKNOWN
        pooled: dict[SignColor, float] = defaultdict(float)
        for other in _pool_cells(cell, pool_r):
            for colour, w in self.votes.get(other, {}).items():
                pooled[colour] += w
        return max(pooled, key=lambda c: pooled[c]) if pooled else SignColor.UNKNOWN

    def assignment(self) -> dict[str, list[tuple[tuple[float, float], SignColor]]]:
        """Top cells per section, best first. At most ``max_per_section`` each."""
        return {
            section: [(c, self.colour_for(c, 0.0)) for c in cells[: self.max_per_section]]
            for section, cells in self.ranked().items()
        }

    def separation(self) -> list[tuple[str, float, float, float]]:
        """Per section: the last accepted weight, the first rejected one, ratio.

        The assignment is only meaningful if that cut is a real gap rather than
        a coin flip between two cells holding the same evidence.
        """
        by_section: dict[str, list[float]] = defaultdict(list)
        for cell, w in self.weight.items():
            by_section[_CELL_SECTION[cell]].append(w)
        rows = []
        for section, ws in by_section.items():
            ws.sort(reverse=True)
            k = self.max_per_section
            last = ws[k - 1] if len(ws) >= k else (ws[-1] if ws else 0.0)
            nxt = ws[k] if len(ws) > k else 0.0
            rows.append((section, last, nxt, last / nxt if nxt > 0 else float("inf")))
        return rows

    def lane_separation(self) -> list[tuple[float, float, float]]:
        """Per ASSIGNED cell: its weight, its lane partner's, the ratio.

        The cut measured by :meth:`separation` mixes three axes. This one
        isolates the axis the pass-side rule actually keys on: given that a
        pillar is in this section at this depth, how confidently does the
        evidence pick the INNER line over the OUTER one?
        """
        rows = []
        for _section, cells in self.ranked().items():
            for cell in cells[: self.max_per_section]:
                mine = self.weight[cell]
                theirs = self.weight.get(_LANE_PARTNER[cell], 0.0)
                rows.append((mine, theirs, mine / theirs if theirs > 0 else float("inf")))
        return rows


@dataclass
class ArmStats:
    """One assignment arm's churn, cardinality and verdicts over one bag."""

    slots: int = 0
    peak: int = 0
    ticks_over_max: int = 0
    repoints: int = 0
    repoints_committed: int = 0
    repoints_late: int = 0
    repoints_lane_only: int = 0
    repoints_depth: int = 0
    colour_flips: int = 0
    colour_flips_repoint: int = 0
    colour_flips_vote: int = 0
    colour_flips_committed: int = 0
    frozen_changes: int = 0
    repoints_passed: int = 0
    fresh_slots: int = 0
    delay_ticks: list[float] = field(default_factory=list)
    delay_s: list[float] = field(default_factory=list)
    stranded: int = 0
    stranded_s: list[float] = field(default_factory=list)
    verdicts: Counter = field(default_factory=Counter)


@dataclass
class Arm:
    """A slot map's publication policy, plus the router it publishes into."""

    label: str
    margin: float
    pool_r: float
    lane_flip: bool = False
    freeze_committed: bool = False
    new_slot_on_passed: bool = False
    """Never re-point an index the router has RETIRED: give the challenger a
    FRESH slot instead. ``_passed`` then keeps meaning 'this pillar is behind
    us', and a real pillar can never be hidden by inheriting a retired index.
    Count stays monotone and ``active_sign_count`` stays 8-bounded, because the
    retired index is excluded from it."""
    """Refuse to re-point or re-colour the slot the router is COMMITTED to.

    The rule-9.24.5 exposure is not churn in general, it is churn on the ONE
    slot the robot is currently steering around. A commitment is short and the
    evidence that would have moved the slot is still there when it ends, so
    this costs a delay rather than a decision."""
    router: SignRouter | None = None
    slot_index: dict[tuple[str, int], int] = field(default_factory=dict)
    slot_cell: dict[tuple[str, int], tuple[float, float]] = field(default_factory=dict)
    slot_age: dict[tuple[str, int], int] = field(default_factory=dict)
    lead_since: dict[tuple[str, int], tuple[int, float]] = field(default_factory=dict)
    """Slot -> (map-tick, bag time) at which a challenger FIRST out-weighed the
    incumbent. The hysteresis cost is exactly the distance from here to the
    re-point, and a slot still holding an entry at the end was STRANDED."""
    ever_cells: set[tuple[float, float]] = field(default_factory=set)
    store: dict = field(default_factory=dict)
    st: ArmStats = field(default_factory=ArmStats)

    def _choose(self, smap: SlotMap, cells: list[tuple[float, float]], section: str, mtick: int, rel: float):  # noqa: ANN202
        """Cells for this section's slots, oldest slot first, with hysteresis."""
        k = smap.max_per_section
        chosen: list[tuple[float, float]] = []
        used: set[tuple[float, float]] = set()
        rank = 0
        while (section, rank) in self.slot_index:
            inc = self.slot_cell[(section, rank)]
            best = next((c for c in cells if c not in used), None)
            if best is None:
                # Nothing qualifies any more; the slot keeps what it has rather
                # than pointing at nothing, which is what keeps COUNT monotone.
                chosen.append(inc)
                used.add(inc)
            elif best == inc:
                self.lead_since.pop((section, rank), None)
                chosen.append(inc)
                used.add(inc)
            elif inc in used:
                # An EARLIER slot of this section already holds the incumbent:
                # the two slots swapped ranks. Holding it here would point two
                # slots at one cell, so this slot must take the challenger.
                chosen.append(best)
                used.add(best)
            elif (
                smap.weight.get(inc, 0.0) >= smap.min_evidence
                and smap.weight[best] < self.margin * smap.weight[inc]
            ):
                self.lead_since.setdefault((section, rank), (mtick, rel))
                chosen.append(inc)
                used.add(inc)
            else:
                chosen.append(best)
                used.add(best)
            rank += 1
        for cell in cells:
            if len(chosen) >= k:
                break
            if cell not in used:
                chosen.append(cell)
                used.add(cell)
        return chosen

    def step(self, smap: SlotMap, ranked, mtick: int, rel: float) -> None:  # noqa: ANN001, C901
        router = self.router
        committed_idx = router._committed  # noqa: SLF001
        for section, cells in ranked.items():
            for rank, cell in enumerate(self._choose(smap, cells, section, mtick, rel)):
                key = (section, rank)
                published = _LANE_PARTNER[cell] if self.lane_flip else cell
                colour = smap.colour_for(cell, self.pool_r)
                spec = SignSpec(x=published[0], y=published[1], color=colour)
                self.ever_cells.add(published)
                idx = self.slot_index.get(key)
                if idx is None:
                    self.slot_index[key] = len(router._signs)  # noqa: SLF001
                    self.slot_cell[key] = cell
                    self.slot_age[key] = 0
                    router._signs.append(spec)  # noqa: SLF001
                    router._sign_corridors.append(router._corridor_for_spec(spec))  # noqa: SLF001
                    continue
                old = router._signs[idx]  # noqa: SLF001
                if self.freeze_committed and committed_idx == idx and old != spec:
                    self.st.frozen_changes += 1
                    self.slot_age[key] += 1
                    continue
                repointed = self.slot_cell[key] != cell
                if repointed:
                    was = self.slot_cell[key]
                    self.st.repoints += 1
                    if self.slot_age[key] >= CONVERGED_TICKS:
                        self.st.repoints_late += 1
                    if committed_idx == idx:
                        self.st.repoints_committed += 1
                    if idx in router._passed and self.new_slot_on_passed:  # noqa: SLF001
                        self.st.repoints_passed += 1
                        self.st.fresh_slots += 1
                        idx = len(router._signs)  # noqa: SLF001
                        self.slot_index[key] = idx
                        self.slot_cell[key] = cell
                        self.slot_age[key] = 0
                        router._signs.append(spec)  # noqa: SLF001
                        router._sign_corridors.append(router._corridor_for_spec(spec))  # noqa: SLF001
                        continue
                    if idx in router._passed:  # noqa: SLF001
                        # The router has RETIRED this index. Re-pointing it at a
                        # different pillar hides that pillar until the lap reset.
                        self.st.repoints_passed += 1
                    if cell == _LANE_PARTNER[was]:
                        self.st.repoints_lane_only += 1
                    else:
                        self.st.repoints_depth += 1
                    led = self.lead_since.pop(key, None)
                    if led is not None:
                        self.st.delay_ticks.append(mtick - led[0])
                        self.st.delay_s.append(rel - led[1])
                    self.slot_cell[key] = cell
                    self.slot_age[key] = 0
                else:
                    self.slot_age[key] += 1
                if old.color is not colour and old.color is not SignColor.UNKNOWN:
                    self.st.colour_flips += 1
                    if repointed:
                        self.st.colour_flips_repoint += 1
                    else:
                        self.st.colour_flips_vote += 1
                    if committed_idx == idx:
                        self.st.colour_flips_committed += 1
                if old != spec:
                    router._signs[idx] = spec  # noqa: SLF001
                    router._sign_corridors[idx] = router._corridor_for_spec(spec)  # noqa: SLF001

    def finish(self, rel_end: float) -> None:
        self.st.slots = len(self.slot_index)
        for _key, (_tick, t0) in self.lead_since.items():
            self.st.stranded += 1
            self.st.stranded_s.append(rel_end - t0)
        self.st.verdicts = _verdicts(self.store)


@dataclass
class RunStats:
    run: str
    map_ticks: int = 0
    base_peak: int = 0
    base_ticks_over_max: int = 0
    base_published: int = 0
    base_moves: int = 0
    base_moves_committed: int = 0
    base_move_dists: list[float] = field(default_factory=list)
    base_colour_flips: int = 0
    base_colour_flips_committed: int = 0
    claimed: int = 0
    dropped: int = 0
    drop_dists: list[float] = field(default_factory=list)
    separations: list[tuple[str, float, float, float]] = field(default_factory=list)
    lane_seps: list[tuple[float, float, float]] = field(default_factory=list)
    base_verdicts: Counter = field(default_factory=Counter)
    t_first_base: float | None = None
    t_first_slot: float | None = None
    passes_before_slot: int = 0
    base_ticks_blind: int = 0
    proto_ticks_blind: int = 0
    ticks: int = 0
    arms: dict[str, ArmStats] = field(default_factory=dict)
    cover: Counter = field(default_factory=Counter)
    """Shipped passes, by verdict, split on whether the assignment ever held a
    cell near that pillar. The denominator control: a believed count that falls
    because REAL pillars were dropped is a regression wearing a disguise, so the
    passes the shipped map got RIGHT are exactly what must stay covered."""
    uncovered: list[tuple[str, str, float, float]] = field(default_factory=list)
    """(verdict, reason, distance to nearest legal cell, weight there) for every
    shipped pass the assignment did NOT cover. The reason decomposes the 'lost
    capability' claim into phantom / starved / outranked."""


def _record_pass(store: dict, router, d, deformed, direction) -> None:  # noqa: ANN001
    """Keep the NEAREST tick per committed pillar, as diag_bag_pass_side does."""
    committed = router.committed_sign_position
    if committed is None:
        return
    rng = math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
    key = (round(committed.x, 1), round(committed.y, 1))
    if key in store and store[key][0] <= rng:
        return
    colour = next(
        (s.color for s in router.signs if abs(s.x - committed.x) < 1e-9 and abs(s.y - committed.y) < 1e-9),
        SignColor.UNKNOWN,
    )
    rule = pass_side_lateral_axis(d.current_corridor, colour, direction)
    store[key] = (rng, rule, committed, (d.pose_x, d.pose_y), deformed)


def _verdict_of(entry) -> str | None:  # noqa: ANN001
    _rng, rule, sign_pos, robot, deformed = entry
    if rule is None:
        return None
    axis, want = rule
    idx = 0 if axis is Axis.X else 1
    sign_axis = sign_pos.x if idx == 0 else sign_pos.y
    commanded = (1 if deformed[idx] - sign_axis > 0 else -1) * want
    achieved = (1 if robot[idx] - sign_axis > 0 else -1) * want
    return "routing" if commanded < 0 else "execution" if achieved < 0 else "ok"


def _verdicts(store: dict) -> Counter:
    out: Counter = Counter()
    for entry in store.values():
        verdict = _verdict_of(entry)
        if verdict is not None:
            out[verdict] += 1
    return out


def _replay(run, rows, frames, scans, tuning, latency, stamped, accept_r, min_evidence, max_per_section, arm_specs):  # noqa: ANN001, C901, PLR0912, PLR0915
    st = RunStats(run=run)
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    base = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    arms = [
        Arm(
            label=label,
            margin=margin,
            pool_r=pool_r,
            lane_flip=flip,
            freeze_committed=freeze,
            new_slot_on_passed=fresh,
            router=SignRouter(signs=[], direction=direction, discover=False, tuning=tuning),
        )
        for label, margin, pool_r, flip, freeze, fresh in arm_specs
    ]
    smap = SlotMap(accept_r=accept_r, min_evidence=min_evidence, max_per_section=max_per_section)
    max_range = tuning.sign_discovery.MAX_INGEST_RANGE_M
    min_conf = tuning.sign_router.MIN_CONFIDENCE

    poses: dict[float, Pose] = {}
    for rel, d in rows:
        if d.pose_x is not None and d.pose_y is not None and d.pose_yaw is not None:
            poses[rel] = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
    pose_times = sorted(poses)
    if not pose_times:
        return st
    first = poses[pose_times[0]]
    parking_corridor = corridor_for_position(first.x, first.y)

    scan_times = [t for t, _ in scans]
    frame_i = 0
    base_prev: dict[int, SignSpec] = {}
    base_store: dict = {}
    mtick = 0
    last_rel = 0.0

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
        pending: list[tuple[float, list[dict]]] = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            pending.append(frames[frame_i])
            frame_i += 1
        for frame_rel, payload in pending:
            seen_from = _pose_at(pose_times, poses, (frame_rel if stamped else rel) - latency) or pose
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
        if d.steer_target_x is None or d.steer_target_y is None:
            continue
        last_rel = rel
        if obs:
            mtick += 1
            st.map_ticks += 1

        # ---- BASELINE: the shipped map, ingesting inside deform_waypoint.
        base_deformed = base.deform_waypoint(
            (d.steer_target_x, d.steer_target_y), (d.pose_x, d.pose_y), d.pose_yaw, d.current_corridor, obs
        )
        base_committed_idx = base._committed  # noqa: SLF001
        for i, spec in enumerate(base.signs):
            old = base_prev.get(i)
            if old is not None and (abs(old.x - spec.x) > 1e-9 or abs(old.y - spec.y) > 1e-9):
                st.base_moves += 1
                st.base_move_dists.append(math.dist((old.x, old.y), (spec.x, spec.y)))
                if base_committed_idx == i:
                    st.base_moves_committed += 1
            if old is not None and old.color is not spec.color:
                st.base_colour_flips += 1
                if base_committed_idx == i:
                    st.base_colour_flips_committed += 1
            base_prev[i] = spec
        st.ticks += 1
        if not base.signs:
            st.base_ticks_blind += 1
        elif st.t_first_base is None:
            st.t_first_base = rel
        st.base_peak = max(st.base_peak, base.active_sign_count)
        if base.active_sign_count > PHYSICAL_MAX_SIGNS:
            st.base_ticks_over_max += 1
        _record_pass(base_store, base, d, base_deformed, base.direction)

        # ---- ARMS: one shared evidence accumulator, several publication policies.
        smap.observe(
            [o for o in obs if o.confidence >= min_conf and o.color in (SignColor.RED, SignColor.GREEN)],
            max_range,
            (d.pose_x, d.pose_y),
        )
        ranked = smap.ranked()
        if not arms[0].slot_index:
            st.proto_ticks_blind += 1
        elif st.t_first_slot is None:
            st.t_first_slot = rel
        for arm in arms:
            arm.step(smap, ranked, mtick, rel)
            deformed = arm.router.deform_waypoint(
                (d.steer_target_x, d.steer_target_y), (d.pose_x, d.pose_y), d.pose_yaw, d.current_corridor, None
            )
            arm.st.peak = max(arm.st.peak, arm.router.active_sign_count)
            if arm.router.active_sign_count > PHYSICAL_MAX_SIGNS:
                arm.st.ticks_over_max += 1
            _record_pass(arm.store, arm.router, d, deformed, arm.router.direction)

    st.base_published = len(base.signs)
    st.claimed = smap.claimed
    st.dropped = smap.dropped
    st.drop_dists = smap.drop_dists
    st.separations = smap.separation()
    st.lane_seps = smap.lane_separation()
    st.base_verdicts = _verdicts(base_store)
    for arm in arms:
        arm.finish(last_rel)
        st.arms[arm.label] = arm.st

    # ---- COVERAGE, against the REFERENCE arm (the one already measured).
    ref = arms[0]
    ever = ref.ever_cells
    assigned_now = {c for cells in smap.ranked().values() for c in cells[:max_per_section]}
    for entry in base_store.values():
        verdict = _verdict_of(entry)
        if verdict is None:
            continue
        sign_pos = entry[2]
        near = min((math.dist(c, (sign_pos.x, sign_pos.y)) for c in ever), default=99.0)
        st.cover[(verdict, near <= COVER_M)] += 1
        if near <= COVER_M:
            continue
        cell, dist = _nearest_cell(sign_pos.x, sign_pos.y)
        w = smap.weight.get(cell, 0.0)
        if dist > COVER_M:
            reason = "off-lattice"
        elif w < min_evidence:
            reason = "starved"
        elif cell in assigned_now:
            reason = "assigned-late"
        else:
            reason = "outranked"
        st.uncovered.append((verdict, reason, dist, w))
    return st


def _pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(q * len(s)))]


DEFAULT_ARMS = [
    # label,                  margin, pool_r, lane_flip, freeze_committed
    ("m1.0 pool0.00", 1.0, 0.00, False, False, False),
    ("m1.0 pool0.25", 1.0, 0.25, False, False, False),
    ("m1.0 pool0.55", 1.0, 0.55, False, False, False),
    ("m1.5 pool0.25", 1.5, 0.25, False, False, False),
    ("m2.0 pool0.25", 2.0, 0.25, False, False, False),
    ("m3.0 pool0.25", 3.0, 0.25, False, False, False),
    ("m1.0 p0.00 FREEZE", 1.0, 0.00, False, True, False),
    ("m1.5 p0.25 FREEZE", 1.5, 0.25, False, True, False),
    ("m2.0 p0.25 FREEZE", 2.0, 0.25, False, True, False),
    ("LANEFLIP ctrl", 1.0, 0.25, True, False, False),
    ("LANEFLIP m1.5 FRZ", 1.5, 0.25, True, True, False),
]


def main() -> None:  # noqa: C901, PLR0915
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    parser.add_argument("--accept-r", type=float, default=0.30,
                        help="An observation further than this from every legal cell makes NO claim.")
    parser.add_argument("--min-evidence", type=float, default=0.75,
                        help="Summed confidence a cell needs before assignment (MIN_HITS*MIN_CONFIDENCE).")
    parser.add_argument("--max-per-section", type=int, default=2, help="Rulebook cap.")
    parser.add_argument("--arm", action="append", default=[], metavar="MARGIN:POOL[:flip]",
                        help="Extra assignment arm. Repeatable. Replaces the default sweep.")
    args = parser.parse_args()

    arm_specs = DEFAULT_ARMS
    if args.arm:
        arm_specs = []
        for spec in args.arm:
            parts = spec.split(":")
            margin, pool = float(parts[0]), float(parts[1])
            flip = len(parts) > 2 and parts[2].startswith("f")
            freeze = len(parts) > 3 and parts[3].startswith("z")
            fresh = len(parts) > 4 and parts[4].startswith("n")
            arm_specs.append((f"m{margin:.1f} pool{pool:.2f}{' FLIP' if flip else ''}"
                              f"{' FRZ' if freeze else ''}{' NEW' if fresh else ''}",
                              margin, pool, flip, freeze, fresh))

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)
    print(f"== accept_r={args.accept_r}  min_evidence={args.min_evidence}  cap={args.max_per_section}/section"
          f"{'  overrides ' + str(overrides) if overrides else ''}")
    print(f"== arms: {', '.join(a[0] for a in arm_specs)}")

    stats: list[RunStats] = []
    skipped: Counter = Counter()
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
        try:
            stats.append(_replay(Path(bag).name.replace("run_", ""), rows, frames, scans, tuning,
                                 latency, source == "captured_at",
                                 args.accept_r, args.min_evidence, args.max_per_section, arm_specs))
        except Exception as exc:  # noqa: BLE001
            print(f"  !! {Path(bag).name}: {type(exc).__name__}: {exc}")
            skipped["replay_failed"] += 1

    empty = [s for s in stats if not s.arms]
    stats = [s for s in stats if s.arms]
    if empty:
        skipped["no_pose"] += len(empty)
    print(f"\n== CORPUS  {len(stats)} bags replayed, {sum(skipped.values())} skipped {dict(skipped)}")
    if not stats:
        return
    n = len(stats)

    drops = [d for s in stats for d in s.drop_dists]
    claimed = sum(s.claimed for s in stats)
    dropped = sum(s.dropped for s in stats)
    print("\n== CAN THE EVIDENCE SUPPORT AN ASSIGNMENT AT ALL")
    print(f"  observations claiming a cell:   {claimed}")
    print(f"  claiming NONE (> accept_r):     {dropped}   ({100 * dropped / max(1, claimed + dropped):.1f}%)")
    print(f"  distance to nearest legal cell: p50 {_pct(drops, 0.5):.3f}  p90 {_pct(drops, 0.9):.3f}  "
          f"max {max(drops, default=float('nan')):.3f}")

    seps = [r for s in stats for r in s.separations]
    clear = sum(1 for _sec, last, nxt, _r in seps if nxt == 0.0 or last >= 2 * nxt)
    print(f"\n== SEPARATION at the cut (last accepted cell vs first rejected), {len(seps)} section-runs")
    print(f"  cut is CLEAR (>=2x, or nothing below):  {clear}  ({100 * clear / max(1, len(seps)):.1f}%)")
    ratios = [min(r, 99.0) for _s, _l, _n, r in seps]
    print(f"  ratio p50 {_pct(ratios, 0.5):.2f}   p10 {_pct(ratios, 0.1):.2f}")

    lanes = [r for s in stats for r in s.lane_seps]
    lane_clear = sum(1 for _m, t, _r in lanes if t == 0.0)
    lane_2x = sum(1 for _m, t, r in lanes if t == 0.0 or r >= 2.0)
    lane_ratios = [min(r, 99.0) for _m, _t, r in lanes]
    print(f"\n== LANE separation ONLY (assigned cell vs its lane partner), {len(lanes)} assigned cells")
    print(f"  partner has NO evidence at all:        {lane_clear}  "
          f"({100 * lane_clear / max(1, len(lanes)):.1f}%)")
    print(f"  lane cut is CLEAR (>=2x or partner 0): {lane_2x}  "
          f"({100 * lane_2x / max(1, len(lanes)):.1f}%)")
    print(f"  ratio p50 {_pct(lane_ratios, 0.5):.2f}   p10 {_pct(lane_ratios, 0.1):.2f}")

    print("\n== CARDINALITY  (the track holds at most 8)")
    card_rows = [[
        "SHIPPED track map",
        max(s.base_peak for s in stats),
        sum(1 for s in stats if s.base_peak > PHYSICAL_MAX_SIGNS),
        round(sum(s.base_published for s in stats) / n, 1),
    ]]
    for label, *_ in arm_specs:
        card_rows.append([
            label,
            max(s.arms[label].peak for s in stats),
            sum(1 for s in stats if s.arms[label].peak > PHYSICAL_MAX_SIGNS),
            round(sum(s.arms[label].slots for s in stats) / n, 1),
        ])
    print_table(card_rows, ["map", "worst peak", "runs over max", "mean published"])

    print("\n== CHURN  (a published sign changing under the router)")
    base_moves = sum(s.base_moves for s in stats)
    churn_rows = [[
        "SHIPPED in-place move", base_moves, round(base_moves / n, 1),
        sum(s.base_moves_committed for s in stats), "-", "-",
        sum(s.base_colour_flips for s in stats), sum(s.base_colour_flips_committed for s in stats),
    ]]
    for label, *_ in arm_specs:
        a = [s.arms[label] for s in stats]
        rep = sum(x.repoints for x in a)
        churn_rows.append([
            label, rep, round(rep / n, 1),
            sum(x.repoints_committed for x in a),
            sum(x.repoints_lane_only for x in a),
            sum(x.repoints_late for x in a),
            sum(x.colour_flips for x in a),
            sum(x.colour_flips_committed for x in a),
        ])
    print_table(churn_rows, ["map", "re-points", "per run", "while COMMITTED", "lane-only",
                             f"late(>{CONVERGED_TICKS}t)", "colour flips", "flips COMMITTED"])

    print("\n== COLOUR FLIP DECOMPOSITION  (risk 1)")
    flip_rows = []
    for label, *_ in arm_specs:
        a = [s.arms[label] for s in stats]
        flip_rows.append([
            label, sum(x.colour_flips for x in a),
            sum(x.colour_flips_repoint for x in a),
            sum(x.colour_flips_vote for x in a),
            sum(x.colour_flips_committed for x in a),
        ])
    print_table(flip_rows, ["arm", "flips", "caused by RE-POINT", "same-cell VOTE change", "while COMMITTED"])
    print("  fresh slots allocated instead of re-pointing a RETIRED index: " + ", ".join(
        f"{label} {sum(s.arms[label].fresh_slots for s in stats)}" for label, *_ in arm_specs))
    print("  re-points landing on a slot the router already RETIRED (_passed): " + ", ".join(
        f"{label} {sum(s.arms[label].repoints_passed for s in stats)}" for label, *_ in arm_specs))
    print("  changes SUPPRESSED by the committed-slot freeze: " + ", ".join(
        f"{label} {sum(s.arms[label].frozen_changes for s in stats)}" for label, *_ in arm_specs
        if any(s.arms[label].frozen_changes for s in stats)))

    print("\n== HYSTERESIS COST  (risk 2): delay from the challenger LEADING to the slot MOVING")
    hyst_rows = []
    for label, *_ in arm_specs:
        a = [s.arms[label] for s in stats]
        dt = [x for st_ in a for x in st_.delay_s]
        dticks = [x for st_ in a for x in st_.delay_ticks]
        strand = [x for st_ in a for x in st_.stranded_s]
        hyst_rows.append([
            label, len(dt),
            f"{_pct(dt, 0.5):.2f}" if dt else "-",
            f"{_pct(dt, 0.9):.2f}" if dt else "-",
            f"{_pct(dticks, 0.5):.0f}" if dticks else "-",
            sum(st_.stranded for st_ in a),
            f"{_pct(strand, 0.5):.1f}" if strand else "-",
        ])
    print_table(hyst_rows, ["arm", "delayed re-points", "delay p50 s", "delay p90 s", "delay p50 ticks",
                            "STRANDED at end", "stranded p50 s"])

    cover: Counter = Counter()
    for s_ in stats:
        cover.update(s_.cover)
    print(f"\n== COVERAGE of the SHIPPED map's passes by the assignment (within {COVER_M} m)")
    for verdict in ("ok", "execution", "routing"):
        yes = cover[(verdict, True)]
        no = cover[(verdict, False)]
        print(f"  shipped {verdict:>9}: covered {yes:5d}   NOT covered {no:5d}   "
              f"({100 * yes / max(1, yes + no):.1f}% covered)")

    print("\n== WHAT THE UNCOVERED SHIPPED PASSES ARE")
    unc = [u for s in stats for u in s.uncovered]
    by_reason: Counter = Counter()
    for verdict, reason, _dist, _w in unc:
        by_reason[(verdict, reason)] += 1
    reasons = ("off-lattice", "starved", "outranked", "assigned-late")
    print_table(
        [[grp] + [by_reason[(grp, r)] for r in reasons] + [sum(by_reason[(grp, r)] for r in reasons)]
         for grp in ("ok", "execution", "routing")],
        ["shipped pass was", *reasons, "total"],
    )
    for reason in reasons:
        ds = [d for _v, r, d, _w in unc if r == reason]
        if ds:
            print(f"  {reason:>13}: dist to nearest legal cell p50 {_pct(ds, 0.5):.2f}  p90 {_pct(ds, 0.9):.2f}")

    print("")
    print("== COLD START  (what each map believes before any evidence clears its floor)")
    tb = [s.t_first_base for s in stats if s.t_first_base is not None]
    tp = [s.t_first_slot for s in stats if s.t_first_slot is not None]
    print(f"  first SHIPPED publication:  {len(tb)}/{n} runs   p50 {_pct(tb, 0.5):.1f} s  p90 {_pct(tb, 0.9):.1f} s")
    print(f"  first ASSIGNMENT slot:      {len(tp)}/{n} runs   p50 {_pct(tp, 0.5):.1f} s  p90 {_pct(tp, 0.9):.1f} s")
    bb = sum(s.base_ticks_blind for s in stats)
    pb = sum(s.proto_ticks_blind for s in stats)
    tt = sum(s.ticks for s in stats)
    print(f"  ticks with an EMPTY map (no sign at all, no deformation attempted): "
          f"shipped {bb} ({100 * bb / max(1, tt):.1f}%)   assignment {pb} ({100 * pb / max(1, tt):.1f}%)")
    print("")
    print("\n== ROUTING ERROR, judged identically for every map on the same bags")
    rows_out = []
    tot: Counter = Counter()
    for s in stats:
        tot.update(s.base_verdicts)
    nn = sum(tot.values())
    rows_out.append(["SHIPPED track map", nn, tot["routing"], f"{100 * tot['routing'] / max(1, nn):.1f}%",
                     tot["execution"], tot["ok"]])
    for label, *_ in arm_specs:
        tot = Counter()
        for s in stats:
            tot.update(s.arms[label].verdicts)
        nn = sum(tot.values())
        rows_out.append([label, nn, tot["routing"], f"{100 * tot['routing'] / max(1, nn):.1f}%",
                         tot["execution"], tot["ok"]])
    print_table(rows_out, ["map", "passes", "routing", "routing rate", "execution", "ok"])


if __name__ == "__main__":
    main()
