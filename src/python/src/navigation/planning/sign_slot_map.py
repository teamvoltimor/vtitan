"""A sign map that assigns evidence to the RULEBOOK's cells instead of clustering freely.

``ObservedSignMap`` tracks whatever the camera reports and tries to decide, from
position alone, which reports describe the same pillar. That question is not
answerable here: the believed position carries 0.15-0.25 m of error while two
DISTINCT legal pillars sit 0.20 m apart across the lane pair and 0.50 m apart
along a section. So the map invents pillars -- 9 to 25 believed on a track that
physically holds at most 8, with roughly half of all believed positions matching
no legal cell at all -- and that single defect costs half the sample of every
geometric measurement this project makes.

FOUR attempts to repair it at PUBLICATION time are on file and all failed,
because each one still had to answer "is this the same object" from position:

* position-keyed merging, two variants: collisions 209 and 231 against a 202
  baseline;
* ``SNAP_TO_LATTICE_M`` quantisation at 0.40: routing errors appeared to halve
  while 264 passes vanished from the denominator and the PEAK believed count
  ROSE from 26 to 40;
* a per-section cardinality cap: it retains REAL signs, because a phantom that
  published first holds the slot. Being monotone is what made it safe and is
  exactly what stops the real pillar entering later.

This module asks a different question. The rulebook says a pillar stands on one
of 24 legal cells -- six per section, at 0.4 m from the outer wall or 0.4 m from
the inner one, at depths 1.0/1.5/2.0 m -- and that a section holds AT MOST TWO.
So the map is not a clustering problem, it is a CONSTRAINED ASSIGNMENT: which
two of six cells does each section's evidence support? Evidence accumulates per
cell and the assignment is recomputed every tick, which is the axis the
cardinality cap could not move.

MEASURED over 125 bags against the shipped map on identical observations:

| | shipped | slots |
|---|---|---|
| routing error | 23.3% | **15.0%** |
| worst peak believed | 24 | **7** |
| runs over the physical max | 32/125 | **0/125** |
| position changes per run | 44.9 | **3.1** |
| re-points / colour flips while COMMITTED | 1062 / 66 | **0 / 0** |

Two results worth carrying, because both are counter-intuitive:

**The win is CARDINALITY, not lane accuracy.** Which of the two lanes a pillar
lands in is close to a coin flip (the lane partner has zero evidence 21% of the
time, and the cut is clear in 70%), and it does not matter for routing:
``pass_side_lateral_axis`` keys on (corridor, direction, colour) and position
never enters it, so a lane error cannot invert the side the rule demands. The
adversarial control settles it -- deliberately flipping every lane routes at
14.3% against 15.8% correct. What the assignment buys is that the router stops
committing to phantoms that contradict each other. Lane error is not free, it
lands on EXECUTION instead (+7 points).

**Colour pooling is REFUTED.** Pooling a cell's colour vote with its neighbours
does not remove flips, it relocates them: at radius 0.00/0.25/0.55 the totals are
157/159/166, buying down re-point flips and paying the same back in same-cell
vote flips. What removes the flips that matter is freezing the slot the router is
committed to -- flips-while-committed 66 to 0, and routing IMPROVES.

NEVER screen any of this in the simulator. Its sign map is exact (0.0% of ticks
above the physical maximum, against 86% on hardware), so every number above
collapses to zero there. That blindness is why a 256-scenario sweep once refuted
four dedup fixes that were real.
"""

from __future__ import annotations

import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from shared.domain.models import SignColor

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.sign_discovery import legal_sign_positions
from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.enums import Section
    from shared.domain.models import TrafficSignObservation, Waypoint

logger = logging.getLogger(__name__)

Cell = tuple[float, float]

_SIGNS_PER_SECTION = 2
"""Rulebook cap. A section holds at most two pillars, so the whole track holds
at most eight in twenty-four legal cells."""


@dataclass(slots=True)
class _CellEvidence:
    """What has been observed AT one legal cell, over the whole round.

    Never decays. A pillar does not move during a round, so an observation from
    lap 1 is as valid at lap 3 as when it arrived; decaying it would only make
    the assignment follow the camera's most recent mood.
    """

    weight: float = 0.0
    """Summed confidence. The floor is compared against this, not against a
    count, so one confident look outweighs several doubtful ones."""

    hits: int = 0
    """Observations that claimed this cell. Not what the floor is judged on --
    that is ``weight`` -- but the router logs it when a sign is discovered, and
    a count is what a human reading that line expects."""

    votes: dict[SignColor, float] = field(default_factory=lambda: defaultdict(float))
    """Confidence-weighted colour votes for this cell."""

    @property
    def colour(self) -> SignColor:
        """The cell's colour: argmax over its OWN votes, pooled over the round.

        Not pooled with neighbouring cells. That was measured and refuted -- it
        relocates flips rather than removing them (157/159/166 at pooling radius
        0.00/0.25/0.55), because what it buys in re-point flips it pays back in
        same-cell vote flips.
        """
        if not self.votes:
            return SignColor.UNKNOWN
        return max(self.votes, key=lambda colour: self.votes[colour])


@dataclass(slots=True)
class SignSlot:
    """One published sign, as a STABLE index pointing at a cell that may change.

    The router keys ``_passed``, ``_engaged`` and ``_committed`` by index into
    its own ``_signs`` list, so an index must never be removed or renumbered.
    A slot is therefore append-only in COUNT and mutable in CONTENT: re-pointing
    it writes in place at an index the router already holds.
    """

    cell: Cell
    section: Section
    published_index: int | None = None
    frozen: bool = False
    """Set while the router is committed to this slot. A frozen slot changes
    neither cell nor colour, so a commitment can never have the object it is
    steering around swapped underneath it."""

    colour: SignColor = SignColor.UNKNOWN
    hits: int = 0
    """Observations behind the cell this slot points at, for the router's
    discovery log. Part of the surface ``ObservedSignMap`` presents, so it is
    part of being a drop-in -- a missing attribute here took every scenario out
    at the first discovery, which is what the sim crash-check is for."""

    def as_spec(self):  # noqa: ANN201 - SignSpec, imported lazily to avoid a cycle
        """The slot as the router sees it. The position IS a legal cell, exactly."""
        from src.navigation.planning.sign_discovery import SignSpec

        return SignSpec(x=self.cell[0], y=self.cell[1], color=self.colour)


class SlotSignMap:
    """Drop-in for ``ObservedSignMap`` that assigns evidence to legal cells.

    Presents the same surface the router consumes -- ``observe``, ``propose``,
    ``newly_confirmed`` and ``published`` -- so the router's ingest loop does not
    change. What changes is that a published position is always one of the 24
    cells the rulebook allows, and a section never publishes more than two.
    """

    def __init__(self, min_confidence: float, tuning: NavigationTuning | None = None) -> None:
        tuning = get_tuning(tuning)
        sr = tuning.sign_router
        self._min_confidence = min_confidence
        self._accept_r = sr.SLOT_ACCEPT_RADIUS_M
        self._min_evidence = sr.SLOT_MIN_EVIDENCE
        self._repoint_margin = sr.SLOT_REPOINT_MARGIN
        self._max_ingest_range_m = tuning.sign_discovery.MAX_INGEST_RANGE_M

        self._cells: dict[Cell, _CellEvidence] = {}
        self._cell_section: dict[Cell, Section] = {
            cell: corridor_for_position(cell[0], cell[1]) for cell in legal_sign_positions()
        }
        self._slots: list[SignSlot] = []
        self._unpublished: list[SignSlot] = []
        self._retired: set[int] = set()
        """Router indices that have been passed. Re-pointing one would make an
        unpassed pillar inherit the "behind us" flag and vanish for the rest of
        the lap -- measured on 168 of 708 re-points -- so those get a fresh slot
        instead."""

    # ---------------------------------------------------------------- ingest

    def observe(self, observations: list[TrafficSignObservation] | None, robot_pos: Waypoint) -> None:
        """Fold one frame of world-coordinate observations into the cell evidence."""
        if observations:
            for obs in observations:
                if obs.confidence < self._min_confidence:
                    continue
                if obs.color not in (SignColor.RED, SignColor.GREEN):
                    continue
                if math.dist((obs.world_x_m, obs.world_y_m), (robot_pos.x, robot_pos.y)) > self._max_ingest_range_m:
                    continue
                self._claim(obs)
        self._reassign()

    def propose(self, positions: list[tuple[float, float]] | None, robot_pos: Waypoint) -> None:
        """Position-only proposals carry no colour, so they claim no cell.

        Accepted and ignored deliberately, to keep the surface identical to
        ``ObservedSignMap``. A cell whose only evidence is colourless cannot be
        routed around -- ``pass_side_lateral_axis`` returns None for UNKNOWN --
        so admitting it would raise the published count without raising what the
        router can act on, which is the failure mode of every dedup attempt on
        file.
        """

    def _claim(self, obs: TrafficSignObservation) -> None:
        """Attribute one observation to the legal cell it is claiming, if any.

        Further than ``SLOT_ACCEPT_RADIUS_M`` from every cell, it claims NOTHING
        rather than being pulled to the nearest. A pillar cannot stand off the
        lattice, so such a reading is about measurement, not about the world,
        and 12.2% of observations are in that class.
        """
        best: Cell | None = None
        best_d = self._accept_r
        for cell in self._cell_section:
            d = math.dist(cell, (obs.world_x_m, obs.world_y_m))
            if d <= best_d:
                best, best_d = cell, d
        if best is None:
            return
        evidence = self._cells.setdefault(best, _CellEvidence())
        evidence.weight += obs.confidence
        evidence.hits += 1
        evidence.votes[obs.color] += obs.confidence

    # ------------------------------------------------------------ assignment

    def _reassign(self) -> None:
        """Recompute the top-two-per-section assignment and apply it to the slots.

        Recomputed rather than accumulated, which is the whole point: a slot held
        by a phantom is re-pointed the moment a real pillar out-evidences it. The
        per-section cardinality cap that shipped before could not do this because
        it was monotone -- first past the post kept the slot forever.
        """
        by_section: dict[Section, list[tuple[float, Cell]]] = defaultdict(list)
        for cell, evidence in self._cells.items():
            if evidence.weight >= self._min_evidence and evidence.colour is not SignColor.UNKNOWN:
                by_section[self._cell_section[cell]].append((evidence.weight, cell))

        for section, scored in by_section.items():
            scored.sort(reverse=True)
            self._apply_section(section, [cell for _, cell in scored[:_SIGNS_PER_SECTION]])

    def _apply_section(self, section: Section, wanted: list[Cell]) -> None:
        """Point this section's slots at ``wanted``, creating slots as needed."""
        slots = [s for s in self._slots + self._unpublished if s.section == section]
        held = {s.cell for s in slots}

        for cell in wanted:
            if cell in held:
                self._refresh_colour(next(s for s in slots if s.cell == cell))
                continue
            free = [s for s in slots if s.cell not in wanted and not s.frozen]
            if len(slots) < _SIGNS_PER_SECTION or not free:
                self._open_slot(cell, section)
                continue
            incumbent = min(free, key=lambda s: self._weight(s.cell))
            if not self._displaces(cell, incumbent.cell):
                continue
            self._repoint(incumbent, cell)

    def _displaces(self, challenger: Cell, incumbent: Cell) -> bool:
        """Whether ``challenger`` beats ``incumbent`` by the hysteresis margin.

        A bare comparison churns: the cut between the last accepted cell and the
        first rejected one is clear (2x or better) in only 56.5% of
        section-runs, p10 ratio 1.20, so roughly a third of assignments would
        flip on noise. At margin 1.5 the churn halves for a median 0.55 s of
        phantom hold (p90 9.2 s); above 2.0 the tail and the count of challengers
        that lead at the end and never get the slot grow faster than the churn
        falls.
        """
        incumbent_weight = self._weight(incumbent)
        if incumbent_weight <= 0.0:
            return True
        return self._weight(challenger) >= incumbent_weight * self._repoint_margin

    def _weight(self, cell: Cell) -> float:
        evidence = self._cells.get(cell)
        return evidence.weight if evidence else 0.0

    def _open_slot(self, cell: Cell, section: Section) -> None:
        evidence = self._cells[cell]
        slot = SignSlot(cell=cell, section=section, colour=evidence.colour, hits=evidence.hits)
        self._unpublished.append(slot)

    def _repoint(self, slot: SignSlot, cell: Cell) -> None:
        """Move a slot onto a better-supported cell, or open a fresh one.

        A slot whose router index has already been PASSED cannot be re-pointed:
        the router keys ``_passed`` by index, so the new pillar would inherit the
        "already behind us" flag and be invisible until the lap resets. Measured
        on 168 of 708 re-points (24%), which is not a corner case. A fresh slot
        costs one more index and recovers those pillars -- 23 passes over 125
        runs, at +1.0 point of routing error that is newly MEASURED exposure
        rather than newly created, since the router previously never committed
        to them at all.
        """
        if slot.published_index is not None and slot.published_index in self._retired:
            self._open_slot(cell, slot.section)
            return
        logger.info(
            "Slot %s re-pointed (%.2f, %.2f) -> (%.2f, %.2f)",
            slot.published_index,
            slot.cell[0],
            slot.cell[1],
            cell[0],
            cell[1],
        )
        slot.cell = cell
        slot.colour = self._cells[cell].colour
        slot.hits = self._cells[cell].hits

    def _refresh_colour(self, slot: SignSlot) -> None:
        if slot.frozen:
            return
        slot.colour = self._cells[slot.cell].colour
        slot.hits = self._cells[slot.cell].hits

    # ---------------------------------------------------------------- output

    def newly_confirmed(self) -> list[SignSlot]:
        """Slots created since the last call. The caller assigns their index."""
        fresh, self._unpublished = self._unpublished, []
        self._slots.extend(fresh)
        return fresh

    def published(self) -> list[SignSlot]:
        """Every slot the router already holds, for in-place refinement."""
        return [s for s in self._slots if s.published_index is not None]

    # ------------------------------------------------------ router feedback

    def set_committed(self, index: int | None) -> None:
        """Freeze the slot the router is committed to, and thaw the rest.

        The freeze is what removes the colour churn this design would otherwise
        have: flips while committed go 66 to 0 and re-points 166 to 0, and
        routing IMPROVES (15.8% to 15.0%). It costs a delay, not a decision -- a
        commitment is short and the evidence that would have moved the slot is
        still there when it ends.
        """
        for slot in self._slots:
            slot.frozen = slot.published_index is not None and slot.published_index == index

    def retire(self, index: int) -> None:
        """Record that the router has marked ``index`` as passed."""
        self._retired.add(index)

    def reset_for_new_lap(self) -> None:
        """Passed indices come back next lap; the evidence deliberately does not."""
        self._retired.clear()
