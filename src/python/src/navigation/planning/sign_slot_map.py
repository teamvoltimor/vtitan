"""A sign map that assigns evidence to the RULEBOOK's cells instead of clustering freely.

``ObservedSignMap`` tracks whatever the camera reports and tries to decide, from
position alone, which reports describe the same pillar. That question is not
answerable here: the believed position carries enough error that two DISTINCT
legal pillars sitting close together cannot be told apart, so the map invents
pillars, with roughly half of all believed positions matching no legal cell at
all, and that single defect costs half the sample of every geometric
measurement this project makes.

FOUR attempts to repair it at PUBLICATION time are on file and all failed,
because each one still had to answer "is this the same object" from position:

* position-keyed merging, two variants: measured worse than baseline;
* ``SNAP_TO_LATTICE_M`` quantisation: routing errors appeared to halve while
  passes vanished from the denominator and the PEAK believed count ROSE;
* a per-section cardinality cap: it retains REAL signs, because a phantom that
  published first holds the slot. Being monotone is what made it safe and is
  exactly what stops the real pillar entering later.

The measurements behind these are in
``adr:0058-sign-discovery-range-and-barrier-belief``.

This module asks a different question. The rulebook says a pillar stands on one
of 24 legal cells -- six per section, at 0.4 m from the outer wall or 0.4 m from
the inner one, at depths 1.0/1.5/2.0 m -- and that a section holds AT MOST TWO.
So the map is not a clustering problem, it is a CONSTRAINED ASSIGNMENT: which
two of six cells does each section's evidence support? Evidence accumulates per
cell and the assignment is recomputed every tick, which is the axis the
cardinality cap could not move.

Measured over the bag corpus against the shipped map on identical observations
(see the ADR for the table):

- a much lower worst peak believed count, and no runs over the physical max;
- no re-points or colour flips while COMMITTED.

The headline routing error and per-run position changes are in
``adr:0058-sign-discovery-range-and-barrier-belief``.

**The win is CARDINALITY, not lane accuracy.** Which of the two lanes a pillar
lands in is close to a coin flip (the lane partner often has zero evidence, and
the cut is only sometimes clear), and it does not matter for routing:
``pass_side_lateral_axis`` keys on (corridor, direction, colour) and position
never enters it, so a lane error cannot invert the side the rule demands. The
adversarial control settles it: deliberately flipping every lane routes at about
the same rate as the correct assignment. What the assignment buys is that the
router stops committing to phantoms that contradict each other. Lane error is
not free, it lands on EXECUTION instead.

**Colour pooling is REFUTED.** Pooling a cell's colour vote with its neighbours
does not remove flips, it relocates them. What removes the flips that matter is
freezing the slot the router is committed to. See
``adr:0058-sign-discovery-range-and-barrier-belief``.

NEVER screen any of this in the simulator. Its sign map is exact (almost no ticks
above the physical maximum, against most ticks on hardware), so every number
above collapses to zero there. That blindness is why a simulator sweep once
refuted four dedup fixes that were real.
"""

from __future__ import annotations

import logging
import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from shared.domain.enums import Section
from shared.domain.models import SignColor

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.sign_discovery import legal_sign_positions
from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
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
        relocates flips rather than removing them. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
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
        self._accept_r = sr.slot_accept_radius_m
        self._min_evidence = sr.slot_min_evidence
        self._repoint_margin = sr.slot_repoint_margin
        self._max_ingest_range_m = tuning.sign_discovery.max_ingest_range_m

        self._cells: dict[Cell, _CellEvidence] = {}
        self._cell_section: dict[Cell, Section] = {
            cell: corridor_for_position(cell[0], cell[1]) for cell in legal_sign_positions()
        }
        self._slots: list[SignSlot] = []
        self._unpublished: list[SignSlot] = []
        self._retired: set[int] = set()
        """Router indices that have been passed. Re-pointing one would make an
        unpassed pillar inherit the "behind us" flag and vanish for the rest of
        the lap, so those get a fresh slot instead. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``."""

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
        lattice, so such a reading is about measurement, not about the world.
        See ``adr:0058-sign-discovery-range-and-barrier-belief``.
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
            self._apply_section(section, self._one_per_depth(section, [cell for _, cell in scored]))

    @staticmethod
    def _depth(cell: Cell, section: Section) -> float:
        """The cell's coordinate ALONG its section: x on north/south, y on east/west."""
        return cell[0] if section in (Section.NORTH, Section.SOUTH) else cell[1]

    def _one_per_depth(self, section: Section, ranked: list[Cell]) -> list[Cell]:
        """The section's top cells, at most one per depth line, up to the cap.

        The rulebook never puts two pillars on the same depth line: every double
        is depth 1.0 plus depth 2.0, and 1.5 only ever appears alone. So a
        section that believes BOTH laterals of one depth is not believing two
        pillars, it is believing one pillar twice, which is exactly what a small
        pose bias does to a pillar standing between the 0.4 m and 0.6 m lanes.
        The corpus verification and the traced failure are in
        ``adr:0058-sign-discovery-range-and-barrier-belief``.

        Choosing the heavier lateral per depth frees the second slot for a real
        pillar at another depth and drops nothing the rules could have placed.
        """
        wanted: list[Cell] = []
        taken: set[float] = set()
        for cell in ranked:
            depth = self._depth(cell, section)
            if depth in taken:
                continue
            wanted.append(cell)
            taken.add(depth)
            if len(wanted) == _SIGNS_PER_SECTION:
                break
        return wanted

    def _apply_section(self, section: Section, wanted: list[Cell]) -> None:
        """Point this section's LIVE slots at ``wanted``, never opening a third.

        The cap is the whole point of this map, so it is enforced on every path
        out of this method rather than assumed.         An earlier version opened a slot
        whenever no incumbent was displaceable, which let a section hold three.
        It still beat the shipped map, which is exactly why the leak needed
        catching rather than celebrating. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``.

        Slots at a RETIRED index do not count: the router excludes ``_passed``
        from ``active_sign_count``, so they are not live pillars and refusing to
        replace them would strand a real one for the rest of the lap.

        An incumbent that shares a depth line with a wanted cell is the other
        lateral of the same pillar (see ``_one_per_depth``), so it is re-pointed
        without the hysteresis margin: the margin exists to stop two CANDIDATE
        pillars churning on noise, and a twin is not a candidate the rulebook
        allows at all.
        """
        for cell in wanted:
            slots = [s for s in self._slots + self._unpublished if s.section == section]
            live = [s for s in slots if s.published_index not in self._retired]
            if any(s.cell == cell for s in slots):
                self._refresh_colour(next(s for s in slots if s.cell == cell))
                continue
            free = [s for s in live if s.cell not in wanted and not s.frozen]
            # THIS cell's depth, not the set of all wanted depths: an incumbent
            # at another wanted depth is a different pillar, and hijacking its
            # slot would hand the router's index for the depth-2.0 pillar to the
            # depth-1.0 one.
            twins = [s for s in free if self._depth(s.cell, section) == self._depth(cell, section)]
            if twins:
                # Checked BEFORE the cap: a twin below the cap would otherwise
                # be joined by its own other lateral, and the section would
                # hold one pillar twice with a slot to spare.
                self._repoint(min(twins, key=lambda s: self._weight(s.cell)), cell)
                continue
            if len(live) < _SIGNS_PER_SECTION:
                self._open_slot(cell, section)
                continue
            if not free:
                # At the cap with nothing displaceable. Wait: the evidence does
                # not expire, so this cell takes a slot as soon as one frees,
                # and opening a third here is what the rulebook forbids.
                continue
            incumbent = min(free, key=lambda s: self._weight(s.cell))
            if not self._displaces(cell, incumbent.cell):
                continue
            self._repoint(incumbent, cell)

    def _displaces(self, challenger: Cell, incumbent: Cell) -> bool:
        """Whether ``challenger`` beats ``incumbent`` by the hysteresis margin.

        A bare comparison churns: the cut between the last accepted cell and the
        first rejected one is rarely clear, so a large share of assignments
        would flip on noise. The hysteresis margin halves the churn for a short
        phantom hold; above a certain point the tail and the count of challengers
        that lead at the end and never get the slot grow faster than the churn
        falls. See ``adr:0058-sign-discovery-range-and-barrier-belief``.
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
        "already behind us" flag and be invisible until the lap resets, which is
        not a corner case. A fresh slot costs one more index and recovers those
        pillars, at a small routing-error cost that is newly MEASURED exposure
        rather than newly created, since the router previously never committed
        to them at all. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
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
        have: flips while committed and re-points both fall to zero, and routing
        improves rather than degrades. It costs a delay, not a decision -- a
        commitment is short and the evidence that would have moved the slot is
        still there when it ends. See
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
        """
        for slot in self._slots:
            slot.frozen = slot.published_index is not None and slot.published_index == index

    def retire(self, index: int) -> None:
        """Record that the router has marked ``index`` as passed.

        WIRED. ``router.py`` calls this on every published-map refresh::

            retire = getattr(self._sign_map, "retire", None)
            if retire is not None:
                for passed_index in self._passed:
                    retire(passed_index)

        A note here once claimed the opposite and cost a session: the call is
        duck-typed through a local, so ``grep for a dotted .retire(`` and
        ``grep '_sign_map.retire'`` both miss it and only ``grep 'retire('``
        finds it. ``ObservedSignMap`` really has no ``retire``, so the NON-slot
        arm is unwired, which is what made the wrong reading look plausible.

        WHAT IS ACTUALLY BROKEN is the other half, and it is an asymmetry.
        ``SignRouter.reset_for_new_lap`` clears ``_passed`` and never forwards
        to this map, so at a lap line ``_passed`` empties while ``_retired``
        keeps every lap-1 index. ``_apply_section`` caps LIVE (non-retired)
        slots, so each section is then free to open two more, and a 3-lap round
        ratchets toward 6 published indices per section against a physical 2.

        Forwarding the lap reset is therefore NO LONGER a no-op. It is a real
        behaviour change and needs its own A/B: the design was measured as a
        package and it trades index growth for recovered pillars. The tick
        distribution and the recovered-pass cost are in
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
        """
        self._retired.add(index)

    def reset_for_new_lap(self) -> None:
        """Passed indices come back next lap; the evidence deliberately does not.

        NO PRODUCTION CALLER, which is the live defect rather than a dead
        branch: ``retire`` above IS wired, so ``_retired`` is populated, and
        ``SignRouter.reset_for_new_lap`` clearing ``_passed`` without clearing
        this leaves the two sets disagreeing for the rest of the round. See
        ``retire``'s docstring, and the measurement in
        ``adr:0058-sign-discovery-range-and-barrier-belief``.
        """
        self._retired.clear()
