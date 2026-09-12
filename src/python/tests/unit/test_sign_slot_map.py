"""The rules-constrained sign map: at most two pillars per section, on legal cells only.

The shipped ``ObservedSignMap`` decides "is this the same pillar" from position,
which is unanswerable here -- the belief carries 0.15-0.25 m of error while two
DISTINCT legal cells sit 0.20 m apart. It believes 9-25 pillars on a track that
holds 8. These tests pin the properties that make the slot map different, each
with the control that would otherwise let it pass vacuously.
"""

from __future__ import annotations

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import SignColor, TrafficSignObservation, Waypoint

from src.navigation.planning.sign_discovery import legal_sign_positions
from src.navigation.planning.sign_slot_map import SlotSignMap
from src.navigation.planning.waypoints import corridor_for_position


def _obs(x: float, y: float, colour: SignColor, confidence: float = 0.8) -> TrafficSignObservation:
    return TrafficSignObservation(
        world_x_m=x, world_y_m=y, color=colour, confidence=confidence, detected_at_timestamp=0.0
    )


def _map() -> SlotSignMap:
    return SlotSignMap(0.25, tuning=NavigationTuning.load_default())


def _cells_of(section) -> list[tuple[float, float]]:  # noqa: ANN001
    return [c for c in legal_sign_positions() if corridor_for_position(c[0], c[1]) is section]


def _feed(sign_map: SlotSignMap, cell, colour: SignColor, times: int, confidence: float = 0.8) -> None:  # noqa: ANN001
    here = Waypoint(cell[0], cell[1] - 0.4)
    for _ in range(times):
        sign_map.observe([_obs(cell[0], cell[1], colour, confidence)], here)


class TestLegalPositionsOnly:
    def test_a_published_position_is_exactly_a_legal_cell(self) -> None:
        sign_map = _map()
        cell = legal_sign_positions()[0]
        # Offset inside the accept radius, so the claim is real but the reading is not exact.
        here = Waypoint(cell[0], cell[1] - 0.4)
        for _ in range(3):
            sign_map.observe([_obs(cell[0] + 0.12, cell[1] - 0.09, SignColor.RED)], here)

        published = sign_map.newly_confirmed()

        assert [s.cell for s in published] == [cell]

    def test_an_off_lattice_reading_claims_nothing(self) -> None:
        """The control for the test above: a reading far from every cell must not
        be pulled onto the nearest one. A pillar cannot stand off the lattice, so
        that reading is about measurement, not about the world."""
        sign_map = _map()
        cell = legal_sign_positions()[0]
        here = Waypoint(cell[0], cell[1] - 0.4)
        for _ in range(8):
            sign_map.observe([_obs(cell[0] + 0.9, cell[1] + 0.9, SignColor.RED, 0.95)], here)

        assert sign_map.newly_confirmed() == []


class TestSectionCap:
    def test_a_section_never_publishes_a_third_pillar(self) -> None:
        sign_map = _map()
        cells = _cells_of(corridor_for_position(*legal_sign_positions()[0]))
        assert len(cells) >= 3, "a section has six legal cells; the fixture is wrong"
        for cell in cells[:3]:
            _feed(sign_map, cell, SignColor.RED, times=3)

        published = sign_map.newly_confirmed()

        assert len(published) <= 2, f"published {len(published)} in one section"

    def test_the_control_three_cells_really_did_carry_evidence(self) -> None:
        """Without this the cap above could pass because nothing was ingested."""
        sign_map = _map()
        cells = _cells_of(corridor_for_position(*legal_sign_positions()[0]))
        for cell in cells[:3]:
            _feed(sign_map, cell, SignColor.RED, times=3)

        assert sum(1 for c in cells[:3] if sign_map._weight(c) > 0) == 3  # noqa: SLF001


class TestRepointing:
    def test_a_better_supported_cell_takes_the_slot(self) -> None:
        """What the refuted cardinality cap could not do: first past the post
        held the slot forever, so a phantom kept a real pillar out."""
        sign_map = _map()
        cells = _cells_of(corridor_for_position(*legal_sign_positions()[0]))
        phantom, real = cells[0], cells[1]
        _feed(sign_map, phantom, SignColor.RED, times=1)
        held = [s.cell for s in sign_map.newly_confirmed()]
        assert held == [phantom], "the phantom did not take the slot first"

        # Fill the section's other slot, then out-evidence the phantom well past
        # the hysteresis margin.
        _feed(sign_map, cells[2], SignColor.RED, times=2)
        sign_map.newly_confirmed()
        _feed(sign_map, real, SignColor.GREEN, times=12)
        sign_map.observe(None, Waypoint(real[0], real[1] - 0.4))

        assert real in {s.cell for s in sign_map._slots + sign_map._unpublished}  # noqa: SLF001

    def test_a_committed_slot_is_frozen(self) -> None:
        """A commitment must never have the object it is steering around swapped."""
        sign_map = _map()
        cells = _cells_of(corridor_for_position(*legal_sign_positions()[0]))
        _feed(sign_map, cells[0], SignColor.RED, times=1)
        _feed(sign_map, cells[1], SignColor.RED, times=1)
        for i, slot in enumerate(sign_map.newly_confirmed()):
            slot.published_index = i
        sign_map.set_committed(0)
        frozen_cell = sign_map._slots[0].cell  # noqa: SLF001

        _feed(sign_map, cells[2], SignColor.GREEN, times=20)
        sign_map.observe(None, Waypoint(cells[2][0], cells[2][1] - 0.4))

        assert sign_map._slots[0].cell == frozen_cell  # noqa: SLF001

    def test_a_passed_index_gets_a_fresh_slot_instead_of_being_repointed(self) -> None:
        """Re-pointing a retired index would make an unpassed pillar inherit the
        'behind us' flag and vanish for the lap. Measured on 168 of 708
        re-points, so it is not a corner case."""
        sign_map = _map()
        cells = _cells_of(corridor_for_position(*legal_sign_positions()[0]))
        _feed(sign_map, cells[0], SignColor.RED, times=1)
        _feed(sign_map, cells[1], SignColor.RED, times=1)
        for i, slot in enumerate(sign_map.newly_confirmed()):
            slot.published_index = i
        sign_map.retire(0)
        sign_map.retire(1)

        _feed(sign_map, cells[2], SignColor.GREEN, times=20)
        sign_map.observe(None, Waypoint(cells[2][0], cells[2][1] - 0.4))

        assert [s.cell for s in sign_map.newly_confirmed()] == [cells[2]]
        assert sign_map._slots[0].cell == cells[0], "a retired index was re-pointed"  # noqa: SLF001


class TestFailSafe:
    def test_thin_evidence_publishes_nothing(self) -> None:
        """Before any cell clears the floor the map is empty, which makes the
        router fall back to pure corridor following -- no colour claimed, so
        rule 9.24.5 cannot be violated."""
        sign_map = _map()
        cell = legal_sign_positions()[0]
        sign_map.observe([_obs(cell[0], cell[1], SignColor.RED, 0.3)], Waypoint(cell[0], cell[1] - 0.4))

        assert sign_map.newly_confirmed() == []
        assert sign_map.published() == []

    def test_a_colourless_cell_never_publishes(self) -> None:
        """pass_side_lateral_axis returns None for UNKNOWN, so a colourless
        pillar raises the published count without raising what the router can
        act on -- the failure mode of every dedup attempt on file."""
        sign_map = _map()
        cell = legal_sign_positions()[0]
        here = Waypoint(cell[0], cell[1] - 0.4)
        for _ in range(6):
            sign_map.propose([cell], here)

        assert sign_map.newly_confirmed() == []
