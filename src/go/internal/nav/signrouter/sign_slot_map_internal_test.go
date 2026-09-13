package signrouter

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// These tests port platform/robot/tests/unit/test_sign_slot_map.py on the
// package-internal surface (slots, unpublished, weight) the Python tests reach
// into. Each property is paired with the control that would otherwise let it
// pass vacuously.

func slotTestMap() *SlotSignMap {
	return NewSlotSignMap(DefaultConfig(), DefaultDiscoveryConfig(), nil)
}

func slotObs(x, y float64, color SignColor, confidence float64) TrafficSignObservation {
	return TrafficSignObservation{WorldXM: x, WorldYM: y, Color: color, Confidence: confidence}
}

func cellsOf(m *SlotSignMap, section trackmodel.Section) []trackmodel.Waypoint {
	var out []trackmodel.Waypoint
	for _, cell := range m.lattice {
		if m.cellSection[cell] == section {
			out = append(out, cell)
		}
	}
	return out
}

func feed(m *SlotSignMap, cell trackmodel.Waypoint, color SignColor, times int, confidence float64) {
	here := trackmodel.Waypoint{X: cell.X, Y: cell.Y - 0.4}
	for range times {
		m.Observe([]TrafficSignObservation{slotObs(cell.X, cell.Y, color, confidence)}, here)
	}
}

// TestSlotMap_LegalPositionsOnly ports TestLegalPositionsOnly.
func TestSlotMap_LegalPositionsOnly(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cell := m.lattice[0]
	here := trackmodel.Waypoint{X: cell.X, Y: cell.Y - 0.4}
	for range 3 {
		m.Observe([]TrafficSignObservation{slotObs(cell.X+0.12, cell.Y-0.09, SignColorRed, 0.8)}, here)
	}
	published := m.newlyConfirmed()
	if len(published) != 1 || published[0].Cell != cell {
		t.Errorf("published = %+v, want exactly [%+v]", published, cell)
	}
}

// TestSlotMap_OffLatticeClaimsNothing is the control: a reading far from every
// cell must not be pulled onto the nearest one.
func TestSlotMap_OffLatticeClaimsNothing(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cell := m.lattice[0]
	here := trackmodel.Waypoint{X: cell.X, Y: cell.Y - 0.4}
	for range 8 {
		m.Observe([]TrafficSignObservation{slotObs(cell.X+0.9, cell.Y+0.9, SignColorRed, 0.95)}, here)
	}
	if got := m.newlyConfirmed(); len(got) != 0 {
		t.Errorf("published = %d slots, want 0 (off-lattice claims nothing)", len(got))
	}
}

// TestSlotMap_SectionNeverPublishesAThird ports TestSectionCap.
func TestSlotMap_SectionNeverPublishesAThird(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	if len(cells) < 3 {
		t.Fatalf("a section should have six legal cells, fixture has %d", len(cells))
	}
	for _, cell := range cells[:3] {
		feed(m, cell, SignColorRed, 3, 0.8)
	}
	published := m.newlyConfirmed()
	if len(published) > 2 {
		t.Errorf("published %d in one section, want <= 2", len(published))
	}
}

// TestSlotMap_ControlThreeCellsHadEvidence is the control for the cap.
func TestSlotMap_ControlThreeCellsHadEvidence(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	for _, cell := range cells[:3] {
		feed(m, cell, SignColorRed, 3, 0.8)
	}
	withEvidence := 0
	for _, cell := range cells[:3] {
		if m.weight(cell) > 0 {
			withEvidence++
		}
	}
	if withEvidence != 3 {
		t.Errorf("cells with evidence = %d, want 3 (the cap test would be vacuous)", withEvidence)
	}
}

// TestSlotMap_BetterSupportedCellTakesTheSlot ports
// test_a_better_supported_cell_takes_the_slot.
func TestSlotMap_BetterSupportedCellTakesTheSlot(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	phantom, real := cells[0], cells[1]
	feed(m, phantom, SignColorRed, 1, 0.8)
	held := m.newlyConfirmed()
	if len(held) != 1 || held[0].Cell != phantom {
		t.Fatalf("held = %+v, want the phantom %+v first", held, phantom)
	}

	feed(m, cells[2], SignColorRed, 2, 0.8)
	m.newlyConfirmed()
	feed(m, real, SignColorGreen, 12, 0.8)
	m.Observe(nil, trackmodel.Waypoint{X: real.X, Y: real.Y - 0.4})

	found := false
	for _, slot := range append(append([]*SignSlot{}, m.slots...), m.unpublished...) {
		if slot.Cell == real {
			found = true
		}
	}
	if !found {
		t.Error("the better-supported cell did not take a slot")
	}
}

// TestSlotMap_CommittedSlotIsFrozen ports test_a_committed_slot_is_frozen.
func TestSlotMap_CommittedSlotIsFrozen(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	feed(m, cells[0], SignColorRed, 1, 0.8)
	feed(m, cells[1], SignColorRed, 1, 0.8)
	for i, slot := range m.newlyConfirmed() {
		idx := i
		slot.PublishedIndex = &idx
	}
	committed := 0
	m.setCommitted(&committed)
	frozenCell := m.slots[0].Cell

	feed(m, cells[2], SignColorGreen, 20, 0.8)
	m.Observe(nil, trackmodel.Waypoint{X: cells[2].X, Y: cells[2].Y - 0.4})

	if m.slots[0].Cell != frozenCell {
		t.Errorf("committed slot moved from %+v to %+v", frozenCell, m.slots[0].Cell)
	}
}

// TestSlotMap_PassedIndexGetsFreshSlot ports
// test_a_passed_index_gets_a_fresh_slot_instead_of_being_repointed.
func TestSlotMap_PassedIndexGetsFreshSlot(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	feed(m, cells[0], SignColorRed, 1, 0.8)
	feed(m, cells[1], SignColorRed, 1, 0.8)
	for i, slot := range m.newlyConfirmed() {
		idx := i
		slot.PublishedIndex = &idx
	}
	m.retire(0)
	m.retire(1)

	feed(m, cells[2], SignColorGreen, 20, 0.8)
	m.Observe(nil, trackmodel.Waypoint{X: cells[2].X, Y: cells[2].Y - 0.4})

	fresh := m.newlyConfirmed()
	if len(fresh) != 1 || fresh[0].Cell != cells[2] {
		t.Errorf("newlyConfirmed = %+v, want [%+v]", fresh, cells[2])
	}
	if m.slots[0].Cell != cells[0] {
		t.Errorf("retired slot re-pointed to %+v, want untouched %+v", m.slots[0].Cell, cells[0])
	}
}

// TestSlotMap_ThinEvidencePublishesNothing ports TestFailSafe's thin-evidence
// case.
func TestSlotMap_ThinEvidencePublishesNothing(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cell := m.lattice[0]
	m.Observe([]TrafficSignObservation{slotObs(cell.X, cell.Y, SignColorRed, 0.3)},
		trackmodel.Waypoint{X: cell.X, Y: cell.Y - 0.4})
	if got := m.newlyConfirmed(); len(got) != 0 {
		t.Errorf("newlyConfirmed = %d, want 0", len(got))
	}
	if got := m.published(); len(got) != 0 {
		t.Errorf("published = %d, want 0", len(got))
	}
}

// TestSlotMap_ColourlessCellNeverPublishes ports
// test_a_colourless_cell_never_publishes.
func TestSlotMap_ColourlessCellNeverPublishes(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cell := m.lattice[0]
	here := trackmodel.Waypoint{X: cell.X, Y: cell.Y - 0.4}
	for range 6 {
		m.Propose([]trackmodel.Waypoint{cell}, here)
	}
	if got := m.newlyConfirmed(); len(got) != 0 {
		t.Errorf("newlyConfirmed = %d, want 0 (colourless cells never publish)", len(got))
	}
}

// TestSlotMap_FrozenSectionDoesNotGrowAThird ports
// TestTheCapHoldsUnderPressure's leak case.
func TestSlotMap_FrozenSectionDoesNotGrowAThird(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	for _, cell := range cells[:2] {
		feed(m, cell, SignColorRed, 3, 0.8)
	}
	for i, slot := range m.newlyConfirmed() {
		idx := i
		slot.PublishedIndex = &idx
	}
	committed := 0
	m.setCommitted(&committed)

	feed(m, cells[2], SignColorGreen, 30, 0.8)
	m.Observe(nil, trackmodel.Waypoint{X: cells[2].X, Y: cells[2].Y - 0.4})

	live := append(append([]*SignSlot{}, m.slots...), m.unpublished...)
	if len(live) > 2 {
		t.Errorf("section grew to %d slots, want <= 2", len(live))
	}
}

// TestSlotMap_ControlThirdCellOutEvidenced is the control for the cap.
func TestSlotMap_ControlThirdCellOutEvidenced(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	for _, cell := range cells[:2] {
		feed(m, cell, SignColorRed, 3, 0.8)
	}
	feed(m, cells[2], SignColorGreen, 30, 0.8)
	if m.weight(cells[2]) <= m.weight(cells[0])*1.5 {
		t.Errorf("weight(cell2)=%v not > 1.5x weight(cell0)=%v", m.weight(cells[2]), m.weight(cells[0]))
	}
}

// TestSlotMap_RetiredSlotMayStillBeReplaced ports
// test_a_retired_slot_may_still_be_replaced.
func TestSlotMap_RetiredSlotMayStillBeReplaced(t *testing.T) {
	t.Parallel()

	m := slotTestMap()
	cells := cellsOf(m, m.cellSection[m.lattice[0]])
	for _, cell := range cells[:2] {
		feed(m, cell, SignColorRed, 3, 0.8)
	}
	for i, slot := range m.newlyConfirmed() {
		idx := i
		slot.PublishedIndex = &idx
	}
	m.retire(0)
	m.retire(1)

	feed(m, cells[2], SignColorGreen, 30, 0.8)
	m.Observe(nil, trackmodel.Waypoint{X: cells[2].X, Y: cells[2].Y - 0.4})

	fresh := m.newlyConfirmed()
	if len(fresh) != 1 || fresh[0].Cell != cells[2] {
		t.Errorf("newlyConfirmed = %+v, want [%+v]", fresh, cells[2])
	}
}
