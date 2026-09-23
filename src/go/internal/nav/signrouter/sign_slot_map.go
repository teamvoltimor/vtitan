// sign_slot_map.go ports
// src/python/src/navigation/planning/sign_slot_map.py: a sign map that
// assigns evidence to the rulebook's 24 legal cells (at most two per section)
// instead of clustering freely.
//
// ObservedSignMap tracks whatever the camera reports and decides "which reports
// describe the same pillar" from position alone -- not answerable here, because
// the believed position is too noisy to separate two DISTINCT legal pillars.
// The rulebook says a pillar stands on one of 24 legal cells -- six per section,
// at 0.4 m from the outer wall or 0.4 m from the inner one, at depths
// 1.0/1.5/2.0 m -- and that a section holds AT MOST TWO. So this is a
// CONSTRAINED ASSIGNMENT, recomputed every tick, not a clustering problem.
//
// Measured rationale and the 125-bag comparison are in
// adr:0058-sign-discovery-range-and-barrier-belief.
//
// NEVER screen it in the simulator: the sim's sign map is exact, so its
// advantage collapses to zero there.

package signrouter

import (
	"math"
	"slices"
	"sort"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// cellEvidence is what has been observed AT one legal cell over the whole
// round, matching _CellEvidence. Never decays: a pillar does not move during a
// round, so an observation from lap 1 is as valid at lap 3.
type cellEvidence struct {
	// weight is summed confidence; the floor is compared against this, not a
	// count, so one confident look outweighs several doubtful ones.
	weight float64
	// hits is observations that claimed this cell (for the discovery log).
	hits  int
	votes map[SignColor]float64
}

// SignSlot is one published sign, as a STABLE index pointing at a cell that may
// change, matching SignSlot. The router keys its passed/engaged/committed state
// by index, so a slot is append-only in COUNT and mutable in CONTENT: a
// re-point writes in place at an index the router already holds.
type SignSlot struct {
	Cell           trackmodel.Waypoint
	Section        trackmodel.Section
	PublishedIndex *int
	// Frozen is set while the router is committed to this slot. A frozen slot
	// changes neither cell nor colour, so a commitment can never have the
	// object it is steering around swapped underneath it.
	Frozen bool
	Colour SignColor
	// Hits is observations behind the cell this slot points at, for the
	// router's discovery log.
	Hits int
}

// SlotSignMap is a drop-in for ObservedSignMap that assigns evidence to legal
// cells. Presents the same surface the navigator consumes, so the ingest loop
// does not change; what changes is that a published position is always one of
// the 24 cells the rulebook allows, and a section never publishes more than
// two.
type SlotSignMap struct {
	minConfidence   float64
	acceptR         float64
	minEvidence     float64
	repointMargin   float64
	maxIngestRangeM float64
	cornerMinM      float64
	cornerMaxM      float64
	sd              *SignRouter

	// lattice is all 24 legal cells, in legal_sign_positions order. claim
	// iterates it (not just claimed cells) so a near-tie resolves the same way
	// the Python dict-comprehension order does.
	lattice     []trackmodel.Waypoint
	cellSection map[trackmodel.Waypoint]trackmodel.Section

	cells       map[trackmodel.Waypoint]*cellEvidence
	cellOrder   []trackmodel.Waypoint
	slots       []*SignSlot
	unpublished []*SignSlot
	// retired holds router indices that have been passed. Re-pointing one
	// would make an unpassed pillar inherit the "behind us" flag and vanish
	// for the rest of the lap, so those get a fresh slot instead.
	retired map[int]struct{}
}

// weightedCell pairs a cell with its evidence weight for the assignment sort.
type weightedCell struct {
	weight float64
	cell   trackmodel.Waypoint
}

// signsPerSection is the rulebook cap. A section holds at most two pillars, so
// the whole track holds at most eight in twenty-four legal cells. Matches
// sign_slot_map._SIGNS_PER_SECTION.
const signsPerSection = 2

// trackSections and legalCellsPerSection size the legal grid: four sections,
// three depth rows by two width lines each.
const (
	trackSections        = 4
	legalCellsPerSection = 6
)

// colour is the cell's colour: argmax over its OWN votes, pooled over the
// round -- NOT pooled with neighbouring cells; pooling relocates flips rather
// than removing them. See adr:0058-sign-discovery-range-and-barrier-belief.
func (e *cellEvidence) colour() SignColor {
	best := SignColorUnknown
	bestV := math.Inf(-1)
	for color, v := range e.votes {
		if v > bestV {
			bestV = v
			best = color
		}
	}
	return best
}

// asSpec materialises the slot as the router sees it. The position IS a legal
// cell, exactly.
func (s *SignSlot) asSpec() SignSpec {
	return SignSpec{X: s.Cell.X, Y: s.Cell.Y, Color: s.Colour}
}

// NewSlotSignMap builds an empty rulebook-constrained map, matching
// SlotSignMap.__init__. cfg supplies the SLOT_* knobs, the sign grid and the
// track bounds; discovery supplies the ingest range and minimum confidence; sd,
// when non-nil, is the router confirmed slots are published into.
func NewSlotSignMap(cfg Config, discovery DiscoveryConfig, sd *SignRouter) *SlotSignMap {
	if sd != nil {
		discovery.CornerMinM = sd.cornerMinM()
		discovery.CornerMaxM = sd.cornerMaxM()
	}
	m := &SlotSignMap{
		minConfidence:   cfg.MinConfidence,
		acceptR:         cfg.SlotAcceptRadiusM,
		minEvidence:     cfg.SlotMinEvidence,
		repointMargin:   cfg.SlotRepointMargin,
		maxIngestRangeM: discovery.MaxIngestRangeM,
		cornerMinM:      cfg.TrackCornerMinM,
		cornerMaxM:      cfg.TrackCornerMaxM,
		sd:              sd,
		cellSection:     map[trackmodel.Waypoint]trackmodel.Section{},
		cells:           map[trackmodel.Waypoint]*cellEvidence{},
		retired:         map[int]struct{}{},
	}
	m.lattice = legalSignPositions(cfg)
	for _, cell := range m.lattice {
		m.cellSection[cell] = waypoints.CorridorForPosition(cell.X, cell.Y, cfg.TrackCornerMinM, cfg.TrackCornerMaxM)
	}
	return m
}

// legalSignPositions returns every world position a pillar may legally stand
// at, matching sign_discovery.legal_sign_positions. Built from the rulebook
// geometry: three depth rows, two width lines near the outer wall and their
// mirror near the far one, six per section, four sections.
func legalSignPositions(cfg Config) []trackmodel.Waypoint {
	depths := []float64{cfg.GridDepthNear, cfg.GridDepthMiddle, cfg.GridDepthFar}
	near := []float64{cfg.GridWidthOuter, cfg.GridWidthInner}
	far := []float64{cfg.TrackSizeM - near[0], cfg.TrackSizeM - near[1]}
	points := make([]trackmodel.Waypoint, 0, trackSections*legalCellsPerSection)
	for _, d := range depths {
		for _, w := range near {
			points = append(points, trackmodel.Waypoint{X: d, Y: w}) // SOUTH
		}
		for _, w := range far {
			points = append(points, trackmodel.Waypoint{X: d, Y: w}) // NORTH
		}
		for _, w := range near {
			points = append(points, trackmodel.Waypoint{X: w, Y: d}) // WEST
		}
		for _, w := range far {
			points = append(points, trackmodel.Waypoint{X: w, Y: d}) // EAST
		}
	}
	return points
}

// IsDiscovering reports whether this map is actively feeding a SignRouter
// (discover mode), matching SignRouter.is_discovering.
func (m *SlotSignMap) IsDiscovering() bool {
	return m != nil && m.sd != nil
}

// Observe folds one frame of world-coordinate observations into the cell
// evidence and recomputes the assignment, matching SlotSignMap.observe.
func (m *SlotSignMap) Observe(observations []TrafficSignObservation, robotPos trackmodel.Waypoint) {
	for i := range observations {
		obs := observations[i]
		if obs.Confidence < m.minConfidence {
			continue
		}
		if obs.Color != SignColorRed && obs.Color != SignColorGreen {
			continue
		}
		if math.Hypot(obs.WorldXM-robotPos.X, obs.WorldYM-robotPos.Y) > m.maxIngestRangeM {
			continue
		}
		m.claim(obs)
	}
	m.reassign()
}

// Propose is deliberately ignored, matching SlotSignMap.propose: position-only
// proposals carry no colour, so they claim no cell. A cell whose only evidence
// is colourless cannot be routed around (PassSideLateralAxis returns false for
// UNKNOWN), so admitting it would raise the published count without raising
// what the router can act on. Accepted for surface parity with ObservedSignMap.
func (m *SlotSignMap) Propose(_ []trackmodel.Waypoint, _ trackmodel.Waypoint) {}

// ResetForNewLap clears the retired set. Passed indices come back next lap; the
// evidence deliberately does not.
func (m *SlotSignMap) ResetForNewLap() {
	m.retired = map[int]struct{}{}
}

// Publish appends newly-confirmed slots into the router, feeds the router's
// committed/passed state back, then refines published slots in place.
func (m *SlotSignMap) Publish() {
	if m.sd == nil {
		return
	}
	for _, slot := range m.newlyConfirmed() {
		idx := m.sd.AppendSign(slot.asSpec())
		i := idx
		slot.PublishedIndex = &i
	}
	// The map cannot see these two facts and both change what it may do: a
	// committed slot must not be re-pointed underneath the router, and a
	// PASSED index must never be re-pointed at all.
	m.setCommitted(m.sd.CommittedIndex())
	for index := range m.sd.PassedIndices() {
		m.retire(index)
	}
	for _, slot := range m.published() {
		m.sd.UpdateSign(*slot.PublishedIndex, slot.asSpec())
	}
}

// claim attributes one observation to the legal cell it is claiming, if any.
// Further than acceptR from every cell, it claims NOTHING rather than being
// pulled to the nearest: a pillar cannot stand off the lattice, so a reading
// that far out is about measurement, not the world. See
// adr:0058-sign-discovery-range-and-barrier-belief.
func (m *SlotSignMap) claim(obs TrafficSignObservation) {
	best := trackmodel.Waypoint{}
	bestD := m.acceptR
	found := false
	for _, cell := range m.lattice {
		d := math.Hypot(cell.X-obs.WorldXM, cell.Y-obs.WorldYM)
		if d <= bestD {
			best, bestD, found = cell, d, true
		}
	}
	if !found {
		return
	}
	evidence, ok := m.cells[best]
	if !ok {
		evidence = &cellEvidence{votes: map[SignColor]float64{}}
		m.cells[best] = evidence
		m.cellOrder = append(m.cellOrder, best)
	}
	evidence.weight += obs.Confidence
	evidence.hits++
	evidence.votes[obs.Color] += obs.Confidence
}

// reassign recomputes the top-two-per-section assignment and applies it to the
// slots. Recomputed rather than accumulated: a slot held by a phantom is
// re-pointed the moment a real pillar out-evidences it.
func (m *SlotSignMap) reassign() {
	sectionOrder := make([]trackmodel.Section, 0, trackSections)
	bySection := map[trackmodel.Section][]weightedCell{}
	for _, cell := range m.cellOrder {
		evidence := m.cells[cell]
		if evidence.weight < m.minEvidence || evidence.colour() == SignColorUnknown {
			continue
		}
		section := m.cellSection[cell]
		if _, seen := bySection[section]; !seen {
			sectionOrder = append(sectionOrder, section)
		}
		bySection[section] = append(bySection[section], weightedCell{evidence.weight, cell})
	}
	for _, section := range sectionOrder {
		scored := bySection[section]
		// Reverse tuple sort: weight desc, then cell desc, matching Python's
		// scored.sort(reverse=True).
		sort.SliceStable(scored, func(i, j int) bool {
			if scored[i].weight != scored[j].weight {
				return scored[i].weight > scored[j].weight
			}
			if scored[i].cell.X != scored[j].cell.X {
				return scored[i].cell.X > scored[j].cell.X
			}
			return scored[i].cell.Y > scored[j].cell.Y
		})
		wanted := make([]trackmodel.Waypoint, 0, signsPerSection)
		for _, entry := range scored[:min(signsPerSection, len(scored))] {
			wanted = append(wanted, entry.cell)
		}
		m.applySection(section, wanted)
	}
}

// applySection points this section's LIVE slots at wanted, never opening a
// third. The cap is enforced on every path rather than assumed: an earlier
// version opened a slot whenever no incumbent was displaceable, which let a
// section hold three. Slots at a RETIRED index do not count as live.
func (m *SlotSignMap) applySection(section trackmodel.Section, wanted []trackmodel.Waypoint) {
	for _, cell := range wanted {
		slots := m.slotsForSection(section)
		if existing := slotAt(slots, cell); existing != nil {
			m.refreshColour(existing)
			continue
		}
		live := m.liveSlots(slots)
		if len(live) < signsPerSection {
			m.openSlot(cell, section)
			continue
		}
		// At the cap. With nothing displaceable the evidence does not
		// expire, so this cell takes a slot as soon as one frees.
		incumbent := m.weakestDisplaceable(live, wanted)
		if incumbent == nil || !m.displaces(cell, incumbent.Cell) {
			continue
		}
		m.repoint(incumbent, cell)
	}
}

// slotAt returns the slot already pointed at cell, or nil.
func slotAt(slots []*SignSlot, cell trackmodel.Waypoint) *SignSlot {
	for _, slot := range slots {
		if slot.Cell == cell {
			return slot
		}
	}
	return nil
}

// liveSlots drops the slots at a RETIRED index.
func (m *SlotSignMap) liveSlots(slots []*SignSlot) []*SignSlot {
	live := make([]*SignSlot, 0, len(slots))
	for _, slot := range slots {
		if !m.isRetired(slot) {
			live = append(live, slot)
		}
	}
	return live
}

// weakestDisplaceable is the lowest-weight live slot that is neither wanted
// nor frozen (the first on a tie), or nil when every slot is protected.
func (m *SlotSignMap) weakestDisplaceable(live []*SignSlot, wanted []trackmodel.Waypoint) *SignSlot {
	var weakest *SignSlot
	for _, slot := range live {
		if slices.Contains(wanted, slot.Cell) || slot.Frozen {
			continue
		}
		if weakest == nil || m.weight(slot.Cell) < m.weight(weakest.Cell) {
			weakest = slot
		}
	}
	return weakest
}

// displaces reports whether challenger beats incumbent by the hysteresis
// margin. A bare comparison churns: the cut is rarely clear by a wide margin,
// so about a third of assignments would flip on noise. See
// adr:0058-sign-discovery-range-and-barrier-belief.
func (m *SlotSignMap) displaces(challenger, incumbent trackmodel.Waypoint) bool {
	incumbentWeight := m.weight(incumbent)
	if incumbentWeight <= 0.0 {
		return true
	}
	return m.weight(challenger) >= incumbentWeight*m.repointMargin
}

func (m *SlotSignMap) weight(cell trackmodel.Waypoint) float64 {
	if evidence, ok := m.cells[cell]; ok {
		return evidence.weight
	}
	return 0.0
}

// slotsForSection returns this section's slots (published first, then
// unpublished), preserving order so a tie-break matches Python's
// self._slots + self._unpublished.
func (m *SlotSignMap) slotsForSection(section trackmodel.Section) []*SignSlot {
	out := make([]*SignSlot, 0, len(m.slots)+len(m.unpublished))
	for _, slot := range m.slots {
		if slot.Section == section {
			out = append(out, slot)
		}
	}
	for _, slot := range m.unpublished {
		if slot.Section == section {
			out = append(out, slot)
		}
	}
	return out
}

// isRetired reports whether a slot's router index has been marked passed.
func (m *SlotSignMap) isRetired(slot *SignSlot) bool {
	if slot.PublishedIndex == nil {
		return false
	}
	_, ok := m.retired[*slot.PublishedIndex]
	return ok
}

func (m *SlotSignMap) openSlot(cell trackmodel.Waypoint, section trackmodel.Section) {
	evidence := m.cells[cell]
	m.unpublished = append(m.unpublished, &SignSlot{
		Cell:    cell,
		Section: section,
		Colour:  evidence.colour(),
		Hits:    evidence.hits,
	})
}

// repoint moves a slot onto a better-supported cell, or opens a fresh one when
// the slot's index has already been passed.
func (m *SlotSignMap) repoint(slot *SignSlot, cell trackmodel.Waypoint) {
	if m.isRetired(slot) {
		m.openSlot(cell, slot.Section)
		return
	}
	slot.Cell = cell
	slot.Colour = m.cells[cell].colour()
	slot.Hits = m.cells[cell].hits
}

func (m *SlotSignMap) refreshColour(slot *SignSlot) {
	if slot.Frozen {
		return
	}
	slot.Colour = m.cells[slot.Cell].colour()
	slot.Hits = m.cells[slot.Cell].hits
}

// newlyConfirmed returns slots created since the last call and moves them into
// the published list.
func (m *SlotSignMap) newlyConfirmed() []*SignSlot {
	fresh := m.unpublished
	m.unpublished = nil
	m.slots = append(m.slots, fresh...)
	return fresh
}

// published returns every slot the router already holds, for in-place
// refinement.
func (m *SlotSignMap) published() []*SignSlot {
	out := make([]*SignSlot, 0, len(m.slots))
	for _, slot := range m.slots {
		if slot.PublishedIndex != nil {
			out = append(out, slot)
		}
	}
	return out
}

// setCommitted freezes the slot the router is committed to and thaws the rest.
// The freeze removes the colour churn this design would otherwise have, and
// routing improves. See adr:0058-sign-discovery-range-and-barrier-belief.
func (m *SlotSignMap) setCommitted(index *int) {
	for _, slot := range m.slots {
		slot.Frozen = slot.PublishedIndex != nil && index != nil && *slot.PublishedIndex == *index
	}
}

// retire records that the router has marked index as passed.
func (m *SlotSignMap) retire(index int) {
	m.retired[index] = struct{}{}
}
