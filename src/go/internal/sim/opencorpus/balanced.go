package opencorpus

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// balancedCombo is one (layout, section, direction) cell of the grid, plus
// how many start cells that combination admits -- which is what the
// round-robin assignment below balances over.
type balancedCombo struct {
	widths    WidthSet
	section   simconfig.Section
	direction trackmodel.Direction
	cellCount int
}

// balancedKey identifies a scenario by its parameters rather than its index,
// for the lookup back into the full space.
type balancedKey struct {
	widths    WidthSet
	section   simconfig.Section
	direction trackmodel.Direction
	startCell int
}

// Balanced128Size is 16 layouts x 4 sections x 2 directions. The start CELL
// is what varies within it, not the count.
const Balanced128Size = WidthLayoutCount * len(comboSectionOrder) * len(DirectionOrder)

// comboSectionOrder is the section order balanced_128_cases enumerates
// combos in: Python's `for section in Section`, i.e. the ENUM DECLARATION
// order, north-first.
//
// This is NOT SectionOrder, which is south-first because the 640-case space
// iterates _OPEN_SECTIONS instead. The two orders coexist in the Python
// original and both are load-bearing: this one decides which combo each
// shuffled position maps to, and SectionOrder decides the case indices.
// Using either in the other's place still yields 128 valid scenarios under
// the wrong seed-to-corpus mapping.
var comboSectionOrder = [4]simconfig.Section{
	simconfig.SectionNorth,
	simconfig.SectionSouth,
	simconfig.SectionEast,
	simconfig.SectionWest,
}

// Balanced128 returns the 128-scenario screening corpus for a seed, matching
// scripts.common.open_cases.balanced_128_cases.
//
// The historical open128 corpus is the same 128-combo grid with start_cell
// pinned to 0 -- "the cell hard against the outer wall", one fixed spawn,
// never varied. That is the wrong constant to freeze: a blind robot's
// opening readings depend on where across the corridor it begins, and those
// readings feed corridor-width estimation and the side ranges the direction
// estimator votes on. Every Open pass rate in this project's history
// (96 -> 125 -> 126) was measured against that single spawn.
//
// Sampling 128 uniformly from the full 640 covers cells but leaves
// layout/section coverage to chance, so two runs at different seeds are not
// comparable scenario-for-scenario. This keeps the grid COMPLETE and varies
// the cell: one case per combo, so every layout/section/direction appears
// exactly once while the cell is assigned round-robin per cell-count class.
// The combo order is shuffled before assignment purely to decorrelate the
// cell from enumeration order.
//
// Deterministic for a given seed, and deterministic ACROSS LANGUAGES: the
// shuffle runs on a reimplementation of CPython's Mersenne Twister (see
// pyrandom.go), so "balanced128 seed 0" names the same 128 scenarios here
// as in Python. Varying the seed draws an independent balanced corpus, which
// is the honest way to check a result is not an artifact of one spawn
// assignment.
func Balanced128(seed uint64) ([]Params, error) {
	combos := make([]balancedCombo, 0, Balanced128Size)
	for i := range WidthLayoutCount {
		widths := widthSetFromProductIndex(i)
		for _, section := range comboSectionOrder {
			for _, direction := range DirectionOrder {
				combos = append(combos, balancedCombo{
					widths:    widths,
					section:   section,
					direction: direction,
					cellCount: widths.StartCellCount(section),
				})
			}
		}
	}

	order := make([]int, len(combos))
	for i := range order {
		order[i] = i
	}
	newPyRandom(seed).shuffle(order)

	// Round-robin PER CELL-COUNT CLASS, so the 4-cell and 6-cell combos each
	// spread evenly over their own range rather than one class starving the
	// other's high indices.
	taken := make(map[int]int, 2)
	assigned := make([]int, len(combos))
	for _, position := range order {
		count := combos[position].cellCount
		assigned[position] = taken[count] % count
		taken[count]++
	}

	// The chosen combos are looked up in the full space rather than built
	// fresh, so a balanced case keeps the space's own INDEX. A scenario has
	// to mean the same thing here as in every other harness -- otherwise
	// "case 300" would name one scenario in a 640 sweep and a different one
	// in a screening run.
	byKey := make(map[balancedKey]Params, 640)
	for _, p := range Space() {
		byKey[keyOf(p)] = p
	}

	params := make([]Params, 0, len(combos))
	for i, combo := range combos {
		key := balancedKey{
			widths:    combo.widths,
			section:   combo.section,
			direction: combo.direction,
			startCell: assigned[i],
		}
		p, ok := byKey[key]
		if !ok {
			return nil, fmt.Errorf(
				"opencorpus: balanced128 selected %v/%v/%v cell %d, which is not in the full space",
				combo.widths, combo.section, combo.direction, assigned[i])
		}
		params = append(params, p)
	}
	return params, nil
}

// widthSetFromProductIndex maps 0..15 to a layout in the order
// itertools.product((NARROW, WIDE), repeat=4) yields over
// open_cases.SIDES = (south, north, east, west) -- LAST element varying
// fastest, so west flips every step and south every eight.
//
// This is NOT WidthSetFromBits, and the difference is invisible in the
// output: both enumerate the same sixteen layouts, in different orders.
// scenario_catalog builds them from a bit mask (south = bit 0, varying
// fastest) and open_cases builds them with itertools.product (west varying
// fastest), so index 1 is a west-wide layout here and a south-wide layout
// there. Substituting one for the other still produces 128 valid, balanced
// scenarios -- under a different seed-to-corpus mapping, which is exactly
// the thing this corpus's seed is supposed to pin.
func widthSetFromProductIndex(i int) WidthSet {
	widthFor := func(bit int) int {
		if i&(1<<bit) != 0 {
			return WideMM
		}
		return NarrowMM
	}
	return WidthSet{
		SouthMM: widthFor(3),
		NorthMM: widthFor(2),
		EastMM:  widthFor(1),
		WestMM:  widthFor(0),
	}
}

func keyOf(p Params) balancedKey {
	return balancedKey{
		widths:    p.Widths,
		section:   p.Section,
		direction: p.Direction,
		startCell: p.StartCell,
	}
}
