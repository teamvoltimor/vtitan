package opencorpus

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// Corridor widths the Open Challenge can present, in millimetres. The rules
// offer these two and nothing between them, which is what makes the space
// enumerable at all -- see waypoints' narrow_width_threshold_m for why the
// planner treats width as a two-way classification rather than a continuum.
const (
	NarrowMM = 600
	WideMM   = 1000
)

// SectionOrder is the section enumeration order of Python's _OPEN_SECTIONS.
// See the package doc: this is NOT simconfig.AllSections, and substituting
// it renumbers every case.
var SectionOrder = [4]simconfig.Section{
	simconfig.SectionSouth,
	simconfig.SectionNorth,
	simconfig.SectionEast,
	simconfig.SectionWest,
}

// DirectionOrder is the direction enumeration order of
// OpenChallengeScenarioSpace.directions.
var DirectionOrder = [2]trackmodel.Direction{
	trackmodel.Clockwise,
	trackmodel.Counterclockwise,
}

// WidthLayoutCount is the number of distinct corridor-width assignments: one
// per value of the 4-bit mask WidthSetFromBits decodes.
const WidthLayoutCount = 16

// WidthSet is one complete Open Challenge corridor-width assignment, in
// millimetres. Matches scenario_catalog.CorridorWidthSet.
type WidthSet struct {
	SouthMM int
	NorthMM int
	EastMM  int
	WestMM  int
}

// WidthSetFromBits maps a 4-bit value to the four corridor widths, bit set
// meaning wide. The bit-to-section assignment is
// CorridorWidthSet.from_bits': south is bit 0, north bit 1, east bit 2,
// west bit 3. It is arbitrary but fixed -- it decides which layout each
// case index lands on.
func WidthSetFromBits(bits int) WidthSet {
	widthFor := func(mask int) int {
		if bits&mask != 0 {
			return WideMM
		}
		return NarrowMM
	}
	return WidthSet{
		SouthMM: widthFor(0b0001),
		NorthMM: widthFor(0b0010),
		EastMM:  widthFor(0b0100),
		WestMM:  widthFor(0b1000),
	}
}

// WidthMMFor returns the width of one corridor.
func (w WidthSet) WidthMMFor(section simconfig.Section) int {
	switch section {
	case simconfig.SectionSouth:
		return w.SouthMM
	case simconfig.SectionNorth:
		return w.NorthMM
	case simconfig.SectionEast:
		return w.EastMM
	case simconfig.SectionWest:
		return w.WestMM
	default:
		return 0
	}
}

// IsWide reports whether the requested corridor takes the wide width.
func (w WidthSet) IsWide(section simconfig.Section) bool {
	return w.WidthMMFor(section) == WideMM
}

// MetresByName returns the widths keyed by lowercase section name, the shape
// generate.Metadata and startconditions.StartPose both want.
func (w WidthSet) MetresByName() map[simconfig.Section]float64 {
	widths := make(map[simconfig.Section]float64, len(SectionOrder))
	for _, section := range SectionOrder {
		widths[section] = float64(w.WidthMMFor(section)) / 1000.0
	}
	return widths
}

// StartCellCount is how many legal starting cells this layout offers on the
// given side: four in a narrow corridor, six in a wide one.
//
// Derived from generate.StartCells rather than hardcoded, unlike Python's
// CorridorWidthSet.start_cell_count, which carries its own 6/4 literal
// alongside start_cells' band arithmetic. The two agree today; deriving
// means they cannot stop agreeing here, and
// TestStartCellCount_MatchesPythonLiterals pins the values Python hardcodes
// so a band-layout edit in track.toml fails loudly instead of quietly
// resizing the corpus.
func (w WidthSet) StartCellCount(section simconfig.Section) int {
	return len(generate.StartCells(section, float64(w.WidthMMFor(section))/1000.0))
}

// Params uniquely identifies one Open Challenge scenario. Matches
// scenario_catalog.OpenChallengeScenarioParams.
type Params struct {
	Widths    WidthSet
	Section   simconfig.Section
	Direction trackmodel.Direction
	Index     int
	StartCell int
}

// Label is the human-readable scenario label, byte-identical to
// OpenChallengeScenarioParams.label so a Go report and a Python one name the
// same scenario the same way.
func (p Params) Label() string {
	abbrev := "ccw"
	if p.Direction == trackmodel.Clockwise {
		abbrev = "cw"
	}
	return fmt.Sprintf("open_%04d[%s/%s]", p.Index, p.Section, abbrev)
}

// ID is the label reduced to a filesystem- and CLI-safe token,
// e.g. "open_0000". Zero-padded to four digits so a lexicographic sort of
// the written metadata filenames -- which is what corpus.Load does -- is
// also the index order.
func (p Params) ID() string {
	return fmt.Sprintf("open_%04d", p.Index)
}

// Space returns the complete legal Open Challenge scenario space in Python's
// deterministic enumeration order. See the package doc on why the nesting
// below is load-bearing.
func Space() []Params {
	// 2 directions x 16 layouts x 4 sections x 4..6 cells. The exact total
	// (640 for the shipped band layout) is deliberately not asserted here --
	// it follows from the geometry, and hardcoding it would turn a track
	// change into a mismatch rather than a new, larger corpus.
	params := make([]Params, 0, 640)
	index := 0
	for _, direction := range DirectionOrder {
		for bits := range WidthLayoutCount {
			widths := WidthSetFromBits(bits)
			for _, section := range SectionOrder {
				for cell := range widths.StartCellCount(section) {
					params = append(params, Params{
						Index:     index,
						Direction: direction,
						Widths:    widths,
						Section:   section,
						StartCell: cell,
					})
					index++
				}
			}
		}
	}
	return params
}

// OuterWallCells filters a space down to start cell 0 only -- the legacy
// "open128" corpus, which is the same layout/section/direction grid with the
// spawn pinned hard against the outer wall.
//
// Kept because every Open pass rate in this project's history before the
// balanced corpora (96 -> 125 -> 126) was measured on it, so it is the only
// corpus those figures can be compared against. It is NOT the corpus to
// screen anything new on: a blind robot's opening readings depend on where
// across the corridor it starts, and this freezes exactly that.
func OuterWallCells(params []Params) []Params {
	filtered := make([]Params, 0, len(params)/4)
	for _, p := range params {
		if p.StartCell == 0 {
			filtered = append(filtered, p)
		}
	}
	return filtered
}
