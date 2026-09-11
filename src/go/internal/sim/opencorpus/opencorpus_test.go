package opencorpus_test

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/opencorpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// poseToleranceM is well under a millimeter: the golden carries six decimal
// places and both sides compute the same closed-form geometry, so anything
// above float noise is a real divergence rather than a rounding difference.
const poseToleranceM = 1e-6

// TestSpace_MatchesPythonEnumeration is the parity gate for this package.
//
// The golden is the exact output of Python's
// scenario_catalog._OPEN_CHALLENGE_SPACE.all_params(), one line per case,
// regenerated with:
//
//	cd platform/robot && PYTHONPATH=. \
//	  VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
//	  pixi run -e dev python <dump script>
//
// It pins the whole tuple -- index, direction, the four widths, section,
// cell, label AND the resulting spawn pose -- because the index is only
// meaningful if everything it maps to is identical. A reordering that kept
// the same 640 scenarios would still invalidate every case number recorded
// against this space, and only a positional comparison catches that.
func TestSpace_MatchesPythonEnumeration(t *testing.T) {
	t.Parallel()

	want := readGolden(t)
	got := opencorpus.Space()
	cfg := startconditions.DefaultConfig()

	if len(got) != len(want) {
		t.Fatalf("Space() has %d scenarios, want %d (the Python space)", len(got), len(want))
	}

	for i, p := range got {
		row := want[i]
		if p.Index != row.index {
			t.Fatalf("case %d: Index = %d, want %d", i, p.Index, row.index)
		}
		if p.Direction.String() != row.direction {
			t.Errorf("case %d: Direction = %q, want %q", i, p.Direction, row.direction)
		}
		if string(p.Section) != row.section {
			t.Errorf("case %d: Section = %q, want %q", i, p.Section, row.section)
		}
		if p.StartCell != row.startCell {
			t.Errorf("case %d: StartCell = %d, want %d", i, p.StartCell, row.startCell)
		}
		gotWidths := [4]int{
			p.Widths.WidthMMFor(simconfig.SectionSouth),
			p.Widths.WidthMMFor(simconfig.SectionNorth),
			p.Widths.WidthMMFor(simconfig.SectionEast),
			p.Widths.WidthMMFor(simconfig.SectionWest),
		}
		if gotWidths != row.widthsMM {
			t.Errorf("case %d: widths (s,n,e,w) = %v, want %v", i, gotWidths, row.widthsMM)
		}
		if p.Label() != row.label {
			t.Errorf("case %d: Label() = %q, want %q", i, p.Label(), row.label)
		}

		meta, err := opencorpus.Metadata(p, cfg)
		if err != nil {
			t.Fatalf("case %d: Metadata: %v", i, err)
		}
		sc := meta.StartingConditions
		if math.Abs(sc.Position.X-row.x) > poseToleranceM ||
			math.Abs(sc.Position.Y-row.y) > poseToleranceM {
			t.Errorf("case %d: spawn = (%.6f, %.6f), want (%.6f, %.6f)",
				i, sc.Position.X, sc.Position.Y, row.x, row.y)
		}
		if math.Abs(sc.Yaw-row.yaw) > poseToleranceM {
			t.Errorf("case %d: yaw = %.6f, want %.6f", i, sc.Yaw, row.yaw)
		}
	}
}

// TestSpace_SectionOrderIsNotAllSections guards the one substitution that
// silently renumbers the corpus: simconfig.AllSections is north-first and
// the Open space is south-first, so reaching for the ready-made constant
// produces 640 valid scenarios under the wrong indices. The golden test
// above would catch it too; this one names the mistake.
func TestSpace_SectionOrderIsNotAllSections(t *testing.T) {
	t.Parallel()

	if opencorpus.SectionOrder[0] != simconfig.SectionSouth {
		t.Errorf("SectionOrder[0] = %q, want %q (Python's _OPEN_SECTIONS)",
			opencorpus.SectionOrder[0], simconfig.SectionSouth)
	}
	if opencorpus.SectionOrder == simconfig.AllSections {
		t.Error("SectionOrder equals simconfig.AllSections; the Open space enumerates south-first")
	}
}

// TestStartCellCount_MatchesPythonLiterals pins the 4/6 counts that
// scenario_catalog.CorridorWidthSet.start_cell_count hardcodes. WidthSet
// derives them from the band layout instead, so this is where a track.toml
// band edit surfaces: it would resize the Go corpus silently while Python's
// literal kept claiming 4 and 6.
func TestStartCellCount_MatchesPythonLiterals(t *testing.T) {
	t.Parallel()

	narrow := opencorpus.WidthSetFromBits(0)
	wide := opencorpus.WidthSetFromBits(0b1111)
	for _, section := range opencorpus.SectionOrder {
		if got := narrow.StartCellCount(section); got != 4 {
			t.Errorf("narrow %s: StartCellCount = %d, want 4", section, got)
		}
		if got := wide.StartCellCount(section); got != 6 {
			t.Errorf("wide %s: StartCellCount = %d, want 6", section, got)
		}
	}
}

// TestOuterWallCells_IsTheLegacy128 checks the open128 filter keeps the full
// layout/section/direction grid: 16 layouts x 4 sections x 2 directions,
// one spawn each.
func TestOuterWallCells_IsTheLegacy128(t *testing.T) {
	t.Parallel()

	legacy := opencorpus.OuterWallCells(opencorpus.Space())
	const want = opencorpus.WidthLayoutCount * len(opencorpus.SectionOrder) * len(opencorpus.DirectionOrder)
	if len(legacy) != want {
		t.Fatalf("OuterWallCells returned %d scenarios, want %d", len(legacy), want)
	}
	for _, p := range legacy {
		if p.StartCell != 0 {
			t.Fatalf("%s: StartCell = %d, want 0", p.ID(), p.StartCell)
		}
	}
}

// TestMetadata_IsAnOpenRound checks the fields the rules fix for every Open
// scenario, and that sign_positions serializes as [] rather than null --
// Python's model_dump emits a list, and a null would make the Go-written
// corpus parse differently in the two languages.
func TestMetadata_IsAnOpenRound(t *testing.T) {
	t.Parallel()

	meta, err := opencorpus.Metadata(opencorpus.Space()[0], startconditions.DefaultConfig())
	if err != nil {
		t.Fatalf("Metadata: %v", err)
	}
	if meta.ChallengeType != "open" {
		t.Errorf("ChallengeType = %q, want \"open\"", meta.ChallengeType)
	}
	if meta.NumSigns != 0 || len(meta.SignPositions) != 0 {
		t.Errorf("NumSigns = %d, SignPositions = %v, want 0 and empty", meta.NumSigns, meta.SignPositions)
	}
	if meta.HasParkingLot || meta.ParkingLot != nil {
		t.Errorf("HasParkingLot = %t, ParkingLot = %v, want false and nil", meta.HasParkingLot, meta.ParkingLot)
	}
	raw, err := json.Marshal(meta)
	if err != nil {
		t.Fatalf("marshalling metadata: %v", err)
	}
	if !strings.Contains(string(raw), `"sign_positions":[]`) {
		t.Errorf("metadata JSON has no empty sign_positions array: %s", raw)
	}
}

// TestWrite_RoundTripsThroughTheCorpusLoader is the contract that matters
// for sim-runner: what Write puts on disk must come back out of the same
// loader every other corpus uses, in index order, and parse into the schema
// NativeRunner reads.
func TestWrite_RoundTripsThroughTheCorpusLoader(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	params := opencorpus.Space()
	written, err := opencorpus.Write(dir, params, startconditions.DefaultConfig())
	if err != nil {
		t.Fatalf("Write: %v", err)
	}
	if len(written) != len(params) {
		t.Fatalf("Write returned %d scenarios, want %d", len(written), len(params))
	}

	loaded, err := corpus.Load(dir)
	if err != nil {
		t.Fatalf("loading the written corpus: %v", err)
	}
	if len(loaded) != len(params) {
		t.Fatalf("corpus.Load found %d scenarios, want %d", len(loaded), len(params))
	}
	// Lexicographic filename order must be index order, or a "case N" in a
	// Go report would not be case N in the space.
	for i, sc := range loaded {
		if sc.ID != params[i].ID() {
			t.Fatalf("loaded[%d].ID = %q, want %q", i, sc.ID, params[i].ID())
		}
	}

	raw, err := os.ReadFile(filepath.Clean(loaded[0].MetadataPath))
	if err != nil {
		t.Fatalf("reading %s: %v", loaded[0].MetadataPath, err)
	}
	var meta generate.Metadata
	if err := json.Unmarshal(raw, &meta); err != nil {
		t.Fatalf("parsing the written metadata as generate.Metadata: %v", err)
	}
	if meta.ScenarioID != 0 || meta.StartingConditions.Section != "south" {
		t.Errorf("round-tripped metadata = id %d section %q, want 0 and \"south\"",
			meta.ScenarioID, meta.StartingConditions.Section)
	}
}

// goldenRow is one line of the Python-generated space dump.
type goldenRow struct {
	direction string
	section   string
	label     string
	widthsMM  [4]int
	index     int
	startCell int
	x, y, yaw float64
}

func readGolden(t *testing.T) []goldenRow {
	t.Helper()

	raw, err := os.ReadFile(filepath.Join("testdata", "python_open_space.csv"))
	if err != nil {
		t.Fatalf("reading the Python space golden: %v", err)
	}

	var rows []goldenRow
	for _, line := range strings.Split(strings.ReplaceAll(string(raw), "\r\n", "\n"), "\n") {
		if line == "" {
			continue
		}
		var r goldenRow
		n, err := fmt.Sscanf(
			strings.ReplaceAll(line, ",", " "),
			"%d %s %d %d %d %d %s %d %s %g %g %g",
			&r.index, &r.direction,
			&r.widthsMM[0], &r.widthsMM[1], &r.widthsMM[2], &r.widthsMM[3],
			&r.section, &r.startCell, &r.label, &r.x, &r.y, &r.yaw,
		)
		if err != nil || n != 12 {
			t.Fatalf("parsing golden line %q: got %d fields, err %v", line, n, err)
		}
		rows = append(rows, r)
	}
	return rows
}

// TestBalanced128_MatchesPython is the cross-language parity gate for the
// screening corpus, and the only thing that makes its SEED meaningful.
//
// The golden is scripts.common.open_cases.balanced_128_cases dumped at four
// seeds. Matching at one seed could be luck in the round-robin; matching at
// four, in order, means the Mersenne Twister reimplementation in pyrandom.go
// is reproducing CPython's stream rather than merely producing a plausible
// permutation.
func TestBalanced128_MatchesPython(t *testing.T) {
	t.Parallel()

	bySeed := map[uint64][]balancedGoldenRow{}
	var seeds []uint64
	for _, row := range readBalancedGolden(t) {
		if _, seen := bySeed[row.seed]; !seen {
			seeds = append(seeds, row.seed)
		}
		bySeed[row.seed] = append(bySeed[row.seed], row)
	}
	if len(seeds) < 2 {
		t.Fatalf("golden covers %d seeds, want several", len(seeds))
	}

	for _, seed := range seeds {
		want := bySeed[seed]
		got, err := opencorpus.Balanced128(seed)
		if err != nil {
			t.Fatalf("seed %d: Balanced128: %v", seed, err)
		}
		if len(got) != len(want) {
			t.Fatalf("seed %d: got %d scenarios, want %d", seed, len(got), len(want))
		}
		for i, p := range got {
			row := want[i]
			gotWidths := [4]int{
				p.Widths.WidthMMFor(simconfig.SectionSouth),
				p.Widths.WidthMMFor(simconfig.SectionNorth),
				p.Widths.WidthMMFor(simconfig.SectionEast),
				p.Widths.WidthMMFor(simconfig.SectionWest),
			}
			if gotWidths != row.widthsMM || string(p.Section) != row.section ||
				p.Direction.String() != row.direction || p.StartCell != row.startCell {
				t.Fatalf("seed %d case %d: got widths %v %s/%s cell %d, want %v %s/%s cell %d",
					seed, i, gotWidths, p.Section, p.Direction, p.StartCell,
					row.widthsMM, row.section, row.direction, row.startCell)
			}
		}
	}
}

// TestBalanced128_CoversTheGridExactlyOnce is the property the corpus is
// named for: 16 layouts x 4 sections x 2 directions, each appearing once.
// A uniform sample of 128 from the full 640 would also be 128 scenarios and
// would NOT have this, which is the whole reason this corpus exists.
func TestBalanced128_CoversTheGridExactlyOnce(t *testing.T) {
	t.Parallel()

	got, err := opencorpus.Balanced128(0)
	if err != nil {
		t.Fatalf("Balanced128: %v", err)
	}
	if len(got) != opencorpus.Balanced128Size {
		t.Fatalf("got %d scenarios, want %d", len(got), opencorpus.Balanced128Size)
	}
	type combo struct {
		widths    opencorpus.WidthSet
		section   simconfig.Section
		direction string
	}
	seen := map[combo]int{}
	cells := map[int]int{}
	for _, p := range got {
		seen[combo{p.Widths, p.Section, p.Direction.String()}]++
		cells[p.StartCell]++
	}
	if len(seen) != opencorpus.Balanced128Size {
		t.Errorf("covered %d distinct layout/section/direction combos, want %d",
			len(seen), opencorpus.Balanced128Size)
	}
	for c, n := range seen {
		if n != 1 {
			t.Errorf("%v appears %d times, want exactly once", c, n)
		}
	}
	// And the cell actually VARIES -- the failure this corpus replaced was
	// every scenario pinned to cell 0.
	if len(cells) < 4 {
		t.Errorf("start cells used = %v, want the cell to vary across the corpus", cells)
	}
}

// balancedGoldenRow is one line of the Python balanced128 dump.
type balancedGoldenRow struct {
	section   string
	direction string
	widthsMM  [4]int
	seed      uint64
	startCell int
}

func readBalancedGolden(t *testing.T) []balancedGoldenRow {
	t.Helper()

	raw, err := os.ReadFile(filepath.Join("testdata", "python_balanced128.csv"))
	if err != nil {
		t.Fatalf("reading the balanced128 golden: %v", err)
	}
	var rows []balancedGoldenRow
	for _, line := range strings.Split(strings.ReplaceAll(string(raw), "\r\n", "\n"), "\n") {
		if line == "" {
			continue
		}
		var r balancedGoldenRow
		n, scanErr := fmt.Sscanf(
			strings.ReplaceAll(line, ",", " "),
			"%d %d %d %d %d %s %s %d",
			&r.seed, &r.widthsMM[0], &r.widthsMM[1], &r.widthsMM[2], &r.widthsMM[3],
			&r.section, &r.direction, &r.startCell,
		)
		if scanErr != nil || n != 8 {
			t.Fatalf("parsing golden line %q: got %d fields, err %v", line, n, scanErr)
		}
		rows = append(rows, r)
	}
	return rows
}
