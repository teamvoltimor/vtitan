package generate_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

const _cellTolerance = 1e-9

// acrossAlong projects a track-coordinate point onto the starting section's own
// axes: distance out from the outer wall, and distance along the corridor.
func acrossAlong(section simconfig.Section, p simconfig.Vec2) (across, along float64) {
	trackMax := simconfig.TrackMaxCoord
	switch section {
	case simconfig.SectionSouth:
		return p[1], p[0]
	case simconfig.SectionNorth:
		return trackMax - p[1], p[0]
	case simconfig.SectionWest:
		return p[0], p[1]
	default: // East
		return trackMax - p[0], p[1]
	}
}

// TestStartCells_CountPerWidth pins the rule that makes the layout conditional:
// 0.40+0.20 = 0.60 fills a narrow corridor exactly, so its innermost band falls
// under the centre square and cannot be a start.
func TestStartCells_CountPerWidth(t *testing.T) {
	for _, tc := range []struct {
		name  string
		width float64
		want  int
	}{
		{"narrow", simconfig.CorridorNarrow, 4},
		{"wide", simconfig.CorridorWide, 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			for _, section := range simconfig.AllSections {
				if got := len(generate.StartCells(section, tc.width)); got != tc.want {
					t.Errorf("%s/%s: %d cells, want %d", section, tc.name, got, tc.want)
				}
			}
		})
	}
}

// TestStartCells_SpawnOffsets checks the spawn sits at the chosen offset out
// from the outer wall — an absolute distance, identical in narrow and wide
// corridors, unlike the fraction-of-width model this replaced.
//
// The values are derived: the chassis is pushed flush against one edge of its
// band rather than centred in it, so they follow the chassis width rather than
// being round numbers. Band 1 is the one that matters — it is barely wider than
// the robot, and hugging its outer edge doubles the clearance to the inner
// block over centring.
func TestStartCells_SpawnOffsets(t *testing.T) {
	o := simconfig.StartingZoneSpawnOffsets
	for _, tc := range []struct {
		name  string
		width float64
		want  []float64
	}{
		{"narrow", simconfig.CorridorNarrow, []float64{o[0], o[0], o[1], o[1]}},
		{"wide", simconfig.CorridorWide, []float64{o[0], o[0], o[1], o[1], o[2], o[2]}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			for _, section := range simconfig.AllSections {
				cells := generate.StartCells(section, tc.width)
				for i, cell := range cells {
					across, _ := acrossAlong(section, cell.Spawn)
					if math.Abs(across-tc.want[i]) > _cellTolerance {
						t.Errorf("%s cell %d: spawn %.4f m from outer wall, want %.4f",
							section, i, across, tc.want[i])
					}
				}
			}
		})
	}
}

// TestStartCells_ChassisInsideBand is the property the offsets exist to satisfy:
// no start may straddle a band boundary, or the robot begins in two sections of
// the starting square at once.
func TestStartCells_ChassisInsideBand(t *testing.T) {
	half := simconfig.RobotWidth / 2
	for _, width := range []float64{simconfig.CorridorNarrow, simconfig.CorridorWide} {
		for _, section := range simconfig.AllSections {
			for i, cell := range generate.StartCells(section, width) {
				zoneAcross, _ := acrossAlong(section, cell.ZoneCentre)
				spawnAcross, _ := acrossAlong(section, cell.Spawn)
				bandLo := zoneAcross - cell.BandWidth/2
				bandHi := zoneAcross + cell.BandWidth/2

				if spawnAcross-half < bandLo-_cellTolerance {
					t.Errorf("%s w=%.1f cell %d: chassis outer edge %.4f < band start %.4f",
						section, width, i, spawnAcross-half, bandLo)
				}
				if spawnAcross+half > bandHi+_cellTolerance {
					t.Errorf("%s w=%.1f cell %d: chassis inner edge %.4f > band end %.4f",
						section, width, i, spawnAcross+half, bandHi)
				}
			}
		}
	}
}

// TestStartCells_WithinCorridor guards the innermost start: the chassis must
// stay clear of the inner block, and every cell must stay on the mat.
func TestStartCells_WithinCorridor(t *testing.T) {
	half := simconfig.RobotWidth / 2
	for _, width := range []float64{simconfig.CorridorNarrow, simconfig.CorridorWide} {
		for _, section := range simconfig.AllSections {
			for i, cell := range generate.StartCells(section, width) {
				across, along := acrossAlong(section, cell.Spawn)
				if across-half < -_cellTolerance {
					t.Errorf("%s w=%.1f cell %d: chassis crosses the outer wall", section, width, i)
				}
				if across+half > width+_cellTolerance {
					t.Errorf("%s w=%.1f cell %d: chassis crosses into the inner block (%.4f > %.4f)",
						section, width, i, across+half, width)
				}
				if along != simconfig.GridLengthSectionLeft && along != simconfig.GridLengthSectionRight {
					t.Errorf("%s w=%.1f cell %d: along-corridor %.4f is not a cell midpoint",
						section, width, i, along)
				}
			}
		}
	}
}

// TestStartCells_OrderedOuterInward pins index 0 as the cell against the outer
// wall, which callers rely on for a deterministic default start.
func TestStartCells_OrderedOuterInward(t *testing.T) {
	for _, section := range simconfig.AllSections {
		cells := generate.StartCells(section, simconfig.CorridorWide)
		prev := math.Inf(-1)
		for i, cell := range cells {
			across, _ := acrossAlong(section, cell.Spawn)
			if across < prev-_cellTolerance {
				t.Errorf("%s cell %d: offset %.4f goes back outward from %.4f", section, i, across, prev)
			}
			prev = across
		}
	}
}

// TestStartCells_ZoneIsBandCentre keeps the painted rectangle centred on its
// band even though the spawn inside it is deliberately off-centre.
func TestStartCells_ZoneIsBandCentre(t *testing.T) {
	wantCentres := []float64{0.20, 0.20, 0.50, 0.50, 0.80, 0.80}
	wantBands := []float64{0.40, 0.40, 0.20, 0.20, 0.40, 0.40}
	for _, section := range simconfig.AllSections {
		for i, cell := range generate.StartCells(section, simconfig.CorridorWide) {
			across, _ := acrossAlong(section, cell.ZoneCentre)
			if math.Abs(across-wantCentres[i]) > _cellTolerance {
				t.Errorf("%s cell %d: zone centre %.4f, want %.4f", section, i, across, wantCentres[i])
			}
			if math.Abs(cell.BandWidth-wantBands[i]) > _cellTolerance {
				t.Errorf("%s cell %d: band width %.4f, want %.4f", section, i, cell.BandWidth, wantBands[i])
			}
		}
	}
}
