package generate

import "github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"

// StartCell is one legal starting position on a side's starting square.
type StartCell struct {
	// ZoneCentre is the midpoint of the painted cell rectangle, in track
	// coordinates. The rectangle spans StartingZoneDefaultLength along the
	// corridor by BandWidth across it.
	ZoneCentre simconfig.Vec2
	// Spawn is where the robot is placed inside that cell. It is offset from
	// ZoneCentre across the corridor - see simconfig.StartingZoneSpawnOffsets
	// for why the two differ.
	Spawn simconfig.Vec2
	// BandWidth is the cross-corridor width of the band the cell sits in, and
	// so the width of the rectangle painted on the mat.
	BandWidth float64
}

// bandFitEpsilon absorbs float noise when testing whether a band's inner edge
// still falls inside the corridor. Without it a 0.20 m band closing exactly on
// a 0.60 m narrow corridor can miss by one ulp and drop two legal cells.
const bandFitEpsilon = 1e-9

// StartCells returns every legal starting cell for a side, ordered outer wall
// inward and, within a band, along the travel axis — so index 0 is always the
// cell hard against the outer wall.
//
// Each side of the mat carries a marked square, a meter along the corridor by
// a meter across, split into simconfig.StartingZoneBandWidths out from the
// outer wall, each band into two cells of simconfig.StartingZoneDefaultLength.
// A band is a legal start only while it lies inside the corridor; past the
// corridor's inner edge the band is under the centre square. So a narrow
// (0.6 m) corridor yields four cells and a wide (1.0 m) one yields six.
//
// Where the robot starts across the corridor is not cosmetic for a blind run:
// it is the input to corridor-width estimation and to the side ranges the
// direction estimator votes on. The fraction-of-width model this replaced
// scaled its offsets with the corridor, so it emitted six positions whatever
// the width and could straddle a band boundary — at 0.42 of a narrow corridor
// the 0.20 m chassis spans 0.32 to 0.52, sitting in two bands at once.
func StartCells(section simconfig.Section, corridorWidth float64) []StartCell {
	trackMax := simconfig.TrackMaxCoord
	// The square occupies the middle meter of the side, leaving a meter of
	// corner region at each end; the two cells sit either side of the midpoint.
	alongs := [2]float64{simconfig.GridLengthSectionLeft, simconfig.GridLengthSectionRight}

	appendCells := func(cells []StartCell, zoneAcross, spawnAcross, band float64) []StartCell {
		for _, along := range alongs {
			zx, zy := computeZoneCoords(section, along, zoneAcross, trackMax)
			sx, sy := computeZoneCoords(section, along, spawnAcross, trackMax)
			cells = append(cells, StartCell{
				ZoneCentre: simconfig.Vec2{zx, zy},
				Spawn:      simconfig.Vec2{sx, sy},
				BandWidth:  band,
			})
		}
		return cells
	}

	cells := make([]StartCell, 0, 2*len(simconfig.StartingZoneBandWidths))
	edge := 0.0
	for i, band := range simconfig.StartingZoneBandWidths {
		farEdge := edge + band
		if farEdge > corridorWidth+bandFitEpsilon {
			break
		}
		cells = appendCells(cells, edge+band/2, simconfig.StartingZoneSpawnOffsets[i], band)
		edge = farEdge
	}

	if len(cells) == 0 {
		// Unreachable for any corridor at or above CorridorMinWidth (0.5 m),
		// which already exceeds the outermost band. Kept so a malformed width
		// degrades to the old centreline spawn rather than panicking callers
		// on an empty slice.
		cells = appendCells(cells, corridorWidth/2, corridorWidth/2, corridorWidth)
	}
	return cells
}
