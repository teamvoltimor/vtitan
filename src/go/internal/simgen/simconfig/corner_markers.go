package simconfig

import "math"

// CornerMarker is one of the eight diagonal decorative markers on the WRO
// 2026 field: two per corner (one blue, one orange). Positions and
// rotations are fixed by field geometry (pi/6 increments). Both the SDF
// world generator (sdf/world.go) and the SVG preview renderer
// (preview/svg.go) iterate this single table so they cannot drift apart.
type CornerMarker struct {
	Name   string  // SDF model name (ignored by the SVG renderer)
	Blue   bool    // true = blue marker, false = orange
	CX, CY float64 // center position, meters
	YawRad float64 // rotation about Z, radians
}

const (
	// markerInsetHalf and markerInsetDiag are the two distances a marker
	// centre sits in from the nearest track edge (0.50 m and approximately
	// 0.50*sqrt(2) m). The far variants are their complements measured from
	// track.TrackMaxCoord, where each corner's twin pair sits.
	markerInsetHalf = 0.50
	markerInsetDiag = 0.71

	// Corner marker yaw angles, in 30 degree increments.
	cornerYaw30  = math.Pi / 6
	cornerYaw60  = 2 * math.Pi / 6
	cornerYaw120 = 4 * math.Pi / 6
	cornerYaw150 = 5 * math.Pi / 6
)

// CornerMarkers returns the shared geometry table, ordered NE, SE, SW, NW.
// It is a function of the loaded Track because the far variants depend on
// TrackMaxCoord, which is runtime data now.
func CornerMarkers(track *Track) []CornerMarker {
	farHalf := track.TrackMaxCoord - markerInsetHalf
	farDiag := track.TrackMaxCoord - markerInsetDiag
	return []CornerMarker{
		{Name: ModelCornerNEBlue, Blue: true, CX: farHalf, CY: farDiag, YawRad: cornerYaw30},
		{Name: ModelCornerNEOrange, Blue: false, CX: farDiag, CY: farHalf, YawRad: cornerYaw60},
		{Name: ModelCornerSEOrange, Blue: false, CX: farHalf, CY: markerInsetDiag, YawRad: -cornerYaw30},
		{Name: ModelCornerSEBlue, Blue: true, CX: farDiag, CY: markerInsetHalf, YawRad: -cornerYaw60},
		{Name: ModelCornerSWBlue, Blue: true, CX: markerInsetHalf, CY: markerInsetDiag, YawRad: -cornerYaw150},
		{Name: ModelCornerSWOrange, Blue: false, CX: markerInsetDiag, CY: markerInsetHalf, YawRad: -cornerYaw120},
		{Name: ModelCornerNWOrange, Blue: false, CX: markerInsetHalf, CY: farDiag, YawRad: cornerYaw150},
		{Name: ModelCornerNWBlue, Blue: true, CX: markerInsetDiag, CY: farHalf, YawRad: cornerYaw120},
	}
}
