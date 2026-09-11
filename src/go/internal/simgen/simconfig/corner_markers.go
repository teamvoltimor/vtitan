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

// CornerMarkers is the shared geometry table, ordered NE, SE, SW, NW.
var CornerMarkers = []CornerMarker{
	{Name: ModelCornerNEBlue, Blue: true, CX: 2.50, CY: 2.29, YawRad: 1 * math.Pi / 6},
	{Name: ModelCornerNEOrange, Blue: false, CX: 2.29, CY: 2.50, YawRad: 2 * math.Pi / 6},
	{Name: ModelCornerSEOrange, Blue: false, CX: 2.50, CY: 0.71, YawRad: -1 * math.Pi / 6},
	{Name: ModelCornerSEBlue, Blue: true, CX: 2.29, CY: 0.50, YawRad: -2 * math.Pi / 6},
	{Name: ModelCornerSWBlue, Blue: true, CX: 0.50, CY: 0.71, YawRad: -5 * math.Pi / 6},
	{Name: ModelCornerSWOrange, Blue: false, CX: 0.71, CY: 0.50, YawRad: -4 * math.Pi / 6},
	{Name: ModelCornerNWOrange, Blue: false, CX: 0.50, CY: 2.29, YawRad: 5 * math.Pi / 6},
	{Name: ModelCornerNWBlue, Blue: true, CX: 0.71, CY: 2.50, YawRad: 4 * math.Pi / 6},
}
