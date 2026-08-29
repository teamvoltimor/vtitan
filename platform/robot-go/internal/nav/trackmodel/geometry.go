package trackmodel

// Section identifies one of the four navigable corridors of the WRO track,
// matching shared.domain.enums.Section.
type Section int

// InnerBlock is the bounding box of the track's central obstacle block,
// matching shared.domain.models.InnerBlock.
type InnerBlock struct {
	XMin, YMin, XMax, YMax float64
}

// CorridorGeometry is the complete track corridor width layout (meters),
// matching shared.domain.models.CorridorGeometry.
type CorridorGeometry struct {
	NorthWidthM, SouthWidthM, EastWidthM, WestWidthM float64
	InnerBlock                                       InnerBlock
}

const (
	North Section = iota
	South
	East
	West
)

// MinWidthM returns the narrowest corridor width across all four sides.
func (g CorridorGeometry) MinWidthM() float64 {
	return min(g.NorthWidthM, g.SouthWidthM, g.EastWidthM, g.WestWidthM)
}

// MeanWidthM returns the mean corridor width across all four sides.
func (g CorridorGeometry) MeanWidthM() float64 {
	const sides = 4
	return (g.NorthWidthM + g.SouthWidthM + g.EastWidthM + g.WestWidthM) / sides
}

// CorridorGeometryFromWidths builds a CorridorGeometry from a per-section
// width map, matching CorridorGeometry.from_width_dict / track_geometry.py's
// corridor_geometry_from_widths. maxCoord is the track's outer boundary
// (profile.TrackConfig.Track.MaxCoord) -- only the max, not the min, feeds
// the inner-block corners, matching the Python original exactly.
func CorridorGeometryFromWidths(widths map[Section]float64, maxCoord float64) CorridorGeometry {
	north, south, east, west := widths[North], widths[South], widths[East], widths[West]
	return CorridorGeometry{
		NorthWidthM: north,
		SouthWidthM: south,
		EastWidthM:  east,
		WestWidthM:  west,
		InnerBlock: InnerBlock{
			XMin: west,
			YMin: south,
			XMax: maxCoord - east,
			YMax: maxCoord - north,
		},
	}
}
