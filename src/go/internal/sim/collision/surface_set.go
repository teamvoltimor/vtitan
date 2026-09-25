package collision

// SurfaceSet is a set of ContactSurface values. Every rulebook-driven
// surface classification (forbidden walls per challenge, unforgivable
// contacts, solid surfaces blocking a step) uses it, so the semantics are
// spelled out once instead of in three map[ContactSurface]bool literals.
type SurfaceSet map[ContactSurface]struct{}

// NewSurfaceSet builds a set from the given surfaces.
func NewSurfaceSet(surfaces ...ContactSurface) SurfaceSet {
	s := make(SurfaceSet, len(surfaces))
	for _, c := range surfaces {
		s[c] = struct{}{}
	}
	return s
}

// Contains reports whether the surface is in the set. A nil or empty set
// contains nothing.
func (s SurfaceSet) Contains(c ContactSurface) bool {
	_, ok := s[c]
	return ok
}

// SolidSurfacesFor is the set of surfaces that physically stop the chassis
// in a challenge whose run-ending surfaces are terminal, matching
// simulator.py: every surface whose contact does NOT end the run, plus the
// parking lot always. A terminal surface is left passable because touching
// it ends the run anyway (after the start-of-run grace, during which the
// body must be free to work itself clear); the parking fins are solid even
// though touching them is terminal, since a chassis resting against a fin
// is touching it (adr:0062-sim-contact-model-and-parking).
func SolidSurfacesFor(terminal SurfaceSet) SurfaceSet {
	solid := NewSurfaceSet(SurfaceParkingLot)
	for _, c := range []ContactSurface{SurfaceOuterWall, SurfaceInnerWall, SurfaceObstacle} {
		if !terminal.Contains(c) {
			solid[c] = struct{}{}
		}
	}
	return solid
}
