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
