package navv1

// NavigatorDebugSubject/LapsCompletedSubject/CurrentCorridorSubject are the
// NATS subjects declared in navigator_debug.proto/race_progress.proto's own
// docstrings -- named here, next to the generated message types, so every
// publisher/subscriber pair references one source of truth instead of each
// cmd/* binary retyping the same literal.
const (
	NavigatorDebugSubject  = "vtitan.nav.v1.navigator_debug"
	LapsCompletedSubject   = "vtitan.nav.v1.laps_completed"
	CurrentCorridorSubject = "vtitan.nav.v1.current_corridor"
)
