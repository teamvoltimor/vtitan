package uiv1

// ButtonEventSubject/TelemetrySummarySubject and friends are the NATS
// subjects declared in this package's .proto docstrings -- named here, next
// to the generated message types, so every publisher/subscriber pair
// references one source of truth instead of each cmd/* binary retyping the
// same literal.
const (
	ButtonEventSubject      = "vtitan.ui.v1.button_event"
	TelemetrySummarySubject = "vtitan.ui.v1.telemetry_summary"
	ButtonHoldSubject       = "vtitan.ui.v1.button_hold"
	JumperInsertedSubject   = "vtitan.ui.v1.jumper_inserted"
	// ChallengeModeActiveSubject carries the RESOLVED mode the stack should
	// act on; JumperInsertedSubject carries the raw hardware reading. See
	// challenge_mode.proto for why both exist.
	ChallengeModeActiveSubject = "vtitan.ui.v1.challenge_mode_active"
)
