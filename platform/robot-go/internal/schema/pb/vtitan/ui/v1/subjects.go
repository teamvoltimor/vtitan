package uiv1

// ButtonEventSubject/TelemetrySummarySubject are the NATS subjects declared
// in button_event.proto/telemetry_summary.proto's own docstrings -- named
// here, next to the generated message types, so every publisher/subscriber
// pair references one source of truth instead of each cmd/* binary
// retyping the same literal.
const (
	ButtonEventSubject      = "vtitan.ui.v1.button_event"
	TelemetrySummarySubject = "vtitan.ui.v1.telemetry_summary"
)
