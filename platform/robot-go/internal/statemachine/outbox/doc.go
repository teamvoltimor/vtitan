// Package outbox implements the keep-latest, non-blocking handoff
// TelemetryIngestChannel uses to move outbound telemetry from a fast
// producer to a slower/reconnecting backend stream
// (platform/robot/ros2_ws/src/vtitan_state_machine/vtitan_state_machine/
// telemetry_ingest_channel.py's `push_snapshot`/`push_topics`/
// `_push_latest`, backed by a `queue.Queue(maxsize=1)`). Slot is a
// concrete generic type, not an interface, because there is nothing here
// to swap out -- it's pure in-process synchronization with no external
// dependency, so an interface would be the kind of speculative
// abstraction go-architect §10 and hexagonal-arch §7 both warn against
// ("a shared seam is only real once two consumers need it"). The
// interface half of this package's design is Streamer, in drain.go: what
// a background transport loop needs to consume a Slot, defined at the
// point of use exactly like internal/telemetry/diag.Source and
// internal/sim/scenario.Runner.
package outbox
