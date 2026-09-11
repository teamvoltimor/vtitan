// Package backoff implements the reconnect-backoff policy shared by the
// state machine node's two backend gRPC channels today
// (platform/robot/ros2_ws/src/vtitan_state_machine/vtitan_state_machine/
// command_channel.py and telemetry_ingest_channel.py, via
// grpc_backoff.py's BACKOFF_INITIAL_S/BACKOFF_MAX_S constants) and by
// whatever replaces them once NATS wiring lands
// (docs/internal/plans/go-migration-plan.md flags NATS reconnect behavior
// over the USB-gadget link as still unverified). The retry pacing itself
// is a real Config-worthy tunable -- how fast a lost connection should be
// retried is an operational judgment call, not a fixed fact -- unlike,
// say, the single-slot queue size in internal/statemachine/outbox, which
// is architectural and not meant to be tuned.
package backoff
