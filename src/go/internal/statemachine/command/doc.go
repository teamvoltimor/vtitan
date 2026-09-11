// Package command implements the backend->robot command-dispatch
// decision logic: given a decoded Command, decide what local action it
// maps to and what ack status/message to report. This is the pure half of
// platform/robot/ros2_ws/src/vtitan_state_machine/vtitan_state_machine/
// command_channel.py's CommandChannel -- specifically `_dispatch_command`
// and its per-kind `_dispatch_*` helpers.
//
// Scoping note, worked out by reading command_channel.py, its sibling
// telemetry_ingest_channel.py, and state_machine_node.py together: despite
// living in the vtitan_state_machine ROS2 package, neither channel is
// owned by state_machine_node.py or its StateMachine (internal/statemachine/core)
// -- both are constructed and driven by telemetry_bridge_node.py instead.
// CommandChannel's only connection to the actual state machine is
// indirect, through one dispatch case: a start/stop/e-stop command
// resolves to publishing a synthetic /button/event, the same String topic
// state_machine_node.py's own physical-button subscription feeds off of.
// Everything else CommandChannel and TelemetryIngestChannel do --
// resolving a robot ID from the backend, opening/retrying a gRPC
// client-stream connection, threading.Event synchronization with a ROS2
// service call -- is backend-transport plumbing with no equivalent in
// this Go tree yet (no NATS wiring, no generated commands.proto/
// ingest.proto client, per docs/internal/plans/go-migration-plan.md's
// "nothing gets wired to real hardware/transport yet" framing for this
// stage). This package therefore ports only the transport-independent
// decision logic -- what a Command maps to, and what ack status/message
// results -- behind narrow ButtonSink/ChannelSink interfaces (defined
// here, at the point of use, per go-architect §4 and matching
// internal/telemetry/diag's Source pattern) that a real gRPC- or
// NATS-backed implementation satisfies later without this package
// changing.
//
// The reconnect-backoff constants both real Python channels share
// (grpc_backoff.py) are ported separately, in internal/statemachine/backoff,
// since they're policy shared by both directions of the eventual real
// wiring, not specific to command dispatch.
package command
