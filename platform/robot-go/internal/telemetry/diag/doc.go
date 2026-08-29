// Package diag aggregates several sensor/perception inputs (LIDAR scan,
// IMU orientation, vision detections) into the low-rate summary the OLED/UI
// needs, mirroring telemetry_bridge_node.py's `_publish_ui_summary` path
// (platform/robot/ros2_ws/src/vtitan_state_machine/vtitan_state_machine/
// telemetry_bridge_node.py) and the wire shape of TelemetrySummaryWire
// (platform/robot/src/ros2/wire_models.py).
//
// This is a telemetry-aggregation node, not a hardware driver: it owns no
// sensor I/O of its own, only the fan-in/summarize logic that sits between
// several input topics and one low-rate output topic. Real NATS
// subscriptions (internal/transport/nats is still a one-line stub) and the
// corresponding ui/telemetry_summary.proto (not yet designed — only 5
// proto files exist in proto/ so far, none for this output) are both out
// of scope for this package until those two pieces exist elsewhere in the
// tree; Aggregator is built against the small Source interface (source.go)
// so both can be wired in later without touching this logic, the same
// pattern internal/sim/scenario.Orchestrator uses against its Runner
// interface.
package diag
