// Package foxglove implements enough of the Foxglove WebSocket Protocol
// (subprotocol "foxglove.websocket.v1") for Foxglove Studio to connect live
// and render published channels -- the rviz2 replacement's server side,
// per platform/robot/docs/internal/plans/go-migration-plan.md's
// "Visualization" row.
//
// Scope: channel advertisement (server -> client, one per registered
// subject/schema pair), client subscribe/unsubscribe, and binary Message
// Data frames carrying already-serialized protobuf payloads. NOT
// implemented: client-side publishing, services, parameters, connection
// graph, or any capability this bridge's one-way (robot -> Studio)
// telemetry use case doesn't need -- see the protocol's own spec at
// https://github.com/foxglove/ws-protocol for the full surface.
//
// This package owns the WIRE PROTOCOL only. cmd/foxglove-bridge owns
// deciding WHAT to bridge (which NATS subjects, which protobuf types).
package foxglove
