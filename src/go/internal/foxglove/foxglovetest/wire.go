// Package foxglovetest provides the wire shapes shared by the Foxglove
// protocol tests in internal/foxglove and cmd/foxglove-bridge. It mirrors
// only the parts of the JSON protocol those tests decode, keeping the
// camelCase tags the Foxglove spec mandates rather than the snake_case
// this repo otherwise prefers for JSON.
package foxglovetest

import "github.com/coder/websocket"

// AdvertiseResponse matches the "advertise" server->client JSON message.
type AdvertiseResponse struct {
	Op       string              `json:"op"`
	Channels []AdvertisedChannel `json:"channels"`
}

// AdvertisedChannel is one entry of an AdvertiseResponse's channels array.
type AdvertisedChannel struct {
	ID         uint32 `json:"id"`
	Topic      string `json:"topic"`
	SchemaName string `json:"schemaName"`
}

// ReadResult carries one websocket frame read and its error across the
// goroutine boundary in tests that read once while retrying a write.
type ReadResult struct {
	Kind websocket.MessageType
	Data []byte
	Err  error
}
