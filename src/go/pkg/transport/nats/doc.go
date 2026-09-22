// Package nats is a thin wrapper around nats.go providing typed
// Publisher[T]/Subscriber[T] with protobuf encode/decode at the edge.
// Reconnection is handled by nats.go's own native logic (see Connect), not
// reimplemented here. Read(ctx) on Subscriber matches the same blocking
// pattern used by driver.Driver[T] and button.Driver elsewhere in this
// module, for one consistent "block until the next value or ctx is done"
// shape across hardware drivers and network subscriptions alike.
package nats
