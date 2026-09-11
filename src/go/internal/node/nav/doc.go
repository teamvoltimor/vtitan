// Package nav adapts the navigator domain package onto the wire.
//
// It converts navigator.DebugSnapshot and the navigator's lap/corridor
// progress into their vtitan.nav.v1 protobuf forms, and publishes them over
// NATS. The conversion lives here rather than in internal/nav/navigator for
// the same reason internal/node/motor owns StatusFor: the domain packages
// must not depend on the generated schema, so an adapter sits between them.
//
// # Presence is the contract
//
// NavigatorDebug's sub-messages carry meaning by being ABSENT. A nil group
// means that branch of Step did not run this tick, and a nil field inside a
// present group means Python's own "not computed on this branch" null. Both
// layers are preserved here deliberately: collapsing either would turn a
// deliberately-absent value into a measured 0.0 and quietly corrupt the
// parity comparison the whole migration stage is gated on.
//
// Parking and blind-creep groups are never populated. ParkController and the
// blind-mode bootstrap have no Go counterpart yet (see the navigator
// package's doc.go), so emitting an empty group would assert "this branch
// ran and found nothing" when the truth is "this branch does not exist here".
package nav
