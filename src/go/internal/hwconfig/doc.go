// Package hwconfig resolves each driver's Config from the shipped TOML tree
// and the active hardware profile.
//
// It exists because the drivers live in pkg/driver, outside this module's
// internal tree, and so cannot import internal/config/{profile,generated}.
// Every other package in this repo keeps its own ConfigFor (navigator,
// waypoints, signrouter, controllers, ...); the drivers are the deliberate
// exception, because being reusable outside vTitan means not knowing how
// vTitan stores its configuration. The split is the price of that, and this
// package is where the knowledge went: it is the only place that knows both
// the TOML tree and the driver Config types.
//
// Each function keeps the fallback behavior its driver had before the move,
// including which failures warn and continue and which return an error --
// Encoder is the one that refuses to guess, for the reason its own doc
// comment gives.
package hwconfig
