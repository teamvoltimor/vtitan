package profile

// DefaultCorridorFollowerTOMLPath is
// src/config/navigation/blind_nav/corridor_follower.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
//
// The file's shape is the generated
// blind_nav.NavigationBlindNavCorridorFollower DTO; the generated
// BayExitSpeedMps spelling is kept, and the registry in defaults.go carries
// the fallbacks its schema does not tag.
const DefaultCorridorFollowerTOMLPath = "src/config/navigation/blind_nav/corridor_follower.toml"

// DefaultBayWallClearanceM matches
// NavigationBlindNavCorridorFollower.BayWallClearanceM's registry fallback.
const DefaultBayWallClearanceM = 0.20
