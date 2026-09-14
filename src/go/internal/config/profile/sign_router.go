package profile

// DefaultSignRouterTOMLPath is
// src/config/navigation/signs/sign_router.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated signs.NavigationSignsSignRouter
// DTO; the generated SignLane* spellings for the relabel and depth-consistency
// fields are kept, and the registry in defaults.go carries the fallbacks its
// schema does not tag.
const DefaultSignRouterTOMLPath = "src/config/navigation/signs/sign_router.toml"

// Default* match SignRouterParams' Pydantic defaults for the four fields
// missing from the checked-in sign_router.toml.
const (
	DefaultDepthPin                = true
	DefaultPinCornerGuard          = true
	DefaultPinHeadingGuard         = true
	DefaultPinHeadingGuardDeg      = 35.0
	DefaultRelabelUnsatisfiable    = true
	DefaultDepthConsistentCorridor = true
)
