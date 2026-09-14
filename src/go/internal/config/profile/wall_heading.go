package profile

// DefaultWallHeadingTOMLPath is
// src/config/navigation/sensors/wall_heading.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load. The file's shape is the generated
// sensors.NavigationSensorsWallHeading DTO, the parameters for
// internal/nav/wallheading's absolute-heading estimate.
const DefaultWallHeadingTOMLPath = "src/config/navigation/sensors/wall_heading.toml"
