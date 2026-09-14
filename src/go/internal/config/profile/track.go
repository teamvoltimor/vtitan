package profile

// DefaultTrackTOMLPath is src/config/track.toml, relative to
// the repo root. Unlike robot.toml, it has no per-component profile
// overlays -- pass nil profileNames to Load. The file's shape is the
// generated generated.TrackConfig DTO.
const DefaultTrackTOMLPath = "src/config/track.toml"
