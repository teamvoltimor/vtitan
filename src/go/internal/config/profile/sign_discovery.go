package profile

// DefaultSignDiscoveryTOMLPath is where sign_discovery.toml lives, relative to
// the repo root. The file's shape is the generated
// signs.NavigationSignsSignDiscovery DTO; the generated
// MinReliableBboxHeightPx spelling and int type are kept, and the registry in
// defaults.go carries the fallbacks its schema does not tag.
const DefaultSignDiscoveryTOMLPath = "src/config/navigation/signs/sign_discovery.toml"
