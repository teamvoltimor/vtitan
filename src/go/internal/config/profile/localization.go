package profile

// DefaultLocalizationTOMLPath is
// src/config/navigation/blind_nav/localization.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load. The file's shape is the generated
// blind_nav.NavigationBlindNavLocalization DTO; the generated MaxSpeedMps
// spelling is kept and the shipped value and its rationale are unchanged.
const DefaultLocalizationTOMLPath = "src/config/navigation/blind_nav/localization.toml"
