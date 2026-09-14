package profile

// DefaultWaypointsTOMLPath is
// src/config/navigation/waypoint/waypoints.toml, relative to
// the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
//
// The file's shape is the generated waypoint.NavigationWaypointWaypoints DTO;
// WideCenterBiasSide/NarrowCenterBiasSide stay strings because
// viper/mapstructure has no decode hook for trackmodel.CorridorSide's
// "inner"/"outer" TOML values, so internal/nav/waypoints.ConfigFor parses them
// itself. The fallbacks its schema does not tag live in the defaults.go
// registry.
const DefaultWaypointsTOMLPath = "src/config/navigation/waypoint/waypoints.toml"
