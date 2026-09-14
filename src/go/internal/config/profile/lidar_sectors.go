package profile

// DefaultLidarSectorsTOMLPath is
// src/config/navigation/sensors/lidar_sectors.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// sensors.NavigationSensorsLidarSectors DTO; its one untagged fallback
// (rear_self_detection_from_chassis) lives in the defaults.go registry.
const DefaultLidarSectorsTOMLPath = "src/config/navigation/sensors/lidar_sectors.toml"
