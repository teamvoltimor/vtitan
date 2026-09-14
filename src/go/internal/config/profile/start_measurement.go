package profile

// DefaultStartMeasurementTOMLPath is
// src/config/navigation/sensors/start_measurement.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// sensors.NavigationSensorsStartMeasurement DTO, the parameters for
// internal/nav/startmeasurement's scan-derived starting pose.
const DefaultStartMeasurementTOMLPath = "src/config/navigation/sensors/start_measurement.toml"
