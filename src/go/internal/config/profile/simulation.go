package profile

// DefaultSimulationTOMLPath is
// src/config/navigation/simulation/simulation.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
//
// The file's shape is the generated simulation.NavigationSimulationSimulation
// DTO; internal/sim/collision consumes its collision-check keep-out margin and
// the axis-alignment tolerance used to swap an obstacle box's extents for a
// quarter-turned pose. The remaining simulator-only fields (start-collision
// grace window, LIDAR dropout rate, detection confidence, no-progress
// detection) belong to scoring/sensor-emulation logic not ported to Go yet.
const DefaultSimulationTOMLPath = "src/config/navigation/simulation/simulation.toml"
