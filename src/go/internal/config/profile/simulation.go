package profile

// SimulationConfig mirrors the subset of
// platform/shared/config/navigation/simulation/simulation.toml
// (shared.config.navigation_tuning.simulation_params.SimulationParams)
// that internal/sim/collision consumes: the collision-check keep-out
// margin and the axis-alignment tolerance used to swap an obstacle box's
// extents for a quarter-turned pose. The remaining simulator-only fields
// (start-collision grace window, LIDAR dropout rate, detection confidence,
// no-progress detection) belong to scoring/sensor-emulation logic not
// ported to Go yet, so they're omitted here rather than mirrored unused.
type SimulationConfig struct {
	// CollisionMarginM matches COLLISION_MARGIN_M.
	CollisionMarginM float64 `mapstructure:"collision_margin_m"`
	// AxisAlignTolerance matches AXIS_ALIGN_TOLERANCE.
	AxisAlignTolerance float64 `mapstructure:"axis_align_tolerance"`
}

// DefaultSimulationTOMLPath is
// platform/shared/config/navigation/simulation/simulation.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultSimulationTOMLPath = "platform/shared/config/navigation/simulation/simulation.toml"
