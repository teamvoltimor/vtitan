package profile

// ControlConfig mirrors src/config/navigation/motion/control.toml
// (shared.config.navigation_tuning.motion.ControlLoopParams) in full.
type ControlConfig struct {
	// ControlHz matches CONTROL_HZ -- the navigation control loop's rate,
	// on the robot and in simulation.
	ControlHz float64 `mapstructure:"control_hz"`
}

// DefaultControlTOMLPath is
// src/config/navigation/motion/control.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultControlTOMLPath = "src/config/navigation/motion/control.toml"
