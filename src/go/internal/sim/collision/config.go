package collision

// Config parameterizes TrackModel with the two values this package's own
// tuning group (SimulationParams) owns: the collision-check keep-out
// margin and the axis-alignment tolerance for a quarter-turned obstacle
// pose. Robot/track physical constants (chassis length/width, the track's
// outer boundary) come from profile.RobotConfig/generated.TrackConfig
// instead -- see NewTrackModelParams, which takes those directly.
type Config struct {
	// CollisionMarginM matches SimulationParams.COLLISION_MARGIN_M: how
	// far past the visual wall face the chassis keep-out extends.
	CollisionMarginM float64
	// AxisAlignTolerance matches SimulationParams.AXIS_ALIGN_TOLERANCE:
	// |cos(yaw)| below this counts as a quarter-turn for
	// NewObstacleBoxFromPose.
	AxisAlignTolerance float64
	// SlideOnContact matches simulation.toml's
	// contact_slides_along_surfaces: a blocked translation slides along the
	// surface it hit instead of being scaled to nothing (AllowedStep's
	// slide).
	SlideOnContact bool
}

// Default* match
// shared.config.navigation_tuning.simulation_params.SimulationParams's
// Pydantic field defaults, which the shipped simulation.toml does not
// currently override.
const (
	DefaultCollisionMarginM   = 0.0
	DefaultAxisAlignTolerance = 1e-6
	// DefaultSlideOnContact is the shipped simulation.toml value; the
	// Pydantic field is required and has no default of its own.
	DefaultSlideOnContact = true
)

// DefaultConfig returns the Config matching the Python tuning defaults.
func DefaultConfig() Config {
	return Config{
		CollisionMarginM:   DefaultCollisionMarginM,
		AxisAlignTolerance: DefaultAxisAlignTolerance,
		SlideOnContact:     DefaultSlideOnContact,
	}
}
