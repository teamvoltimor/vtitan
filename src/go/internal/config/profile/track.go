package profile

// TrackMat mirrors track.toml's [track] section.
type TrackMat struct {
	MatSize   float64 `mapstructure:"mat_size"`
	Size      float64 `mapstructure:"size"`
	MinCoord  float64 `mapstructure:"min_coord"`
	MaxCoord  float64 `mapstructure:"max_coord"`
	CornerMin float64 `mapstructure:"corner_min"`
	CornerMax float64 `mapstructure:"corner_max"`
}

// TrackWall mirrors track.toml's [wall] section.
type TrackWall struct {
	Height             float64    `mapstructure:"height"`
	Thickness          float64    `mapstructure:"thickness"`
	CollisionThickness float64    `mapstructure:"collision_thickness"`
	ExteriorOffset     float64    `mapstructure:"exterior_offset"`
	InteriorOffset     float64    `mapstructure:"interior_offset"`
	Color              [3]float64 `mapstructure:"color"`
}

// TrackCorridor mirrors track.toml's [corridor] section.
type TrackCorridor struct {
	Narrow        float64    `mapstructure:"narrow"`
	Wide          float64    `mapstructure:"wide"`
	Obstacles     float64    `mapstructure:"obstacles"`
	MinWidth      float64    `mapstructure:"min_width"`
	MaxWidth      float64    `mapstructure:"max_width"`
	DivisionLines [2]float64 `mapstructure:"division_lines"`
}

// TrackSign mirrors track.toml's [sign] section.
type TrackSign struct {
	Width                   float64    `mapstructure:"width"`
	Depth                   float64    `mapstructure:"depth"`
	Height                  float64    `mapstructure:"height"`
	ZPosition               float64    `mapstructure:"z_position"`
	GridDepthNear           float64    `mapstructure:"grid_depth_near"`
	GridDepthMiddle         float64    `mapstructure:"grid_depth_middle"`
	GridDepthFar            float64    `mapstructure:"grid_depth_far"`
	PlacementCircleDiameter float64    `mapstructure:"placement_circle_diameter"`
	MinCount                int        `mapstructure:"min_count"`
	MaxCount                int        `mapstructure:"max_count"`
	RedColor                [3]float64 `mapstructure:"red_color"`
	GreenColor              [3]float64 `mapstructure:"green_color"`
	RedStd                  [3]float64 `mapstructure:"red_std"`
	GreenStd                [3]float64 `mapstructure:"green_std"`
}

// TrackParking mirrors track.toml's [parking] section.
type TrackParking struct {
	Length        float64    `mapstructure:"length"`
	Width         float64    `mapstructure:"width"`
	Height        float64    `mapstructure:"height"`
	ZPosition     float64    `mapstructure:"z_position"`
	WallOffset    float64    `mapstructure:"wall_offset"`
	SpacingFactor float64    `mapstructure:"spacing_factor"`
	Color         [3]float64 `mapstructure:"color"`
}

// TrackStartingZone mirrors track.toml's [starting_zone] section.
type TrackStartingZone struct {
	DefaultLength         float64    `mapstructure:"default_length"`
	Thickness             float64    `mapstructure:"thickness"`
	ObstaclesSizeFactor   float64    `mapstructure:"obstacles_size_factor"`
	IndicatorRadius       float64    `mapstructure:"indicator_radius"`
	Color                 [3]float64 `mapstructure:"color"`
	ClockwiseColor        [3]float64 `mapstructure:"clockwise_color"`
	CounterclockwiseColor [3]float64 `mapstructure:"counterclockwise_color"`
	SpawnAlignment        [3]string  `mapstructure:"spawn_alignment"`
}

// TrackMarkings mirrors track.toml's [markings] section.
type TrackMarkings struct {
	OrangeColor [3]float64 `mapstructure:"orange_color"`
	BlueColor   [3]float64 `mapstructure:"blue_color"`
	Angle       float64    `mapstructure:"angle"`
}

// TrackConfig mirrors shared.config.track_constants.TrackConstants's full
// schema, as loaded from src/config/track.toml (the WRO mat
// geometry, distinct from robot.toml's chassis geometry).
type TrackConfig struct {
	Track        TrackMat          `mapstructure:"track"`
	Wall         TrackWall         `mapstructure:"wall"`
	Corridor     TrackCorridor     `mapstructure:"corridor"`
	Sign         TrackSign         `mapstructure:"sign"`
	Parking      TrackParking      `mapstructure:"parking"`
	StartingZone TrackStartingZone `mapstructure:"starting_zone"`
	Markings     TrackMarkings     `mapstructure:"markings"`
}

// DefaultTrackTOMLPath is src/config/track.toml, relative to
// the repo root. Unlike robot.toml, it has no per-component profile
// overlays -- pass nil profileNames to Load.
const DefaultTrackTOMLPath = "src/config/track.toml"
