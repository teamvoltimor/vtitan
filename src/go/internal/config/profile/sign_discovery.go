package profile

// SignDiscoveryConfig mirrors platform/config/navigation/signs/sign_discovery.toml
// (NavigationTuning.sign_discovery): the gates the camera-discovery map applies
// before it will publish a sign to the router.
type SignDiscoveryConfig struct {
	// MinReliableBBoxHeightPX matches MIN_RELIABLE_BBOX_HEIGHT_PX: below this
	// the pinhole range estimate is not trusted and the detection is dropped.
	MinReliableBBoxHeightPX float64 `mapstructure:"min_reliable_bbox_height_px" default:"5.0"`
	// MaxIngestRangeM matches MAX_INGEST_RANGE_M.
	MaxIngestRangeM float64 `mapstructure:"max_ingest_range_m" default:"2.0"`
	// AssociationDistM matches ASSOCIATION_DIST_M: how close two observations
	// must land to be treated as the same sign.
	AssociationDistM float64 `mapstructure:"association_dist_m" default:"0.25"`
	// MinHits matches MIN_HITS: confirming observations before publishing.
	MinHits int `mapstructure:"min_hits" default:"3"`
	// RobotCorridorFlipTicks matches ROBOT_CORRIDOR_FLIP_TICKS: ticks the
	// robot's own corridor must settle before discovery accepts the change.
	RobotCorridorFlipTicks int `mapstructure:"robot_corridor_flip_ticks" default:"5"`
}

// DefaultSignDiscoveryTOMLPath is where sign_discovery.toml lives, relative to
// the repo root.
const DefaultSignDiscoveryTOMLPath = "src/config/navigation/signs/sign_discovery.toml"
