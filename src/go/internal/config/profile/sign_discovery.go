package profile

// SignDiscoveryConfig mirrors platform/config/navigation/signs/sign_discovery.toml
// (NavigationTuning.sign_discovery): the gates the camera-discovery map applies
// before it will publish a sign to the router.
type SignDiscoveryConfig struct {
	// MinReliableBBoxHeightPX matches MIN_RELIABLE_BBOX_HEIGHT_PX: below this
	// the pinhole range estimate is not trusted and the detection is dropped.
	MinReliableBBoxHeightPX float64 `mapstructure:"min_reliable_bbox_height_px"`
	// MaxIngestRangeM matches MAX_INGEST_RANGE_M.
	MaxIngestRangeM float64 `mapstructure:"max_ingest_range_m"`
	// AssociationDistM matches ASSOCIATION_DIST_M: how close two observations
	// must land to be treated as the same sign.
	AssociationDistM float64 `mapstructure:"association_dist_m"`
	// MinHits matches MIN_HITS: confirming observations before publishing.
	MinHits int `mapstructure:"min_hits"`
	// RobotCorridorFlipTicks matches ROBOT_CORRIDOR_FLIP_TICKS: ticks the
	// robot's own corridor must settle before discovery accepts the change.
	RobotCorridorFlipTicks int `mapstructure:"robot_corridor_flip_ticks"`
}

// DefaultSignDiscoveryTOMLPath is where sign_discovery.toml lives, relative to
// the repo root.
const DefaultSignDiscoveryTOMLPath = "platform/config/navigation/signs/sign_discovery.toml"

// SignDiscoveryDefaults mirrors the shipped file, for viper to overlay a
// partial TOML onto.
func SignDiscoveryDefaults() map[string]any {
	return map[string]any{
		"min_reliable_bbox_height_px": 5.0,
		"max_ingest_range_m":          2.0,
		"association_dist_m":          0.25,
		"min_hits":                    3,
		"robot_corridor_flip_ticks":   5,
	}
}
