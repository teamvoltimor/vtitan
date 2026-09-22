package wallheading

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig, overlaid with
// sensors.NavigationSensorsWallHeading (from
// <configRoot>/profile.DefaultWallHeadingTOMLPath), when configRoot is
// non-empty and the load succeeds; otherwise the default, logged.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultWallHeadingTOMLPath), nil,
		func(loaded sensors.NavigationSensorsWallHeading) {
			cfg.MinConcentration = loaded.MinConcentration
			cfg.BaselineRays = loaded.BaselineRays
			cfg.MaxSegmentJumpM = loaded.MaxSegmentJumpM
			cfg.MinSegmentM = loaded.MinSegmentM
			cfg.NearMaxRangeM = loaded.NearMaxRangeM
			cfg.MinReturns = loaded.MinReturns
		})

	return cfg
}
