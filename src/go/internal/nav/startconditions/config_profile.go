package startconditions

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with waypoints.ConfigFor's own resolution and
// profile.TrackConfig (from <configRoot>/profile.DefaultTrackTOMLPath) when
// configRoot is non-empty and loading succeeds; otherwise, or on load
// failure, the literal defaults, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	cfg.Waypoints = waypoints.ConfigFor(logger, configRoot)
	if configRoot == "" {
		return cfg
	}

	trackPath := filepath.Join(configRoot, profile.DefaultTrackTOMLPath)
	loaded, err := profile.Load[profile.TrackConfig](trackPath, nil)
	if err != nil {
		logger.Warn("startconditions: loading track.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}
	cfg.TrackMaxCoordM = loaded.Track.MaxCoord
	cfg.NarrowWidthM = loaded.Corridor.Narrow

	return cfg
}
