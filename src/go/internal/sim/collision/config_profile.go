package collision

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with profile.SimulationConfig loaded from
// <configRoot>/profile.DefaultSimulationTOMLPath if configRoot is
// non-empty and loading succeeds; otherwise, or on any load failure, the
// literal defaults, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	path := filepath.Join(configRoot, profile.DefaultSimulationTOMLPath)
	sim, err := profile.Load[profile.SimulationConfig](path, nil)
	if err != nil {
		logger.Warn("collision: loading simulation.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.CollisionMarginM = sim.CollisionMarginM
	cfg.AxisAlignTolerance = sim.AxisAlignTolerance
	return cfg
}
