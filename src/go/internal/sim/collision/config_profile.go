package collision

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/simulation"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with simulation.NavigationSimulationSimulation loaded from
// <configRoot>/profile.DefaultSimulationTOMLPath if configRoot is
// non-empty and loading succeeds; otherwise, or on any load failure, the
// literal defaults, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultSimulationTOMLPath), nil,
		func(sim simulation.NavigationSimulationSimulation) {
			cfg.CollisionMarginM = sim.CollisionMarginM
			cfg.AxisAlignTolerance = sim.AxisAlignTolerance
		})
	return cfg
}
