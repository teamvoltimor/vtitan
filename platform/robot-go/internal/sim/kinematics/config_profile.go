package kinematics

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literal,
// overlaid with profile.PursuitConfig loaded from
// <configRoot>/profile.DefaultPursuitTOMLPath if configRoot is non-empty
// and loading succeeds; otherwise, or on any load failure, the literal
// default, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	path := filepath.Join(configRoot, profile.DefaultPursuitTOMLPath)
	pursuit, err := profile.Load[profile.PursuitConfig](path, nil)
	if err != nil {
		logger.Warn("kinematics: loading pursuit.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.MaxSteeringRateRadPerS = pursuit.MaxSteeringRate
	return cfg
}
