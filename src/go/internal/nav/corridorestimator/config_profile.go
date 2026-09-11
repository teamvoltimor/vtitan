package corridorestimator

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with profile.CorridorEstimatorConfig and (for the alignment gate)
// profile.DirectionEstimatorConfig, each loaded from
// <configRoot>/profile.DefaultXxxTOMLPath if configRoot is non-empty and
// loading succeeds; otherwise, or on any load failure, the literal defaults,
// logging why.
//
// The two files load and fall back independently, since they are unrelated
// failure domains.
//
// NarrowWidthM/WideWidthM are NOT overlaid: they live in track.toml, which
// has no mirror yet. They are rule constants (0.6/1.0 m) rather than tuning,
// so the literals are the shipped values either way.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	cePath := filepath.Join(configRoot, profile.DefaultCorridorEstimatorTOMLPath)
	if ce, err := profile.Load[profile.CorridorEstimatorConfig](cePath, nil); err != nil {
		logger.Warn("corridorestimator: loading corridor_estimator.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.MinSamples = ce.MinSamples
		cfg.PlausibleWidthMarginM = ce.PlausibleWidthMarginM
		cfg.MaxStartSamples = ce.MaxStartSamples
		cfg.DecisionBoundaryM = ce.DecisionBoundaryM
	}

	dePath := filepath.Join(configRoot, profile.DefaultDirectionEstimatorTOMLPath)
	if de, err := profile.Load[profile.DirectionEstimatorConfig](dePath, nil); err != nil {
		logger.Warn("corridorestimator: loading direction_estimator.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.AlignmentToleranceRad = de.AlignmentToleranceRad
	}

	return cfg
}
