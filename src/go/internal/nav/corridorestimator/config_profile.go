package corridorestimator

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/blind_nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with blind_nav.NavigationBlindNavCorridorEstimator and (for the alignment gate)
// blind_nav.NavigationBlindNavDirectionEstimator, each loaded from
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

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultCorridorEstimatorTOMLPath), nil,
		func(ce blind_nav.NavigationBlindNavCorridorEstimator) {
			cfg.MinSamples = ce.MinSamples
			cfg.PlausibleWidthMarginM = ce.PlausibleWidthMarginM
			cfg.MaxStartSamples = ce.MaxStartSamples
			cfg.DecisionBoundaryM = ce.DecisionBoundaryM
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultDirectionEstimatorTOMLPath), nil,
		func(de blind_nav.NavigationBlindNavDirectionEstimator) {
			cfg.AlignmentToleranceRad = de.AlignmentToleranceRad
		})

	return cfg
}
