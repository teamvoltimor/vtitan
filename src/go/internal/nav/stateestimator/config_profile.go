package stateestimator

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/blind_nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig, overlaid with
// blind_nav.NavigationBlindNavStateEstimator (from
// <configRoot>/profile.DefaultStateEstimatorTOMLPath), when configRoot is
// non-empty and the load succeeds; otherwise the default, logged.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultStateEstimatorTOMLPath), nil,
		func(loaded blind_nav.NavigationBlindNavStateEstimator) {
			cfg.YawCorrectionGain = loaded.YawCorrectionGain
		})

	return cfg
}
