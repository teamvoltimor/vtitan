package main

import (
	"fmt"
	"strings"

	"github.com/spf13/pflag"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensormodel"
)

// registerSensorModelFlag binds --sensor-models into cfg.
func registerSensorModelFlag(flags *pflag.FlagSet, cfg *cliConfig) {
	flags.StringVar(
		&cfg.sensorModels,
		"sensor-models",
		"",
		"--runner native only: measured sensor models to switch on, 'all' or a comma list of "+
			strings.Join(sensormodel.Names(), ", ")+"; none by default (platform plan 2.12)",
	)
}

// sensorModelsFor parses --sensor-models.
func sensorModelsFor(cfg cliConfig) (sensormodel.Set, error) {
	models, err := sensormodel.ParseSet(cfg.sensorModels)
	if err != nil {
		return 0, fmt.Errorf("sim-runner: --sensor-models: %w", err)
	}
	return models, nil
}
