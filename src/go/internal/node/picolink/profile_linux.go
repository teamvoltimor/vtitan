//go:build linux

package picolink

import (
	"fmt"
	"log/slog"

	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	nodemotor "github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
)

// DefaultCommandTimeout is the Zero's own command deadline
// (internal/node/motor.DefaultCommandTimeout), so a board swap does not
// quietly change how long the car keeps driving on a silent command stream.
const DefaultCommandTimeout = nodemotor.DefaultCommandTimeout

// LoadProfile resolves a Profile from configRoot and the active hardware
// profile (VTITAN_HARDWARE_PROFILE), calling the same resolvers the Zero's
// cmd/pi-zero calls, so the two boards cannot read the profile differently.
//
// Steering and servo are required, as on the Zero: neither the linkage ratio
// nor the servo's range has a default that is safe to guess. The encoder is
// optional, as on the Zero: without it Profile.Encoder is nil and a warning is
// logged, and the board still drives.
func LoadProfile(logger *slog.Logger, configRoot string) (Profile, error) {
	steering, err := nodemotor.SteeringFor(configRoot)
	if err != nil {
		return Profile{}, fmt.Errorf("picolink: %w", err)
	}
	servoCfg, err := hwconfig.Servo(configRoot)
	if err != nil {
		return Profile{}, fmt.Errorf("picolink: %w", err)
	}

	p := Profile{
		Steering:                steering,
		Servo:                   servoCfg,
		SpeedScalePercentPerMPS: nodemotor.SpeedScaleFor(logger, configRoot),
		Invert:                  hwconfig.Motor(logger, configRoot).Invert,
	}

	p.Button = &ButtonParams{Thresholds: hwconfig.Button(logger, configRoot).Thresholds}

	encCfg, encErr := hwconfig.Encoder(configRoot)
	if encErr != nil {
		logger.Warn("picolink: no wheel encoder configured, not publishing joint_states", "error", encErr)
		return p, nil
	}
	p.Encoder = &EncoderParams{CountsPerRev: encCfg.CountsPerRev, Invert: encCfg.Invert}
	return p, nil
}
