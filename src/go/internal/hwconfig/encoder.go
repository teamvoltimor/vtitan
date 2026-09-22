package hwconfig

import (
	"errors"
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	driverencoder "github.com/teamvoltimor/vtitan/src/go/pkg/driver/encoder"
)

// Encoder resolves the Config to Connect with, from configRoot's
// encoder.toml (pins, counts_per_rev) and robot.toml (wheel radius), both
// overlaid with the profiles named in profile.ActiveNames().
//
// Unlike Button this returns an error rather than falling back to
// literals: counts_per_rev has no shared default anywhere in the stack by
// design, and inventing one here would reintroduce exactly the silent
// cross-motor misconfiguration encoder.toml's split was made to prevent.
func Encoder(configRoot string) (driverencoder.Config, error) {
	if configRoot == "" {
		return driverencoder.Config{}, errors.New("encoder: a config root is required to load encoder.toml")
	}

	names := profile.ActiveNames()
	encCfg, err := profile.LoadEncoderConfig(
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultEncoderTOMLPath)),
		names,
	)
	if err != nil {
		return driverencoder.Config{}, fmt.Errorf("encoder: loading encoder.toml: %w", err)
	}
	robotCfg, err := profile.LoadRobotConfig(
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultRobotTOMLPath)),
		names,
	)
	if err != nil {
		return driverencoder.Config{}, fmt.Errorf("encoder: loading robot.toml: %w", err)
	}

	cfg := driverencoder.Config{
		GPIOChip:       driverencoder.DefaultGPIOChip,
		PinA:           encCfg.PinA,
		PinB:           encCfg.PinB,
		CountsPerRev:   encCfg.CountsPerRev,
		WheelDiameterM: robotCfg.Wheel.Radius * 2,
	}
	if validateErr := cfg.Validate(); validateErr != nil {
		//nolint:wrapcheck // Validate's errors already carry "encoder: ..." context
		return driverencoder.Config{}, validateErr
	}
	return cfg, nil
}
