package hwconfig

import (
	"errors"
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/motors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	driverservo "github.com/teamvoltimor/vtitan/src/go/pkg/driver/servo"
)

// Servo resolves the steering servo's Config from configRoot's servo.toml
// overlaid with the profiles named in profile.ActiveNames().
//
// Like Encoder, and unlike Motor, it returns an error rather than falling
// back to literals: the servo profile overlays range_deg, and a 270 degree
// servo driven with the 180 degree base value turns every command into two
// thirds of itself. There is no safe shipped default to fall back to.
func Servo(configRoot string) (driverservo.Config, error) {
	if configRoot == "" {
		return driverservo.Config{}, errors.New("servo: a config root is required to load servo.toml")
	}

	loaded, err := profile.Load[motors.HardwareMotorsServo](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultServoTOMLPath)),
		profile.ActiveNames(),
	)
	if err != nil {
		return driverservo.Config{}, fmt.Errorf("servo: loading servo.toml: %w", err)
	}

	return driverservo.Config{
		GPIOPin:       loaded.GpioPin,
		PWMChip:       loaded.Pwmchip,
		PWMChannel:    loaded.PwmChannel,
		FrequencyHz:   loaded.PwmFrequencyHz,
		MinPulseUS:    loaded.MinPulseUs,
		MaxPulseUS:    loaded.MaxPulseUs,
		CenterPulseUS: loaded.CenterPulseUs,
		RangeDeg:      loaded.RangeDeg,
		Reversed:      loaded.Reversed,
	}, nil
}
