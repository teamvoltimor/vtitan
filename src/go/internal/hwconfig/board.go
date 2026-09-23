package hwconfig

import (
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// Board is the actuation board the active hardware profile selects
// (adr:0098-pico-actuation-board-and-portable-cores).
type Board struct {
	// Kind is which board drives the steering servo and the drive motor.
	Kind hardware.HardwareBoardKind
	// SerialPort is the Pico 2's serial device on the Pi 5. Meaningful only
	// when Kind is hardware.HardwareBoardKindPico2.
	SerialPort string
}

// ActuationBoard resolves configRoot's board.toml overlaid with the profiles
// named in profile.ActiveNames(). An empty configRoot is the Pi Zero, the
// board every build had before the Pico, so a dev machine behaves as before.
//
// Unlike Motor it does not fall back on a load failure: the answer decides
// which process commands the actuators, and guessing wrong either leaves the
// car with no driver or with two. An unknown kind is an error too, since the
// TOML decoder does not apply the schema's enum.
func ActuationBoard(configRoot string) (Board, error) {
	if configRoot == "" {
		return Board{Kind: hardware.HardwareBoardKindZero}, nil
	}

	loaded, err := profile.Load[hardware.HardwareBoard](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultBoardTOMLPath)),
		profile.ActiveNames(),
	)
	if err != nil {
		return Board{}, fmt.Errorf("board: loading board.toml: %w", err)
	}

	switch loaded.Kind {
	case hardware.HardwareBoardKindZero, hardware.HardwareBoardKindPico2:
		return Board{Kind: loaded.Kind, SerialPort: loaded.SerialPort}, nil
	default:
		return Board{}, fmt.Errorf("board: board.toml kind %q is neither %q nor %q",
			loaded.Kind, hardware.HardwareBoardKindZero, hardware.HardwareBoardKindPico2)
	}
}
