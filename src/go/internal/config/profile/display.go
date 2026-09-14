package profile

import (
	"fmt"
	"strconv"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/display"
)

// DefaultSSD1306TOMLPath is
// src/config/hardware/display/ssd1306.toml, relative to the
// repo root. The file's shape is the generated
// display.HardwareDisplaySsd1306 DTO, whose field is I2CAddress (a
// "0x.."-formatted string in TOML, not a plain int); see ParseI2CAddress for
// parsing it into internal/driver/display/ssd1306's uint16 field.
const DefaultSSD1306TOMLPath = "src/config/hardware/display/ssd1306.toml"

// ParseI2CAddress parses cfg.I2CAddress (e.g. "0x3C") into the uint16
// internal/driver/display/ssd1306.Config.I2CAddress expects. It is a free
// function because the generated DTO it reads carries no methods.
func ParseI2CAddress(cfg *display.HardwareDisplaySsd1306) (uint16, error) {
	addr, err := strconv.ParseUint(cfg.I2CAddress, 0, 16)
	if err != nil {
		return 0, fmt.Errorf("profile: parsing i2c_address %q: %w", cfg.I2CAddress, err)
	}
	return uint16(addr), nil
}
