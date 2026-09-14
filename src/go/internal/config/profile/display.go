package profile

import (
	"fmt"
	"strconv"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/display"
)

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
