package profile

import (
	"fmt"
	"strconv"
)

// SSD1306Config mirrors ssd1306.toml (src/hardware/display/ssd1306/config.py).
// I2CAddressHex is a "0x.."-formatted string in TOML, not a plain int --
// see I2CAddress for parsing it into internal/driver/display/ssd1306's
// uint16 field.
type SSD1306Config struct {
	// Width matches internal/driver/display/ssd1306.Config.Width.
	Width int `mapstructure:"width"`
	// Height matches internal/driver/display/ssd1306.Config.Height.
	Height int `mapstructure:"height"`
	// I2CAddressHex matches internal/driver/display/ssd1306.Config.I2CAddress
	// once parsed via I2CAddress.
	I2CAddressHex string `mapstructure:"i2c_address"`
	// I2CBus matches internal/driver/display/ssd1306.Config.I2CBus.
	I2CBus int `mapstructure:"i2c_bus"`
}

// DefaultSSD1306TOMLPath is
// platform/config/hardware/display/ssd1306.toml, relative to the
// repo root.
const DefaultSSD1306TOMLPath = "platform/config/hardware/display/ssd1306.toml"

// I2CAddress parses I2CAddressHex (e.g. "0x3C") into the uint16
// internal/driver/display/ssd1306.Config.I2CAddress expects.
func (c *SSD1306Config) I2CAddress() (uint16, error) {
	addr, err := strconv.ParseUint(c.I2CAddressHex, 0, 16)
	if err != nil {
		return 0, fmt.Errorf("profile: parsing i2c_address %q: %w", c.I2CAddressHex, err)
	}
	return uint16(addr), nil
}
