package ssd1306

// Config configures a Driver's panel geometry and I2C wiring. Field names
// and defaults mirror platform/robot/src/hardware/display/ssd1306/config.py's
// Config: Width/Height are real panel-geometry facts (which SSD1306 variant
// is wired up), I2CAddress/I2CBus are real per-board wiring facts — all four
// genuinely vary by hardware, unlike the protocol command bytes in
// commands.go, which are fixed SSD1306 datasheet values and stay package
// constants rather than Config fields.
type Config struct {
	Width      int    `validate:"required,gt=0"`
	Height     int    `validate:"required,gt=0"`
	I2CAddress uint16 `validate:"required"`
	I2CBus     int    `validate:"gte=0"`
}

// Default panel geometry and wiring, matching Config's Python defaults
// (platform/robot/src/hardware/display/ssd1306/config.py): a 128x64 SSD1306
// at I2C address 0x3C on bus 1 (/dev/i2c-1 on Raspberry Pi).
const (
	DefaultWidth      = 128
	DefaultHeight     = 64
	DefaultI2CAddress = 0x3C
	DefaultI2CBus     = 1
)

// DefaultConfig returns the Config matching this project's current wiring.
func DefaultConfig() Config {
	return Config{
		Width:      DefaultWidth,
		Height:     DefaultHeight,
		I2CAddress: DefaultI2CAddress,
		I2CBus:     DefaultI2CBus,
	}
}
