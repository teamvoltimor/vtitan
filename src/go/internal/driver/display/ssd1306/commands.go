package ssd1306

// Control bytes prefixing every I2C write, distinguishing a command stream
// from a data (framebuffer) stream. Ported verbatim from
// driver_raw_i2c.py's _CONTROL_COMMAND/_CONTROL_DATA — these are fixed
// SSD1306 protocol facts, not configuration.
const (
	controlCommand byte = 0x00
	controlData    byte = 0x40
)

// SSD1306 command bytes, ported verbatim from driver_raw_i2c.py's
// module-level constants (same names, same values). Fixed by the SSD1306
// datasheet, not configuration — see Config's doc comment for the
// config/constant boundary this package draws.
const (
	cmdDisplayOff         byte = 0xAE
	cmdDisplayOn          byte = 0xAF
	cmdSetDisplayClockDiv byte = 0xD5
	cmdSetMultiplex       byte = 0xA8
	cmdSetDisplayOffset   byte = 0xD3
	cmdSetStartLine       byte = 0x40
	cmdChargePump         byte = 0x8D
	cmdMemoryMode         byte = 0x20
	cmdSegRemap           byte = 0xA1
	cmdComScanDec         byte = 0xC8
	cmdSetComPins         byte = 0xDA
	cmdSetContrast        byte = 0x81
	cmdSetPrecharge       byte = 0xD9
	cmdSetVCOMDetect      byte = 0xDB
	cmdDisplayAllOnResume byte = 0xA4
	cmdNormalDisplay      byte = 0xA6
	cmdColumnAddr         byte = 0x21
	cmdPageAddr           byte = 0x22
)

// Fixed command parameter bytes sent during Connect's init sequence.
// Ported verbatim from the bare hex literals driver_raw_i2c.py's connect()
// passes alongside the command bytes above. These are display-tuning
// values from the SSD1306 datasheet/reference design (clock ratio, charge
// pump enable, addressing mode, contrast, precharge period, VCOMH deselect
// level) — hoisted to named package constants per the same
// config-vs-constant boundary as commands.go's command bytes, not exposed
// on Config since nothing in the Python driver ever varies them.
const (
	displayClockDivValue  byte = 0x80
	displayOffsetValue    byte = 0x00
	chargePumpEnableValue byte = 0x14
	memoryModeHorizontal  byte = 0x00
	contrastValue         byte = 0xCF
	prechargeValue        byte = 0xF1
	vcomDetectValue       byte = 0x40
)

// height128x64 is the pixel height of the 128x64 SSD1306 variant (vs.
// 128x32), which selects the COM pin configuration byte below. Ported from
// driver_raw_i2c.py's _DISPLAY_HEIGHT_128X64.
const height128x64 = 64

// COM pin hardware configuration bytes: alternative COM pin config with
// left/right remap disabled for the 128x64 variant, sequential COM pin
// config (no remap) for the 128x32 variant. Ported from driver_raw_i2c.py's
// connect(): `0x12 if height == 64 else 0x02`.
const (
	comPinsConfig64 byte = 0x12
	comPinsConfig32 byte = 0x02
)
