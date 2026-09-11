package ssd1306_test

import (
	"fmt"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/display/ssd1306"
)

// TestController_Init_SendsSSD1306InitSequence is the actual point of this
// package: it asserts the exact I2C byte sequence Controller.Init sends
// matches platform/robot/src/hardware/display/ssd1306/driver_raw_i2c.py's
// connect() body byte-for-byte -- not a generic SSD1306 datasheet
// assumption, the real bytes that file sends (including its
// _write_command-per-byte and single-transaction _write_data framing).
func TestController_Init_SendsSSD1306InitSequence(t *testing.T) {
	t.Parallel()

	fake := &fakeI2C{}
	cfg := ssd1306.Config{Width: 128, Height: 64, I2CAddress: 0x3C, I2CBus: 1}
	ctrl := ssd1306.NewController(fake, cfg)

	if err := ctrl.Init(); err != nil {
		t.Fatalf("Init() error = %v, want nil", err)
	}

	// Command bytes, in order, ported verbatim from driver_raw_i2c.py's
	// connect(): each entry is one _write_command() argument, sent as its
	// own [controlCommand, byte] I2C write.
	wantCommandBytes := []byte{
		0xAE,       // DISPLAYOFF
		0xD5, 0x80, // SETDISPLAYCLOCKDIV
		0xA8, 0x3F, // SETMULTIPLEX (height-1 = 63)
		0xD3, 0x00, // SETDISPLAYOFFSET
		0x40,       // SETSTARTLINE
		0x8D, 0x14, // CHARGEPUMP
		0x20, 0x00, // MEMORYMODE
		0xA1,       // SEGREMAP
		0xC8,       // COMSCANDEC
		0xDA, 0x12, // SETCOMPINS (128x64 variant -> 0x12)
		0x81, 0xCF, // SETCONTRAST
		0xD9, 0xF1, // SETPRECHARGE
		0xDB, 0x40, // SETVCOMDETECT
		0xA4, // DISPLAYALLON_RESUME
		0xA6, // NORMALDISPLAY
		0xAF, // DISPLAYON
		// Init() ends by calling Clear(), which sets the addressing
		// window to the whole display before writing the blank buffer:
		0x21, 0x00, 0x7F, // COLUMNADDR: start 0, end width-1
		0x22, 0x00, 0x07, // PAGEADDR: start 0, end pages-1 (64/8 - 1 = 7)
	}

	wantWrites := make([]string, 0, len(wantCommandBytes)+1)
	for _, b := range wantCommandBytes {
		wantWrites = append(wantWrites, fmt.Sprintf("%02X%02X", 0x00, b))
	}
	// Clear()'s data write: one blank (all-zero) buffer, width*pages
	// bytes, prefixed once with the data control byte (0x40) -- not one
	// write per byte, matching _write_data's single-transaction shape.
	blank := strings.Repeat("00", cfg.Width*cfg.Height/8)
	wantWrites = append(wantWrites, "40"+blank)

	assertWrites(t, fake.writes, wantWrites)
}

// TestController_Init_128x32SelectsSequentialComPins covers the other
// SSD1306 panel variant driver_raw_i2c.py distinguishes: for a 128x32
// panel it selects the sequential (non-alternative) COM pin config byte
// (0x02, not 0x12) and a 4-page multiplex/addressing window instead of 8.
func TestController_Init_128x32SelectsSequentialComPins(t *testing.T) {
	t.Parallel()

	fake := &fakeI2C{}
	cfg := ssd1306.Config{Width: 128, Height: 32, I2CAddress: 0x3C, I2CBus: 1}
	ctrl := ssd1306.NewController(fake, cfg)

	if err := ctrl.Init(); err != nil {
		t.Fatalf("Init() error = %v, want nil", err)
	}

	wantCommandBytes := []byte{
		0xAE,
		0xD5, 0x80,
		0xA8, 0x1F, // SETMULTIPLEX (height-1 = 31)
		0xD3, 0x00,
		0x40,
		0x8D, 0x14,
		0x20, 0x00,
		0xA1,
		0xC8,
		0xDA, 0x02, // SETCOMPINS: 128x32 variant -> 0x02
		0x81, 0xCF,
		0xD9, 0xF1,
		0xDB, 0x40,
		0xA4,
		0xA6,
		0xAF,
		0x21, 0x00, 0x7F,
		0x22, 0x00, 0x03, // PAGEADDR: pages-1 = 32/8 - 1 = 3
	}

	wantWrites := make([]string, 0, len(wantCommandBytes)+1)
	for _, b := range wantCommandBytes {
		wantWrites = append(wantWrites, fmt.Sprintf("%02X%02X", 0x00, b))
	}
	blank := strings.Repeat("00", cfg.Width*cfg.Height/8)
	wantWrites = append(wantWrites, "40"+blank)

	assertWrites(t, fake.writes, wantWrites)
}

// TestController_WriteFramebuffer_SetsAddressingWindowThenData asserts
// WriteFramebuffer sets the full-display addressing window before writing
// the framebuffer's page-addressed bytes, matching driver_raw_i2c.py's
// show_image (_set_addressing_window() then one _write_data() call).
func TestController_WriteFramebuffer_SetsAddressingWindowThenData(t *testing.T) {
	t.Parallel()

	fake := &fakeI2C{}
	cfg := ssd1306.Config{Width: 8, Height: 8, I2CAddress: 0x3C, I2CBus: 1}
	ctrl := ssd1306.NewController(fake, cfg)

	fb, err := ssd1306.NewFramebuffer(cfg.Width, cfg.Height)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}
	fb.SetPixel(0, 0, true) // top-left pixel -> bit 0 of column 0's byte

	if writeErr := ctrl.WriteFramebuffer(fb); writeErr != nil {
		t.Fatalf("WriteFramebuffer() error = %v, want nil", writeErr)
	}

	wantWrites := []string{
		"0021", "0000", "0007", // COLUMNADDR 0..7
		"0022", "0000", "0000", // PAGEADDR 0..0 (one page for an 8px-tall panel)
		"4001" + strings.Repeat("00", cfg.Width-1), // data: column 0 = 0x01, rest 0x00
	}
	assertWrites(t, fake.writes, wantWrites)
}

func TestController_WriteFramebuffer_SizeMismatchReturnsError(t *testing.T) {
	t.Parallel()

	fake := &fakeI2C{}
	cfg := ssd1306.Config{Width: 128, Height: 64, I2CAddress: 0x3C, I2CBus: 1}
	ctrl := ssd1306.NewController(fake, cfg)

	fb, err := ssd1306.NewFramebuffer(64, 32)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	if writeErr := ctrl.WriteFramebuffer(fb); writeErr == nil {
		t.Fatal("WriteFramebuffer() with mismatched size: got nil error, want an error")
	}
}

func TestController_Off_SendsDisplayOffCommand(t *testing.T) {
	t.Parallel()

	fake := &fakeI2C{}
	cfg := ssd1306.Config{Width: 128, Height: 64, I2CAddress: 0x3C, I2CBus: 1}
	ctrl := ssd1306.NewController(fake, cfg)

	if err := ctrl.Off(); err != nil {
		t.Fatalf("Off() error = %v, want nil", err)
	}

	assertWrites(t, fake.writes, []string{"00AE"})
}

// assertWrites is a small local test helper (t.Helper() per go-architect
// §9) shared by this file's table-free, sequence-sensitive assertions.
func assertWrites(t *testing.T, got, want []string) {
	t.Helper()

	if len(got) != len(want) {
		t.Fatalf("sent %d writes, want %d\ngot:  %v\nwant: %v", len(got), len(want), got, want)
	}
	for i, w := range want {
		if got[i] != w {
			t.Fatalf("write[%d] = %s, want %s\ngot:  %v\nwant: %v", i, got[i], w, got, want)
		}
	}
}
