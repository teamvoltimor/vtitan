//go:build hw

package ssd1306_test

import (
	"context"
	"fmt"
	"os"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/display/ssd1306"
)

// TestHW_OLED_I2C verifies the only periph.io consumer in the codebase still
// opens the real I2C bus and runs the SSD1306 init sequence on current Pi OS
// kernel/userspace. periph.io is the one genuinely stale dependency; this is
// the canary for whether it must be replaced with a local ioctl shim.
//
// PASS -> host.Init() + i2creg.Open succeed and Init() completes; the OLED is
//         live on the bus with periph.io's I2C stack intact.
// FAIL -> host.Init errors (periph.io driver registry broke) OR i2creg.Open
//         fails (bus /dev/i2c-N gone / permission) OR Init() errors (device
//         unresponsive on the bus). Any of these means periph.io's I2C path
//         is dead and the ssd1306/i2c_linux.go ioctl shim is needed.
func TestHW_OLED_I2C(t *testing.T) {
	cfg := ssd1306.DefaultConfig()
	if v := os.Getenv("SSD1306_I2C_BUS"); v != "" {
		var bus int
		if _, err := fmt.Sscanf(v, "%d", &bus); err != nil {
			t.Fatalf("invalid SSD1306_I2C_BUS=%q: %v", v, err)
		}
		cfg.I2CBus = bus
	}
	if v := os.Getenv("SSD1306_I2C_ADDR"); v != "" {
		var addr uint64
		if _, err := fmt.Sscanf(v, "0x%X", &addr); err != nil {
			t.Fatalf("invalid SSD1306_I2C_ADDR=%q: %v", v, err)
		}
		cfg.I2CAddress = uint16(addr)
	}

	d, err := ssd1306.New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: ssd1306.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: ssd1306.Connect (periph.io I2C): %v", err)
	}
	defer func() {
		if cerr := d.Close(); cerr != nil {
			t.Logf("close note: %v", cerr)
		}
	}()

	// Prove the panel is addressable: clear it (a real bus write). A
	// no-device bus would surface an I/O error here or in Connect's Init.
	if err := d.Clear(context.Background()); err != nil {
		t.Fatalf("HW FAIL: ssd1306.Clear (panel unresponsive on bus): %v", err)
	}

	t.Logf("HW PASS: periph.io I2C open + SSD1306 init OK on bus %d addr 0x%X",
		cfg.I2CBus, cfg.I2CAddress)
}
