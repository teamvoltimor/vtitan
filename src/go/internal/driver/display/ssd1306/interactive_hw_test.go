//go:build hw && hwinteractive

package ssd1306_test

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/display/ssd1306"
)

func waitForAck(prompt string) bool {
	fmt.Printf("\n%s\n  [type y/yes to PASS, anything else to FAIL]: ", prompt)
	line, err := bufio.NewReader(os.Stdin).ReadString('\n')
	if err != nil {
		return false
	}
	line = strings.TrimSpace(strings.ToLower(line))
	return line == "y" || line == "yes"
}

// TestHW_OLED_Render_Dynamic draws a known pattern (border box + "HW TEST"
// text + a filled rectangle) and asks a human to eyeball that it actually
// rendered -- pixels can't be read back programmatically from an SSD1306.
// Complements TestHW_OLED_I2C, which only proves the bus/init path works;
// this proves the panel visibly displays what we wrote.
//
// PASS -> operator confirms the rendered pattern (box + "HW TEST" + block)
//         is visible on the physical panel.
// FAIL  -> WriteFramebuffer errors (bus write failed) OR operator reports a
//         blank/wrong display. Either means the pixels aren't reaching the
//         panel even though init succeeded.
func TestHW_OLED_Render_Dynamic(t *testing.T) {
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
		t.Fatalf("HW FAIL: ssd1306.Connect: %v", err)
	}
	defer func() { _ = d.Close() }()

	fb, err := ssd1306.NewFramebuffer(cfg.Width, cfg.Height)
	if err != nil {
		t.Fatalf("HW FAIL: NewFramebuffer: %v", err)
	}
	fb.Clear()
	// Border box.
	fb.FillRect(0, 0, cfg.Width, 1, true)
	fb.FillRect(0, cfg.Height-1, cfg.Width, 1, true)
	fb.FillRect(0, 0, 1, cfg.Height, true)
	fb.FillRect(cfg.Width-1, 0, 1, cfg.Height, true)
	// Known text.
	ssd1306.DrawString(fb, 8, 8, "HW TEST", true)
	// Filled block so a human can sanity-check fill too.
	fb.FillRect(8, 24, 40, 16, true)

	if err := d.WriteFramebuffer(context.Background(), fb); err != nil {
		t.Fatalf("HW FAIL: ssd1306.WriteFramebuffer (panel unresponsive?): %v", err)
	}

	if !waitForAck("OLED: do you see a bordered box with 'HW TEST' and a solid block?") {
		t.Fatalf("HW FAIL: operator did not confirm OLED rendered the pattern")
	}

	t.Logf("HW PASS: OLED rendered known pattern per operator (bus %d addr 0x%X)", cfg.I2CBus, cfg.I2CAddress)
}
