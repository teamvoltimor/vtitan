//go:build hw

package button_test

import (
	"fmt"
	"os"
	"testing"
	"time"

	"github.com/warthog618/go-gpiocdev"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
)

// TestHW_GPIO_Bias verifies that go-gpiocdev's internal pull-up bias is
// honored by the kernel on the real button line. The button is wired
// GND-to-pin with an external pull-up (config/hardware/button/gpio.toml:
// gpio_pin=4, pull_up=true), so an unpressed line must read a stable 1
// (HIGH) -- a kernel that ignores the bias request would float and read
// intermittently. This is the single real risk flagged for the
// go-gpiocdev dependency.
//
// PASS -> requesting GPIO4 as input with WithPullUp reads a stable 1 across
//         samples (matches the unpressed, wired-pull-up level).
// FAIL -> the line request errors (wrong chip/pin, already claimed) OR the
//         level is not a stable 1 -- the kernel is not applying
//         go-gpiocdev's bias against the real wiring.
//
// Set BUTTON_GPIO_LINE to override (defaults to 4 to match gpio.toml).
func TestHW_GPIO_Bias(t *testing.T) {
	line := 4 // BCM GPIO4, per config/hardware/button/gpio.toml
	if v := os.Getenv("BUTTON_GPIO_LINE"); v != "" {
		var n int
		if _, err := fmt.Sscanf(v, "%d", &n); err != nil {
			t.Fatalf("invalid BUTTON_GPIO_LINE=%q: %v", v, err)
		}
		line = n
	}
	chip := os.Getenv("BUTTON_GPIO_CHIP")
	if chip == "" {
		chip = button.DefaultGPIOChip
	}

	// The button is wired with an external pull-up (pull_up=true in
	// gpio.toml), so the unpressed level is HIGH (1). Requesting WithPullUp
	// must preserve that stable 1.
	l, err := gpiocdev.RequestLine(chip, line, gpiocdev.AsInput, gpiocdev.WithPullUp)
	if err != nil {
		t.Fatalf("HW FAIL: requesting line %s:%d with pull-up: %v", chip, line, err)
	}
	defer func() { _ = l.Close() }()

	if !stableLevel(t, l, 1, "pull-up (unpressed button, wired pull-up)") {
		t.Fatalf("HW FAIL: button line %s:%d did not read stable 1 under WithPullUp", chip, line)
	}

	t.Logf("HW PASS: go-gpiocdev WithPullUp honored on %s:%d (stable 1 = unpressed wired-pull-up button)", chip, line)
}

func stableLevel(t *testing.T, line *gpiocdev.Line, want int, label string) bool {
	t.Helper()
	for i := 0; i < 50; i++ {
		v, err := line.Value()
		if err != nil {
			t.Logf("read error during %s stability check: %v", label, err)
			return false
		}
		if v != want {
			t.Logf("%s: got %d, want stable %d (sample %d/50)", label, v, want, i)
			return false
		}
		time.Sleep(10 * time.Millisecond)
	}
	return true
}
