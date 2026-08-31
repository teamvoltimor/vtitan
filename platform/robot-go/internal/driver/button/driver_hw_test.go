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

// TestHW_GPIO_Bias verifies that go-gpiocdev's internal pull-up/pull-down
// bias requests are honored by the kernel on a real floating line. This is
// the single real risk flagged for the go-gpiocdev dependency: a kernel that
// ignores the bias request leaves the line floating and produces unstable
// readings.
//
// PASS -> kernel honors WithPullUp (stable 1) and WithPullDown (stable 0).
// FAIL -> either request errors, or the line does not read the biased level
//         steadily -- the kernel is not applying go-gpiocdev's bias.
//
// Set BUTTON_GPIO_LINE to the BCM offset of a FLOATING (unwired) pin.
func TestHW_GPIO_Bias(t *testing.T) {
	line := requireEnvInt(t, "BUTTON_GPIO_LINE")
	chip := os.Getenv("BUTTON_GPIO_CHIP")
	if chip == "" {
		chip = button.DefaultGPIOChip
	}

	// --- Pull-up bias: a floating line with an internal pull-up must read 1.
	up, err := gpiocdev.RequestLine(chip, line, gpiocdev.AsInput, gpiocdev.WithPullUp)
	if err != nil {
		t.Fatalf("HW FAIL: requesting line %s:%d with pull-up: %v", chip, line, err)
	}
	if !stableLevel(t, up, 1, "pull-up") {
		_ = up.Close()
		t.Fatalf("HW FAIL: floating line did not read stable 1 under WithPullUp")
	}
	_ = up.Close()

	// --- Pull-down bias: the same floating line must now read 0.
	down, err := gpiocdev.RequestLine(chip, line, gpiocdev.AsInput, gpiocdev.WithPullDown)
	if err != nil {
		t.Fatalf("HW FAIL: requesting line %s:%d with pull-down: %v", chip, line, err)
	}
	if !stableLevel(t, down, 0, "pull-down") {
		_ = down.Close()
		t.Fatalf("HW FAIL: floating line did not read stable 0 under WithPullDown")
	}
	_ = down.Close()

	t.Log("HW PASS: kernel honors go-gpiocdev WithPullUp/WithPullDown bias")
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

func requireEnvInt(t *testing.T, key string) int {
	t.Helper()
	v := os.Getenv(key)
	if v == "" {
		t.Skipf("skipping: %s not set (BCM offset of a floating pin)", key)
	}
	var n int
	if _, err := fmt.Sscanf(v, "%d", &n); err != nil {
		t.Fatalf("invalid %s=%q: %v", key, v, err)
	}
	return n
}
