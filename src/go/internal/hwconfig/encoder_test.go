package hwconfig_test

import (
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
)

// The shipped motors.toml sets drive.encoder_reversed = true, which is not
// the zero value, so this fails for a loader that never reads the key.
// That loader is what Go had: Invert stayed false and odometry came out
// with the opposite sign to Python's.
func TestEncoder_ReadsEncoderReversedFromMotorsTOML(t *testing.T) {
	t.Setenv("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")

	cfg, err := hwconfig.Encoder(filepath.Join("..", "..", "..", ".."))
	if err != nil {
		t.Fatalf("Encoder: %v", err)
	}
	if !cfg.Invert {
		t.Fatal("Invert = false, want true from motors.toml drive.encoder_reversed")
	}
}
