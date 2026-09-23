//go:build linux

package main

import (
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/display/ssd1306"
)

func TestRenderSummary(t *testing.T) {
	t.Parallel()

	cfg := ssd1306.DefaultConfig()

	t.Run("no detection", func(t *testing.T) {
		t.Parallel()

		fb, err := renderSummary(cfg, &uiv1.TelemetrySummary{
			LidarFrontCm: 42, LidarLeftCm: 10, LidarRightCm: 20, GyroYawDeg: 180,
		})
		if err != nil {
			t.Fatalf("renderSummary() error = %v, want nil", err)
		}
		if fb.Width() != cfg.Width || fb.Height() != cfg.Height {
			t.Errorf(
				"framebuffer = %dx%d, want %dx%d",
				fb.Width(),
				fb.Height(),
				cfg.Width,
				cfg.Height,
			)
		}
	})

	t.Run("with detection", func(t *testing.T) {
		t.Parallel()

		fb, err := renderSummary(cfg, &uiv1.TelemetrySummary{
			HasBestDetection: true, BestDetectionClassId: "sign_red", BestDetectionConfidence: 0.9,
		})
		if err != nil {
			t.Fatalf("renderSummary() error = %v, want nil", err)
		}
		if fb == nil {
			t.Fatal("renderSummary() framebuffer = nil, want non-nil")
		}
	})
}

func TestRenderSummary_InvalidConfigIsAnError(t *testing.T) {
	t.Parallel()

	if _, err := renderSummary(ssd1306.Config{Width: 0, Height: 0}, &uiv1.TelemetrySummary{}); err == nil {
		t.Fatal("renderSummary() with an invalid framebuffer size: got nil error, want non-nil")
	}
}

// With the pico2 profile the Zero must refuse before touching any driver;
// without it, the shipped board.toml is the Zero and it proceeds.
func TestRequireZeroBoard(t *testing.T) {
	shippedRoot := filepath.Join("..", "..", "..", "..")

	t.Setenv(profile.EnvVar, "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")
	if err := requireZeroBoard(shippedRoot); err != nil {
		t.Errorf("requireZeroBoard without pico2 = %v, want nil", err)
	}

	t.Setenv(profile.EnvVar, "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm,pico2")
	if err := requireZeroBoard(shippedRoot); err == nil {
		t.Error("requireZeroBoard with pico2 = nil, want a refusal")
	}
}
