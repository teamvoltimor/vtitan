//go:build linux

package main

import (
	"errors"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/display/ssd1306"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
)

var errBoom = errors.New("boom")

func TestButtonKindToProto(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		in   button.Kind
		want uiv1.ButtonEvent_Kind
	}{
		{name: "pressed", in: button.KindPressed, want: uiv1.ButtonEvent_KIND_PRESSED},
		{name: "long press", in: button.KindLongPress, want: uiv1.ButtonEvent_KIND_LONG_PRESS},
		{name: "shutdown press", in: button.KindShutdownPress, want: uiv1.ButtonEvent_KIND_SHUTDOWN_PRESS},
		{name: "short press", in: button.KindShortPress, want: uiv1.ButtonEvent_KIND_SHORT_PRESS},
		{name: "released", in: button.KindReleased, want: uiv1.ButtonEvent_KIND_RELEASED},
		{name: "unknown", in: button.Kind(99), want: uiv1.ButtonEvent_KIND_UNSPECIFIED},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := buttonKindToProto(tt.in); got != tt.want {
				t.Errorf("buttonKindToProto(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}

func TestButtonEventMessageFor(t *testing.T) {
	t.Parallel()

	got := buttonEventMessageFor(button.Event{Kind: button.KindLongPress, HeldSec: 3.5})

	if got.GetKind() != uiv1.ButtonEvent_KIND_LONG_PRESS {
		t.Errorf("Kind = %v, want KIND_LONG_PRESS", got.GetKind())
	}
	if got.GetHeldSec() != 3.5 {
		t.Errorf("HeldSec = %v, want 3.5", got.GetHeldSec())
	}
}

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
			t.Errorf("framebuffer = %dx%d, want %dx%d", fb.Width(), fb.Height(), cfg.Width, cfg.Height)
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

func TestMotorStatusFor_FaultTakesPriority(t *testing.T) {
	t.Parallel()

	got := motorStatusFor(0.5, 0, errBoom)
	if got.GetDetail() != errBoom.Error() {
		t.Errorf("Detail = %q, want %q", got.GetDetail(), errBoom.Error())
	}
}
