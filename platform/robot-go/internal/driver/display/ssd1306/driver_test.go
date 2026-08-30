package ssd1306_test

import (
	"context"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/display/ssd1306"
)

func TestNew_RejectsInvalidConfig(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		cfg  ssd1306.Config
	}{
		{
			name: "zero width",
			cfg:  ssd1306.Config{Width: 0, Height: 64, I2CAddress: 0x3C, I2CBus: 1},
		},
		{
			name: "zero height",
			cfg:  ssd1306.Config{Width: 128, Height: 0, I2CAddress: 0x3C, I2CBus: 1},
		},
		{
			name: "zero I2C address",
			cfg:  ssd1306.Config{Width: 128, Height: 64, I2CAddress: 0, I2CBus: 1},
		},
		{
			name: "negative I2C bus",
			cfg:  ssd1306.Config{Width: 128, Height: 64, I2CAddress: 0x3C, I2CBus: -1},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if _, err := ssd1306.New(tt.cfg); err == nil {
				t.Fatalf("New(%+v): got nil error, want an error", tt.cfg)
			}
		})
	}
}

func TestNew_AcceptsDefaultConfig(t *testing.T) {
	t.Parallel()

	if _, err := ssd1306.New(ssd1306.DefaultConfig()); err != nil {
		t.Fatalf("New(DefaultConfig()) error = %v, want nil", err)
	}
}

// TestDriver_BeforeConnect_ReturnsErrNotConnected asserts Clear and
// WriteFramebuffer fail fast (no real I2C bus access attempted) when
// called before Connect -- these don't need real hardware to test since
// the not-connected check runs before anything touches periph.io.
func TestDriver_BeforeConnect_ReturnsErrNotConnected(t *testing.T) {
	t.Parallel()

	d, err := ssd1306.New(ssd1306.DefaultConfig())
	if err != nil {
		t.Fatalf("New() error = %v, want nil", err)
	}
	ctx := context.Background()

	if clearErr := d.Clear(ctx); clearErr == nil {
		t.Error("Clear() before Connect: got nil error, want an error")
	}

	fb, err := ssd1306.NewFramebuffer(ssd1306.DefaultWidth, ssd1306.DefaultHeight)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}
	if writeErr := d.WriteFramebuffer(ctx, fb); writeErr == nil {
		t.Error("WriteFramebuffer() before Connect: got nil error, want an error")
	}
}

func TestDriver_Close_BeforeConnect_IsANoOp(t *testing.T) {
	t.Parallel()

	d, err := ssd1306.New(ssd1306.DefaultConfig())
	if err != nil {
		t.Fatalf("New() error = %v, want nil", err)
	}

	if closeErr := d.Close(); closeErr != nil {
		t.Fatalf("Close() before Connect: got error %v, want nil", closeErr)
	}
}
