package ssd1306_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/display/ssd1306"
)

// TestDrawString_SupportedGlyphSetsExpectedPixels spot-checks 'I' -- a
// full-height vertical bar in this package's 3x5 block font (see text.go)
// -- against the pixels DrawString actually sets, rather than asserting
// against the font table's own literal (which would be tautological).
func TestDrawString_SupportedGlyphSetsExpectedPixels(t *testing.T) {
	t.Parallel()

	fb, err := ssd1306.NewFramebuffer(8, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	ssd1306.DrawString(fb, 0, 0, "I", true)

	tests := []struct {
		x, y int
		want bool
	}{
		{x: 0, y: 0, want: true}, // top row of 'I' is a full bar: 111
		{x: 1, y: 0, want: true},
		{x: 2, y: 0, want: true},
		{x: 0, y: 1, want: false}, // second row is just the middle column: 010
		{x: 1, y: 1, want: true},
		{x: 2, y: 1, want: false},
		{x: 0, y: 4, want: true}, // bottom row is a full bar again: 111
		{x: 2, y: 4, want: true},
	}
	for _, tt := range tests {
		if got := fb.Pixel(tt.x, tt.y); got != tt.want {
			t.Errorf(
				"Pixel(%d, %d) after DrawString(\"I\") = %v, want %v",
				tt.x,
				tt.y,
				got,
				tt.want,
			)
		}
	}
}

// TestDrawString_UnsupportedRuneStillAdvancesCursor asserts an
// out-of-character-set rune is skipped (draws nothing) but still consumes
// one glyphAdvance of cursor width, so a later supported character in the
// same string lands where a supported character would have put it -- text
// after an unsupported rune doesn't collapse leftward.
func TestDrawString_UnsupportedRuneStillAdvancesCursor(t *testing.T) {
	t.Parallel()

	const glyphAdvance = 4 // glyphWidth(3) + 1, mirrored from text.go's unexported constant

	fbWithGap, err := ssd1306.NewFramebuffer(16, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}
	ssd1306.DrawString(fbWithGap, 0, 0, "I~I", true) // '~' is unsupported

	fbNoGap, err := ssd1306.NewFramebuffer(16, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}
	ssd1306.DrawString(fbNoGap, 0, 0, "I", true)
	ssd1306.DrawString(fbNoGap, 2*glyphAdvance, 0, "I", true)

	for x := range 16 {
		for y := range 8 {
			if fbWithGap.Pixel(x, y) != fbNoGap.Pixel(x, y) {
				t.Fatalf(
					"Pixel(%d, %d): DrawString(\"I~I\") = %v, want %v (matching two I's %d px apart)",
					x,
					y,
					fbWithGap.Pixel(x, y),
					fbNoGap.Pixel(x, y),
					2*glyphAdvance,
				)
			}
		}
	}
}

func TestDrawString_EmptyStringSetsNoPixels(t *testing.T) {
	t.Parallel()

	fb, err := ssd1306.NewFramebuffer(8, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	ssd1306.DrawString(fb, 0, 0, "", true)

	for x := range 8 {
		for y := range 8 {
			if fb.Pixel(x, y) {
				t.Fatalf("Pixel(%d, %d) after DrawString(\"\") = true, want false", x, y)
			}
		}
	}
}
