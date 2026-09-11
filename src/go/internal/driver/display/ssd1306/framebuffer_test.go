package ssd1306_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/display/ssd1306"
)

func TestNewFramebuffer_RejectsInvalidSizes(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name   string
		width  int
		height int
	}{
		{name: "zero width", width: 0, height: 8},
		{name: "negative height", width: 8, height: -8},
		{name: "height not a multiple of 8", width: 8, height: 5},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if _, err := ssd1306.NewFramebuffer(tt.width, tt.height); err == nil {
				t.Fatalf(
					"NewFramebuffer(%d, %d): got nil error, want an error",
					tt.width,
					tt.height,
				)
			}
		})
	}
}

func TestFramebuffer_SetPixel_RoundTrips(t *testing.T) {
	t.Parallel()

	fb, err := ssd1306.NewFramebuffer(16, 16)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	if fb.Pixel(3, 4) {
		t.Fatal("Pixel(3, 4) before SetPixel: got true, want false")
	}

	fb.SetPixel(3, 4, true)
	if !fb.Pixel(3, 4) {
		t.Fatal("Pixel(3, 4) after SetPixel(true): got false, want true")
	}

	fb.SetPixel(3, 4, false)
	if fb.Pixel(3, 4) {
		t.Fatal("Pixel(3, 4) after SetPixel(false): got true, want false")
	}
}

func TestFramebuffer_SetPixel_OutOfBoundsIsIgnored(t *testing.T) {
	t.Parallel()

	fb, err := ssd1306.NewFramebuffer(8, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	fb.SetPixel(-1, 0, true)
	fb.SetPixel(0, -1, true)
	fb.SetPixel(8, 0, true)
	fb.SetPixel(0, 8, true)

	if fb.Pixel(-1, 0) || fb.Pixel(0, -1) || fb.Pixel(8, 0) || fb.Pixel(0, 8) {
		t.Fatal(
			"out-of-bounds Pixel() reported true, want false for every out-of-bounds coordinate",
		)
	}
}

func TestFramebuffer_FillRect_ClipsToBounds(t *testing.T) {
	t.Parallel()

	fb, err := ssd1306.NewFramebuffer(8, 8)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	fb.FillRect(6, 6, 4, 4, true) // extends 2px past both edges

	tests := []struct {
		x, y int
		want bool
	}{
		{x: 6, y: 6, want: true},
		{x: 7, y: 7, want: true},
		{x: 5, y: 6, want: false}, // just outside the rect
		{x: 6, y: 5, want: false},
	}
	for _, tt := range tests {
		if got := fb.Pixel(tt.x, tt.y); got != tt.want {
			t.Errorf("Pixel(%d, %d) = %v, want %v", tt.x, tt.y, got, tt.want)
		}
	}
}

// TestFramebuffer_Bytes_PageAddressedColumnMajorLayout is the load-bearing
// test for Bytes(): it asserts the exact page-addressed, column-major byte
// layout driver_raw_i2c.py's _image_to_pages doc comment describes ("Each
// output byte packs 8 vertically-stacked pixels ... bit 0 as the topmost
// row"), across two pages so the page-boundary arithmetic (Page.startRow)
// is actually exercised, not just a single-page case.
func TestFramebuffer_Bytes_PageAddressedColumnMajorLayout(t *testing.T) {
	t.Parallel()

	const width, height = 3, 16 // 2 pages
	fb, err := ssd1306.NewFramebuffer(width, height)
	if err != nil {
		t.Fatalf("NewFramebuffer() error = %v, want nil", err)
	}

	fb.SetPixel(0, 0, true)  // page 0, column 0, bit 0
	fb.SetPixel(0, 7, true)  // page 0, column 0, bit 7
	fb.SetPixel(1, 8, true)  // page 1, column 1, bit 0 (first row of page 1)
	fb.SetPixel(2, 15, true) // page 1, column 2, bit 7 (last row of page 1)

	got := fb.Bytes()
	want := []byte{
		// page 0: columns 0,1,2
		0b1000_0001, 0x00, 0x00,
		// page 1: columns 0,1,2
		0x00, 0b0000_0001, 0b1000_0000,
	}

	if len(got) != len(want) {
		t.Fatalf("Bytes() length = %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("Bytes()[%d] = %#08b, want %#08b", i, got[i], want[i])
		}
	}
}

func TestFramebuffer_NumPages(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name       string
		height     int
		wantNumber int
	}{
		{name: "128x64 panel", height: 64, wantNumber: 8},
		{name: "128x32 panel", height: 32, wantNumber: 4},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			fb, err := ssd1306.NewFramebuffer(128, tt.height)
			if err != nil {
				t.Fatalf("NewFramebuffer() error = %v, want nil", err)
			}
			if got := int(fb.NumPages()); got != tt.wantNumber {
				t.Errorf("NumPages() = %d, want %d", got, tt.wantNumber)
			}
		})
	}
}
