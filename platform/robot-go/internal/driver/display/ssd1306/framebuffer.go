package ssd1306

import (
	"errors"
	"fmt"
)

// Page identifies one horizontal 8-pixel-tall row-band of the SSD1306's
// page-addressed memory — the display's own hardware addressing unit, not
// a UI/application "screen" (that page-orchestration concept lives in
// oled_display_node.py and is out of scope here — see doc.go). Page 0 is
// the topmost bitsPerPage rows, Page 1 the next bitsPerPage rows, and so
// on. A given panel's page count is Height/bitsPerPage — not a fixed enum
// set, since panel height varies by Config (128x32 vs 128x64) — so Page is
// a distinct named type rather than a bare int, and NumPages/startRow below
// are the only places page arithmetic happens.
type Page int

// Framebuffer is a 1-bit pixel buffer sized to one SSD1306 panel, drawn
// into with SetPixel/FillRect/DrawString and read out with Bytes() in the
// panel's native page-addressed, column-major byte layout.
type Framebuffer struct {
	width  int
	height int
	pixels []bool // row-major, index = y*width + x
}

// bitsPerPage is the number of vertically-stacked pixel rows one Page packs
// into a single output byte, per the SSD1306's page-addressed
// horizontal-addressing-mode layout (see driver_raw_i2c.py's
// _image_to_pages doc comment: "Each output byte packs 8 vertically-stacked
// pixels").
const bitsPerPage = 8

// ErrInvalidFramebufferSize is returned by NewFramebuffer for a
// non-positive or non-page-aligned width/height.
var ErrInvalidFramebufferSize = errors.New("ssd1306: invalid framebuffer size")

// startRow returns the topmost pixel row Page p covers.
func (p Page) startRow() int {
	return int(p) * bitsPerPage
}

// NewFramebuffer returns a blank Framebuffer sized width x height. height
// must be a multiple of bitsPerPage (8) — the SSD1306's page addressing
// has no partial-page concept, matching driver_raw_i2c.py's
// `self._pages = self.config.height // 8` (an implicit assumption there,
// made an explicit validated error here).
func NewFramebuffer(width, height int) (*Framebuffer, error) {
	if width <= 0 || height <= 0 {
		return nil, fmt.Errorf("%w: width=%d height=%d", ErrInvalidFramebufferSize, width, height)
	}
	if height%bitsPerPage != 0 {
		return nil, fmt.Errorf(
			"%w: height=%d is not a multiple of %d",
			ErrInvalidFramebufferSize,
			height,
			bitsPerPage,
		)
	}
	return &Framebuffer{
		width:  width,
		height: height,
		pixels: make([]bool, width*height),
	}, nil
}

// Width returns the framebuffer's width in pixels.
func (f *Framebuffer) Width() int {
	return f.width
}

// Height returns the framebuffer's height in pixels.
func (f *Framebuffer) Height() int {
	return f.height
}

// NumPages returns the number of hardware Pages this framebuffer's height
// spans (Height/bitsPerPage).
func (f *Framebuffer) NumPages() Page {
	return Page(f.height / bitsPerPage)
}

// SetPixel sets the pixel at (x, y) on or off. Out-of-bounds coordinates
// are silently ignored, matching the common framebuffer convention of
// letting callers draw shapes that partially clip off-screen without a
// bounds-check at every call site.
func (f *Framebuffer) SetPixel(x, y int, on bool) {
	if i := f.index(x, y); i >= 0 {
		f.pixels[i] = on
	}
}

// Pixel reports whether the pixel at (x, y) is set. Out-of-bounds
// coordinates report false.
func (f *Framebuffer) Pixel(x, y int) bool {
	if i := f.index(x, y); i >= 0 {
		return f.pixels[i]
	}
	return false
}

// Clear sets every pixel off.
func (f *Framebuffer) Clear() {
	for i := range f.pixels {
		f.pixels[i] = false
	}
}

// FillRect sets every pixel in the w x h rectangle with top-left corner
// (x0, y0) to on. Any portion of the rectangle outside the framebuffer's
// bounds is clipped.
func (f *Framebuffer) FillRect(x0, y0, w, h int, on bool) {
	for y := y0; y < y0+h; y++ {
		for x := x0; x < x0+w; x++ {
			f.SetPixel(x, y, on)
		}
	}
}

// Bytes returns the framebuffer's contents in the SSD1306's native
// page-addressed, column-major layout: one byte per (page, column) pair,
// each byte packing bitsPerPage vertically-stacked pixels with bit 0 as
// the topmost row of that page. This is the same layout
// driver_raw_i2c.py's _image_to_pages produces from a PIL image, ported
// here to read from Framebuffer's own pixel storage instead.
func (f *Framebuffer) Bytes() []byte {
	numPages := f.NumPages()
	buffer := make([]byte, f.width*int(numPages))
	for page := range numPages {
		for x := range f.width {
			var b byte
			for bit := range bitsPerPage {
				y := page.startRow() + bit
				if f.Pixel(x, y) {
					b |= 1 << bit
				}
			}
			buffer[int(page)*f.width+x] = b
		}
	}
	return buffer
}

// index returns pixels' flat-slice index for (x, y), or -1 if (x, y) is
// outside the framebuffer's bounds.
func (f *Framebuffer) index(x, y int) int {
	if x < 0 || x >= f.width || y < 0 || y >= f.height {
		return -1
	}
	return y*f.width + x
}
