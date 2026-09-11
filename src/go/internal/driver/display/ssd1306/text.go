package ssd1306

// This file is the minimal text-rendering primitive the task scope calls
// for ("enough to draw a string or fill a rectangle") — it is intentionally
// not a port of anything in oled_display_node.py, which renders through
// PIL's ImageDraw/a real TTF font (see doc.go's scope note). glyph5x7 below
// is a small hand-authored block font covering digits, uppercase letters,
// and a few punctuation marks commonly needed for status text (e.g.
// "READY", "12.3V") — legible but not a port of any specific existing font
// asset. Replace with a real font table if/when the page-rendering pass
// needs closer visual parity with the Python UI.

// glyphRow is one row of a glyph's pixels: bit 0 is the leftmost column,
// bit (glyphWidth-1) the rightmost.
type glyphRow byte

// glyph is a supported character's fixed-size bitmap, glyphHeight rows of
// glyphWidth columns each.
type glyph struct {
	char rune
	rows [glyphHeight]glyphRow
}

// glyphWidth and glyphHeight are the fixed pixel cell every glyph is drawn
// into. glyphAdvance is how far DrawString moves the cursor per character,
// one column wider than glyphWidth to leave a blank spacing column between
// characters without a separate kerning pass.
const (
	glyphWidth   = 3
	glyphHeight  = 5
	glyphAdvance = glyphWidth + 1
)

// glyphs is the supported character set, looked up by linear scan in
// glyphFor — a plain slice rather than a map (see project style
// preference: avoid maps, prefer []struct{...}), which is also fine
// performance-wise for a set this small and drawn at a UI refresh rate,
// not a hot path.
var glyphs = []glyph{
	{' ', [glyphHeight]glyphRow{0b000, 0b000, 0b000, 0b000, 0b000}},
	{'0', [glyphHeight]glyphRow{0b111, 0b101, 0b101, 0b101, 0b111}},
	{'1', [glyphHeight]glyphRow{0b010, 0b110, 0b010, 0b010, 0b111}},
	{'2', [glyphHeight]glyphRow{0b111, 0b001, 0b111, 0b100, 0b111}},
	{'3', [glyphHeight]glyphRow{0b111, 0b001, 0b111, 0b001, 0b111}},
	{'4', [glyphHeight]glyphRow{0b101, 0b101, 0b111, 0b001, 0b001}},
	{'5', [glyphHeight]glyphRow{0b111, 0b100, 0b111, 0b001, 0b111}},
	{'6', [glyphHeight]glyphRow{0b111, 0b100, 0b111, 0b101, 0b111}},
	{'7', [glyphHeight]glyphRow{0b111, 0b001, 0b001, 0b001, 0b001}},
	{'8', [glyphHeight]glyphRow{0b111, 0b101, 0b111, 0b101, 0b111}},
	{'9', [glyphHeight]glyphRow{0b111, 0b101, 0b111, 0b001, 0b111}},
	{':', [glyphHeight]glyphRow{0b000, 0b010, 0b000, 0b010, 0b000}},
	{'-', [glyphHeight]glyphRow{0b000, 0b000, 0b111, 0b000, 0b000}},
	{'.', [glyphHeight]glyphRow{0b000, 0b000, 0b000, 0b000, 0b010}},
	{'%', [glyphHeight]glyphRow{0b101, 0b001, 0b010, 0b100, 0b101}},
	{'A', [glyphHeight]glyphRow{0b010, 0b101, 0b111, 0b101, 0b101}},
	{'B', [glyphHeight]glyphRow{0b110, 0b101, 0b110, 0b101, 0b110}},
	{'C', [glyphHeight]glyphRow{0b011, 0b100, 0b100, 0b100, 0b011}},
	{'D', [glyphHeight]glyphRow{0b110, 0b101, 0b101, 0b101, 0b110}},
	{'E', [glyphHeight]glyphRow{0b111, 0b100, 0b111, 0b100, 0b111}},
	{'F', [glyphHeight]glyphRow{0b111, 0b100, 0b111, 0b100, 0b100}},
	{'G', [glyphHeight]glyphRow{0b011, 0b100, 0b101, 0b101, 0b011}},
	{'H', [glyphHeight]glyphRow{0b101, 0b101, 0b111, 0b101, 0b101}},
	{'I', [glyphHeight]glyphRow{0b111, 0b010, 0b010, 0b010, 0b111}},
	{'J', [glyphHeight]glyphRow{0b001, 0b001, 0b001, 0b101, 0b011}},
	{'K', [glyphHeight]glyphRow{0b101, 0b110, 0b100, 0b110, 0b101}},
	{'L', [glyphHeight]glyphRow{0b100, 0b100, 0b100, 0b100, 0b111}},
	{'M', [glyphHeight]glyphRow{0b101, 0b111, 0b111, 0b101, 0b101}},
	{'N', [glyphHeight]glyphRow{0b101, 0b111, 0b111, 0b111, 0b101}},
	{'O', [glyphHeight]glyphRow{0b111, 0b101, 0b101, 0b101, 0b111}},
	{'P', [glyphHeight]glyphRow{0b111, 0b101, 0b111, 0b100, 0b100}},
	{'Q', [glyphHeight]glyphRow{0b111, 0b101, 0b101, 0b111, 0b001}},
	{'R', [glyphHeight]glyphRow{0b111, 0b101, 0b111, 0b110, 0b101}},
	{'S', [glyphHeight]glyphRow{0b011, 0b100, 0b111, 0b001, 0b110}},
	{'T', [glyphHeight]glyphRow{0b111, 0b010, 0b010, 0b010, 0b010}},
	{'U', [glyphHeight]glyphRow{0b101, 0b101, 0b101, 0b101, 0b111}},
	{'V', [glyphHeight]glyphRow{0b101, 0b101, 0b101, 0b101, 0b010}},
	{'W', [glyphHeight]glyphRow{0b101, 0b101, 0b111, 0b111, 0b101}},
	{'X', [glyphHeight]glyphRow{0b101, 0b101, 0b010, 0b101, 0b101}},
	{'Y', [glyphHeight]glyphRow{0b101, 0b101, 0b010, 0b010, 0b010}},
	{'Z', [glyphHeight]glyphRow{0b111, 0b001, 0b010, 0b100, 0b111}},
}

// glyphFor returns the glyph for c and true, or the zero glyph and false if
// c isn't in the supported set (see glyphs' doc comment).
func glyphFor(c rune) (glyph, bool) {
	for _, g := range glyphs {
		if g.char == c {
			return g, true
		}
	}
	return glyph{}, false
}

// DrawString draws s into f, left to right, with its top-left corner at
// (x0, y0), each glyph rendered on if on (background pixels are left
// untouched — callers wanting a solid background should FillRect first).
// Characters outside the supported set (see glyphs) are skipped but still
// advance the cursor, so column alignment of surrounding text isn't thrown
// off by one unsupported character.
func DrawString(f *Framebuffer, x0, y0 int, s string, on bool) {
	x := x0
	for _, c := range s {
		if g, ok := glyphFor(c); ok {
			drawGlyph(f, x, y0, g, on)
		}
		x += glyphAdvance
	}
}

// drawGlyph draws one glyph into f with its top-left corner at (x0, y0).
func drawGlyph(f *Framebuffer, x0, y0 int, g glyph, on bool) {
	for row := range glyphHeight {
		for col := range glyphWidth {
			if g.rows[row]&(1<<col) != 0 {
				f.SetPixel(x0+col, y0+row, on)
			}
		}
	}
}
