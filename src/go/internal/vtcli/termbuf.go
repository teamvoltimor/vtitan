package vtcli

import (
	"strconv"
	"strings"
	"unicode/utf8"
)

// termBuf turns a task's PTY output into the lines the run pane shows. It is
// not a terminal emulator: it keeps scrollback as a growing list of lines and
// honors only what tools use to redraw in place -- carriage returns, erase
// line, cursor up/down/column, erase below -- so a progress bar rewrites its
// own line instead of printing a thousand copies of it. Colors (SGR) are
// kept per cell; every other escape sequence is dropped.
type termBuf struct {
	lines [][]termCell
	// rendered caches each line as a string; lines from dirtyFrom on are
	// stale and re-rendered on the next render call.
	rendered  []string
	dirtyFrom int
	row, col  int
	style     string
	width     int
	height    int
	maxLines  int
	color     bool
	pending   []byte
}

// termCell is one character and the SGR sequence it was written with.
type termCell struct {
	r     rune
	style string
}

// Escape-sequence bytes the parser recognises.
const (
	escByte   = 0x1b
	bellByte  = 0x07
	csiByte   = '['
	oscByte   = ']'
	tabStop   = 8
	sgrReset  = "\x1b[0m"
	maxStyle  = 64
	csiMinFin = 0x40
	csiMaxFin = 0x7e
)

// newTermBuf returns an empty buffer that wraps at width, treats the last
// height lines as the screen cursor positioning is relative to, and keeps at
// most maxLines of scrollback.
func newTermBuf(width, height, maxLines int, color bool) *termBuf {
	return &termBuf{
		lines:    [][]termCell{nil},
		width:    max(width, 1),
		height:   max(height, 1),
		maxLines: maxLines,
		color:    color,
	}
}

// resize changes where new output wraps. Lines already written keep their
// shape: re-flowing scrollback would move what the user is reading.
func (b *termBuf) resize(width, height int) {
	b.width, b.height = max(width, 1), max(height, 1)
}

// feed appends task output. A sequence or rune split across two reads is
// held back until the rest arrives.
func (b *termBuf) feed(data []byte) {
	b.pending = append(b.pending, data...)
	rest := b.pending

	for len(rest) > 0 {
		used := b.step(rest)
		if used == 0 {
			break
		}

		rest = rest[used:]
	}

	b.pending = append(b.pending[:0], rest...)
	b.trim()
}

// step consumes one control byte, escape sequence or rune from data and
// returns how many bytes it used, or 0 if data holds only part of one.
func (b *termBuf) step(data []byte) int {
	switch data[0] {
	case escByte:
		return b.escape(data)
	case '\n':
		b.lineFeed()
	case '\r':
		b.col = 0
	case '\b':
		b.col = max(b.col-1, 0)
	case '\t':
		b.col = min((b.col/tabStop+1)*tabStop, b.width-1)
	default:
		if data[0] < ' ' || data[0] == 0x7f {
			return 1
		}

		if !utf8.FullRune(data) {
			return 0
		}

		r, size := utf8.DecodeRune(data)
		b.put(r)

		return size
	}

	return 1
}

// escape consumes one escape sequence, acting on the few that redraw lines.
func (b *termBuf) escape(data []byte) int {
	if len(data) < 2 {
		return 0
	}

	switch data[1] {
	case csiByte:
		for i := 2; i < len(data); i++ {
			if data[i] >= csiMinFin && data[i] <= csiMaxFin {
				b.csi(string(data[2:i]), data[i])

				return i + 1
			}
		}

		return 0
	case oscByte:
		// Titles and hyperlinks: dropped, terminated by BEL or ESC \.
		for i := 2; i < len(data); i++ {
			if data[i] == bellByte {
				return i + 1
			}

			if data[i] == escByte && i+1 < len(data) && data[i+1] == '\\' {
				return i + 2
			}
		}

		return 0
	default:
		return 2
	}
}

// csi applies one control sequence: params is what sits between ESC [ and
// the final byte.
func (b *termBuf) csi(params string, final byte) {
	if strings.HasPrefix(params, "?") || strings.HasPrefix(params, ">") {
		return // private modes: cursor visibility, alt screen, bracketed paste
	}

	count := func() int {
		n, err := strconv.Atoi(strings.SplitN(params, ";", 2)[0])
		if err != nil || n < 1 {
			return 1
		}

		return n
	}

	switch final {
	case 'm':
		b.sgr(params)
	case 'K':
		b.eraseLine(params)
	case 'J':
		if params == "" || params == "0" {
			b.eraseLine("0")
			b.lines = b.lines[:b.row+1]
			b.dirty(b.row)
		}
	case 'A':
		b.row = max(b.row-count(), b.screenTop())
	case 'B':
		b.moveDown(count())
	case 'C':
		b.col = min(b.col+count(), b.width-1)
	case 'D':
		b.col = max(b.col-count(), 0)
	case 'G':
		b.col = min(count()-1, b.width-1)
	case 'H', 'f':
		b.position(params)
	}
}

// sgr updates the style new characters are written with.
func (b *termBuf) sgr(params string) {
	if params == "" || params == "0" {
		b.style = ""

		return
	}

	seq := "\x1b[" + params + "m"
	if len(b.style)+len(seq) > maxStyle {
		b.style = seq

		return
	}

	b.style += seq
}

// eraseLine clears part of the cursor's line: 0 to the end, 1 to the start,
// 2 all of it.
func (b *termBuf) eraseLine(params string) {
	line := b.lines[b.row]

	switch params {
	case "", "0":
		if b.col < len(line) {
			b.lines[b.row] = line[:b.col]
		}
	case "1":
		for i := 0; i < min(b.col+1, len(line)); i++ {
			line[i] = termCell{r: ' '}
		}
	case "2":
		b.lines[b.row] = nil
	}

	b.dirty(b.row)
}

// position moves the cursor to a 1-based row;col of the visible screen.
func (b *termBuf) position(params string) {
	row, col := 1, 1

	parts := strings.SplitN(params, ";", 2)
	if n, err := strconv.Atoi(parts[0]); err == nil && n > 0 {
		row = n
	}

	if len(parts) == 2 {
		if n, err := strconv.Atoi(parts[1]); err == nil && n > 0 {
			col = n
		}
	}

	b.row = b.screenTop()
	b.moveDown(row - 1)
	b.col = min(col-1, b.width-1)
}

// screenTop is the first line of what a real terminal would be showing.
func (b *termBuf) screenTop() int {
	return max(len(b.lines)-b.height, 0)
}

// moveDown moves the cursor n lines down, adding lines at the bottom.
func (b *termBuf) moveDown(n int) {
	for range n {
		b.row++
		if b.row == len(b.lines) {
			b.lines = append(b.lines, nil)
		}
	}
}

// lineFeed starts the next line; a PTY sends \r\n, but a bare \n from a
// raw write still starts at column 0 rather than leaving a staircase.
func (b *termBuf) lineFeed() {
	b.moveDown(1)
	b.col = 0
}

// put writes r at the cursor, wrapping at the width like a terminal does.
func (b *termBuf) put(r rune) {
	if b.col >= b.width {
		b.lineFeed()
	}

	line := b.lines[b.row]
	for len(line) < b.col {
		line = append(line, termCell{r: ' '})
	}

	cell := termCell{r: r, style: b.style}
	if b.col < len(line) {
		line[b.col] = cell
	} else {
		line = append(line, cell)
	}

	b.lines[b.row] = line
	b.col++
	b.dirty(b.row)
}

// dirty marks row and everything after it for re-rendering.
func (b *termBuf) dirty(row int) {
	b.dirtyFrom = min(b.dirtyFrom, row)
}

// trim drops the oldest lines beyond maxLines.
func (b *termBuf) trim() {
	excess := len(b.lines) - b.maxLines
	if b.maxLines <= 0 || excess <= 0 {
		return
	}

	b.lines = b.lines[excess:]
	b.row = max(b.row-excess, 0)

	if excess < len(b.rendered) {
		b.rendered = b.rendered[excess:]
	} else {
		b.rendered = nil
	}

	b.dirtyFrom = max(b.dirtyFrom-excess, 0)
}

// render returns every line as a string, colors included when the UI has
// them, re-rendering only what changed since the last call.
func (b *termBuf) render() []string {
	b.rendered = b.rendered[:min(b.dirtyFrom, len(b.rendered))]

	for i := len(b.rendered); i < len(b.lines); i++ {
		b.rendered = append(b.rendered, b.renderLine(b.lines[i]))
	}

	b.dirtyFrom = len(b.lines)

	return b.rendered
}

// renderLine turns one line of cells into a string.
func (b *termBuf) renderLine(line []termCell) string {
	var out strings.Builder

	style := ""
	for _, cell := range line {
		if b.color && cell.style != style {
			out.WriteString(sgrReset + cell.style)
			style = cell.style
		}

		out.WriteRune(cell.r)
	}

	if style != "" {
		out.WriteString(sgrReset)
	}

	return out.String()
}

// plain returns the buffer without colors, for tests and the exit summary.
func (b *termBuf) plain() string {
	lines := make([]string, 0, len(b.lines))

	for _, line := range b.lines {
		var out strings.Builder
		for _, cell := range line {
			out.WriteRune(cell.r)
		}

		lines = append(lines, out.String())
	}

	return strings.TrimRight(strings.Join(lines, "\n"), "\n")
}
