package vtcli

import (
	"strings"
	"testing"
)

// TestTermBufRedrawsInPlace checks the redraws tools actually emit: a
// carriage-return progress line, erase-to-end, and a two-line progress block
// that moves the cursor up to rewrite itself.
func TestTermBufRedrawsInPlace(t *testing.T) {
	t.Parallel()

	for name, tc := range map[string]struct {
		input string
		want  string
	}{
		"plain lines":        {"one\r\ntwo\r\n", "one\ntwo"},
		"carriage return":    {"10%\r50%\r100%\r\ndone", "100%\ndone"},
		"erase to end":       {"downloading big-file\r\x1b[Kok\r\n", "ok"},
		"cursor up redraw":   {"a 1\r\nb 1\r\n\x1b[2Aa 2\r\nb 2\r\n", "a 2\nb 2"},
		"erase whole line":   {"noise\x1b[2K\rclean", "clean"},
		"erase below":        {"keep\r\nx\r\ny\x1b[1A\r\x1b[J", "keep"},
		"column move":        {"abcdef\x1b[3Gx", "abxdef"},
		"bare newline":       {"a\nb", "a\nb"},
		"tab stops":          {"a\tb", "a       b"},
		"private modes skip": {"\x1b[?25lhidden cursor\x1b[?25h", "hidden cursor"},
		"osc title dropped":  {"\x1b]0;title\x07text", "text"},
		"backspace":          {"abc\b\bX", "aXc"},
	} {
		buf := newTermBuf(80, 24, 100, false)
		buf.feed([]byte(tc.input))

		if got := buf.plain(); got != tc.want {
			t.Errorf("%s: got %q, want %q", name, got, tc.want)
		}
	}
}

// TestTermBufSplitWrites checks that a color sequence and a multi-byte rune
// cut across two reads come out the same as one read.
func TestTermBufSplitWrites(t *testing.T) {
	t.Parallel()

	whole := "\x1b[31mred\x1b[0m ✓ ok"
	for cut := 1; cut < len(whole); cut++ {
		buf := newTermBuf(80, 24, 100, true)
		buf.feed([]byte(whole[:cut]))
		buf.feed([]byte(whole[cut:]))

		if got := buf.plain(); got != "red ✓ ok" {
			t.Fatalf("cut at %d: got %q", cut, got)
		}

		if got := buf.render()[0]; !strings.Contains(got, "\x1b[31mred") {
			t.Fatalf("cut at %d: color lost: %q", cut, got)
		}
	}
}

// TestTermBufWrapsAndTrims checks the width wrap and the scrollback cap, and
// that the render cache follows a trim.
func TestTermBufWrapsAndTrims(t *testing.T) {
	t.Parallel()

	buf := newTermBuf(4, 24, 3, false)
	buf.feed([]byte("abcdefgh"))

	if got := buf.plain(); got != "abcd\nefgh" {
		t.Errorf("wrap: got %q", got)
	}

	_ = buf.render()
	buf.feed([]byte("\r\n1\r\n2\r\n3"))

	if got := strings.Join(buf.render(), "\n"); got != "1\n2\n3" {
		t.Errorf("trim: got %q", got)
	}
}

// TestTermBufColorOff checks that NO_COLOR output carries no escapes.
func TestTermBufColorOff(t *testing.T) {
	t.Parallel()

	buf := newTermBuf(80, 24, 10, false)
	buf.feed([]byte("\x1b[1;32mPASS\x1b[0m"))

	if got := buf.render()[0]; got != "PASS" {
		t.Errorf("got %q, want no escapes", got)
	}
}

// TestTermBufAnswersColorQueries checks that the terminal colour queries a
// task waits on before it draws are answered for the detected background,
// while a title still is not.
func TestTermBufAnswersColorQueries(t *testing.T) {
	t.Parallel()

	dark := newTermBuf(80, 24, 10, false)
	dark.feed([]byte("\x1b]11;?\x1b\\drawing"))

	if got := dark.plain(); got != "drawing" {
		t.Errorf("query leaked into the output: %q", got)
	}

	if got := dark.takeReplies(); len(got) != 1 || !strings.Contains(got[0], "11;rgb:0000/0000/0000") {
		t.Errorf("dark background reply: %q", got)
	}

	if got := dark.takeReplies(); got != nil {
		t.Errorf("replies were not cleared: %q", got)
	}

	light := newTermBuf(80, 24, 10, false)
	light.light = true
	light.feed([]byte("\x1b]10;?\x07"))

	if got := light.takeReplies(); len(got) != 1 || !strings.Contains(got[0], "10;rgb:0000/0000/0000") {
		t.Errorf("foreground reply on a light background: %q", got)
	}

	title := newTermBuf(80, 24, 10, false)
	title.feed([]byte("\x1b]0;a title\x07"))

	if got := title.takeReplies(); got != nil {
		t.Errorf("a title produced a reply: %q", got)
	}
}
