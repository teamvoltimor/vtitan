package vtcli

import (
	"fmt"
	"strings"
	"time"

	"github.com/charmbracelet/bubbles/spinner"
	"github.com/charmbracelet/bubbles/viewport"
	tea "github.com/charmbracelet/bubbletea"
)

// runModel is the run pane: the equivalent vt line, a live status, and the
// task's output scrolling underneath, kept after the task ends so it can be
// read before going back to the picker. It reports what the user chose next
// through back, rerun and quit; the session acts on them.
type runModel struct {
	line    string
	run     *taskRun
	buf     *termBuf
	view    viewport.Model
	spin    spinner.Model
	ui      UI
	started time.Time
	result  *runDoneMsg
	// note replaces the output when there is none to show: a dry run, or a
	// task that was handed the whole terminal.
	note          string
	width, height int
	interrupts    int
	back          bool
	rerun         bool
	quit          bool
}

// Run pane layout and limits.
const (
	// runChromeLines is the header (line, status, gap) plus the footer.
	runChromeLines  = 4
	runScrollback   = 10000
	minRunViewLines = 3
	// runPageLines is how far the arrows scroll; pgup/pgdn move a page.
	runPageLines = 1
)

// newRunModel builds the pane for one run. run is nil when nothing streams
// into it (dry run, whole-terminal handoff); note then says why.
func newRunModel(ui UI, line string, run *taskRun, note string, width, height int) *runModel {
	m := &runModel{
		line: line, run: run, note: note, ui: ui, started: time.Now(),
		spin: spinner.New(spinner.WithSpinner(spinner.MiniDot)),
	}
	m.view = viewport.New(width, max(height-runChromeLines, minRunViewLines))
	m.view.MouseWheelEnabled = true
	m.resize(width, height)
	m.buf = newTermBuf(m.view.Width, m.view.Height, runScrollback, ui.color)
	m.buf.light = !ui.dark

	return m
}

// Init starts reading output and the spinner.
func (m *runModel) Init() tea.Cmd {
	if m.run == nil {
		return nil
	}

	return tea.Batch(m.run.next(), m.spin.Tick)
}

// Update implements tea.Model.
func (m *runModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) { //nolint:ireturn // tea.Model forces it
	switch msg := msg.(type) {
	case tea.WindowSizeMsg:
		m.resize(msg.Width, msg.Height)
	case runOutputMsg:
		m.buf.feed(msg.data)

		// A task that queried the terminal is waiting on the answer; dropping
		// it stalls the task until its own timeout.
		for _, reply := range m.buf.takeReplies() {
			m.run.input(reply)
		}

		m.refresh()

		return m, m.run.next()
	case tea.MouseMsg:
		var cmd tea.Cmd
		m.view, cmd = m.view.Update(msg)

		return m, cmd
	case runDoneMsg:
		m.result = &msg
		if msg.err != nil {
			m.note = msg.err.Error()
		}

		m.refresh()
	case spinner.TickMsg:
		if m.finished() {
			return m, nil
		}

		var cmd tea.Cmd
		m.spin, cmd = m.spin.Update(msg)

		return m, cmd
	case tea.KeyMsg:
		m.key(msg)
	}

	return m, nil
}

// View implements tea.Model.
func (m *runModel) View() string {
	body := m.view.View()
	if m.note != "" && m.buf.plain() == "" {
		body = m.ui.Muted(m.note)
	}

	return strings.Join([]string{
		m.ui.Accent(equivalentPrefix+m.line, true),
		m.status(),
		"",
		body,
		m.footer(),
	}, "\n")
}

// finished reports whether the task has ended (or never ran).
func (m *runModel) finished() bool {
	return m.run == nil || m.result != nil
}

// key handles scrolling in both states; while the task runs every other key
// is typed into it, and once it ends the keys choose what happens next.
func (m *runModel) key(key tea.KeyMsg) {
	if m.scroll(key.String()) {
		return
	}

	if !m.finished() {
		m.typeKey(key)

		return
	}

	switch key.String() {
	case "enter", "esc", "backspace", "b":
		m.back = true
	case "r":
		m.rerun = true
	case "q", "ctrl+c":
		m.quit = true
	}
}

// scroll moves through the output and reports whether key was a scroll key.
// Scrolling up stops following the tail; reaching the bottom resumes it.
func (m *runModel) scroll(key string) bool {
	switch key {
	case "up":
		m.view.ScrollUp(runPageLines)
	case "down":
		m.view.ScrollDown(runPageLines)
	case "pgup":
		m.view.PageUp()
	case "pgdown":
		m.view.PageDown()
	case "home":
		m.view.GotoTop()
	case "end":
		m.view.GotoBottom()
	default:
		return false
	}

	return true
}

// typeKey forwards one key to the running task. The first ctrl+c interrupts
// it like in a terminal; a second one kills it, for a task that ignores
// SIGINT.
func (m *runModel) typeKey(key tea.KeyMsg) {
	switch key.Type {
	case tea.KeyCtrlC:
		m.interrupts++
		if m.interrupts > 1 {
			m.run.kill()

			return
		}

		m.run.interrupt()
	case tea.KeyEnter:
		m.run.input("\r")
	case tea.KeyBackspace:
		m.run.input("\x7f")
	case tea.KeyTab:
		m.run.input("\t")
	case tea.KeySpace:
		m.run.input(" ")
	case tea.KeyCtrlD:
		m.run.input("\x04")
	case tea.KeyEsc:
		m.run.input("\x1b")
	case tea.KeyRunes:
		m.run.input(string(key.Runes))
	default:
	}
}

// resize fits the output to the terminal and tells the task its new size.
func (m *runModel) resize(width, height int) {
	m.width, m.height = width, height
	m.view.Width = width
	m.view.Height = max(height-runChromeLines, minRunViewLines)

	if m.buf != nil {
		m.buf.resize(m.view.Width, m.view.Height)
		m.refresh()
	}

	if m.run != nil && !m.finished() {
		m.run.resize(m.view.Width, m.view.Height)
	}
}

// refresh re-renders the output, following the tail unless the user has
// scrolled up to read something.
func (m *runModel) refresh() {
	follow := m.view.AtBottom()

	m.view.SetContent(strings.Join(m.buf.render(), "\n"))

	if follow {
		m.view.GotoBottom()
	}
}

// status is the running/exited line under the command.
func (m *runModel) status() string {
	switch {
	case m.run == nil && m.result == nil:
		return m.ui.Muted("not run")
	case m.result == nil:
		return m.spin.View() + " " + m.ui.Muted("running · "+elapsed(time.Since(m.started)))
	case m.result.err != nil:
		return m.ui.Danger("✗ could not run: " + m.result.err.Error())
	case m.result.code == 0:
		return m.ui.Accent("✓ exited 0 after "+elapsed(m.result.elapsed), false)
	default:
		return m.ui.Danger(fmt.Sprintf("✗ exited %d after %s", m.result.code, elapsed(m.result.elapsed)))
	}
}

// footer lists the keys for the current state.
func (m *runModel) footer() string {
	if !m.finished() {
		stop := "ctrl+c: interrupt"
		if m.interrupts > 0 {
			stop = "ctrl+c again: kill"
		}

		return m.ui.Muted(stop + " · ↑↓ pgup/pgdn/wheel: scroll · other keys go to the task")
	}

	return m.ui.Muted("enter/esc: back to the menu · r: run again · q: quit · ↑↓ pgup/pgdn/wheel: scroll")
}

// summary is the line printed after the session ends, one per run, so the
// commands can be copied once the alt screen is gone.
func (m *runModel) summary() string {
	switch {
	case m.result == nil:
		return equivalentPrefix + m.line
	case m.result.err != nil:
		return equivalentPrefix + m.line + "  (could not run)"
	default:
		return fmt.Sprintf("%s%s  (exit %d)", equivalentPrefix, m.line, m.result.code)
	}
}

// elapsed renders a duration to a tenth of a second under a minute, and to
// the second above.
func elapsed(d time.Duration) string {
	if d < time.Minute {
		return fmt.Sprintf("%.1fs", d.Seconds())
	}

	return d.Truncate(time.Second).String()
}
