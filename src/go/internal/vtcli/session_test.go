package vtcli

import (
	"os/exec"
	"strings"
	"testing"
	"time"

	tea "github.com/charmbracelet/bubbletea"
)

// newTestSession builds a session over the real spec, remembering nothing.
func newTestSession(t *testing.T) *sessionModel {
	t.Helper()

	app := newSpecApp(t)
	app.recentPath = ""

	session := &sessionModel{
		app: app, ctx: t.Context(), screen: screenPicker,
		picker: newPickerModel(app.buildPickerRoot(), app.childLevel, nil, app.ui),
		width:  defaultPickerWidth, height: defaultPickerHeight,
	}

	return session
}

// key sends one key press to the session.
func key(s *sessionModel, name string) tea.Cmd {
	var msg tea.KeyMsg

	switch name {
	case "enter":
		msg = tea.KeyMsg{Type: tea.KeyEnter}
	case "esc":
		msg = tea.KeyMsg{Type: tea.KeyEsc}
	default:
		msg = tea.KeyMsg{Type: tea.KeyRunes, Runes: []rune(name)}
	}

	_, cmd := s.Update(msg)

	return cmd
}

// specCommand returns the spec entry at path.
func specCommand(t *testing.T, path string) *Command {
	t.Helper()

	for i := range curatedSpec {
		if strings.Join(curatedSpec[i].Path, " ") == path {
			command := curatedSpec[i]

			return &command
		}
	}

	t.Fatalf("no command %q in the spec", path)

	return nil
}

// TestSessionStaysOpenAcrossRuns walks the loop the session exists for: pick
// a command, fill the form, see the result in the run pane, go back to the
// same picker level, and quit only when asked.
func TestSessionStaysOpenAcrossRuns(t *testing.T) {
	t.Parallel()

	s := newTestSession(t)
	s.app.dryRun = true

	s.picker.descend("hailo")
	s.open(&pendingRun{command: specCommand(t, "hailo clean")})

	if s.screen != screenForm {
		t.Fatalf("a command with switches opened screen %d, want the form", s.screen)
	}

	key(s, "esc")

	if s.screen != screenPicker || s.picker.cancelled {
		t.Fatalf("esc in the form: screen %d cancelled %v, want back in the picker", s.screen, s.picker.cancelled)
	}

	s.open(&pendingRun{command: specCommand(t, "hailo clean")})
	// enter walks the fields, opens the confirmation, then confirms.
	for range len(commandFields(*s.pending.command)) + 1 {
		key(s, "enter")
	}

	if s.screen != screenRun || !strings.Contains(s.View(), "task hailo:clean:all") {
		t.Fatalf("after confirming: screen %d, view:\n%s", s.screen, s.View())
	}

	key(s, "enter")

	if s.screen != screenPicker {
		t.Fatalf("enter in a finished run pane: screen %d, want the picker", s.screen)
	}

	if got := strings.Join(s.picker.trail, " "); got != "hailo" {
		t.Errorf("back from a run lands at %q, want the level it was picked from (hailo)", got)
	}

	if cmd := key(s, "q"); cmd == nil || !s.picker.cancelled {
		t.Error("q in the picker did not end the session")
	}
}

// TestSessionRunsWithoutFormAndReruns checks a command with nothing to ask
// goes straight to the run pane, and r runs it again.
func TestSessionRunsWithoutFormAndReruns(t *testing.T) {
	t.Parallel()

	s := newTestSession(t)
	s.app.dryRun = true

	s.open(&pendingRun{command: specCommand(t, "hailo test")})

	if s.screen != screenRun {
		t.Fatalf("a command without fields opened screen %d, want the run pane", s.screen)
	}

	first := s.run
	key(s, "r")

	if s.run == first || s.screen != screenRun {
		t.Error("r did not start a new run")
	}
}

// TestTaskRunStreamsOnAPTY runs a real `task` on a PTY and checks its output
// and exit status reach the run pane, for a success and a failure.
func TestTaskRunStreamsOnAPTY(t *testing.T) {
	t.Parallel()

	if _, err := exec.LookPath(taskBinary); err != nil {
		t.Skipf("task not installed: %v", err)
	}

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	for name, tc := range map[string]struct {
		args     []string
		wantCode int
		wantText string
	}{
		"success": {args: []string{"--version"}, wantCode: 0, wantText: "3."},
		"failure": {args: []string{"this:does:not:exist"}, wantCode: 200, wantText: "does not exist"},
	} {
		run, startErr := startTaskRun(t.Context(), root, tc.args[0], tc.args[1:], 80, 24)
		if startErr != nil {
			t.Fatalf("%s: start: %v", name, startErr)
		}

		pane := newRunModel(NewUI(), "vt test", run, "", 80, 30)
		deadline := time.After(20 * time.Second)

		for pane.result == nil {
			msgs := make(chan tea.Msg, 1)
			go func() { msgs <- run.next()() }()

			select {
			case msg := <-msgs:
				pane.Update(msg)
			case <-deadline:
				t.Fatalf("%s: task did not finish", name)
			}
		}

		if pane.result.code != tc.wantCode {
			t.Errorf("%s: exit %d, want %d", name, pane.result.code, tc.wantCode)
		}

		if got := pane.buf.plain(); !strings.Contains(got, tc.wantText) {
			t.Errorf("%s: output %q does not contain %q", name, got, tc.wantText)
		}
	}
}
