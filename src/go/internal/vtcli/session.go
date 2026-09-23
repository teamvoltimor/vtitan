package vtcli

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"strings"

	tea "github.com/charmbracelet/bubbletea"
)

// sessionModel is the whole interactive vt: the picker, the argument form and
// the run pane in one program, so running a command does not end the session.
// A run's output stays on screen until the user goes back, and going back
// returns to the level the command was picked from. Each screen reports its
// outcome through fields (picker.selected, form.submit, run.back, ...), and
// the session drops the tea.Quit they return for it, since here only the
// session quits.
type sessionModel struct {
	app    *App
	ctx    context.Context //nolint:containedctx // tea.Model has no ctx parameter; runs started from Update need one
	screen sessionScreen
	picker *pickerModel
	form   *formModel
	run    *runModel
	// pending is what the open form will run once submitted, and what `r`
	// runs again from the run pane.
	pending       *pendingRun
	history       []string
	width, height int
}

// sessionScreen is the screen the session is showing.
type sessionScreen int

// pendingRun is a picked command or task waiting for its values.
type pendingRun struct {
	command *Command
	task    string
	values  map[string]string
}

// execDoneMsg ends a whole-terminal run (Command.Terminal).
type execDoneMsg struct{ result runDoneMsg }

const (
	screenPicker sessionScreen = iota
	screenForm
	screenRun
)

// runSession runs the interactive session until the user quits, then prints
// one line per run so the commands survive the alt screen.
func (a *App) runSession(ctx context.Context, out *os.File) error {
	session := &sessionModel{
		app: a, ctx: ctx, screen: screenPicker,
		picker: newPickerModel(a.buildPickerRoot(), a.childLevel, a.taskLevel(), a.ui),
		width:  defaultPickerWidth, height: defaultPickerHeight,
	}

	program := tea.NewProgram(session, tea.WithAltScreen(), tea.WithMouseCellMotion(),
		tea.WithInput(os.Stdin), tea.WithOutput(out))
	if _, err := program.Run(); err != nil {
		return fmt.Errorf("interactive session: %w", err)
	}

	for _, line := range session.history {
		fmt.Fprintln(os.Stderr, a.ui.Muted(line))
	}

	return nil
}

// Init implements tea.Model.
func (s *sessionModel) Init() tea.Cmd { return s.picker.Init() }

// Update implements tea.Model: sizes go to every screen, everything else to
// the one showing.
func (s *sessionModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) { //nolint:ireturn // tea.Model forces it
	switch msg := msg.(type) {
	case tea.WindowSizeMsg:
		s.width, s.height = msg.Width, msg.Height
		s.picker.Update(msg)

		if s.form != nil {
			s.form.Update(msg)
		}

		if s.run != nil {
			s.run.Update(msg)
		}

		return s, nil
	case execDoneMsg:
		if s.run != nil {
			s.run.result = &msg.result
			s.history = append(s.history, s.run.summary())
		}

		return s, nil
	}

	var cmd tea.Cmd

	switch s.screen {
	case screenForm:
		cmd = s.updateForm(msg)
	case screenRun:
		cmd = s.updateRun(msg)
	default:
		cmd = s.updatePicker(msg)
	}

	return s, cmd
}

// View implements tea.Model.
func (s *sessionModel) View() string {
	switch s.screen {
	case screenForm:
		return s.form.View()
	case screenRun:
		return s.run.View()
	default:
		return s.picker.View()
	}
}

// updatePicker forwards to the picker and opens what it selects.
func (s *sessionModel) updatePicker(msg tea.Msg) tea.Cmd {
	_, cmd := s.picker.Update(msg)

	if s.picker.cancelled {
		return tea.Quit
	}

	selected := s.picker.selected
	if selected == nil {
		return cmd
	}

	s.picker.selected = nil

	pending := &pendingRun{command: selected.spec, task: selected.task}
	if selected.spec != nil {
		// The picker's item points into the spec; copy so a later edit of
		// the pending run cannot reach it.
		command := *selected.spec
		pending.command = &command
	}

	return s.open(pending)
}

// open shows the form for pending, or runs it directly when it has nothing
// to ask.
func (s *sessionModel) open(pending *pendingRun) tea.Cmd {
	s.pending = pending

	form := s.app.formFor(pending)
	if form == nil {
		pending.values = map[string]string{}

		return s.start()
	}

	s.form = form
	s.form.Update(tea.WindowSizeMsg{Width: s.width, Height: s.height})
	s.screen = screenForm

	return s.form.Init()
}

// updateForm forwards to the form; esc goes back to the picker, submitting
// runs.
func (s *sessionModel) updateForm(msg tea.Msg) tea.Cmd {
	_, cmd := s.form.Update(msg)

	switch {
	case s.form.cancelled:
		s.form, s.screen = nil, screenPicker

		return nil
	case s.form.submit:
		s.pending.values = s.form.values()
		s.form = nil
		return s.start()
	}

	return cmd
}

// updateRun forwards to the run pane and acts on its back/rerun/quit.
func (s *sessionModel) updateRun(msg tea.Msg) tea.Cmd {
	_, cmd := s.run.Update(msg)

	if _, done := msg.(runDoneMsg); done {
		s.history = append(s.history, s.run.summary())
	}

	switch {
	case s.run.quit:
		return tea.Quit
	case s.run.rerun:
		return s.start()
	case s.run.back:
		s.run, s.screen = nil, screenPicker
		// A run is now the most recent pick; show it at the top of the root.
		s.picker.setRoot(s.app.buildPickerRoot())

		return nil
	}

	return cmd
}

// start resolves the pending run and shows it in the run pane: streamed on a
// PTY, handed the whole terminal, or only displayed under --dry-run.
func (s *sessionModel) start() tea.Cmd {
	name, args, line, display, err := s.app.resolve(s.pending)
	s.screen = screenRun

	if err != nil {
		s.run = newRunModel(s.app.ui, line, nil, err.Error(), s.width, s.height)

		return nil
	}

	if s.app.dryRun {
		s.run = newRunModel(s.app.ui, line, nil, "dry run, not executed:\n\n"+display, s.width, s.height)

		return nil
	}

	s.app.remember(s.recentEntry())

	if s.pending.command != nil && s.pending.command.Terminal {
		s.run = newRunModel(s.app.ui, line, nil,
			"this command used the whole terminal; its output is not kept here", s.width, s.height)

		return tea.ExecProcess(
			taskCommand(s.ctx, s.app.repoRoot, name, args),
			func(err error) tea.Msg { return execDoneMsg{result: exitResult(err)} },
		)
	}

	run, err := startTaskRun(s.ctx, s.app.repoRoot, name, args, s.width, max(s.height-runChromeLines, 1))
	if err != nil {
		s.run = newRunModel(s.app.ui, line, nil, err.Error(), s.width, s.height)

		return nil
	}

	s.run = newRunModel(s.app.ui, line, run, "", s.width, s.height)

	return s.run.Init()
}

// recentEntry is how the pending run is remembered for the picker's top rows.
func (s *sessionModel) recentEntry() string {
	if s.pending.command != nil {
		return recentCommandPrefix + strings.Join(s.pending.command.Path, " ")
	}

	return recentTaskPrefix + s.pending.task
}

// exitResult turns a finished process's error into the run's outcome.
func exitResult(err error) runDoneMsg {
	var exitErr *exec.ExitError

	switch {
	case err == nil:
		return runDoneMsg{}
	case errors.As(err, &exitErr):
		return runDoneMsg{code: exitErr.ExitCode()}
	default:
		return runDoneMsg{code: -1, err: err}
	}
}
