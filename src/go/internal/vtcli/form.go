package vtcli

import (
	"fmt"
	"os"
	"strconv"
	"strings"

	"github.com/charmbracelet/bubbles/textinput"
	tea "github.com/charmbracelet/bubbletea"
)

// formModel drives the argument form after a command is chosen, ending in a
// confirmation step so a stray enter never launches anything heavy.
type formModel struct {
	title     string
	fields    []*formField
	index     int
	submit    bool
	cancelled bool
	err       string
	confirm   bool
	heavy     bool
	ui        UI
	// preview renders the invocation the current values resolve to, or why
	// they do not; it is planInvocation's own output, not a second copy.
	preview func(values map[string]string) (string, error)
}

// formField is one prompt in the argument form: a text input, or a checkbox
// for a boolean flag or a variant switch.
type formField struct {
	label    string
	help     string
	kind     FlagKind
	required bool
	isBool   bool
	boolVal  bool
	input    textinput.Model
}

// backRowTitle labels the row that returns to the parent level, and
// backSegment is the sentinel choose() recognises for it.
const (
	backRowTitle = ".. back"
	backSegment  = "\x00back"
)

// newTextField creates a text field for one argument or flag.
func newTextField(name, help string, required bool, kind FlagKind) *formField {
	input := textinput.New()
	input.Placeholder = help
	input.CharLimit = formCharLimit
	input.Width = defaultFormInputWidth

	return &formField{label: name, help: help, kind: kind, required: required, input: input}
}

// newBoolField creates a checkbox field starting at value.
func newBoolField(name, help string, value bool) *formField {
	field := newTextField(name, help, false, FlagBool)
	field.isBool = true
	field.boolVal = value

	return field
}

// secret hides what is typed into the field.
func (f *formField) secret(on bool) {
	if on {
		f.input.EchoMode = textinput.EchoPassword
	}
}

// runForm runs the argument form and returns the collected values, after the
// confirmation screen shows what preview says will run.
func runForm(
	ui UI,
	title string,
	fields []*formField,
	preview func(map[string]string) (string, error),
	heavy bool,
) (values map[string]string, ok bool, err error) {
	model := formModel{title: title, fields: fields, heavy: heavy, ui: ui, preview: preview}
	model.focus(0)

	program := tea.NewProgram(&model, tea.WithAltScreen(), tea.WithInput(os.Stdin), tea.WithOutput(os.Stdout))

	final, err := program.Run()
	if err != nil {
		return nil, false, fmt.Errorf("form: %w", err)
	}

	result, isForm := final.(*formModel)
	if !isForm || result.cancelled || !result.submit {
		return nil, false, nil
	}

	return result.values(), true, nil
}

// Init implements tea.Model.
func (m *formModel) Init() tea.Cmd { return textinput.Blink }

// Update implements tea.Model.
func (m *formModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	if size, isResize := msg.(tea.WindowSizeMsg); isResize {
		m.resizeInputs(size.Width)

		return m, nil
	}

	if key, ok := msg.(tea.KeyMsg); ok {
		if handled, cmd := m.handleKey(key); handled {
			return m, cmd
		}
	}

	var cmd tea.Cmd
	m.fields[m.index].input, cmd = m.fields[m.index].input.Update(msg)

	return m, cmd
}

// View implements tea.Model.
func (m *formModel) View() string {
	lines := []string{m.ui.Accent(m.title, true), ""}

	for i, field := range m.fields {
		cursor := "  "
		if i == m.index {
			cursor = m.ui.Accent("> ", true)
		}

		if field.isBool {
			mark := " "
			if field.boolVal {
				mark = "x"
			}

			lines = append(lines, cursor+field.label+"  ["+mark+"]  "+m.ui.Muted(field.help))

			continue
		}

		lines = append(lines, cursor+field.label, "    "+field.input.View())
	}

	if m.err != "" {
		lines = append(lines, "", m.ui.Danger(m.err))
	}

	if m.confirm {
		line, err := m.preview(m.values())
		if err != nil {
			line = err.Error()
		}
		lines = append(lines, "", "Will run:  "+m.ui.Accent(line, true))

		if m.heavy {
			lines = append(lines, m.ui.Warning("WARNING: this task starts long-running processes (simulator/service)."))
		}

		lines = append(lines, "", m.ui.Muted("(enter: run · esc: cancel)"))

		return strings.Join(lines, "\n")
	}

	lines = append(lines, "", m.ui.Muted("(tab: next · space: toggle · enter: next · esc: cancel) · "+
		"untouched defaults stay with Task"))

	return strings.Join(lines, "\n")
}

// resizeInputs keeps the text inputs as wide as the terminal allows.
func (m *formModel) resizeInputs(termWidth int) {
	width := max(termWidth-formWidthMargin, minFormInputWidth)

	for _, field := range m.fields {
		field.input.Width = width
	}
}

// handleKey processes navigation and toggles. It reports whether the key was
// consumed by the form rather than the focused input.
func (m *formModel) handleKey(key tea.KeyMsg) (bool, tea.Cmd) {
	switch key.String() {
	case "ctrl+c", "esc":
		m.cancelled = true

		return true, tea.Quit
	case "tab", "down":
		m.focus(m.index + 1)

		return true, nil
	case "shift+tab", "up":
		m.focus(m.index - 1)

		return true, nil
	case "enter":
		return m.enter()
	}

	if m.fields[m.index].isBool && (key.String() == "left" || key.String() == "right" || key.String() == " ") {
		m.fields[m.index].boolVal = !m.fields[m.index].boolVal

		return true, nil
	}

	return false, nil
}

// enter advances a field, or on the last one validates and opens the
// confirmation, or on the confirmation submits.
func (m *formModel) enter() (bool, tea.Cmd) {
	if m.confirm {
		m.submit = true

		return true, tea.Quit
	}

	if m.index < len(m.fields)-1 {
		m.focus(m.index + 1)

		return true, nil
	}

	if err := m.validate(); err != nil {
		m.err = err.Error()

		return true, nil
	}

	m.confirm = true
	m.err = ""

	return true, nil
}

// focus moves the highlight to index, wrapping around.
func (m *formModel) focus(index int) {
	if len(m.fields) == 0 {
		return
	}

	if index < 0 {
		index = len(m.fields) - 1
	}

	if index >= len(m.fields) {
		index = 0
	}

	for i, field := range m.fields {
		if i == index {
			field.input.Focus()
		} else {
			field.input.Blur()
		}
	}

	m.index = index
}

// validate enforces required fields and integers, then asks the planner, so
// an unknown board or two exclusive switches are caught before confirming.
func (m *formModel) validate() error {
	for _, field := range m.fields {
		if field.isBool {
			continue
		}

		value := strings.TrimSpace(field.input.Value())
		if field.required && value == "" {
			return fmt.Errorf("field %q is required", field.label)
		}

		if field.kind == FlagInt && value != "" {
			if _, err := strconv.Atoi(value); err != nil {
				return fmt.Errorf("field %q must be an integer", field.label)
			}
		}
	}

	_, err := m.preview(m.values())

	return err
}

// values collects the form state as a name -> value map.
func (m *formModel) values() map[string]string {
	values := make(map[string]string, len(m.fields))
	for _, field := range m.fields {
		if field.isBool {
			values[field.label] = strconv.FormatBool(field.boolVal)

			continue
		}

		values[field.label] = strings.TrimSpace(field.input.Value())
	}

	return values
}
