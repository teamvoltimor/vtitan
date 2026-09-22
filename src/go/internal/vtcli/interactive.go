package vtcli

import (
	"context"
	"fmt"
	"os"
	"strconv"
	"strings"

	"github.com/charmbracelet/bubbles/list"
	"github.com/charmbracelet/bubbles/textinput"
	tea "github.com/charmbracelet/bubbletea"
	"github.com/spf13/cobra"
)

// leafItem is one selectable row in the interactive picker.
type leafItem struct {
	title  string
	desc   string
	spec   Command
	escape bool
}

// pickerModel drives the command list. Selecting an item quits with selected
// set; q/ctrl+c sets cancelled.
type pickerModel struct {
	list      list.Model
	selected  *leafItem
	cancelled bool
}

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
	taskName  string
}

// formField is one prompt in the argument form.
type formField struct {
	label    string
	help     string
	kind     FlagKind
	required bool
	isBool   bool
	boolVal  bool
	input    textinput.Model
}

// formCharLimit bounds how much a single argument field accepts.
const formCharLimit = 512

// picker sizing: a default for the first frame, before the terminal reports
// its real dimensions via tea.WindowSizeMsg.
const (
	defaultPickerWidth  = 80
	defaultPickerHeight = 24
)

// canPick reports whether an interactive picker is appropriate: both ends are
// a terminal and the user has not opted out.
func (a *App) canPick() bool {
	return os.Getenv("VT_NO_PICKER") == "" && isTerminal(os.Stdin) && isTerminal(os.Stdout)
}

// home is what a bare `vt` does: pick interactively on a terminal, otherwise
// print the static menu so pipes and CI stay readable.
func (a *App) home(cmd *cobra.Command) error {
	if a.canPick() {
		return a.pickAndRun(cmd)
	}

	return a.printHome(cmd)
}

// printHome renders the non-interactive banner + menu.
func (a *App) printHome(cmd *cobra.Command) error {
	out := cmd.OutOrStdout()
	home := a.ui.Banner() + "\n\n" + a.ui.Menu(a.menuEntries()) +
		"\n\nUsa `vt <dominio> --help` para el detalle, o `vt run <tarea>` para el resto."

	if _, err := fmt.Fprintln(out, home); err != nil {
		return fmt.Errorf("write home: %w", err)
	}

	return nil
}

// pickerItems flattens the curated leaves into pickable rows, plus one escape
// hatch entry for any Task.
func (a *App) pickerItems() []leafItem {
	items := make([]leafItem, 0, len(a.spec)+1)
	for _, command := range a.spec {
		items = append(items, leafItem{
			title: strings.Join(command.Path, " "),
			desc:  command.Short,
			spec:  command,
		})
	}

	items = append(items, leafItem{
		title:  "run <tarea>",
		desc:   "Escape hatch: cualquier tarea de Task, con sus argumentos",
		escape: true,
	})

	return items
}

// pickAndRun opens the picker, then the argument form, then runs the task.
func (a *App) pickAndRun(cmd *cobra.Command) error {
	program := tea.NewProgram(
		newPickerModel(a.pickerItems()),
		tea.WithAltScreen(),
		tea.WithInput(os.Stdin),
		tea.WithOutput(cmd.OutOrStdout()),
	)

	final, err := program.Run()
	if err != nil {
		return fmt.Errorf("selector: %w", err)
	}

	result, ok := final.(pickerModel)
	if !ok || result.cancelled || result.selected == nil {
		return nil
	}

	if result.selected.escape {
		return a.promptEscape(cmd.Context())
	}

	values, submitted, err := a.promptFields(result.selected.spec)
	if err != nil || !submitted {
		return err
	}

	passthrough := values["args"]
	delete(values, "args")

	return a.invoke(cmd.Context(), result.selected.spec, values, passthrough)
}

// promptEscape asks for a raw task name and its arguments.
func (a *App) promptEscape(ctx context.Context) error {
	fields := []*formField{
		newTextField("tarea", "nombre exacto, p. ej. go:test:hw", true, FlagString),
		newTextField("args", "VAR=valor y flags, tal cual", false, FlagString),
	}

	values, submitted, err := runForm("run <tarea>", "", fields)
	if err != nil || !submitted {
		return err
	}

	name := values["tarea"]
	if !KnownTask(a.tasks, name) {
		return fmt.Errorf("tarea %q desconocida", name)
	}

	return runTask(ctx, a.repoRoot, name, splitArgs(values["args"]))
}

// promptFields builds and runs the argument form for one command.
func (a *App) promptFields(command Command) (values map[string]string, ok bool, err error) {
	fields := make([]*formField, 0, len(command.Args)+len(command.Flags)+1)

	for _, arg := range command.Args {
		fields = append(fields, newTextField(arg.Name, arg.Usage, arg.Required, FlagString))
	}

	for _, flag := range command.Flags {
		field := newTextField(flag.Name, flag.Usage, flag.Required, flag.Kind)
		if flag.Kind == FlagBool {
			field.isBool = true
			field.boolVal = boolDefault(flag.Default)
		} else {
			field.input.SetValue(flag.Default)
		}

		fields = append(fields, field)
	}

	if command.Passthrough {
		fields = append(fields, newTextField("args", "argumentos extra tras --", false, FlagString))
	}

	if len(fields) == 0 {
		return map[string]string{}, true, nil
	}

	return runFormHeavy(strings.Join(command.Path, " "), command.Task, fields, command.Heavy)
}

// newTextField creates a text field for one argument.
func newTextField(name, help string, required bool, kind FlagKind) *formField {
	input := textinput.New()
	input.Placeholder = help
	input.CharLimit = formCharLimit

	return &formField{label: name, help: help, kind: kind, required: required, input: input}
}

// runForm runs the argument form and returns the collected values.
func runForm(title, taskName string, fields []*formField) (values map[string]string, ok bool, err error) {
	return runFormHeavy(title, taskName, fields, false)
}

// runFormHeavy runs the form and, before executing, asks for confirmation.
func runFormHeavy(title, taskName string, fields []*formField, heavy bool) (
	values map[string]string, ok bool, err error,
) {
	model := formModel{title: title, fields: fields, heavy: heavy, taskName: taskName}
	model.focus(0)

	program := tea.NewProgram(&model, tea.WithAltScreen(), tea.WithInput(os.Stdin), tea.WithOutput(os.Stdout))

	final, err := program.Run()
	if err != nil {
		return nil, false, fmt.Errorf("formulario: %w", err)
	}

	result, isForm := final.(*formModel)
	if !isForm || result.cancelled || !result.submit {
		return nil, false, nil
	}

	return result.values(), true, nil
}

// invoke maps the collected values to Task vars and runs the task.
func (a *App) invoke(ctx context.Context, command Command, values map[string]string, passed string) error {
	extra, err := buildExtra(command, values, passed)
	if err != nil {
		return err
	}

	return runTask(ctx, a.repoRoot, command.Task, extra)
}

// buildExtra turns form values into the KEY=value pairs Task expects.
func buildExtra(command Command, values map[string]string, passed string) ([]string, error) {
	extra := make([]string, 0, len(command.Args)+len(command.Flags)+2)

	for _, arg := range command.Args {
		value := strings.TrimSpace(values[arg.Name])
		if value == "" {
			if arg.Required {
				return nil, fmt.Errorf("falta el argumento %q", arg.Name)
			}

			continue
		}

		extra = append(extra, arg.Var+"="+value)
	}

	for _, flag := range command.Flags {
		value := strings.TrimSpace(values[flag.Name])
		if value == "" && flag.Kind != FlagBool {
			continue
		}

		if value == "" {
			value = strconv.FormatBool(false)
		}

		extra = append(extra, flag.Var+"="+value)
	}

	if command.Passthrough && strings.TrimSpace(passed) != "" {
		extra = append(extra, "--")
		extra = append(extra, splitArgs(passed)...)
	}

	return extra, nil
}

// splitArgs splits a free-text argument line on spaces, honouring quotes.
func splitArgs(raw string) []string {
	var (
		args  []string
		buf   []rune
		quote rune
	)

	flush := func() {
		if len(buf) > 0 {
			args = append(args, string(buf))
			buf = nil
		}
	}

	for _, char := range raw {
		switch {
		case quote != 0:
			if char == quote {
				quote = 0
			} else {
				buf = append(buf, char)
			}
		case char == '\'' || char == '"':
			quote = char
		case char == ' ' || char == '\t':
			flush()
		default:
			buf = append(buf, char)
		}
	}

	flush()

	return args
}

// Title implements list.Item.
func (i leafItem) Title() string { return i.title }

// Description implements list.Item.
func (i leafItem) Description() string { return i.desc }

// FilterValue implements list.Item.
func (i leafItem) FilterValue() string { return i.title }

// newPickerModel builds the list model for the picker.
func newPickerModel(items []leafItem) pickerModel {
	entries := make([]list.Item, 0, len(items))
	for index := range items {
		entries = append(entries, items[index])
	}

	model := list.New(entries, list.NewDefaultDelegate(), defaultPickerWidth, defaultPickerHeight)
	model.Title = "vt — elige un comando"
	model.SetFilteringEnabled(true)
	model.SetShowHelp(true)
	model.SetShowStatusBar(true)
	model.SetShowPagination(true)

	return pickerModel{list: model}
}

// Init implements tea.Model.
func (m pickerModel) Init() tea.Cmd { return nil }

// Update implements tea.Model. Keys are always delegated to the list first:
// inside the filter input, enter applies the filter rather than choosing, and
// that distinction is the list's to make, not ours.
func (m pickerModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	if size, isResize := msg.(tea.WindowSizeMsg); isResize {
		m.list.SetSize(size.Width, size.Height)

		return m, nil
	}

	key, isKey := msg.(tea.KeyMsg)
	if isKey && m.list.FilterState() != list.Filtering {
		switch key.String() {
		case "ctrl+c", "q":
			m.cancelled = true

			return m, tea.Quit
		case "enter":
			if item, selected := m.list.SelectedItem().(leafItem); selected {
				m.selected = &item

				return m, tea.Quit
			}
		}
	}

	var cmd tea.Cmd
	m.list, cmd = m.list.Update(msg)

	return m, cmd
}

// View implements tea.Model.
func (m pickerModel) View() string { return m.list.View() }

// Init implements tea.Model.
func (m *formModel) Init() tea.Cmd { return textinput.Blink }

// Update implements tea.Model.
func (m *formModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
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
	lines := []string{m.title, ""}

	for i, field := range m.fields {
		cursor := "  "
		if i == m.index {
			cursor = "> "
		}

		if field.isBool {
			mark := " "
			if field.boolVal {
				mark = "x"
			}

			lines = append(lines, cursor+field.label+"  ["+mark+"]  "+field.help)

			continue
		}

		lines = append(lines, cursor+field.label, "    "+field.input.View())
	}

	if m.err != "" {
		lines = append(lines, "", m.err)
	}

	if m.confirm {
		lines = append(lines, "", "Se ejecutará:  "+m.commandLine())

		if m.heavy {
			lines = append(lines, "AVISO: esta tarea lanza procesos de larga duración (simulador/servicio).")
		}

		lines = append(lines, "", "(enter: ejecutar · esc: cancelar)")

		return strings.Join(lines, "\n")
	}

	lines = append(lines, "", "(tab: siguiente · enter: siguiente · esc: cancelar)")

	return strings.Join(lines, "\n")
}

// commandLine renders the task command the form is about to run.
func (m *formModel) commandLine() string {
	target := m.taskName
	if target == "" {
		target = m.title
	}

	parts := []string{"task", target}
	for _, field := range m.fields {
		if field.isBool {
			parts = append(parts, field.label+"="+strconv.FormatBool(field.boolVal))

			continue
		}

		value := strings.TrimSpace(field.input.Value())
		if value != "" {
			parts = append(parts, field.label+"="+value)
		}
	}

	return strings.Join(parts, " ")
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
		if m.confirm {
			m.submit = true

			return true, tea.Quit
		}

		if m.index == len(m.fields)-1 {
			if err := m.validate(); err != nil {
				m.err = err.Error()

				return true, nil
			}

			m.confirm = true
			m.err = ""

			return true, nil
		}

		m.focus(m.index + 1)

		return true, nil
	}

	if m.fields[m.index].isBool && (key.String() == "left" || key.String() == "right" || key.String() == " ") {
		m.fields[m.index].boolVal = !m.fields[m.index].boolVal

		return true, nil
	}

	return false, nil
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

// validate enforces required fields and integer parsing.
func (m *formModel) validate() error {
	for _, field := range m.fields {
		if field.isBool {
			continue
		}

		value := strings.TrimSpace(field.input.Value())
		if field.required && value == "" {
			return fmt.Errorf("el campo %q es obligatorio", field.label)
		}

		if field.kind == FlagInt && value != "" {
			if _, err := strconv.Atoi(value); err != nil {
				return fmt.Errorf("el campo %q debe ser un entero", field.label)
			}
		}
	}

	return nil
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
