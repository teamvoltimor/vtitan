package vtcli

import (
	"context"
	"fmt"
	"os"
	"strconv"
	"strings"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/spf13/cobra"
)

// argsField is the form field that carries what goes after `--`, or, for the
// escape hatch, the task's raw arguments.
const argsField = "args"

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
	home := a.ui.Banner() + "\n\n" + a.ui.MenuSections(a.menuSections()) + "\n\n" +
		fmt.Sprintf(
			"Use `vt <domain> --help` for details, `vt %[1]s` to list every task, or `vt %[1]s <task>` to run one.",
			catchAllName,
		)

	if _, err := fmt.Fprintln(cmd.OutOrStdout(), home); err != nil {
		return fmt.Errorf("write home: %w", err)
	}

	return nil
}

// pickAndRun opens the picker, then the argument form, then runs the task.
func (a *App) pickAndRun(cmd *cobra.Command) error {
	program := tea.NewProgram(
		newPickerModel(a.buildPickerRoot(), a.childLevel, a.taskLevel(), a.ui),
		tea.WithAltScreen(),
		tea.WithInput(os.Stdin),
		tea.WithOutput(cmd.OutOrStdout()),
	)

	final, err := program.Run()
	if err != nil {
		return fmt.Errorf("selector: %w", err)
	}

	result, ok := final.(*pickerModel)
	if !ok || result.cancelled || result.selected == nil {
		return nil
	}

	if result.selected.task != "" {
		return a.promptTaskArgs(cmd.Context(), result.selected.task)
	}

	if result.selected.spec == nil {
		return nil
	}

	command := *result.selected.spec

	values, submitted, err := a.promptFields(command)
	if err != nil || !submitted {
		return err
	}

	passthrough := splitArgs(values[argsField])
	a.echo(vtLine(command, values, passthrough))

	return a.execute(cmd.Context(), command, values, passthrough)
}

// execute resolves a command against its values and runs the task. Every path
// into a curated command ends here, so the picker and the CLI cannot diverge.
func (a *App) execute(ctx context.Context, command Command, values map[string]string, passthrough []string) error {
	inv, err := planInvocation(command, values, passthrough)
	if err != nil {
		return err
	}

	if a.dryRun {
		fmt.Fprintln(os.Stdout, inv.display())

		return nil
	}

	a.remember(recentCommandPrefix + strings.Join(command.Path, " "))

	return runTask(ctx, a.repoRoot, inv.task, inv.argv())
}

// promptTaskArgs asks for the free-form arguments of a task picked from the
// full inventory, which has no typed flags to build a form from.
func (a *App) promptTaskArgs(ctx context.Context, name string) error {
	field := newTextField(argsField, "VAR=value and flags, verbatim (optional)", false, FlagString)

	preview := func(values map[string]string) (string, error) {
		line := "task " + name
		if args := strings.TrimSpace(values[argsField]); args != "" {
			line += " " + args
		}

		return line, nil
	}

	values, submitted, err := runForm(a.ui, catchAllName+" "+name, []*formField{field}, preview, false)
	if err != nil || !submitted {
		return err
	}

	args := splitArgs(values[argsField])
	a.echo(strings.Join(append([]string{"vt", catchAllName, name}, args...), " "))
	a.remember(recentTaskPrefix + name)

	return runTask(ctx, a.repoRoot, name, args)
}

// promptFields builds and runs the argument form for one command. The preview
// is planInvocation's own rendering, so the confirmation shows exactly what
// will run.
func (a *App) promptFields(command Command) (values map[string]string, ok bool, err error) {
	fields := make([]*formField, 0, len(command.Args)+len(command.Flags)+len(command.Variants)+1)

	for _, arg := range command.Args {
		usage := arg.Usage
		if usage == "" && arg.Tasks != nil {
			usage = useLine(Command{Path: []string{""}, Args: []Arg{arg}})
		}

		fields = append(fields, newTextField(arg.Name, usage, arg.Required, FlagString))
	}

	for _, variant := range command.Variants {
		fields = append(fields, newBoolField(variant.Flag, variant.Usage+" (runs "+variant.Task+")", false))
	}

	for _, flag := range command.Flags {
		if flag.Kind == FlagBool {
			fields = append(fields, newBoolField(flag.Name, flag.Usage, boolDefault(flag.Default)))

			continue
		}

		field := newTextField(flag.Name, flag.Usage, flag.Required, flag.Kind)
		field.input.SetValue(flag.Default)
		field.secret(flag.Secret)
		fields = append(fields, field)
	}

	if command.Passthrough {
		fields = append(fields, newTextField(argsField, "extra arguments after --", false, FlagString))
	}

	preview := func(values map[string]string) (string, error) {
		inv, planErr := planInvocation(command, values, splitArgs(values[argsField]))
		if planErr != nil {
			return "", planErr
		}

		return inv.display(), nil
	}

	if len(fields) == 0 {
		return map[string]string{}, true, nil
	}

	return runForm(a.ui, strings.Join(command.Path, " "), fields, preview, command.Heavy)
}

// echo prints the command line that reproduces a picker run, so the flags can
// be learned and the run repeated without the picker.
func (a *App) echo(line string) {
	fmt.Fprintln(os.Stderr, a.ui.Muted(equivalentPrefix+line))
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

// intDefault parses a spec default, falling back to zero.
func intDefault(raw string) int {
	if raw == "" {
		return 0
	}

	value, err := strconv.Atoi(raw)
	if err != nil {
		return 0
	}

	return value
}

// boolDefault parses a spec default, falling back to false.
func boolDefault(raw string) bool {
	value, err := strconv.ParseBool(raw)
	if err != nil {
		return false
	}

	return value
}
