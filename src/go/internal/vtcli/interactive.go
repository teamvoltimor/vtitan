package vtcli

import (
	"context"
	"fmt"
	"os"
	"strconv"
	"strings"

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

// pickAndRun opens the interactive session: picker, form and run pane in
// one program that stays open between runs.
func (a *App) pickAndRun(cmd *cobra.Command) error {
	out, isFile := cmd.OutOrStdout().(*os.File)
	if !isFile {
		out = os.Stdout
	}

	return a.runSession(cmd.Context(), out)
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

// formFor builds the argument form for a picked command or task, or returns
// nil when there is nothing to ask. The preview is planInvocation's own
// rendering, so the confirmation shows exactly what will run.
func (a *App) formFor(pending *pendingRun) *formModel {
	if pending.command == nil {
		field := newTextField(argsField, "VAR=value and flags, verbatim (optional)", false, FlagString)
		preview := func(values map[string]string) (string, error) {
			_, _, _, display, err := a.resolve(&pendingRun{task: pending.task, values: values})

			return display, err
		}

		return newFormModel(a.ui, catchAllName+" "+pending.task, []*formField{field}, preview, false)
	}

	command := *pending.command

	fields := commandFields(command)
	if len(fields) == 0 {
		return nil
	}

	preview := func(values map[string]string) (string, error) {
		inv, planErr := planInvocation(command, values, splitArgs(values[argsField]))
		if planErr != nil {
			return "", planErr
		}

		return inv.display(), nil
	}

	return newFormModel(a.ui, strings.Join(command.Path, " "), fields, preview, command.Heavy)
}

// commandFields lists the form fields of a command: its positionals, its
// variant switches, its flags, and the passthrough line.
func commandFields(command Command) []*formField {
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

	return fields
}

// resolve turns a pending run and its values into what runs (the task name
// and its argv), the equivalent vt line, and the display line with secrets
// masked. It is planInvocation for a command, and the verbatim arguments for
// a task picked from the full inventory.
func (a *App) resolve(pending *pendingRun) (name string, args []string, line, display string, err error) {
	if pending.command == nil {
		args = splitArgs(pending.values[argsField])
		line = strings.Join(append([]string{"vt", catchAllName, pending.task}, args...), " ")

		return pending.task, args, line, strings.Join(append([]string{"task", pending.task}, args...), " "), nil
	}

	command := *pending.command
	passthrough := splitArgs(pending.values[argsField])
	line = vtLine(command, pending.values, passthrough)

	inv, err := planInvocation(command, pending.values, passthrough)
	if err != nil {
		return "", nil, line, "", err
	}

	return inv.task, inv.argv(), line, inv.display(), nil
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
