package vtcli

import (
	"errors"
	"fmt"
	"maps"
	"regexp"
	"slices"
	"strconv"
	"strings"
)

// invocation is what one run of a command resolves to: the task, the Task
// vars, and what goes after `--`. The CLI, the form's preview and the echoed
// equivalent all come from here, so they cannot disagree.
type invocation struct {
	task        string
	vars        []taskVar
	passthrough []string
}

// taskVar is one KEY=value forwarded to Task.
type taskVar struct {
	name   string
	value  string
	secret bool
}

// errNoTask is returned when a command's task depends on an argument that was
// not given.
var errNoTask = errors.New("no task selected")

// shellSafe matches values that need no quoting when echoed as a command line.
var shellSafe = regexp.MustCompile(`^[A-Za-z0-9_./:=@,+%^-]+$`)

// planInvocation resolves command against values, keyed by argument, flag or
// variant name. A flag whose value is empty or equal to its declared default
// is not forwarded, so Task's own default stays authoritative.
func planInvocation(command Command, values map[string]string, passthrough []string) (invocation, error) {
	inv := invocation{task: command.Task}

	for _, arg := range command.Args {
		value := strings.TrimSpace(values[arg.Name])
		if value == "" {
			if arg.Required {
				return invocation{}, fmt.Errorf("missing argument %q", arg.Name)
			}

			continue
		}

		if arg.Tasks == nil {
			inv.vars = append(inv.vars, taskVar{name: arg.Var, value: value})

			continue
		}

		task, known := arg.Tasks[value]
		if !known {
			return invocation{}, fmt.Errorf("%s must be one of %s, not %q",
				arg.Name, strings.Join(slices.Sorted(maps.Keys(arg.Tasks)), "|"), value)
		}

		inv.task = task
	}

	chosen, err := chosenVariant(command, values)
	if err != nil {
		return invocation{}, err
	}

	if chosen != nil {
		inv.task = chosen.Task
	}

	if inv.task == "" {
		return invocation{}, errNoTask
	}

	for _, flag := range command.Flags {
		value, set := flagValue(flag, values[flag.Name])
		if set {
			inv.vars = append(inv.vars, taskVar{name: flag.Var, value: value, secret: flag.Secret})
		}
	}

	if command.Passthrough {
		inv.passthrough = passthrough
	}

	return inv, nil
}

// chosenVariant returns the variant switched on, if any; two at once is an
// error, since each names a different task.
func chosenVariant(command Command, values map[string]string) (*Variant, error) {
	var chosen *Variant

	for i := range command.Variants {
		if !isOn(values[command.Variants[i].Flag]) {
			continue
		}

		if chosen != nil {
			return nil, fmt.Errorf("--%s and --%s cannot be used together", chosen.Flag, command.Variants[i].Flag)
		}

		chosen = &command.Variants[i]
	}

	return chosen, nil
}

// flagValue normalises one flag's raw value and reports whether it differs
// from the declared default and so should be forwarded.
func flagValue(flag Flag, raw string) (string, bool) {
	value := strings.TrimSpace(raw)

	if flag.Kind == FlagBool {
		if value == "" {
			return "", false
		}

		on, err := strconv.ParseBool(value)
		if err != nil || on == boolDefault(flag.Default) {
			return "", false
		}

		return strconv.FormatBool(on), true
	}

	if value == "" || value == strings.TrimSpace(flag.Default) {
		return "", false
	}

	return value, true
}

// argv is the argument list after the task name, as runTask passes it on.
func (inv invocation) argv() []string {
	args := make([]string, 0, len(inv.vars)+len(inv.passthrough)+1)
	for _, v := range inv.vars {
		args = append(args, v.name+"="+v.value)
	}

	if len(inv.passthrough) > 0 {
		args = append(args, "--")
		args = append(args, inv.passthrough...)
	}

	return args
}

// display renders the invocation as a copyable `task` line, secrets masked.
func (inv invocation) display() string {
	parts := []string{"task", inv.task}
	for _, v := range inv.vars {
		parts = append(parts, v.name+"="+shellQuote(masked(v.value, v.secret)))
	}

	if len(inv.passthrough) > 0 {
		parts = append(parts, "--")
		for _, arg := range inv.passthrough {
			parts = append(parts, shellQuote(arg))
		}
	}

	return strings.Join(parts, " ")
}

// vtLine renders the `vt` command that reproduces values without the picker,
// secrets masked, so the flags can be learned and the run repeated.
func vtLine(command Command, values map[string]string, passthrough []string) string {
	parts := append([]string{"vt"}, command.Path...)

	for _, arg := range command.Args {
		if value := strings.TrimSpace(values[arg.Name]); value != "" {
			parts = append(parts, shellQuote(value))
		}
	}

	for _, variant := range command.Variants {
		if isOn(values[variant.Flag]) {
			parts = append(parts, "--"+variant.Flag)
		}
	}

	for _, flag := range command.Flags {
		value, set := flagValue(flag, values[flag.Name])
		if !set {
			continue
		}

		if flag.Kind == FlagBool {
			parts = append(parts, "--"+flag.Name+"="+value)

			continue
		}

		parts = append(parts, "--"+flag.Name, shellQuote(masked(value, flag.Secret)))
	}

	if command.Passthrough && len(passthrough) > 0 {
		parts = append(parts, "--")
		for _, arg := range passthrough {
			parts = append(parts, shellQuote(arg))
		}
	}

	return strings.Join(parts, " ")
}

// masked hides a secret's value in anything vt prints.
func masked(value string, secret bool) string {
	if secret {
		return secretMask
	}

	return value
}

// shellQuote single-quotes value for display when it is not plainly safe.
func shellQuote(value string) string {
	if value != "" && shellSafe.MatchString(value) {
		return value
	}

	return "'" + strings.ReplaceAll(value, "'", `'\''`) + "'"
}

// isOn reports whether a switch's raw value is a true boolean.
func isOn(raw string) bool {
	on, err := strconv.ParseBool(raw)

	return err == nil && on
}
