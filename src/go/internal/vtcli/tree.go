package vtcli

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/spf13/cobra"
)

// App is the assembled vt command tree plus the state its leaves need to run.
type App struct {
	Root *cobra.Command

	repoRoot string
	spec     []Command
	tasks    []TaskInfo
	ui       UI
}

// flagBinding registers one spec Flag and remembers where its parsed value
// lives. A value is only forwarded to Task when the user changed it.
type flagBinding struct {
	spec  Flag
	text  *string
	whole *int
	truth *bool
}

// rootDomainShort gives each first-level domain its one-line description.
var rootDomainShort = map[string]string{
	"sim":   "Simulation (Gazebo + navigation)",
	"robot": "Robot from the dev machine: deploy, sync runs, vision, ROS2",
	"init":  "One-time setup",
	"lint":  "Lint every module",
	"clean": "Clean generated data",
	"rpi":   "Raspberry Pi (local config)",
	"fleet": "Boards over SSH and network",
	"go":    "Go module",
	"py":    "Python module",
	"docs":  "Documentation",
	"apps":  "Applications",
	"ml":    "ML and models",
	"infra": "Infrastructure",
	"run":   "Escape hatch: any Task task",
}

// segmentHelp gives the intermediate tree nodes a one-line description.
var segmentHelp = map[string]string{
	"build":       "Build",
	"test":        "Tests",
	"hw":          "Hardware",
	"navigate":    "Navigation",
	"visualize":   "Visualize",
	"provision":   "Provision",
	"set-wifi":    "Set WiFi",
	"audit":       "Audit",
	"static":      "CGO off (pure-Go)",
	"capture":     "CGO on (gocv/OpenCV)",
	"interactive": "Interactive tests",
	"all":         "All",
	"run":         "Run",
	"stop":        "Stop",
	"deploy":      "Deploy",
	"ping":        "Ping",
	"ssh":         "SSH",
	"ssh-config":  "~/.ssh/config entries",
	"ethernet":    "Direct Ethernet link (Windows, admin)",
	"route":       "Persistent routes (Windows, admin)",
	"pull":        "Pull from the Pi 5",
	"push":        "Push to the Pi 5",
	"bench-hud":   "Bench vision/HUD session",
	"lint":        "Lint",
}

// NewApp attaches the curated spec and the generated catch-all to root.
func NewApp(repoRoot string, root *cobra.Command, ui UI, spec []Command, tasks []TaskInfo) (*App, error) {
	app := &App{Root: root, repoRoot: repoRoot, spec: spec, tasks: tasks, ui: ui}
	if err := app.build(); err != nil {
		return nil, err
	}

	return app, nil
}

// build wires the curated leaves, the catch-all, and the bare-command banner
// onto the root.
func (a *App) build() error {
	for _, command := range a.spec {
		if err := a.addCurated(command); err != nil {
			return err
		}
	}

	a.Root.AddCommand(a.runCommand())
	a.Root.Long = "Command tree over the repository Taskfiles."
	a.Root.SilenceUsage = true
	a.Root.SilenceErrors = true
	a.Root.RunE = func(cmd *cobra.Command, args []string) error {
		if len(args) > 0 {
			return fmt.Errorf("unknown command %q; try `vt --help`", args[0])
		}

		return a.home(cmd)
	}

	return nil
}

// menuEntries lists the first-level commands for the home menu, skipping
// cobra's own completion/help plumbing.
func (a *App) menuEntries() []MenuEntry {
	entries := make([]MenuEntry, 0, len(a.Root.Commands()))
	for _, child := range a.Root.Commands() {
		if child.Name() == "completion" || child.Name() == "help" {
			continue
		}

		entries = append(entries, MenuEntry{Name: child.Name(), Short: child.Short})
	}

	return entries
}

// addCurated creates the parent path for command and attaches its leaf.
func (a *App) addCurated(command Command) error {
	parent := a.ensurePath(command.Path[:len(command.Path)-1])

	leaf, err := a.newLeaf(command)
	if err != nil {
		return err
	}

	parent.AddCommand(leaf)

	return nil
}

// ensurePath descends path, creating intermediate commands as needed.
func (a *App) ensurePath(path []string) *cobra.Command {
	current := a.Root

	for _, segment := range path {
		child := findChild(current, segment)
		if child == nil {
			child = &cobra.Command{Use: segment}
			if current == a.Root {
				child.Short = rootDomainShort[segment]
			} else {
				child.Short = segmentHelp[segment]
			}

			current.AddCommand(child)
		}

		current = child
	}

	return current
}

// newLeaf builds the cobra command that runs one wrapped Task.
func (a *App) newLeaf(command Command) (*cobra.Command, error) {
	bindings := make([]*flagBinding, len(command.Flags))
	for i := range command.Flags {
		bindings[i] = &flagBinding{spec: command.Flags[i]}
	}

	positional := append([]Arg{}, command.Args...)

	leaf := &cobra.Command{
		Use:   useLine(command),
		Short: command.Short,
		Args:  argsValidator(command),
		RunE: func(cmd *cobra.Command, raw []string) error {
			extra := append([]string{}, collectVars(cmd, bindings, positional, raw)...)
			if command.Passthrough && len(raw) > 0 {
				extra = append(extra, "--")
				extra = append(extra, raw...)
			}

			return runTask(cmd.Context(), a.repoRoot, command.Task, extra)
		},
	}

	registerFlags(leaf, bindings)

	for _, binding := range bindings {
		if binding.spec.Required {
			if err := leaf.MarkFlagRequired(binding.spec.Name); err != nil {
				return nil, fmt.Errorf("mark %s required: %w", binding.spec.Name, err)
			}
		}
	}

	return leaf, nil
}

// runCommand builds the catch-all: any Task name, forwarded verbatim.
func (a *App) runCommand() *cobra.Command {
	return &cobra.Command{
		Use:   "run [<task> [VAR=value ...] [-- <args>]]",
		Short: "Run any Task task, with or without typed flags",
		Long: "Escape hatch for tasks that have no typed flags yet. The name is validated against the real " +
			"inventory before running. With no task, lists every task with its description.",
		Args:               cobra.ArbitraryArgs,
		DisableFlagParsing: true,
		RunE: func(cmd *cobra.Command, raw []string) error {
			if len(raw) == 0 {
				return a.printTasks(cmd)
			}

			name := raw[0]
			if !KnownTask(a.tasks, name) {
				return fmt.Errorf("unknown task %q; try `vt --help` or `task --list-all`", name)
			}

			return runTask(cmd.Context(), a.repoRoot, name, raw[1:])
		},
	}
}

// printTasks lists the whole inventory with descriptions. It is what retired
// the hand-written `help` task, which drifted every time a task was added.
func (a *App) printTasks(cmd *cobra.Command) error {
	listing := fmt.Sprintf("%d tasks. Run one with `vt run <task> [VAR=value ...]`.\n\n%s",
		len(a.tasks), a.ui.Menu(TaskEntries(a.tasks)))

	if _, err := fmt.Fprintln(cmd.OutOrStdout(), listing); err != nil {
		return fmt.Errorf("write task list: %w", err)
	}

	return nil
}

// registerFlags declares each binding on the leaf's flag set.
func registerFlags(leaf *cobra.Command, bindings []*flagBinding) {
	flags := leaf.Flags()

	for _, binding := range bindings {
		switch binding.spec.Kind {
		case FlagInt:
			binding.whole = new(int)
			flags.IntVar(binding.whole, binding.spec.Name, intDefault(binding.spec.Default), binding.spec.Usage)
		case FlagBool:
			binding.truth = new(bool)
			flags.BoolVar(binding.truth, binding.spec.Name, boolDefault(binding.spec.Default), binding.spec.Usage)
		default:
			binding.text = new(string)
			flags.StringVar(binding.text, binding.spec.Name, binding.spec.Default, binding.spec.Usage)
		}
	}
}

// collectVars turns the changed flags and supplied positionals into the
// KEY=value pairs Task expects.
func collectVars(
	cmd *cobra.Command,
	bindings []*flagBinding,
	argSpecs []Arg,
	raw []string,
) []string {
	vars := make([]string, 0, len(bindings)+len(argSpecs))

	for _, binding := range bindings {
		if !cmd.Flags().Changed(binding.spec.Name) {
			continue
		}

		vars = append(vars, binding.spec.Var+"="+binding.value())
	}

	for i, spec := range argSpecs {
		vars = append(vars, spec.Var+"="+raw[i])
	}

	return vars
}

// value formats the flag's current value as Task would receive it.
func (b *flagBinding) value() string {
	switch b.spec.Kind {
	case FlagInt:
		return strconv.Itoa(*b.whole)
	case FlagBool:
		return strconv.FormatBool(*b.truth)
	default:
		return *b.text
	}
}

// argsValidator picks the positional-argument contract for a leaf.
func argsValidator(command Command) cobra.PositionalArgs {
	switch {
	case len(command.Args) > 0:
		return cobra.ExactArgs(len(command.Args))
	case command.Passthrough:
		return cobra.ArbitraryArgs
	default:
		return cobra.NoArgs
	}
}

// useLine renders the `Use` string for a leaf, including placeholders.
func useLine(command Command) string {
	parts := []string{command.Path[len(command.Path)-1]}

	for _, arg := range command.Args {
		if arg.Required {
			parts = append(parts, "<"+arg.Name+">")
		} else {
			parts = append(parts, "["+arg.Name+"]")
		}
	}

	if command.Passthrough {
		parts = append(parts, "[-- <args>]")
	}

	return strings.Join(parts, " ")
}

// findChild returns the existing subcommand named segment, if any.
func findChild(parent *cobra.Command, segment string) *cobra.Command {
	for _, child := range parent.Commands() {
		if child.Name() == segment {
			return child
		}
	}

	return nil
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
