package vtcli

import (
	"fmt"
	"maps"
	"runtime"
	"slices"
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
	// goos decides which platform-limited commands are shown; recentPath is
	// where picks are remembered ("" disables it).
	goos       string
	recentPath string
	// dryRun prints the invocation a curated command resolves to instead of
	// running it (--dry-run).
	dryRun bool
}

// flagBinding registers one flag (a spec Flag or a Variant switch) and
// remembers where its parsed value lives.
type flagBinding struct {
	name  string
	kind  FlagKind
	text  *string
	whole *int
	truth *bool
}

// rootGroup is one section of the root help and menu, in display order.
type rootGroup struct {
	id      string
	title   string
	members []string
}

// rootGroups orders the first level: the domains people work in, the verbs
// that span every module, then the escape hatch. The members list is also the
// help order, so a new domain belongs in exactly one group here.
var rootGroups = []rootGroup{
	{
		id:    "domains",
		title: "Domains:",
		members: []string{
			"sim", "robot", "go", "fleet", "gen", "simgen",
			"frontend", "backend", "annotator", "hailo",
			"config", "models", "openapi", "proto", "docs", "docker",
		},
	},
	{id: "repo", title: "Across modules:", members: []string{"setup", "test", "lint", "clean"}},
	{id: "self", title: "vt and the shared code:", members: []string{"cli", "shared", "workflow"}},
	{id: "any", title: "Any task:", members: []string{catchAllName}},
}

// rootDomainShort gives each first-level namespace its one-line description.
// A namespace missing here renders with an empty description, so this map and
// rootGroups are updated together.
var rootDomainShort = map[string]string{
	"sim":       "Simulation: headless sim + RViz, Gazebo, sim tests",
	"robot":     "The robot, from the dev machine: deploy, runs, vision, ROS2",
	"go":        "Go module: build, deploy, hardware tests",
	"fleet":     "Boards over SSH and network",
	"gen":       "Generate tracks, scenarios, recordings and the sweep corpus",
	"frontend":  "The web frontend: dev server, build, lint",
	"backend":   "The Go telemetry backend: build, dev, sqlc, OpenAPI",
	"config":    "The shared config schemas: generate, verify, validate",
	"workflow":  "Multi-step pipelines that chain the domains",
	"cli":       "vt itself: build, run, test, lint, completions",
	"annotator": "The auto-annotator app: ML service, API, frontend",
	"hailo":     "The Hailo model toolchain: export, compile, evaluate",
	"simgen":    "The Go scenario generator: build, test, lint",
	"docs":      "Prose and diagrams: the drift checks and the renders",
	"shared":    "The shared Python platform code",
	"openapi":   "The aggregated OpenAPI contract",
	"proto":     "The shared proto contract (buf)",
	"docker":    "The repo's docker compose stack",
	"models":    "Tracked model versions: promote, deploy",
	"setup":     "Install and one-time setup",
	"lint":      "Lint every module",
	"clean":     "Clean generated data",
}

// segmentHelp gives the intermediate tree nodes a one-line description.
var segmentHelp = map[string]string{
	"build":      "Build",
	"hw":         "Cross-compile a hardware test for a Pi (does not run it)",
	"view":       "Watch a run in RViz",
	"parts":      "The two halves of view, for separate terminals",
	"vision":     "Detections from the Pi 5 camera",
	"pull":       "Pull from the Pi 5",
	"push":       "Push to the Pi 5",
	"bench-hud":  "Bench vision/HUD session",
	"setup":      "One-time network and SSH setup",
	"ethernet":   "Direct Ethernet link (Windows, admin)",
	"route":      "Persistent routes (Windows, admin)",
	"ssh-config": "~/.ssh/config entries",
}

// NewApp attaches the curated spec and the generated catch-all to root.
func NewApp(repoRoot string, root *cobra.Command, ui UI, spec []Command, tasks []TaskInfo) (*App, error) {
	app := &App{
		Root: root, repoRoot: repoRoot, spec: spec, tasks: tasks, ui: ui,
		goos: runtime.GOOS, recentPath: defaultRecentPath(),
	}
	if err := app.build(); err != nil {
		return nil, err
	}

	return app, nil
}

// build wires the curated leaves, the catch-all, and the bare-command banner
// onto the root, in rootGroups order rather than alphabetical.
func (a *App) build() error {
	// Group order is the point of rootGroups; cobra only exposes it as a
	// package toggle, and vt is the process's only command tree.
	cobra.EnableCommandSorting = false //nolint:reassign // cobra's sole ordering switch; see above

	for _, group := range rootGroups {
		a.Root.AddGroup(&cobra.Group{ID: group.id, Title: group.title})
	}

	for i := range a.spec {
		if err := a.addCurated(a.spec[i]); err != nil {
			return err
		}
	}

	a.Root.AddCommand(a.taskCommand())
	hideEmptyNamespaces(a.Root)
	a.assignGroups()

	a.Root.Long = "Command tree over the repository Taskfiles."
	a.Root.PersistentFlags().BoolVar(&a.dryRun, dryRunFlag, false,
		"print the task invocation a command resolves to, secrets masked, instead of running it")
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

// assignGroups files each first-level command under its root group.
func (a *App) assignGroups() {
	for _, group := range rootGroups {
		for _, member := range group.members {
			if child := findChild(a.Root, member); child != nil {
				child.GroupID = group.id
			}
		}
	}
}

// menuSections lists the visible first-level commands per root group, for
// the static home menu.
func (a *App) menuSections() []MenuSection {
	sections := make([]MenuSection, 0, len(rootGroups))

	for _, group := range rootGroups {
		section := MenuSection{Title: group.title}

		for _, member := range group.members {
			if child := findChild(a.Root, member); child != nil && !child.Hidden {
				section.Entries = append(section.Entries, MenuEntry{Name: child.Name(), Short: child.Short})
			}
		}

		if len(section.Entries) > 0 {
			sections = append(sections, section)
		}
	}

	return sections
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

// newLeaf builds the cobra command that runs one wrapped command. A command
// limited to other platforms is still built, hidden, and refuses to run with
// a message rather than an "unknown command".
func (a *App) newLeaf(command Command) (*cobra.Command, error) {
	bindings := make([]*flagBinding, 0, len(command.Flags)+len(command.Variants))
	for _, flag := range command.Flags {
		bindings = append(bindings, &flagBinding{name: flag.Name, kind: flag.Kind})
	}

	for _, variant := range command.Variants {
		bindings = append(bindings, &flagBinding{name: variant.Flag, kind: FlagBool})
	}

	leaf := &cobra.Command{
		Use:    useLine(command),
		Short:  command.Short,
		Args:   argsValidator(command),
		Hidden: !command.Available(a.goos),
		RunE: func(cmd *cobra.Command, raw []string) error {
			if !command.Available(a.goos) {
				return fmt.Errorf("`vt %s` runs on %s only; this is %s",
					strings.Join(command.Path, " "), strings.Join(command.Platforms, "/"), a.goos)
			}

			values, passthrough := cliValues(cmd, command, bindings, raw)

			return a.execute(cmd.Context(), command, values, passthrough)
		},
	}

	registerFlags(leaf, command, bindings)

	for _, flag := range command.Flags {
		if flag.Required {
			if err := leaf.MarkFlagRequired(flag.Name); err != nil {
				return nil, fmt.Errorf("mark %s required: %w", flag.Name, err)
			}
		}
	}

	return leaf, nil
}

// taskCommand builds the catch-all: any Task name, forwarded verbatim. `run`
// stays as an alias for the name it had before.
func (a *App) taskCommand() *cobra.Command {
	return &cobra.Command{
		Use:     catchAllName + " [<task> [VAR=value ...] [-- <args>]]",
		Aliases: []string{"run"},
		Short:   "Run any Task task, with or without typed flags",
		Long: "Escape hatch for tasks that have no typed flags. The name is validated against the real " +
			"inventory before running. With no task, lists every task with its description.",
		Args:               cobra.ArbitraryArgs,
		DisableFlagParsing: true,
		RunE: func(cmd *cobra.Command, raw []string) error {
			if len(raw) == 0 {
				return a.printTasks(cmd)
			}

			name := raw[0]
			if !KnownTask(a.tasks, name) {
				return fmt.Errorf("unknown task %q; try `vt %s` to list them", name, catchAllName)
			}

			a.remember(recentTaskPrefix + name)

			return runTask(cmd.Context(), a.repoRoot, name, raw[1:])
		},
	}
}

// printTasks lists the whole inventory with descriptions. It is what retired
// the hand-written `help` task, which drifted every time a task was added.
func (a *App) printTasks(cmd *cobra.Command) error {
	listing := fmt.Sprintf("%d tasks. Run one with `vt %s <task> [VAR=value ...]`.\n\n%s",
		len(a.tasks), catchAllName, a.ui.Menu(TaskEntries(a.tasks)))

	if _, err := fmt.Fprintln(cmd.OutOrStdout(), listing); err != nil {
		return fmt.Errorf("write task list: %w", err)
	}

	return nil
}

// registerFlags declares each binding on the leaf's flag set.
func registerFlags(leaf *cobra.Command, command Command, bindings []*flagBinding) {
	flags := leaf.Flags()
	usage, defaults := flagDocs(command)

	for _, binding := range bindings {
		switch binding.kind {
		case FlagInt:
			binding.whole = new(int)
			flags.IntVar(binding.whole, binding.name, intDefault(defaults[binding.name]), usage[binding.name])
		case FlagBool:
			binding.truth = new(bool)
			flags.BoolVar(binding.truth, binding.name, boolDefault(defaults[binding.name]), usage[binding.name])
		default:
			binding.text = new(string)
			flags.StringVar(binding.text, binding.name, defaults[binding.name], usage[binding.name])
		}
	}
}

// flagDocs collects the help text and displayed default of every flag and
// variant switch of command, by name.
func flagDocs(command Command) (usage, defaults map[string]string) {
	usage = make(map[string]string)
	defaults = make(map[string]string)

	for _, flag := range command.Flags {
		usage[flag.Name], defaults[flag.Name] = flag.Usage, flag.Default
	}

	for _, variant := range command.Variants {
		usage[variant.Flag] = variant.Usage + " (runs " + variant.Task + ")"
	}

	return usage, defaults
}

// cliValues turns a parsed command line into the values planInvocation reads:
// changed flags only, positionals by name, and whatever follows `--`.
func cliValues(
	cmd *cobra.Command,
	command Command,
	bindings []*flagBinding,
	raw []string,
) (values map[string]string, passthrough []string) {
	values = make(map[string]string, len(bindings)+len(command.Args))

	for _, binding := range bindings {
		if cmd.Flags().Changed(binding.name) {
			values[binding.name] = binding.value()
		}
	}

	positional := raw
	if dash := cmd.ArgsLenAtDash(); dash >= 0 {
		positional, passthrough = raw[:dash], raw[dash:]
	} else if len(raw) > len(command.Args) {
		positional, passthrough = raw[:len(command.Args)], raw[len(command.Args):]
	}

	for i, arg := range command.Args {
		if i < len(positional) {
			values[arg.Name] = positional[i]
		}
	}

	return values, passthrough
}

// value formats the flag's current value as Task would receive it.
func (b *flagBinding) value() string {
	switch b.kind {
	case FlagInt:
		return strconv.Itoa(*b.whole)
	case FlagBool:
		return strconv.FormatBool(*b.truth)
	default:
		return *b.text
	}
}

// argsValidator picks the positional-argument contract for a leaf: required
// arguments must be there, optional ones may be, and only a passthrough
// command takes more.
func argsValidator(command Command) cobra.PositionalArgs {
	required := 0
	for _, arg := range command.Args {
		if arg.Required {
			required++
		}
	}

	if command.Passthrough {
		return cobra.MinimumNArgs(required)
	}

	return cobra.RangeArgs(required, len(command.Args))
}

// useLine renders the `Use` string for a leaf, including placeholders.
func useLine(command Command) string {
	parts := []string{command.Path[len(command.Path)-1]}

	for _, arg := range command.Args {
		name := arg.Name
		if arg.Tasks != nil {
			name = strings.Join(slices.Sorted(maps.Keys(arg.Tasks)), "|")
		}

		if arg.Required {
			parts = append(parts, "<"+name+">")
		} else {
			parts = append(parts, "["+name+"]")
		}
	}

	if command.Passthrough {
		parts = append(parts, "[-- <args>]")
	}

	return strings.Join(parts, " ")
}

// hideEmptyNamespaces hides every intermediate command whose children are all
// hidden, so a platform-limited branch disappears as a whole.
func hideEmptyNamespaces(parent *cobra.Command) bool {
	children := parent.Commands()
	if len(children) == 0 {
		return parent.Hidden
	}

	allHidden := true
	for _, child := range children {
		if !hideEmptyNamespaces(child) {
			allHidden = false
		}
	}

	if allHidden && parent.RunE == nil && parent.Run == nil {
		parent.Hidden = true
	}

	return parent.Hidden
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
