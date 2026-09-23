package vtcli

import (
	"maps"
	"slices"
)

// FlagKind is the value type a typed flag carries into its Task variable.
type FlagKind string

// Flag maps one CLI flag to one Task variable. Default is display-only: it is
// shown in --help and prefilled in the interactive form, but never forwarded,
// so Task's own default stays authoritative and a stale copy cannot change
// behaviour.
type Flag struct {
	Name     string
	Var      string
	Kind     FlagKind
	Default  string
	Usage    string
	Required bool
	// Secret masks the value wherever vt echoes a command line back, and
	// hides it while typing in the form.
	Secret bool
}

// Arg maps one positional CLI argument to one Task variable, or, when Tasks
// is set, picks which task runs (`vt fleet audit zero`) and is not forwarded.
type Arg struct {
	Name     string
	Var      string
	Usage    string
	Required bool
	Tasks    map[string]string
}

// Variant is a boolean flag that swaps the task a command runs, for tasks that
// are one operation with a modifier (`vt lint --fix` runs lint:fix). At most
// one variant of a command may be set.
type Variant struct {
	Flag  string
	Task  string
	Usage string
}

// Command is one wrapped leaf: a path in the CLI tree that runs one Task.
type Command struct {
	Path        []string
	Task        string
	Short       string
	Flags       []Flag
	Args        []Arg
	Passthrough bool
	// Heavy marks a task that starts long-running processes (a simulator, a
	// service): the interactive form warns before running it.
	Heavy bool
	// Platforms limits the command to these GOOS values; empty means every
	// platform. Elsewhere it is hidden from help, menus and the picker.
	Platforms []string
	Variants  []Variant
	// Terminal hands the task the whole terminal in the interactive session
	// instead of showing its output in the run pane, for a task that draws
	// its own screen or reads raw keys (an SSH shell, vt itself).
	Terminal bool
}

// Domain groups a first-level CLI namespace and the Task-name prefix it owns.
// Only curated domains are checked by the anti-drift test.
type Domain struct {
	ID         string
	Title      string
	TaskPrefix string
}

const (
	// FlagString forwards the flag value verbatim.
	FlagString FlagKind = "string"
	// FlagInt forwards a base-10 integer.
	FlagInt FlagKind = "int"
	// FlagBool forwards "true" or "false".
	FlagBool FlagKind = "bool"
)

// Flag names, positional names and reasons reused across the domain tables.
const (
	flagOut     = "out"
	flagPkg     = "pkg"
	flagSSHHost = "ssh-host"
	flagSDF     = "sdf"
	flagPI5Host = "pi5-host"
	flagSeconds = "seconds"
	argHost     = "host"

	defaultSDF     = "worlds/wro_track_2026.sdf"
	defaultPI5Host = "rpi-5-local"

	// reasonOnBoard marks tasks that run on a Pi. vt is a dev-machine tool
	// (ADR 0096) and the boards keep using task directly.
	reasonOnBoard = "runs on a Pi; vt is dev-machine only, use task there"
)

// profileFlag selects the hardware profile a sim task models. Task's default
// lives in other/tasks/platform.yml.
var profileFlag = Flag{
	Name:    "profile",
	Var:     "PROFILE",
	Default: "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm",
	Usage:   "hardware profile (servo,motor)",
}

// curatedDomains are the domains under the anti-drift contract. Everything
// matching their TaskPrefix, or equal to it without the colon, must be either
// in the spec or excluded by name. A domain is promoted here once its
// exclusion list is closed.
var curatedDomains = []Domain{
	{ID: "go", Title: "Go module", TaskPrefix: "go:"},
	{ID: "fleet", Title: "boards over SSH", TaskPrefix: "windows:"},
	{ID: "sim", Title: "simulation", TaskPrefix: "sim:"},
	{ID: "gen", Title: "generators", TaskPrefix: "gen:"},
	{ID: "frontend", Title: "web frontend", TaskPrefix: "frontend:"},
	{ID: "backend", Title: "telemetry backend", TaskPrefix: "backend:"},
	{ID: "config", Title: "config schemas", TaskPrefix: "config:"},
	{ID: "workflow", Title: "pipelines", TaskPrefix: "workflow:"},
	{ID: "cli", Title: "the vt CLI itself", TaskPrefix: "cli:"},
	{ID: "annotator", Title: "auto-annotator app", TaskPrefix: "auto-annotator:"},
	{ID: "hailo", Title: "Hailo model toolchain", TaskPrefix: "hailo:"},
	{ID: "simgen", Title: "scenario generator", TaskPrefix: "simgen:"},
	{ID: "docs", Title: "prose and diagrams", TaskPrefix: "docs:"},
	{ID: "shared", Title: "shared Python", TaskPrefix: "shared:"},
	{ID: "openapi", Title: "OpenAPI contract", TaskPrefix: "openapi:"},
	{ID: "proto", Title: "proto contract", TaskPrefix: "proto:"},
	{ID: "docker", Title: "docker compose", TaskPrefix: "docker:"},
	{ID: "models", Title: "tracked models", TaskPrefix: "models:"},
	{ID: "robot", Title: "Python/ROS2 runtime", TaskPrefix: "robot:"},
	{ID: "rpi", Title: "on-board Pi tasks", TaskPrefix: "rpi:"},
	{ID: "install", Title: "umbrella install", TaskPrefix: "install:"},
	{ID: "init", Title: "umbrella init", TaskPrefix: "init:"},
	{ID: "test", Title: "umbrella test", TaskPrefix: "test:"},
	{ID: "lint", Title: "umbrella lint", TaskPrefix: "lint:"},
	{ID: "clean", Title: "umbrella clean", TaskPrefix: "clean:"},
}

// curatedSpec is the single declarative table the tree is built from, one
// table per domain, concatenated in rootGroups order. One entry per wrapped
// command; nothing here duplicates how a task runs. Within a table a command
// precedes its children, and table order is help and picker order.
var curatedSpec = slices.Concat(
	simSpec, robotSpec, goSpec, fleetSpec, genSpec,
	frontendSpec, backendSpec, autoAnnotatorSpec, hailoSpec,
	configSpec, contractsSpec, modelsSpec,
	umbrellaSpec, workflowSpec, dockerSpec, docsSpec,
	cliSpec, simgenSpec, sharedSpec)

// exclusions records the tasks deliberately left out of the typed tree and
// why. Adding a task to a curated domain without deciding anything here fails
// the build (see spec_test.go).
var exclusions = mergeExclusions(
	goExclusions, simExclusions, robotExclusions, rpiExclusions,
	autoAnnotatorExclusions, hailoExclusions)

// mergeExclusions joins the per-domain exclusion tables.
func mergeExclusions(tables ...map[string]string) map[string]string {
	merged := make(map[string]string)
	for _, table := range tables {
		maps.Copy(merged, table)
	}

	return merged
}

// CuratedSpec returns the declarative command table.
func CuratedSpec() []Command {
	return curatedSpec
}

// CuratedDomains returns the domains under the anti-drift contract.
func CuratedDomains() []Domain {
	return curatedDomains
}

// Exclusions returns the explicit per-task exclusions for curated domains.
func Exclusions() map[string]string {
	return exclusions
}

// Tasks lists every task the command can run: its own, its variants', and
// its task-picking argument's.
func (c Command) Tasks() []string {
	var tasks []string
	if c.Task != "" {
		tasks = append(tasks, c.Task)
	}

	for _, variant := range c.Variants {
		tasks = append(tasks, variant.Task)
	}

	for _, arg := range c.Args {
		for _, value := range slices.Sorted(maps.Keys(arg.Tasks)) {
			tasks = append(tasks, arg.Tasks[value])
		}
	}

	return tasks
}

// Available reports whether the command runs on goos.
func (c Command) Available(goos string) bool {
	return len(c.Platforms) == 0 || slices.Contains(c.Platforms, goos)
}

// fixVariant is the --fix switch of a lint command: the same linter, fixing.
func fixVariant(task string) Variant {
	return Variant{Flag: "fix", Task: task, Usage: "fix what can be fixed"}
}
