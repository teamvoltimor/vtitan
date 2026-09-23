package vtcli

import (
	"maps"
	"slices"
)

// FlagKind is the value type a typed flag carries into its Task variable.
type FlagKind int

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

// CommandGroup is where a command sits in its level's section order: what you
// run, the checks that gate it, the setup it needs, and the cleanup after it.
type CommandGroup int

// Platform is a GOOS a command is limited to.
type Platform int

// Command is one wrapped leaf: a path in the CLI tree that runs one Task.
type Command struct {
	Path  []string
	Task  string
	Short string
	// Group places the command in its level's section, and is required: the
	// picker draws a header per group, and a check rejects an unclassified
	// command rather than letting it fall into a default.
	Group       CommandGroup
	Flags       []Flag
	Args        []Arg
	Passthrough bool
	// Heavy marks a task that starts long-running processes (a simulator, a
	// service): the interactive form warns before running it.
	Heavy bool
	// Platforms limits the command to these GOOS values; empty means every
	// platform. Elsewhere it is hidden from help, menus and the picker.
	Platforms []Platform
	Variants  []Variant
	// Terminal hands the task the whole terminal in the interactive session
	// instead of showing its output in the run pane, for a task that draws
	// its own screen or reads raw keys (an SSH shell, vt itself).
	Terminal bool
}

// DomainID identifies a curated domain.
type DomainID int

// Domain groups a first-level CLI namespace and the Task-name prefix it owns.
// Only curated domains are checked by the anti-drift test.
type Domain struct {
	ID         DomainID
	Title      string
	TaskPrefix string
}

const (
	// FlagString forwards the flag value verbatim; it is the zero value, so an
	// unset Kind is a string flag.
	FlagString FlagKind = iota
	// FlagInt forwards a base-10 integer.
	FlagInt
	// FlagBool forwards "true" or "false".
	FlagBool
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

	// trainingDataDefault is the display-only default for OUTPUT_DIR, shared by
	// the gen:record:* and sim:analyze tasks. Task anchors its real default to
	// the repo root ({{.ROOT_DIR}}/src/go/training_data) because those tasks run
	// from other/apps/gazebo/runtime, where a relative default would resolve
	// instead. It is worded repo-relative for readability and never forwarded.
	trainingDataDefault = "<repo>/src/go/training_data"

	// reasonOnBoard marks tasks that run on a Pi. vt is a dev-machine tool
	// (ADR 0096) and the boards keep using task directly.
	reasonOnBoard = "runs on a Pi; vt is dev-machine only, use task there"
)

// The command groups, in the order a level shows them: what you run, the
// quality gates, the setup it needs, and the cleanup after it (ADR 0096).
const (
	// groupUnset is the zero value: a command that declares no group, which
	// TestCommandsGrouped rejects.
	groupUnset CommandGroup = iota
	groupRun
	groupCheck
	groupSetup
	groupClean
)

// The platforms a command can be limited to, named by GOOS.
const (
	platformLinux Platform = iota
	platformWindows
	platformDarwin
)

// The curated domain identifiers.
const (
	// domainUnset is the zero value: a task in no curated domain.
	domainUnset DomainID = iota
	domainGo
	domainFleet
	domainSim
	domainGen
	domainFrontend
	domainBackend
	domainConfig
	domainWorkflow
	domainCLI
	domainAnnotator
	domainHailo
	domainSimgen
	domainDocs
	domainShared
	domainOpenAPI
	domainProto
	domainDocker
	domainModels
	domainRobot
	domainRPI
	domainInstall
	domainInit
	domainTest
	domainLint
	domainClean
)

// commandGroupOrder is how a level's sections are ordered.
var commandGroupOrder = []CommandGroup{groupRun, groupCheck, groupSetup, groupClean}

// commandGroupTitles labels each section header.
var commandGroupTitles = map[CommandGroup]string{
	groupRun:   "Run",
	groupCheck: "Check",
	groupSetup: "Setup",
	groupClean: "Clean",
}

// profileFlag selects the hardware profile a sim task models. Task's default
// lives in the root Taskfile.yml.
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
	{ID: domainGo, Title: "Go module", TaskPrefix: "go:"},
	{ID: domainFleet, Title: "boards over SSH", TaskPrefix: "windows:"},
	{ID: domainSim, Title: "simulation", TaskPrefix: "sim:"},
	{ID: domainGen, Title: "generators", TaskPrefix: "gen:"},
	{ID: domainFrontend, Title: "web frontend", TaskPrefix: "frontend:"},
	{ID: domainBackend, Title: "telemetry backend", TaskPrefix: "backend:"},
	{ID: domainConfig, Title: "config schemas", TaskPrefix: "config:"},
	{ID: domainWorkflow, Title: "pipelines", TaskPrefix: "workflow:"},
	{ID: domainCLI, Title: "the vt CLI itself", TaskPrefix: "cli:"},
	{ID: domainAnnotator, Title: "auto-annotator app", TaskPrefix: "auto-annotator:"},
	{ID: domainHailo, Title: "Hailo model toolchain", TaskPrefix: "hailo:"},
	{ID: domainSimgen, Title: "scenario generator", TaskPrefix: "simgen:"},
	{ID: domainDocs, Title: "prose and diagrams", TaskPrefix: "docs:"},
	{ID: domainShared, Title: "shared Python", TaskPrefix: "shared:"},
	{ID: domainOpenAPI, Title: "OpenAPI contract", TaskPrefix: "openapi:"},
	{ID: domainProto, Title: "proto contract", TaskPrefix: "proto:"},
	{ID: domainDocker, Title: "docker compose", TaskPrefix: "docker:"},
	{ID: domainModels, Title: "tracked models", TaskPrefix: "models:"},
	{ID: domainRobot, Title: "Python/ROS2 runtime", TaskPrefix: "robot:"},
	{ID: domainRPI, Title: "on-board Pi tasks", TaskPrefix: "rpi:"},
	{ID: domainInstall, Title: "umbrella install", TaskPrefix: "install:"},
	{ID: domainInit, Title: "umbrella init", TaskPrefix: "init:"},
	{ID: domainTest, Title: "umbrella test", TaskPrefix: "test:"},
	{ID: domainLint, Title: "umbrella lint", TaskPrefix: "lint:"},
	{ID: domainClean, Title: "umbrella clean", TaskPrefix: "clean:"},
}

// curatedSpec is the single declarative table the tree is built from, one
// table per domain, concatenated in rootGroups order. One entry per wrapped
// command; nothing here duplicates how a task runs. Within a table a command
// precedes its children, and table order is help and picker order.
var curatedSpec = slices.Concat(
	simSpec, robotSpec, goSpec, fleetSpec, genSpec,
	frontendSpec, backendSpec, autoAnnotatorSpec, autoAnnotatorToolsSpec,
	hailoSpec, hailoToolsSpec,
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
	if len(c.Platforms) == 0 {
		return true
	}

	platform, ok := platformOf(goos)

	return ok && slices.Contains(c.Platforms, platform)
}

// String implements fmt.Stringer.
func (k FlagKind) String() string {
	switch k {
	case FlagString:
		return "string"
	case FlagInt:
		return "int"
	case FlagBool:
		return "bool"
	default:
		return "unknown"
	}
}

// String implements fmt.Stringer.
func (g CommandGroup) String() string {
	switch g {
	case groupRun:
		return "run"
	case groupCheck:
		return "check"
	case groupSetup:
		return "setup"
	case groupClean:
		return "clean"
	default:
		return "unset"
	}
}

// String implements fmt.Stringer, returning the GOOS name.
func (p Platform) String() string {
	switch p {
	case platformLinux:
		return "linux"
	case platformWindows:
		return "windows"
	case platformDarwin:
		return "darwin"
	default:
		return "unknown"
	}
}

// platformOf maps a GOOS name to its Platform.
func platformOf(goos string) (Platform, bool) {
	switch goos {
	case "linux":
		return platformLinux, true
	case "windows":
		return platformWindows, true
	case "darwin":
		return platformDarwin, true
	default:
		return 0, false
	}
}

// platformNames renders a command's platforms for a message.
func platformNames(platforms []Platform) []string {
	names := make([]string, len(platforms))
	for i, platform := range platforms {
		names[i] = platform.String()
	}

	return names
}

// String implements fmt.Stringer.
func (d DomainID) String() string {
	switch d {
	case domainGo:
		return "go"
	case domainFleet:
		return "fleet"
	case domainSim:
		return "sim"
	case domainGen:
		return "gen"
	case domainFrontend:
		return "frontend"
	case domainBackend:
		return "backend"
	case domainConfig:
		return "config"
	case domainWorkflow:
		return "workflow"
	case domainCLI:
		return "cli"
	case domainAnnotator:
		return "annotator"
	case domainHailo:
		return "hailo"
	case domainSimgen:
		return "simgen"
	case domainDocs:
		return "docs"
	case domainShared:
		return "shared"
	case domainOpenAPI:
		return "openapi"
	case domainProto:
		return "proto"
	case domainDocker:
		return "docker"
	case domainModels:
		return "models"
	case domainRobot:
		return "robot"
	case domainRPI:
		return "rpi"
	case domainInstall:
		return "install"
	case domainInit:
		return "init"
	case domainTest:
		return "test"
	case domainLint:
		return "lint"
	case domainClean:
		return "clean"
	default:
		return "unset"
	}
}

// fixVariant is the --fix switch of a lint command: the same linter, fixing.
func fixVariant(task string) Variant {
	return Variant{Flag: "fix", Task: task, Usage: "fix what can be fixed"}
}
