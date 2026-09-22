package vtcli

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
}

// Arg maps one positional CLI argument to one Task variable.
type Arg struct {
	Name     string
	Var      string
	Usage    string
	Required bool
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

// Flag names and positional names reused across the table.
const (
	flagOut     = "out"
	flagPkg     = "pkg"
	flagSSHHost = "ssh-host"
	flagSDF     = "sdf"
	argHost     = "host"

	defaultSDF = "worlds/wro_track_2026.sdf"

	// reasonPhase1 marks tasks deliberately deferred to phase 1.
	reasonPhase1 = "phase 1"
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
// matching their TaskPrefix must be either in the spec or excluded by name.
// A domain is promoted here once its exclusion list is closed.
var curatedDomains = []Domain{
	{ID: "go", Title: "go — Go module (robot)", TaskPrefix: "go:"},
	{ID: "fleet", Title: "fleet — boards over SSH", TaskPrefix: "windows:"},
	{ID: "sim", Title: "sim — simulation", TaskPrefix: "sim:"},
}

// exclusions records, per curated domain, the tasks deliberately left out of
// the typed tree and why. Adding a task to a curated domain without deciding
// anything here fails the build (see spec_test.go).
var exclusions = map[string]string{
	"go:todo":                         "informational dump of the migration punch list; not a flow",
	"go:hw:stop-notes":                "internal helper for go:hw:stop, not a user command",
	"sim:rviz:navigate:visualize:all": "alias of sim:navigate:visualize:all, reachable as vt sim navigate visualize all",
	"windows:ethernet:configure":      reasonPhase1,
	"windows:ethernet:setup-link":     reasonPhase1,
	"windows:ethernet:unlink":         reasonPhase1,
	"windows:route:add":               reasonPhase1,
	"windows:route:delete":            reasonPhase1,
	"windows:ssh:print-config":        reasonPhase1,
	"windows:ssh:setup-config":        reasonPhase1,
	"windows:stage-windscribe-deb":    reasonPhase1 + " (needs the local .deb path)",
}

// curatedSpec is the single declarative table the tree is built from. One
// entry per wrapped command; nothing here duplicates how a task runs.
var curatedSpec = []Command{
	// sim (curated).
	{
		Path:  []string{"sim", "install"},
		Task:  "sim:install",
		Short: "Install simulation dependencies (uv env + the shared sim pixi env)",
	},
	{
		Path:  []string{"sim", "init"},
		Task:  "sim:init",
		Short: "Initialise the Gazebo/ROS2 pixi environment (once after clone)",
	},
	{
		Path:  []string{"sim", "gazebo"},
		Task:  "sim:gazebo",
		Short: "Launch Gazebo with an SDF world",
		Heavy: true,
		Flags: []Flag{{Name: flagSDF, Var: "SDF", Default: defaultSDF, Usage: "path to the .sdf world"}},
	},
	{
		Path:  []string{"sim", "rviz"},
		Task:  "sim:rviz",
		Short: "Bare RViz, no saved config (Gazebo/hardware work)",
		Heavy: true,
	},
	{
		Path:  []string{"sim", "navigate"},
		Task:  "sim:navigate",
		Short: "Closed-loop simulation test with the real navigator",
	},
	{
		Path:  []string{"sim", "navigate", "rviz"},
		Task:  "sim:navigate:rviz",
		Short: "RViz pre-configured for navigate visualize",
		Heavy: true,
	},
	{
		Path:        []string{"sim", "navigate", "visualize"},
		Task:        "sim:navigate:visualize",
		Short:       "Live scenario(s) published to ROS2 (run navigate rviz first)",
		Passthrough: true,
		Heavy:       true,
		Flags:       []Flag{profileFlag},
	},
	{
		Path:        []string{"sim", "navigate", "visualize", "all"},
		Task:        "sim:navigate:visualize:all",
		Short:       "RViz + scenarios in one command",
		Passthrough: true,
		Heavy:       true,
		Flags:       []Flag{profileFlag},
	},
	{
		Path:  []string{"sim", "navigate", "visualize", "gazebo"},
		Task:  "sim:gz:navigate:visualize:all",
		Short: "Gazebo physics + ROS2 stack + RViz (Linux, needs robot:build-ws)",
		Heavy: true,
		Flags: []Flag{{Name: flagSDF, Var: "SDF", Default: defaultSDF, Usage: "path to the .sdf world"}},
	},
	{
		Path:  []string{"sim", "analyze"},
		Task:  "sim:analyze",
		Short: "Statistics and summaries of generated scenarios",
		Flags: []Flag{{
			Name:    "output-dir",
			Var:     "OUTPUT_DIR",
			Default: "./src/go/training_data",
			Usage:   "scenario directory",
		}},
	},
	{
		Path:  []string{"sim", "test"},
		Task:  "sim:test",
		Short: "Simulator tests",
		Flags: []Flag{
			{Name: "quick", Var: "QUICK", Kind: FlagBool, Default: "false", Usage: "fail fast with short tracebacks"},
		},
	},
	{Path: []string{"sim", "lint"}, Task: "sim:lint", Short: "Lint the simulator"},
	{Path: []string{"sim", "lint", "fix"}, Task: "sim:lint:fix", Short: "Auto-fix simulator lint issues"},

	// go (curated).
	{
		Path:  []string{"go", "build", "static"},
		Task:  "go:build:static",
		Short: "Build the robot binaries (linux/arm64, CGO off)",
		Flags: []Flag{{Name: flagOut, Var: "OUT", Default: "dist", Usage: "output directory"}},
	},
	{
		Path:  []string{"go", "build", "capture"},
		Task:  "go:build:capture",
		Short: "Build the camera binaries with gocv/OpenCV (CGO on)",
		Flags: []Flag{
			{Name: flagOut, Var: "OUT", Default: "dist", Usage: "output directory"},
			{Name: "cc", Var: "CC", Default: "aarch64-linux-gnu-gcc", Usage: "C cross-compiler"},
		},
	},
	{
		Path:  []string{"go", "deploy"},
		Task:  "go:deploy",
		Short: "Build and ship the binaries to a Pi (/opt/vtitan-go)",
		Flags: []Flag{
			{Name: "target-host", Var: "TARGET_HOST", Usage: "destination user@host"},
			{Name: "install-dir", Var: "INSTALL_DIR", Usage: "install directory"},
			{Name: "skip-restart", Var: "SKIP_RESTART", Usage: "do not restart services"},
		},
	},
	{Path: []string{"go", "test"}, Task: "go:test", Short: "Run the Go module tests"},
	{
		Path:  []string{"go", "test", "hw"},
		Task:  "go:test:hw",
		Short: "Cross-compile the hardware tests (linux/arm64)",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Usage: "button|ssd1306|imu|lidar|motor|nats"},
			{Name: flagOut, Var: "OUT", Default: "dist/hw", Usage: "output directory"},
		},
	},
	{
		Path:  []string{"go", "test", "hw", "interactive"},
		Task:  "go:test:hw:interactive",
		Short: "Cross-compile the interactive hardware tests",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Usage: "motor|imu|lidar|ssd1306"},
			{Name: flagOut, Var: "OUT", Default: "dist/hw-interactive", Usage: "output directory"},
		},
	},
	{
		Path:  []string{"go", "hw", "run"},
		Task:  "go:hw:run",
		Short: "Build, ship and run an interactive hardware test on a Pi",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Default: "lidar", Usage: "motor|imu|lidar|ssd1306"},
			{Name: "host", Var: "HOST", Default: "rpi-5-local", Usage: "SSH alias of the Pi"},
			{Name: "yaw-offset", Var: "YAW_OFFSET", Default: "0", Usage: "lidar mount yaw offset"},
			{Name: "inverted", Var: "INVERTED", Kind: FlagBool, Default: "true", Usage: "lidar mounted upside down"},
			{Name: "scan-mode", Var: "SCAN_MODE", Usage: "lidar scan mode"},
			{Name: "dump-scan", Var: "DUMP_SCAN", Usage: "print every valid point"},
			{Name: flagOut, Var: "OUT", Default: "dist/hw-interactive", Usage: "output directory"},
		},
	},
	{
		Path:  []string{"go", "hw", "stop"},
		Task:  "go:hw:stop",
		Short: "Stop the robot services so the Pi ports are free",
		Args:  []Arg{{Name: argHost, Var: "HOST", Required: true, Usage: "SSH alias of the Pi"}},
	},

	// fleet (curated; task prefix windows:).
	{
		Path:  []string{"fleet", "ping"},
		Task:  "windows:ping",
		Short: "Ping a board",
		Args:  []Arg{{Name: argHost, Var: "PING_HOST", Required: true, Usage: "board address"}},
	},
	{
		Path:  []string{"fleet", "ssh"},
		Task:  "windows:ssh",
		Short: "Open an SSH session on a board",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "SSH alias of the Pi"}},
	},
	{
		Path:  []string{"fleet", "run"},
		Task:  "windows:run",
		Short: "Run a command on a board over SSH",
		Args:  []Arg{{Name: argHost, Var: "SSH_HOST", Required: true, Usage: "SSH alias of the Pi"}},
		Flags: []Flag{{Name: "cmd", Var: "CMD", Required: true, Usage: "command to run"}},
	},
	{
		Path:  []string{"fleet", "set-wifi", "pi5"},
		Task:  "windows:set-wifi:pi5",
		Short: "Set the Pi 5 WiFi credentials from Windows",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "network name"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "network password"},
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
		},
	},
	{
		Path:  []string{"fleet", "set-wifi", "zero"},
		Task:  "windows:set-wifi:zero",
		Short: "Set the Pi Zero WiFi credentials, hopping through the Pi 5",
		Flags: []Flag{
			{Name: "ssid", Var: "SSID", Required: true, Usage: "network name"},
			{Name: "password", Var: "PASSWORD", Required: true, Usage: "network password"},
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
		},
	},
	{
		Path:  []string{"fleet", "provision", "pi5"},
		Task:  "windows:provision:pi5",
		Short: "Kick off Pi 5 provisioning over SSH",
		Flags: []Flag{
			{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"},
			{Name: "ip", Var: "PI5_IP", Default: "192.168.251.2", Usage: "Pi 5 address"},
			{Name: "tags", Var: "TAGS", Usage: "Ansible tags"},
			{Name: "skip-tags", Var: "SKIP_TAGS", Usage: "Ansible tags to skip"},
		},
	},
	{
		Path:  []string{"fleet", "audit", "pi5"},
		Task:  "windows:audit:pi5",
		Short: "Audit the Pi 5 provisioning",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-5-direct", Usage: "SSH alias"}},
	},
	{
		Path:  []string{"fleet", "audit", "zero"},
		Task:  "windows:audit:zero",
		Short: "Audit the Pi Zero provisioning",
		Flags: []Flag{{Name: flagSSHHost, Var: "SSH_HOST", Default: "rpi-zero-local", Usage: "SSH alias"}},
	},
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
