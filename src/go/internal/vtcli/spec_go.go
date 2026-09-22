package vtcli

// goSpec wraps the go:* tasks (src/go/Taskfile.yml). The hardware tests sit
// under `go hw`: build them, run one on a Pi, stop the services it needs.
var goSpec = []Command{
	{
		Path:  []string{"go", "build", "static"},
		Task:  "go:build:static",
		Short: "Build the robot binaries (linux/arm64, CGO off)",
		Flags: []Flag{{Name: flagOut, Var: "OUT", Default: "dist", Usage: "output directory"}},
	},
	{
		Path:      []string{"go", "build", "capture"},
		Task:      "go:build:capture",
		Short:     "Build the camera binaries with gocv/OpenCV (CGO on)",
		Platforms: []string{"linux", "windows"},
		Flags: []Flag{
			{Name: flagOut, Var: "OUT", Default: "dist", Usage: "output directory"},
			{Name: "cc", Var: "CC", Default: "aarch64-linux-gnu-gcc", Usage: "C cross-compiler"},
		},
	},
	{
		Path:  []string{"go", "deploy"},
		Task:  "go:deploy",
		Short: "Deploy the Go binaries to a Pi (/opt/vtitan-go); robot deploy ships the Python stack",
		Heavy: true,
		Flags: []Flag{
			{Name: "target-host", Var: "TARGET_HOST", Usage: "destination user@host"},
			{Name: "install-dir", Var: "INSTALL_DIR", Usage: "install directory"},
			{Name: "skip-restart", Var: "SKIP_RESTART", Usage: "do not restart services"},
		},
	},
	{Path: []string{"go", "test"}, Task: "go:test", Short: "Run the Go module tests"},
	{
		Path:  []string{"go", "tinygo-check"},
		Task:  "go:tinygo-check",
		Short: "Build pkg/portable for the Pico 2 under TinyGo",
	},
	{
		Path:  []string{"go", "hw", "build"},
		Task:  "go:test:hw",
		Short: "Cross-compile the hardware tests for linux/arm64 (builds, does not run)",
		Variants: []Variant{{
			Flag: "interactive", Task: "go:test:hw:interactive", Usage: "the interactive tests instead",
		}},
		Flags: []Flag{
			{
				Name:  flagPkg,
				Var:   "PKG",
				Usage: "button|ssd1306|imu|lidar|motor|nats; interactive: motor|imu|lidar|ssd1306",
			},
			{Name: flagOut, Var: "OUT", Usage: "output directory (default dist/hw, or dist/hw-interactive)"},
		},
	},
	{
		Path:  []string{"go", "hw", "run"},
		Task:  "go:hw:run",
		Short: "Build, ship and run an interactive hardware test on a Pi",
		Flags: []Flag{
			{Name: flagPkg, Var: "PKG", Default: "lidar", Usage: "motor|imu|lidar|ssd1306"},
			{Name: "host", Var: "HOST", Default: defaultPI5Host, Usage: "SSH alias of the Pi"},
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
}

// goExclusions are the go:* tasks deliberately left out of the tree.
var goExclusions = map[string]string{
	"go:todo":          "informational dump of the migration punch list; not a flow",
	"go:hw:stop-notes": "internal helper for go:hw:stop, not a user command",
}
