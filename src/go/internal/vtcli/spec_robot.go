package vtcli

// pi5HostFlag and the dir defaults mirror the scripts the robot:* dev-side
// tasks call (src/python/scripts/), which own the real defaults.
var pi5HostFlag = Flag{Name: flagPI5Host, Var: "PI5_HOST", Default: defaultPI5Host, Usage: "SSH target of the Pi 5"}

// patternArg narrows a sync task to the runs whose name starts with it.
var patternArg = Arg{Name: "pattern", Var: "PATTERN", Usage: "run name prefix, e.g. run_20260809"}

// robotSpec wraps the robot:* tasks that run on the dev machine
// (src/python/Taskfile.yml). The ones that run on a Pi are excluded below, and
// the install/build ones live under `vt setup`.
var robotSpec = []Command{
	{
		Path:  []string{"robot", "deploy"},
		Task:  "robot:deploy",
		Short: "Deploy the Python/ROS2 stack + detector to the Pi 5; go deploy ships the Go binaries",
		Heavy: true,
		Flags: []Flag{
			pi5HostFlag,
			{
				Name:  "hef",
				Var:   "HEF",
				Usage: "detector .hef to ship instead of the default; code only: vt task robot:deploy HEF=",
			},
			{
				Name:    "skip-restart",
				Var:     "SKIP_RESTART",
				Kind:    FlagBool,
				Default: "false",
				Usage:   "do not restart the service",
			},
		},
	},
	{
		Path:  []string{"robot", "launch"},
		Task:  "robot:launch",
		Short: "Launch a node set (needs setup ros-ws)",
		Heavy: true,
		Args: []Arg{{
			Name:     "set",
			Var:      "SET",
			Required: true,
			Usage:    "rpi5|rpi5-bench|rpi-zero|state-machine|telemetry|simulator|lidar|joy-teleop",
		}},
	},
	{
		Path:        []string{"robot", "run"},
		Task:        "robot:run",
		Short:       "Run a single node (needs setup ros-ws)",
		Heavy:       true,
		Passthrough: true,
		Args: []Arg{{
			Name:     "node",
			Var:      "NODE",
			Required: true,
			Usage:    "motor|lidar|imu-i2c|imu-rvc|state-machine|oled|telemetry|vision|navigator",
		}},
	},
	{
		Path:  []string{"robot", "vision", "record"},
		Task:  "robot:record-vision",
		Short: "Record the annotated detection video on the Pi 5 and copy it back",
		Heavy: true,
		Flags: []Flag{
			{Name: flagSeconds, Var: "SECONDS_TO_RECORD", Default: "20", Usage: "recording length"},
			{Name: flagOut, Var: "OUT", Usage: "local output path (default: a timestamped .mp4)"},
			{
				Name:    "keep-debug",
				Var:     "KEEP_DEBUG",
				Kind:    FlagBool,
				Default: "false",
				Usage:   "leave the debug video on",
			},
			pi5HostFlag,
		},
	},
	{
		Path:  []string{"robot", "vision", "watch"},
		Task:  "robot:watch-vision",
		Short: "Print live detections, one line per frame (read-only)",
		Flags: []Flag{
			{Name: flagSeconds, Var: "SECONDS_TO_RECORD", Default: "60", Usage: "how long to watch"},
			pi5HostFlag,
		},
	},
	{
		Path:  []string{"robot", "bench-hud", "start"},
		Task:  "robot:bench-hud:start",
		Short: "Start a bench vision/HUD session (stops the race service)",
		Heavy: true,
		Flags: []Flag{pi5HostFlag},
	},
	{
		Path:  []string{"robot", "bench-hud", "stop"},
		Task:  "robot:bench-hud:stop",
		Short: "Stop the bench vision/HUD session",
		Flags: []Flag{pi5HostFlag},
	},
	{
		Path:  []string{"robot", "bench-hud", "record"},
		Task:  "robot:bench-hud:record",
		Short: "Start, record, stop and pull a bench vision/HUD video",
		Heavy: true,
		Flags: []Flag{
			{Name: flagSeconds, Var: "SECONDS_TO_RECORD", Default: "20", Usage: "recording length"},
			pi5HostFlag,
		},
	},
	{
		Path:  []string{"robot", "pull", "runs"},
		Task:  "robot:pull-runs",
		Short: "Pull recorded bags from the Pi 5 into other/data/live/runs",
		Args:  []Arg{patternArg},
		Flags: []Flag{pi5HostFlag, runsDirFlag},
	},
	{
		Path:  []string{"robot", "pull", "videos"},
		Task:  "robot:pull-videos",
		Short: "Pull per-run videos from the Pi 5 into other/data/live/videos",
		Args:  []Arg{patternArg},
		Flags: []Flag{pi5HostFlag, videosDirFlag},
	},
	{
		Path:  []string{"robot", "push", "runs"},
		Task:  "robot:push-runs",
		Short: "Push local bags back up to the Pi 5",
		Args:  []Arg{patternArg},
		Flags: []Flag{pi5HostFlag, runsDirFlag},
	},
	{
		Path:  []string{"robot", "push", "videos"},
		Task:  "robot:push-videos",
		Short: "Push local per-run videos back up to the Pi 5",
		Args:  []Arg{patternArg},
		Flags: []Flag{pi5HostFlag, videosDirFlag},
	},
	{
		Path:  []string{"robot", "test"},
		Task:  "robot:test",
		Short: "Robot tests in parallel (pixi dev env)",
		Flags: []Flag{
			{
				Name:    "scope",
				Var:     "SCOPE",
				Default: "all",
				Usage:   "all|unit|fast|hardware|navigation|vision|drivers|slow",
			},
			{Name: "test", Var: "TEST", Usage: "run one test path instead of a scope"},
			{Name: "workers", Var: "WORKERS", Default: "auto", Usage: "auto|N|serial"},
		},
	},
	{
		Path:     []string{"robot", "lint"},
		Task:     "robot:lint",
		Short:    "Lint the robot code",
		Variants: []Variant{fixVariant("robot:lint:fix")},
	},
	{Path: []string{"robot", "typecheck"}, Task: "robot:typecheck", Short: "mypy over src/ and the ros2_ws packages"},
}

// runsDirFlag and videosDirFlag override where the sync tasks read and write.
var (
	runsDirFlag = Flag{
		Name:    "runs-dir",
		Var:     "RUNS_DIR",
		Default: "other/data/live/runs",
		Usage:   "local runs directory",
	}
	videosDirFlag = Flag{
		Name:    "videos-dir",
		Var:     "VIDEOS_DIR",
		Default: "other/data/live/videos",
		Usage:   "local videos directory",
	}
)

// robotExclusions are the robot:* tasks that run on a Pi.
var robotExclusions = map[string]string{
	"robot:zero":              reasonOnBoard + " (Pi 5)",
	"robot:drive":             reasonOnBoard + " (Pi 5)",
	"robot:reset-motors":      reasonOnBoard + " (Pi 5)",
	"robot:test-motors":       reasonOnBoard + " (Pi 5)",
	"robot:calibrate-encoder": reasonOnBoard + " (Pi 5)",
	"robot:joystick-setup":    reasonOnBoard + " (Pi 5)",
	"robot:sweep-open-loop":   reasonOnBoard + " (Pi Zero)",
}

// rpiExclusions are the rpi:* tasks, all of which run on a Pi: board
// configuration through nmcli/vcgencmd, and Ansible, whose control node is the
// Pi 5. The one exception, rpi:migrate-data, is vt fleet migrate-data.
var rpiExclusions = map[string]string{
	"rpi:ansible:check":        reasonOnBoard + " (Ansible control node)",
	"rpi:ansible:setup":        reasonOnBoard + " (Ansible control node)",
	"rpi:ansible:tags":         reasonOnBoard + " (Ansible control node)",
	"rpi:provision:all":        reasonOnBoard + " (Ansible control node); from here use vt fleet provision pi5",
	"rpi:provision:pi5":        reasonOnBoard + " (Ansible control node); from here use vt fleet provision pi5",
	"rpi:provision:zero":       reasonOnBoard + " (Ansible control node)",
	"rpi:cleanup:old-services": reasonOnBoard + " (Ansible control node)",
	"rpi:camera":               reasonOnBoard,
	"rpi:diagnose":             reasonOnBoard,
	"rpi:hailo":                reasonOnBoard,
	"rpi:health":               reasonOnBoard,
	"rpi:iface-ip":             reasonOnBoard,
	"rpi:nudge":                reasonOnBoard,
	"rpi:reboot":               reasonOnBoard,
	"rpi:repos":                reasonOnBoard,
	"rpi:set-dhcp":             reasonOnBoard,
	"rpi:set-static-ip":        reasonOnBoard,
	"rpi:set-static-ip:direct": reasonOnBoard,
	"rpi:set-static-ip:router": reasonOnBoard,
	"rpi:set-wifi":             reasonOnBoard,
	"rpi:shutdown":             reasonOnBoard,
	"rpi:stack":                reasonOnBoard,
	"rpi:stat":                 reasonOnBoard,
	"rpi:update":               reasonOnBoard,
	"rpi:usb-link:repair":      reasonOnBoard + " (Pi Zero)",
	"rpi:usb-link:status":      reasonOnBoard + " (Pi 5)",
	"rpi:wifi-scan":            reasonOnBoard,
	"rpi:wifi:unlock":          reasonOnBoard + " (Pi 5)",
}
