package vtcli

// simSpec wraps the sim:* tasks (other/tasks/platform.yml).
var simSpec = []Command{
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
}

// simExclusions are the sim:* tasks deliberately left out of the tree.
var simExclusions = map[string]string{
	"sim:rviz:navigate:visualize:all": "alias of sim:navigate:visualize:all, reachable as vt sim navigate visualize all",
}
