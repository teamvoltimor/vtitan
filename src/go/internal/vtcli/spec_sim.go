package vtcli

// simSpec wraps the sim:* tasks (other/tasks/platform.yml). `sim view` is the
// one to reach for: the headless sim and RViz together. Its two halves stay
// reachable under `view parts` for running them in separate terminals.
var simSpec = []Command{
	{
		Path:        []string{"sim", "view"},
		Task:        "sim:navigate:visualize:all",
		Short:       "Watch closed-loop runs in RViz: headless sim + RViz in one command",
		Passthrough: true,
		Heavy:       true,
		Flags:       []Flag{profileFlag},
	},
	{
		Path:      []string{"sim", "view", "gazebo"},
		Task:      "sim:gz:navigate:visualize:all",
		Short:     "Same, on Gazebo physics and the real ROS2 nodes (needs setup ros-ws)",
		Heavy:     true,
		Platforms: []string{"linux"},
		Flags:     []Flag{{Name: flagSDF, Var: "SDF", Default: defaultSDF, Usage: "path to the .sdf world"}},
	},
	{
		Path:  []string{"sim", "view", "parts", "rviz"},
		Task:  "sim:navigate:rviz",
		Short: "Only RViz, pre-configured for view; start it first",
		Heavy: true,
	},
	{
		Path:        []string{"sim", "view", "parts", "scenarios"},
		Task:        "sim:navigate:visualize",
		Short:       "Only the scenarios, publishing to an RViz already running",
		Passthrough: true,
		Heavy:       true,
		Flags:       []Flag{profileFlag},
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
		Short: "Bare RViz with no saved config, for Gazebo or hardware work",
		Heavy: true,
	},
	{
		Path:  []string{"sim", "test"},
		Task:  "sim:test",
		Short: "Simulator tests",
		Flags: []Flag{
			{Name: "quick", Var: "QUICK", Kind: FlagBool, Default: "false", Usage: "fail fast with short tracebacks"},
		},
	},
	{
		Path:  []string{"sim", "test", "navigator"},
		Task:  "sim:navigate",
		Short: "Closed-loop test with the real navigator (headless pytest)",
	},
	{
		Path:  []string{"sim", "analyze"},
		Task:  "sim:analyze",
		Short: "Statistics and summaries of generated scenarios",
		Flags: []Flag{{
			Name:    "output-dir",
			Var:     "OUTPUT_DIR",
			Default: trainingDataDefault,
			Usage:   "scenario directory (override with an absolute path)",
		}},
	},
	{
		Path:     []string{"sim", "lint"},
		Task:     "sim:lint",
		Short:    "Lint the simulator",
		Variants: []Variant{fixVariant("sim:lint:fix")},
	},
}

// simExclusions are the sim:* tasks deliberately left out of the tree.
var simExclusions = map[string]string{
	"sim:rviz:navigate:visualize:all": "alias of sim:navigate:visualize:all, reachable as vt sim view",
}
