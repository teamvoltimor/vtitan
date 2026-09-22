package vtcli

// umbrellaSpec wraps the verbs that span the repo (other/tasks/platform.yml)
// and gathers every install and one-time build under `setup`, which were
// spread over install, init:dev, sim:* and robot:*.
var umbrellaSpec = []Command{
	{Path: []string{"setup", "all"}, Task: "install", Short: "Install all dependencies (sim + robot + Go)"},
	{Path: []string{"setup", "dev"}, Task: "init:dev", Short: "One-time dev setup (install + lint)"},
	{
		Path:  []string{"setup", "sim"},
		Task:  "sim:install",
		Short: "Simulation dependencies (uv env + the shared sim pixi env)",
		Variants: []Variant{{
			Flag: "env-only", Task: "sim:init", Usage: "only initialise the Gazebo/ROS2 pixi env",
		}},
	},
	{Path: []string{"setup", "robot"}, Task: "robot:install", Short: "The robot pixi envs (default + dev)"},
	{
		Path:      []string{"setup", "ros-ws"},
		Task:      "robot:build-ws",
		Short:     "Build the ROS2 workspace with colcon",
		Platforms: []string{"linux"},
	},
	{
		Path:  []string{"setup", "lidar-driver"},
		Task:  "robot:fetch-lidar-driver",
		Short: "Clone the sllidar_ros2 driver into the workspace (before ros-ws)",
	},
	{Path: []string{"test"}, Task: "test", Short: "Run all tests (Python + Go)"},
	{
		Path:     []string{"lint"},
		Task:     "lint",
		Short:    "Run all linters (ruff, golangci-lint, ESLint, buf)",
		Variants: []Variant{fixVariant("lint:fix")},
	},
	{
		Path:  []string{"clean"},
		Task:  "clean",
		Short: "Remove generated training data",
		Variants: []Variant{
			{Flag: "cache", Task: "clean:cache", Usage: "Python caches instead"},
			{Flag: "all", Task: "clean:all", Usage: "training data, caches and build artifacts"},
		},
	},
}
