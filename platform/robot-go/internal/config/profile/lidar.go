package profile

// LidarLaunchConfig mirrors lidar.toml
// (src.config.launch_settings.LidarLaunchDefaults), the sllidar_ros2 node's
// default launch parameters. A flat top-level table, not sectioned.
// ScanMode and AngleCompensate are the sllidar_ros2 driver's own launch
// parameters, with no internal/driver/lidar counterpart (that package reads
// the classic SCAN command directly, not via the ROS2 driver node).
type LidarLaunchConfig struct {
	// SerialPort matches internal/driver/lidar.Config.Port.
	SerialPort string `mapstructure:"serial_port"`
	// SerialBaudrate matches internal/driver/lidar.Config.BaudRate.
	SerialBaudrate  int    `mapstructure:"serial_baudrate"`
	ScanMode        string `mapstructure:"scan_mode"`
	AngleCompensate bool   `mapstructure:"angle_compensate"`
}

// DefaultLidarLaunchTOMLPath is platform/config/launch/lidar.toml,
// relative to the repo root.
const DefaultLidarLaunchTOMLPath = "platform/config/launch/lidar.toml"
