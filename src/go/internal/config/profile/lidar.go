package profile

// DefaultLidarLaunchTOMLPath is src/config/hardware/lidar.toml,
// relative to the repo root. A flat top-level table, not sectioned, whose
// shape is the generated hardware.HardwareLidar DTO. ScanMode and
// AngleCompensate are the sllidar_ros2 driver's own launch parameters, with no
// internal/driver/lidar counterpart (that package reads the classic SCAN
// command directly, not via the ROS2 driver node).
const DefaultLidarLaunchTOMLPath = "src/config/hardware/lidar.toml"
