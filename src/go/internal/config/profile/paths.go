package profile

// Top-level configs.

// DefaultCompetitionTOMLPath is where competition_specs.toml lives, relative
// to the repo root. The file's shape is the generated
// generated.CompetitionSpecs DTO.
const DefaultCompetitionTOMLPath = "src/config/competition_specs.toml"

// DefaultTrackTOMLPath is src/config/track.toml, relative to
// the repo root. Unlike robot.toml, it has no per-component profile
// overlays -- pass nil profileNames to Load. The file's shape is the
// generated generated.TrackConfig DTO.
const DefaultTrackTOMLPath = "src/config/track.toml"

// DefaultRobotTOMLPath is src/config/robot.toml, relative to the repo root.
// Its shape is the hand-written RobotConfig (see robot.go); LoadRobotConfig
// merges the active hardware profile overlays on top.
const DefaultRobotTOMLPath = "src/config/robot.toml"

// Hardware configs.

// DefaultBTS7960TOMLPath is
// src/config/hardware/motors/bts7960.toml, relative to the repo
// root. A flat top-level table, not sectioned, whose shape is the generated
// motors.HardwareMotorsBts7960 DTO.
const DefaultBTS7960TOMLPath = "src/config/hardware/motors/bts7960.toml"

// DefaultButtonGPIOTOMLPath is
// src/config/hardware/button/gpio.toml, relative to the repo
// root. The file's shape is the generated button.HardwareButtonGpio DTO, and
// its [button] section is button.HardwareButtonGpioButton; button_gpio_pin is
// a top-level key (not sectioned).
const DefaultButtonGPIOTOMLPath = "src/config/hardware/button/gpio.toml"

// DefaultButtonNodeTOMLPath is
// src/config/hardware/button/button_node.toml, relative to the
// repo root -- the ROS2 node's own poll cadence, a separate file from
// gpio.toml's driver-level debounce/threshold config, with the generated
// button.HardwareButtonButtonNode shape.
const DefaultButtonNodeTOMLPath = "src/config/hardware/button/button_node.toml"

// DefaultEncoderTOMLPath is
// src/config/hardware/motors/encoder.toml, relative to the repo
// root. Its shape is the hand-written EncoderConfig (see encoder.go).
const DefaultEncoderTOMLPath = "src/config/hardware/motors/encoder.toml"

// DefaultIMUUARTRVCTOMLPath is
// src/config/hardware/imu/bno08x_uart_rvc.toml, relative to the
// repo root.
//
// The file's shape is the generated imu.HardwareImuBno08XUartRvc DTO, whose
// [quaternion] section is imu.HardwareImuBno08XUartRvcQuaternion. Only
// DefaultPort and Baudrate have an pkg/driver/imu.Config counterpart
// (Port, BaudRate) today; PollRateHz/SerialTimeout/DataLockTimeout/Quaternion
// describe driver-internal timing and axis convention the Go port doesn't
// parameterize yet.
const DefaultIMUUARTRVCTOMLPath = "src/config/hardware/imu/bno08x_uart_rvc.toml"

// DefaultLidarLaunchTOMLPath is src/config/hardware/lidar.toml,
// relative to the repo root. A flat top-level table, not sectioned, whose
// shape is the generated hardware.HardwareLidar DTO. ScanMode and
// AngleCompensate are the sllidar_ros2 driver's own launch parameters, with no
// pkg/driver/lidar counterpart (that package reads the classic SCAN
// command directly, not via the ROS2 driver node).
const DefaultLidarLaunchTOMLPath = "src/config/hardware/lidar.toml"

// DefaultBoardTOMLPath is src/config/hardware/board.toml, relative to the
// repo root: which actuation board drives the servo and the motor, and the
// Pico's serial port. The pico2 profile overlays its kind. The file's shape
// is the generated hardware.HardwareBoard DTO.
const DefaultBoardTOMLPath = "src/config/hardware/board.toml"

// DefaultBoardSimTOMLPath is src/config/hardware/board_sim.toml, relative
// to the repo root: the virtual actuation board's loop rate and link
// emulation (pkg/boardsim), read only in simulation. The file's shape is the
// generated hardware.HardwareBoardSim DTO.
const DefaultBoardSimTOMLPath = "src/config/hardware/board_sim.toml"

// DefaultMotorsTOMLPath is
// src/config/hardware/motors/motors.toml, relative to the repo
// root. The file's shape is the generated motors.HardwareMotorsMotors DTO.
const DefaultMotorsTOMLPath = "src/config/hardware/motors/motors.toml"

// DefaultServoTOMLPath is
// src/config/hardware/motors/servo.toml, relative to the repo root. A flat
// top-level table whose shape is the generated motors.HardwareMotorsServo
// DTO. The servo profile overlays it (range_deg = 270.0 for the
// 270deg-hiwonder-35kg servo), so load it with the active profiles.
const DefaultServoTOMLPath = "src/config/hardware/motors/servo.toml"

// DefaultSSD1306TOMLPath is
// src/config/hardware/display/ssd1306.toml, relative to the
// repo root. The file's shape is the generated
// display.HardwareDisplaySsd1306 DTO, whose field is I2CAddress (a
// "0x.."-formatted string in TOML, not a plain int); see ParseI2CAddress for
// parsing it into pkg/driver/display/ssd1306's uint16 field.
const DefaultSSD1306TOMLPath = "src/config/hardware/display/ssd1306.toml"

// Navigation: blind_nav.

// DefaultCorridorEstimatorTOMLPath is
// src/config/navigation/blind_nav/corridor_estimator.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// blind_nav.NavigationBlindNavCorridorEstimator DTO.
const DefaultCorridorEstimatorTOMLPath = "src/config/navigation/blind_nav/corridor_estimator.toml"

// DefaultCorridorFollowerTOMLPath is
// src/config/navigation/blind_nav/corridor_follower.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
//
// The file's shape is the generated
// blind_nav.NavigationBlindNavCorridorFollower DTO; the generated
// BayExitSpeedMps spelling is kept, and every value including its fallbacks
// comes from the TOML.
const DefaultCorridorFollowerTOMLPath = "src/config/navigation/blind_nav/corridor_follower.toml"

// DefaultDirectionEstimatorTOMLPath is
// src/config/navigation/blind_nav/direction_estimator.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load. The file's shape is the generated
// blind_nav.NavigationBlindNavDirectionEstimator DTO; the extra generated
// fields (CornerClearanceM, GateLogPeriodTicks) are carried but still
// unconsumed by Go.
const DefaultDirectionEstimatorTOMLPath = "src/config/navigation/blind_nav/direction_estimator.toml"

// DefaultLocalizationTOMLPath is
// src/config/navigation/blind_nav/localization.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load. The file's shape is the generated
// blind_nav.NavigationBlindNavLocalization DTO; the generated MaxSpeedMps
// spelling is kept and the shipped value and its rationale are unchanged.
const DefaultLocalizationTOMLPath = "src/config/navigation/blind_nav/localization.toml"

// Navigation: escape.

// DefaultEscapeTOMLPath is
// src/config/navigation/escape/escape.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated escape.NavigationEscapeEscape DTO;
// its one untagged fallback is carried by the TOML.
const DefaultEscapeTOMLPath = "src/config/navigation/escape/escape.toml"

// Navigation: motion.

// DefaultClearanceTOMLPath is
// src/config/navigation/motion/clearance.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. Its shape is the hand-written ClearanceConfig (see clearance.go).
const DefaultClearanceTOMLPath = "src/config/navigation/motion/clearance.toml"

// DefaultControlTOMLPath is
// src/config/navigation/motion/control.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated motion.NavigationMotionControl DTO.
const DefaultControlTOMLPath = "src/config/navigation/motion/control.toml"

// DefaultPursuitTOMLPath is
// src/config/navigation/motion/pursuit.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated motion.NavigationMotionPursuit DTO.
// WallMarginSafetyM/MinLookaheadTransitionM are carried even though
// internal/nav/controllers.WaypointController does not (yet) derive a
// crosstrack budget the way CoreNavigator does.
const DefaultPursuitTOMLPath = "src/config/navigation/motion/pursuit.toml"

// Navigation: parking.

// DefaultParkingTOMLPath is
// src/config/navigation/parking/parking.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated
// parking.NavigationParkingParking DTO.
const DefaultParkingTOMLPath = "src/config/navigation/parking/parking.toml"

// Navigation: sensors.

// DefaultLidarSectorsTOMLPath is
// src/config/navigation/sensors/lidar_sectors.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// sensors.NavigationSensorsLidarSectors DTO; rear_self_detection_from_chassis
// is carried by the TOML.
const DefaultLidarSectorsTOMLPath = "src/config/navigation/sensors/lidar_sectors.toml"

// DefaultStartMeasurementTOMLPath is
// src/config/navigation/sensors/start_measurement.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// sensors.NavigationSensorsStartMeasurement DTO, the parameters for
// internal/nav/startmeasurement's scan-derived starting pose.
const DefaultStartMeasurementTOMLPath = "src/config/navigation/sensors/start_measurement.toml"

// DefaultWallHeadingTOMLPath is
// src/config/navigation/sensors/wall_heading.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load. The file's shape is the generated
// sensors.NavigationSensorsWallHeading DTO, the parameters for
// internal/nav/wallheading's absolute-heading estimate.
const DefaultWallHeadingTOMLPath = "src/config/navigation/sensors/wall_heading.toml"

// DefaultSensorTOMLPath is src/config/navigation/sensors/sensor.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load. The file's shape is the generated
// sensors.NavigationSensorsSensor DTO: the staleness gate the nav gateway
// applies to the LIDAR feed.
const DefaultSensorTOMLPath = "src/config/navigation/sensors/sensor.toml"

// Navigation: blind nav.

// DefaultStateEstimatorTOMLPath is
// src/config/navigation/blind_nav/state_estimator.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated
// blind_nav.NavigationBlindNavStateEstimator DTO: the heading-fusion gain
// the nav gateway applies to the IMU yaw.
const DefaultStateEstimatorTOMLPath = "src/config/navigation/blind_nav/state_estimator.toml"

// Navigation: signs.

// DefaultSignDiscoveryTOMLPath is where sign_discovery.toml lives, relative to
// the repo root. The file's shape is the generated
// signs.NavigationSignsSignDiscovery DTO; the generated
// MinReliableBboxHeightPx spelling and int type are kept, and every value
// comes from the TOML.
const DefaultSignDiscoveryTOMLPath = "src/config/navigation/signs/sign_discovery.toml"

// DefaultSignRouterTOMLPath is
// src/config/navigation/signs/sign_router.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load. The file's shape is the generated signs.NavigationSignsSignRouter
// DTO; the generated SignLane* spellings for the relabel and depth-consistency
// fields are kept, and every value comes from the TOML.
const DefaultSignRouterTOMLPath = "src/config/navigation/signs/sign_router.toml"

// Navigation: simulation.

// DefaultSimulationTOMLPath is
// src/config/navigation/simulation/simulation.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
//
// The file's shape is the generated simulation.NavigationSimulationSimulation
// DTO; internal/sim/collision consumes its collision-check keep-out margin and
// the axis-alignment tolerance used to swap an obstacle box's extents for a
// quarter-turned pose. The remaining simulator-only fields (start-collision
// grace window, LIDAR dropout rate, detection confidence, no-progress
// detection) belong to scoring/sensor-emulation logic not ported to Go yet.
const DefaultSimulationTOMLPath = "src/config/navigation/simulation/simulation.toml"

// Navigation: waypoint.

// DefaultWaypointsTOMLPath is
// src/config/navigation/waypoint/waypoints.toml, relative to
// the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
//
// The file's shape is the generated waypoint.NavigationWaypointWaypoints DTO;
// WideCenterBiasSide/NarrowCenterBiasSide stay strings because
// viper/mapstructure has no decode hook for trackmodel.CorridorSide's
// "inner"/"outer" TOML values, so internal/nav/waypoints.ConfigFor parses them
// itself. Every value, including its fallbacks, comes from the TOML.
const DefaultWaypointsTOMLPath = "src/config/navigation/waypoint/waypoints.toml"
