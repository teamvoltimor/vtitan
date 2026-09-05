// Package simconfig mirrors the subset of platform/shared/src/shared/config/
// that the generation pipeline needs. Robot* constants are generated from
// platform/config/robot.toml (see robot_constants.gen.go and
// internal/robotconfig) and must not be hand-edited; the rest of this file's
// values are still hand-maintained and must stay in sync with the Python source.
package simconfig

import "math"

// File and directory permission constants.
const (
	DirPermissions  = 0o750
	FilePermissions = 0o644
)

// Mat geometry — track, wall, corridor, traffic sign, parking and starting
// zone constants — now lives in track_constants.gen.go, generated from
// platform/config/track.toml.
//
// Robot chassis, Ackermann, wheel, LIDAR-mount, and camera-mount constants now live in
// robot_constants.gen.go, generated from platform/config/robot.toml.

// Camera sensor (Raspberry Pi Camera Module 3 Wide).
const (
	CameraHFOV       = 1.7802 // 102° in radians
	CameraWidth      = 1536
	CameraHeight     = 864
	CameraUpdateRate = 30.0
	CameraNearClip   = 0.05
	CameraFarClip    = 10.0
)

// IMU sensor (Adafruit BNO085).
const (
	ImuUpdateRate = 100.0
	ImuGyroNoise  = 0.054  // rad/s
	ImuAccelNoise = 0.3    // m/s²
	ImuMass       = 0.0025 // kg
)

var (
	// ImuSize is the BNO085 board footprint [W, D, H] in meters.
	ImuSize = [3]float64{0.0256, 0.0227, 0.0046}
)

// LIDAR (Slamtec C1).
const (
	LidarMinRange    = 0.05
	LidarSimMinRange = 0.01
	LidarMaxRange    = 12.0
	LidarSamples     = 500
	LidarUpdateRate  = 10.0 // Hz
	LidarNoiseStddev = 0.03
	LidarMinAngle    = -math.Pi // full 360° clockwise bound
	LidarMaxAngle    = math.Pi  // full 360° counterclockwise bound
)

// File path conventions.
const (
	ScenarioPrefix  = "scenario_"
	MetadataSuffix  = "_metadata.json"
	PreviewSuffix   = "_preview.svg"
	BaseWorldPath   = "worlds/wro_track_2026.sdf"
	FolderScenarios = "scenarios"
)

// LightingSpec defines the randomization ranges for one lighting preset.
type LightingSpec struct {
	IntensityMin     float64
	IntensityMax     float64
	AmbientMin       float64
	AmbientMax       float64
	DirXMin, DirXMax float64
	DirYMin, DirYMax float64
	DirZ             float64
	CastShadows      bool
}

// LightingSpecs is the table-driven set of all six lighting presets.
// Keys match the LightingScenario string constants.
var LightingSpecs = map[LightingScenario]LightingSpec{
	LightingDirectSunlight: {0.9, 1.0, 0.3, 0.4, -0.7, -0.3, -0.7, -0.3, -1.0, true},
	LightingCloudy:         {0.6, 0.75, 0.5, 0.6, -0.5, -0.5, -0.5, -0.5, -1.0, true},
	LightingIndoorBright:   {0.7, 0.85, 0.6, 0.7, 0.0, 0.0, 0.0, 0.0, -1.0, false},
	LightingIndoorDim:      {0.5, 0.65, 0.4, 0.5, 0.0, 0.0, 0.0, 0.0, -1.0, false},
	LightingEvening:        {0.6, 0.8, 0.3, 0.4, -0.9, -0.7, -0.5, 0.5, -0.3, true},
	LightingMixed:          {0.7, 0.9, 0.5, 0.65, -0.6, -0.4, -0.6, -0.4, -1.0, true},
}

// Z-layer positions for visual layering in the SDF world.
const (
	ZTrackFloor         = 0.00001
	ZGridLines          = 0.0001
	ZStartingZoneBase   = 0.0002
	ZDirectionIndicator = 0.004
	ZTrafficSign        = 0.05
	ZParkingBlock       = 0.05
)

// Width type string identifiers.
const (
	WidthTypeNarrow = "narrow"
	WidthTypeWide   = "wide"
	WidthTypeFixed  = "fixed"
)

// Color name identifiers.
const (
	ColorNameRed   = "red"
	ColorNameGreen = "green"
)

// Model name constants for Gazebo SDF elements.
const (
	ModelInteriorWallNorth  = "interior_wall_north"
	ModelInteriorWallSouth  = "interior_wall_south"
	ModelInteriorWallEast   = "interior_wall_east"
	ModelInteriorWallWest   = "interior_wall_west"
	ModelExteriorWallNorth  = "exterior_wall_north"
	ModelExteriorWallSouth  = "exterior_wall_south"
	ModelExteriorWallEast   = "exterior_wall_east"
	ModelExteriorWallWest   = "exterior_wall_west"
	ModelGround             = "ground"
	ModelCentralLogo        = "central_logo"
	ModelRedSignPrefix      = "red_sign_"
	ModelGreenSignPrefix    = "green_sign_"
	ModelParkingBlock1      = "parking_limitation_1"
	ModelParkingBlock2      = "parking_limitation_2"
	ModelStartingZonePrefix = "starting_zone_"
	// ModelStartingZonePlaceholder is the static south placeholder in the base template.
	ModelStartingZonePlaceholder = ModelStartingZonePrefix + "south"
	ModelSunLight                = "sun"
	ModelAmbientLight            = "ambient_light"
	ModelRobotName               = "wro_robot"
	ModelDebugCameraName         = "overhead_debug_camera"
)

// Corner marker model names (WRO field corner decorations).
const (
	ModelCornerNEBlue   = "corner_ne_blue"
	ModelCornerNEOrange = "corner_ne_orange"
	ModelCornerSEBlue   = "corner_se_blue"
	ModelCornerSEOrange = "corner_se_orange"
	ModelCornerSWBlue   = "corner_sw_blue"
	ModelCornerSWOrange = "corner_sw_orange"
	ModelCornerNWBlue   = "corner_nw_blue"
	ModelCornerNWOrange = "corner_nw_orange"
)

// Model name prefixes for programmatically generated families of models.
const (
	ModelGridLinePrefix       = "grid_line_"
	ModelCorridorSubdivPrefix = "corridor_"
)

// Exterior wall collision ODE contact parameters.
const (
	WallContactKp       = 1e8
	WallContactKd       = 1000.0
	WallContactMaxVel   = 0.0
	WallContactMinDepth = 0.0
)

// Gazebo system plugin identifiers.
const (
	PluginSensorsFilename   = "gz-sim-sensors-system"
	PluginSensorsName       = "gz::sim::systems::Sensors"
	PluginPhysicsFilename   = "gz-sim-physics-system"
	PluginPhysicsName       = "gz::sim::systems::Physics"
	PluginAckermannFilename = "gz-sim-ackermann-steering-system"
	PluginAckermannName     = "gz::sim::systems::AckermannSteering"
	RenderEngineOgre2       = "ogre2"
)

// Robot ROS2 topics and TF frame IDs.
const (
	RobotCmdVelTopic   = "/wro_robot/cmd_vel"
	RobotOdomTopic     = "/wro_robot/odom"
	RobotOdomFrequency = "50" // Hz, published as string for SDF
	RobotOdomFrameID   = "odom"
	RobotBaseFrameID   = "base_link"
	RobotCameraTopic   = "/wro_robot/camera"
	RobotLidarTopic    = "lidar"
	RobotImuTopic      = "imu"
)

// Robot physics contact/friction parameters (ODE solver).
const (
	RobotWheelFrictionMu      = 1.0
	RobotWheelContactKp       = 1e7
	RobotWheelContactKd       = 500.0
	RobotWheelContactMaxVel   = 0.01
	RobotWheelContactMinDepth = 0.001
	RobotWheelJointFriction   = 0.01
	RobotWheelJointDamping    = 0.01
)

// Robot joint limit/effort/velocity parameters.
const (
	RobotRearJointEffort     = 10.0
	RobotRearJointVelocity   = 100.0
	RobotFrontWheelEffort    = 0.0
	RobotFrontWheelVelocity  = 100.0
	RobotSteeringEffort      = 5.0
	RobotSteeringVelocity    = 10.0
	RobotSteeringLinkMass    = 0.001
	RobotSteeringLinkInertia = 0.00001 // near-zero inertia for hinge links
	RobotCameraLinkMass      = 0.01
	RobotLidarLinkMass       = 0.05
)

// Robot sensor placement offsets (meters). LIDAR/IMU mount z-offsets moved to
// robot_constants.gen.go (RobotLidarMountZOffset/RobotImuMountZOffset), generated from
// platform/config/robot.toml, so Go/xacro/Python share one source instead of three
// hand-maintained copies that could drift.
const (
	RobotFrontIndicatorOffsetX = 0.02  // indicator recessed 20 mm from front face
	RobotFrontIndicatorOffsetZ = 0.003 // indicator floats 3 mm above chassis top
)

// Robot sensor/camera format constants.
const (
	CameraImageFormat    = "R8G8B8"
	LidarHorizResolution = 1.0
	LidarRangeResolution = 0.01
	NoiseTypeGaussian    = "gaussian"
)

var (
	// RobotChassisColor is the blue color for the robot chassis.
	RobotChassisColor = RGB{0.0, 0.0, 0.8}

	// RobotFrontIndicatorColor is the red color for the robot front-facing indicator.
	RobotFrontIndicatorColor = RGB{1.0, 0.0, 0.0}

	// RobotWheelColor is the dark grey color for robot wheels.
	RobotWheelColor = RGB{0.1, 0.1, 0.1}

	// RobotWheelStripeColor is the yellow color for wheel position indicators.
	RobotWheelStripeColor = RGB{1.0, 1.0, 0.0}

	// RobotImuColor is the green color for the IMU sensor visual.
	RobotImuColor = RGB{0.0, 0.4, 0.0}

	// RobotFrontIndicatorSize is the [W, D, H] of the red front indicator box.
	RobotFrontIndicatorSize = [3]float64{0.04, 0.04, 0.005}
)

// Wheel stripe geometry scale factors (applied to wheel radius).
// stripeOffset = r * StripeOffsetFactor
// stripeDims   = [r/RefRadius * StripeDim{X,Y,Z}]
const (
	StripeOffsetFactor = 0.314 // ≈ 0.1π — places stripe at quarter-turn position
	StripeRefRadius    = 0.035 // reference wheel radius used in Python original
	StripeDimX         = 0.004 // stripe box width at reference radius
	StripeDimY         = 0.050 // stripe box depth at reference radius
	StripeDimZ         = 0.008 // stripe box height at reference radius
)

// Debug overhead camera parameters (static model above track center).
const (
	DebugCameraZ          = 2.5
	DebugCameraFOV        = 1.57 // ~90°
	DebugCameraWidth      = 1280
	DebugCameraHeight     = 720
	DebugCameraNearClip   = 0.1
	DebugCameraFarClip    = 10.0
	DebugCameraUpdateRate = 30
)
const DebugCameraTopic = "camera/image_raw"

// Gazebo world physics step parameters.
const (
	PhysicsMaxStepSize    = 0.001
	PhysicsRealTimeFactor = 1.0
	PhysicsUpdateRate     = 1000
)

// World default sun and ambient light parameters.
const (
	SunLightZ                  = 10.0
	AmbientLightZ              = 3.0
	AmbientLightRange          = 20.0
	AmbientLightConstantAtten  = 0.5
	AmbientLightLinearAtten    = 0.01
	AmbientLightQuadraticAtten = 0.001
)

var (
	SunDiffuseColor      = RGB{0.8, 0.8, 0.8}
	SunSpecularColor     = RGB{0.2, 0.2, 0.2}
	SunDefaultDirection  = [3]float64{-0.5, -0.5, -1.0}
	AmbientDiffuseColor  = RGB{0.5, 0.5, 0.5}
	AmbientSpecularColor = RGB{0.1, 0.1, 0.1}

	// GroundColor is the white color for the WRO mat.
	GroundColor = RGB{1.0, 1.0, 1.0}
)

// GroundFrictionMu is the friction coefficient for the ground plane.
const GroundFrictionMu = 0.8

// Track decoration visual constants.
const (
	CornerMarkerLength  = 1.156
	CornerMarkerWidth   = 0.02
	CornerMarkerHeight  = 0.001
	CentralLogoSize     = 0.8
	GridLineThickness   = 0.001
	GridLineHeight      = 0.001
	SubdivLineThickness = 0.001
	SubdivLineHeight    = 0.001
)

var (
	CornerMarkerBlueColor    = RGB{0.0, 0.2, 1.0}
	CornerMarkerOrangeColor  = RGB{1.0, 0.4, 0.0}
	GridLineColor            = RGB{0.6, 0.6, 0.6}
	CentralLogoColor         = RGB{0.9, 0.9, 0.9}
	CorridorSubdivisionColor = RGB{0.5, 0.5, 0.5}

	// StartingZonePlaceholderColor is the slightly lighter grey for the base template placeholder zone.
	StartingZonePlaceholderColor = RGB{0.7, 0.7, 0.7}
)

// Validation clearance constants.
const (
	SignSpacingFactor         = 3.0  // minimum sign-to-sign distance = SignWidth * 3
	ValidationClearanceMargin = 0.05 // extra clearance added to all proximity checks
)

// Scenario generation bounds.
const (
	ScenarioIDMin           = 1
	ScenarioIDMax           = 36
	MaxScenarioRetries      = 10
	SignAdjustmentTolerance = 0.05
	MillimetersPerMeter     = 1000
)

// Default generation parameters (CLI flags).
const (
	DefaultChallengeType = "open"
	DefaultNumScenarios  = 10
	DefaultOutputDir     = "generated"
	SeedRandom           = int64(-1) // sentinel: use random seed
)

// Default deterministic scenario parameters.
const (
	DefaultLightingIntensity = 0.95
	DefaultAmbientIntensity  = 0.35
	DefaultLightingScenario  = string(LightingDirectSunlight)
)

var (
	// DefaultSunDirection is the default direction for sun lighting in the world.
	DefaultSunDirection = [3]float64{-0.5, -0.5, -1.0}
)

// Inertia tensor component names (for robot URDF/SDF).
const (
	InertiaComponentIxx = "ixx"
	InertiaComponentIyy = "iyy"
	InertiaComponentIzz = "izz"
	InertiaComponentIxy = "ixy"
	InertiaComponentIxz = "ixz"
	InertiaComponentIyz = "iyz"
)

// Robot link names — used in SDF models, Gazebo plugins, and ROS2 TF frames.
const (
	RobotLinkRearLeftWheel   = "rear_left_wheel"
	RobotLinkRearRightWheel  = "rear_right_wheel"
	RobotLinkFrontLeftSteer  = "front_left_steering"
	RobotLinkFrontRightSteer = "front_right_steering"
	RobotLinkFrontLeftWheel  = "front_left_wheel"
	RobotLinkFrontRightWheel = "front_right_wheel"
	RobotLinkCamera          = "camera_link"
	RobotLinkLidar           = "lidar_link"
	RobotLinkImu             = "imu_link"
)

// Robot joint names — must match Ackermann plugin references and TF frame parents.
const (
	RobotJointRearLeft        = "rear_left_wheel_joint"
	RobotJointRearRight       = "rear_right_wheel_joint"
	RobotJointFrontLeftSteer  = "front_left_steering_joint"
	RobotJointFrontRightSteer = "front_right_steering_joint"
	RobotJointFrontLeftWheel  = "front_left_wheel_joint"
	RobotJointFrontRightWheel = "front_right_wheel_joint"
	RobotJointCamera          = "camera_joint"
	RobotJointLidar           = "lidar_joint"
	RobotJointImu             = "imu_joint"
)

// Parking block identifiers.
const (
	ParkingBlockIDBlock1 = "block1"
	ParkingBlockIDBlock2 = "block2"
)
