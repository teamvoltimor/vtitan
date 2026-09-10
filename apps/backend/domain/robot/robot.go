// Package robot is the Robot bounded context: fleet-registered robots, their
// live status, tunable configuration, and command dispatch.
package robot

import "time"

// State is the robot's lifecycle state.
type State string

// State values.
const (
	StateBootCheck State = "BOOT_CHECK"
	StateReady     State = "READY"
	StateRacing    State = "RACING"
	StateFinished  State = "FINISHED"
	StateError     State = "ERROR"
	StateOffline   State = "OFFLINE"
)

// Robot is a fleet-registered robot.
type Robot struct {
	ID        string
	Name      string
	FleetID   string
	State     State
	CreatedAt time.Time
	UpdatedAt time.Time
}

// CreateRequest is the payload for registering a new robot.
type CreateRequest struct {
	Name    string
	FleetID string
}

// UpdateRequest is the payload for updating robot metadata. Nil fields are left unchanged.
type UpdateRequest struct {
	Name *string
}

// Pose is a 2-D ground-plane pose.
type Pose struct {
	X   float64
	Y   float64
	Yaw float64
}

// Velocity is the robot's current linear/angular velocity.
type Velocity struct {
	Linear  float64
	Angular float64
}

// Battery is the robot's current power state.
type Battery struct {
	Voltage       float64
	Current       *float64
	StateOfCharge float64
}

// SystemStatus is the robot's onboard subsystem health.
type SystemStatus struct {
	CPUTemp        *float64
	MotorTemps     []float64
	ROSNodesActive *int
	IMUReady       *bool
	LidarReady     *bool
	CameraReady    *bool
	DriveReady     *bool
}

// Status is the robot's live, real-time status.
type Status struct {
	RobotID      string
	State        State
	Pose         Pose
	Velocity     Velocity
	Battery      *Battery
	SystemStatus SystemStatus
	LapNumber    *int
	MissionName  *string
	Timestamp    time.Time
}

// Config is the robot's tunable navigation/speed configuration.
type Config struct {
	MaxLinearSpeed    *float64
	MaxAngularSpeed   *float64
	SteeringKp        *float64
	LookaheadDistance *float64
	SpeedProfiles     []string
}

// UpdateConfigRequest is the payload for a config update. Nil fields are left unchanged;
// SpeedProfile (singular), when set, is appended to Config.SpeedProfiles.
type UpdateConfigRequest struct {
	MaxLinearSpeed    *float64
	MaxAngularSpeed   *float64
	SteeringKp        *float64
	LookaheadDistance *float64
	SpeedProfile      *string
}

// CommandType is a robot control command.
type CommandType string

// CommandType values.
const (
	CommandStartRace      CommandType = "START_RACE"
	CommandStopRace       CommandType = "STOP_RACE"
	CommandPause          CommandType = "PAUSE"
	CommandResume         CommandType = "RESUME"
	CommandEmergencyStop  CommandType = "EMERGENCY_STOP"
	CommandReturnToStart  CommandType = "RETURN_TO_START"
	CommandReboot         CommandType = "REBOOT"
	CommandShutdown       CommandType = "SHUTDOWN"
	CommandSetVisionDebug CommandType = "SET_VISION_DEBUG"
	// CommandDisableCommandChannel shuts down the robot's gRPC command
	// channel. This is one-way: the backend can disable the channel, but
	// cannot remotely re-enable it, since disabling it closes the very
	// stream a re-enable command would need to travel over. Re-enabling
	// requires local access to the robot.
	CommandDisableCommandChannel CommandType = "DISABLE_COMMAND_CHANNEL"
	// CommandSetTelemetryChannel toggles the robot's gRPC telemetry
	// channel. Unlike CommandDisableCommandChannel, this is safely
	// bidirectional since it doesn't affect the command channel.
	CommandSetTelemetryChannel CommandType = "SET_TELEMETRY_CHANNEL"
)

// Command is a control command sent to a robot. Parameters is loosely typed
// at this port boundary; each CommandType defines which keys it reads:
//   - CommandStartRace: "mission_name" (string, optional)
//   - CommandSetVisionDebug: "enabled" (bool, required), "stream_fps" (uint32, optional)
//   - CommandSetTelemetryChannel: "enabled" (bool, required)
//   - all other command types currently take no parameters
type Command struct {
	Type       CommandType
	Parameters map[string]any
}

// CommandStatus is the outcome of dispatching a Command.
type CommandStatus string

// CommandStatus values.
const (
	CommandAccepted CommandStatus = "accepted"
	CommandRejected CommandStatus = "rejected"
	CommandQueued   CommandStatus = "queued"
)

// CommandResult is the response to a dispatched Command.
type CommandResult struct {
	CommandID string
	Status    CommandStatus
	Message   string
}
