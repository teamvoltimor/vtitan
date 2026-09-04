package actuationv1

// AckermannCmdSubject/MotorStatusSubject/JointStatesSubject are the NATS
// subjects declared in ackermann_cmd.proto/motor_status.proto/
// joint_states.proto's own docstrings -- named here, next to the generated
// message types, so every publisher/subscriber pair references one source
// of truth instead of each cmd/* binary retyping the same literal.
const (
	AckermannCmdSubject = "vtitan.actuation.v1.ackermann_cmd"
	MotorStatusSubject  = "vtitan.actuation.v1.motor_status"
	JointStatesSubject  = "vtitan.actuation.v1.joint_states"
)

// DriveJoint/SteeringJoint are the JointStates.name entries this stack
// publishes, matching src/hardware/motors/enums.py's DRIVE_JOINT and
// STEERING_JOINT. Consumers index by NAME, never by array position:
// JointStates carries an arbitrary set of joints in an arbitrary order, and
// assuming index 0 is the drive wheel breaks silently the moment another
// joint is added.
const (
	DriveJoint    = "drive_wheel"
	SteeringJoint = "steering"
)
