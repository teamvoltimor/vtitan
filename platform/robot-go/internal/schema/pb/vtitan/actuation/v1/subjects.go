package actuationv1

// AckermannCmdSubject/MotorStatusSubject are the NATS subjects declared in
// ackermann_cmd.proto/motor_status.proto's own docstrings -- named here,
// next to the generated message types, so every publisher/subscriber pair
// references one source of truth instead of each cmd/* binary retyping the
// same literal.
const (
	AckermannCmdSubject = "vtitan.actuation.v1.ackermann_cmd"
	MotorStatusSubject  = "vtitan.actuation.v1.motor_status"
)
