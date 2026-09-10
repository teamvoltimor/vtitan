package statev1

// RobotStateSubject/RaceMetricsSubject/SystemStatusSubject are the NATS
// subjects declared in robot_state.proto/race_metrics.proto/
// system_status.proto's own docstrings -- named here, next to the generated
// message types, so every publisher/subscriber pair references one source of
// truth instead of each cmd/* binary retyping the same literal.
const (
	RobotStateSubject   = "vtitan.state.v1.robot_state"
	RaceMetricsSubject  = "vtitan.state.v1.race_metrics"
	SystemStatusSubject = "vtitan.state.v1.system_status"
)
