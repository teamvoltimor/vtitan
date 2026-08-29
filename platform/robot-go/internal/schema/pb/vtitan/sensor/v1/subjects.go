package sensorv1

// ImuSubject/ScanSubject are the NATS subjects declared in
// imu.proto/scan.proto's own docstrings -- named here, next to the
// generated message types, so every publisher/subscriber pair references
// one source of truth instead of each cmd/* binary retyping the same
// literal.
const (
	ImuSubject  = "vtitan.sensor.v1.imu"
	ScanSubject = "vtitan.sensor.v1.scan"
)
