package encoder

// Odometry is one full sample from the encoder, mirroring Python's
// motors.base.DriveOdometry. All four fields describe the same instant and
// share the count's sign correction (Config.Invert), so they can never
// disagree about which way the wheel turned.
type Odometry struct {
	// Counts is the raw signed quadrature count in the COMMAND frame.
	Counts int64
	// Revolutions is Counts scaled by CountsPerRev.
	Revolutions float64
	// RPM is the smoothed output-shaft speed from the last SpeedEstimator
	// window, not a fresh differentiation of Counts.
	RPM float64
	// DistanceM is the linear wheel travel, Revolutions * pi * diameter.
	// This is PATH LENGTH, not displacement from the start: it is what a
	// wheel encoder measures and what internal/nav/bayexit differences to
	// decide the chassis has cleared its pocket.
	DistanceM float64
}
