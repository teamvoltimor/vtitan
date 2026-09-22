package quadrature

import "math"

// Unit conversions every encoder consumer shares: revolutions to radians and
// revs/second to rpm. internal/node/motor and internal/node/picolink build
// JointStates from these, as this package's own conversions do; the three
// used to keep a private copy each.
const (
	// RadiansPerRevolution is one output-shaft revolution in radians.
	RadiansPerRevolution = 2 * math.Pi
	// SecondsPerMinute converts the estimator's revs/second into the RPM the
	// rest of the drivetrain speaks (motors.toml, the feedforward calibration
	// and the Python oracle all work in rpm).
	SecondsPerMinute = 60.0
)
