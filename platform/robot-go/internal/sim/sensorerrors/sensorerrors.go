// Package sensorerrors models what the simulated robot can be wrong about
// regarding ITSELF, as opposed to what blind mode withholds about the track.
//
// Ports platform/robot/src/simulation/imu_error_model.py.
//
// Blind mode already withholds the layout. This withholds the two things the
// simulator otherwise hands over for free about the robot:
//
// Where it starts. The estimator is normally seeded with the exact pose the
// body was placed at, which no operator can supply -- the robot is set down
// by hand somewhere inside a starting zone, not on a surveyed point. A
// localizer can work that off by matching scans, so it is a search problem
// rather than a fixed handicap.
//
// Which way it is pointing. IMU yaw is otherwise ground truth forever. A
// BNO085 drifts, and blind mode leans on heading harder than anything else
// does: corridorestimator attributes every width reading by heading, so yaw
// error does not merely steer badly, it can file a measurement under the
// wrong corridor.
//
// All heading error is modelled on the READING, never as a one-off seed of
// the estimator. The estimator takes yaw from the IMU on every update, so a
// seeded yaw offset would be overwritten on the first tick and measure
// nothing. That is also the physical truth: the BNO085's yaw zero is fixed
// at boot, so a chassis set down askew is wrong by that angle for the whole
// round rather than converging out of it.
package sensorerrors

import "math/rand/v2"

// Errors configures the perturbation. Every field is a MAGNITUDE; the sign
// and bearing come from the run's seeded RNG, so a scenario perturbs the
// same way every time it runs while different scenarios perturb differently.
//
// The zero value is a perfect robot, which is what every corpus number in
// this repo was measured on -- switching any of these on is an explicit A/B,
// never a silent change to what "the native runner" means.
type Errors struct {
	// StartPosErrorM is the distance between where the body is and where
	// the estimator is told it is, at a random bearing.
	StartPosErrorM float64
	// YawBiasRad is a constant offset between the IMU's yaw zero and the
	// world frame -- the chassis set down askew, or the IMU zeroed askew.
	// Never corrected, because nothing else observes absolute heading.
	YawBiasRad float64
	// IMUDriftRadPerS is yaw drift accumulated over elapsed time, at a
	// random sign. The BNO085's quoted figure is 0.5 deg/min.
	IMUDriftRadPerS float64
	// GyroScaleError is the fractional error in how much rotation the gyro
	// reports, e.g. 0.005 for 0.5%. It accumulates per DEGREE TURNED rather
	// than per second, which is why it is separate from drift: three laps
	// is twelve 90-degree corners, so the robot banks over 1080 degrees of
	// deliberate rotation and the error scales with the course rather than
	// the clock. A robot vacuum wanders and largely cancels this out; this
	// one does not.
	GyroScaleError float64
	// IMUNoiseRad is per-reading Gaussian yaw noise.
	IMUNoiseRad float64
}

// Any reports whether this configures any perturbation at all, matching
// SensorErrors.any_error.
func (e Errors) Any() bool {
	return e.StartPosErrorM != 0 ||
		e.YawBiasRad != 0 ||
		e.IMUDriftRadPerS != 0 ||
		e.GyroScaleError != 0 ||
		e.IMUNoiseRad != 0
}

// IMUModel turns Errors into the yaw an IMU with those flaws would report.
//
// The bias, drift and scale-error signs are drawn ONCE at construction and
// held for the model's lifetime: a gyro bias is a constant, and a sign that
// wandered would average itself out and understate the damage.
type IMUModel struct {
	rng       *rand.Rand
	errors    Errors
	driftSign float64
	biasSign  float64
	scaleSign float64
}

// NewIMUModel draws the three fixed signs from rng and returns the model.
//
// rng should be a stream of its own, spawned from the run seed rather than
// shared with the LIDAR sampler -- drawing these from the sensor stream
// would shift the LIDAR noise sequence and silently change every existing
// result, including the unperturbed control runs.
func NewIMUModel(errors Errors, rng *rand.Rand) *IMUModel {
	return &IMUModel{
		rng:       rng,
		errors:    errors,
		driftSign: randomSign(rng),
		biasSign:  randomSign(rng),
		scaleSign: randomSign(rng),
	}
}

// randomSign draws -1 or +1 with equal probability, matching
// rng.choice([-1.0, 1.0]).
func randomSign(rng *rand.Rand) float64 {
	if rng.IntN(2) == 0 {
		return -1.0
	}
	return 1.0
}

// Yaw is the heading as the IMU reports it: truth plus accumulated bias,
// drift, scale error and noise.
//
// elapsedS is time since the run started, for IMUDriftRadPerS. rotationRad
// is the signed, UNWRAPPED rotation turned through so far, for
// GyroScaleError -- unwrapped so three laps of one-way cornering accumulate
// rather than cancel.
func (m *IMUModel) Yaw(trueYaw, elapsedS, rotationRad float64) float64 {
	yaw := trueYaw +
		m.biasSign*m.errors.YawBiasRad +
		m.driftSign*m.errors.IMUDriftRadPerS*elapsedS +
		m.scaleSign*m.errors.GyroScaleError*rotationRad
	if m.errors.IMUNoiseRad > 0 {
		yaw += m.rng.NormFloat64() * m.errors.IMUNoiseRad
	}
	return yaw
}
