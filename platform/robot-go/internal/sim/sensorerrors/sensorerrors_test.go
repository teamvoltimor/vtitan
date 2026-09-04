package sensorerrors_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/sensorerrors"
)

func newRNG() *rand.Rand { return rand.New(rand.NewPCG(1, 2)) }

// TestErrors_AnyIsFalseOnlyForAPerfectRobot pins the discriminator the
// gateway uses to decide whether to build a model at all. A false negative
// here would silently drop a configured perturbation and report the run as
// if it had been asked for.
func TestErrors_AnyIsFalseOnlyForAPerfectRobot(t *testing.T) {
	t.Parallel()

	if (sensorerrors.Errors{}).Any() {
		t.Error("the zero Errors reports Any() = true; it is a perfect robot")
	}
	for name, errs := range map[string]sensorerrors.Errors{
		"start pos":  {StartPosErrorM: 0.01},
		"yaw bias":   {YawBiasRad: 0.01},
		"drift":      {IMUDriftRadPerS: 1e-5},
		"gyro scale": {GyroScaleError: 0.001},
		"yaw noise":  {IMUNoiseRad: 0.001},
	} {
		if !errs.Any() {
			t.Errorf("%s alone reports Any() = false", name)
		}
	}
}

// TestIMUModel_NoErrorsReportsTruth checks the model is the identity when
// configured with nothing, so a caller that builds one unconditionally still
// measures an unperturbed robot.
func TestIMUModel_NoErrorsReportsTruth(t *testing.T) {
	t.Parallel()

	m := sensorerrors.NewIMUModel(sensorerrors.Errors{}, newRNG())
	const trueYaw = 0.75
	if got := m.Yaw(trueYaw, 12.0, 30.0); got != trueYaw {
		t.Errorf("Yaw with no errors = %v, want the truth %v", got, trueYaw)
	}
}

// TestIMUModel_BiasIsConstantAndSigned checks the bias neither grows with
// time nor changes sign between readings: a gyro bias IS a constant, and a
// sign that wandered would average itself out and understate the damage.
func TestIMUModel_BiasIsConstantAndSigned(t *testing.T) {
	t.Parallel()

	const bias = 0.05
	m := sensorerrors.NewIMUModel(sensorerrors.Errors{YawBiasRad: bias}, newRNG())

	first := m.Yaw(0, 0, 0) - 0
	later := m.Yaw(1.0, 500.0, 900.0) - 1.0
	if math.Abs(first-later) > 1e-12 {
		t.Errorf("bias moved between readings: %v then %v", first, later)
	}
	if math.Abs(math.Abs(first)-bias) > 1e-12 {
		t.Errorf("bias magnitude = %v, want %v", math.Abs(first), bias)
	}
}

// TestIMUModel_DriftScalesWithTimeAndScaleErrorWithRotation is the
// distinction the two fields exist to draw: drift accumulates on the CLOCK,
// gyro scale error on the COURSE TURNED. Collapsing them into one term would
// make a robot that sits still drift as much as one driving twelve corners.
func TestIMUModel_DriftScalesWithTimeAndScaleErrorWithRotation(t *testing.T) {
	t.Parallel()

	drift := sensorerrors.NewIMUModel(
		sensorerrors.Errors{IMUDriftRadPerS: 0.001}, newRNG(),
	)
	// Same rotation, ten times the elapsed seconds.
	atT1 := math.Abs(drift.Yaw(0, 1, 100))
	atT10 := math.Abs(drift.Yaw(0, 10, 100))
	if math.Abs(atT10-10*atT1) > 1e-12 {
		t.Errorf("drift at 10 s = %v, want 10x the 1 s value %v", atT10, atT1)
	}

	scale := sensorerrors.NewIMUModel(
		sensorerrors.Errors{GyroScaleError: 0.005}, newRNG(),
	)
	// Same elapsed seconds, ten times the rotation.
	atR1 := math.Abs(scale.Yaw(0, 100, 1))
	atR10 := math.Abs(scale.Yaw(0, 100, 10))
	if math.Abs(atR10-10*atR1) > 1e-12 {
		t.Errorf("scale error at 10 rad turned = %v, want 10x the 1 rad value %v", atR10, atR1)
	}
	// And the converse: neither field responds to the other's input.
	if math.Abs(drift.Yaw(0, 5, 1)-drift.Yaw(0, 5, 1000)) > 1e-12 {
		t.Error("drift responded to rotation; it should scale with time only")
	}
	if math.Abs(scale.Yaw(0, 1, 5)-scale.Yaw(0, 1000, 5)) > 1e-12 {
		t.Error("gyro scale error responded to elapsed time; it should scale with rotation only")
	}
}

// TestIMUModel_NoiseVariesPerReading checks noise is drawn per reading
// rather than fixed at construction like the three signs.
func TestIMUModel_NoiseVariesPerReading(t *testing.T) {
	t.Parallel()

	m := sensorerrors.NewIMUModel(sensorerrors.Errors{IMUNoiseRad: 0.01}, newRNG())
	first, second := m.Yaw(0, 0, 0), m.Yaw(0, 0, 0)
	if first == second {
		t.Error("two readings at identical truth returned the same yaw; noise is not per-reading")
	}
}

// TestIMUModel_IsDeterministicForASeed is what makes a perturbed sweep
// comparable to itself: the same scenario must perturb the same way every
// time it runs, or an A/B measures the RNG rather than the change.
func TestIMUModel_IsDeterministicForASeed(t *testing.T) {
	t.Parallel()

	errs := sensorerrors.Errors{
		YawBiasRad:      0.02,
		IMUDriftRadPerS: 0.001,
		GyroScaleError:  0.005,
		IMUNoiseRad:     0.01,
	}
	a := sensorerrors.NewIMUModel(errs, newRNG())
	b := sensorerrors.NewIMUModel(errs, newRNG())
	for i := range 20 {
		t1 := float64(i)
		if got, want := a.Yaw(0.1, t1, t1*2), b.Yaw(0.1, t1, t1*2); got != want {
			t.Fatalf("reading %d diverged between two models on the same seed: %v vs %v", i, got, want)
		}
	}
}
