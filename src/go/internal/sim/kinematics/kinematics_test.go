package kinematics_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// dtS is the 20 Hz control interval the simulator runs at, matching
// test_kinematics_4ws.py's _DT.
const dtS = 0.05

// steerNorm is mid-range steering: away from the saturation limit, so the
// rate limiter and the angle clamp are not what is being measured.
// Matches test_kinematics_4ws.py's _STEER_NORM.
const steerNorm = 0.5

// testWheelbaseM/testMaxSteerRad/testMaxAccelMPS2/testMaxSpeedMPS/
// testMaxSteerRateRadPerS are stand-ins for RobotSpecs' physical
// constants. The formulas under test hold for any wheelbase/steer-angle
// pair -- see test_kinematics_4ws.py's own module docstring, which derives
// the property rather than asserting it against one specific chassis --
// so literal values are chosen for numerical convenience rather than
// copied from robot.toml (this package takes them as explicit Params
// fields precisely so it does not need to read robot.toml itself).
const (
	testWheelbaseM          = 0.19
	testMaxSteerRad         = math.Pi / 6 // 30 degrees
	testMaxAccelMPS2        = 2.0
	testMaxSpeedMPS         = 1.0
	testMaxSteerRateRadPerS = 2.0
)

// idealYawGain is the zero-slip model: the geometry tests below assert
// what the GEOMETRY does, so the measured slip factor is pinned out of
// them. Leaving the shipped gain in would make a geometry regression and
// a re-measured tyre look identical here. See
// TestMeasuredDeparturesFromTheIdealModel's Python counterpart.
const idealYawGain = 1.0

func newTestKinematics(rearSteerRatio float64) *kinematics.AckermannKinematics {
	return newTestKinematicsWith(rearSteerRatio, idealYawGain, 0.0)
}

func newTestKinematicsWith(rearSteerRatio, yawGain, speedTauS float64) *kinematics.AckermannKinematics {
	return kinematics.NewAckermannKinematics(kinematics.Params{
		WheelbaseM:          testWheelbaseM,
		MaxSteerRad:         testMaxSteerRad,
		MaxSteerRateRadPerS: testMaxSteerRateRadPerS,
		MaxAccelMPS2:        testMaxAccelMPS2,
		MaxSpeedMPS:         testMaxSpeedMPS,
		RearSteerRatio:      rearSteerRatio,
		YawGain:             yawGain,
		SpeedTauS:           speedTauS,
	})
}

// TestYawGainScalesTheTurnRadiusInversely: half the yaw for the same
// speed and angle is twice the radius. The gain is the tyre slip the
// zero-slip geometry has no term for -- measured 2026-08-29, when
// replaying a real run's commands through this integrator produced 1.83x
// the yaw the IMU recorded. Matches
// TestMeasuredDeparturesFromTheIdealModel.test_yaw_gain_scales_the_turn_radius_inversely.
func TestYawGainScalesTheTurnRadiusInversely(t *testing.T) {
	t.Parallel()

	ideal := newTestKinematicsWith(1.0, 1.0, 0.0)
	slipping := newTestKinematicsWith(1.0, 0.5, 0.0)

	got := radiusOfCurvature(slipping, steerNorm)
	want := radiusOfCurvature(ideal, steerNorm) * 2

	const relTol = 1e-3
	if math.Abs(got-want) > relTol*math.Abs(want) {
		t.Errorf("half-gain radius = %v, want double the ideal radius = %v", got, want)
	}
}

// TestTheDrivetrainLagsAStepByItsTimeConstant: one tau after a step, a
// first-order lag has closed ~63% of it. Matches
// TestMeasuredDeparturesFromTheIdealModel.test_the_drivetrain_lags_a_step_by_its_time_constant.
func TestTheDrivetrainLagsAStepByItsTimeConstant(t *testing.T) {
	t.Parallel()

	const tauS = 0.4
	const target = 0.3
	const oneTauFraction = 0.632
	// A clamp low enough to bind would be measuring the clamp instead.
	kin := kinematics.NewAckermannKinematics(kinematics.Params{
		WheelbaseM:          testWheelbaseM,
		MaxSteerRad:         testMaxSteerRad,
		MaxSteerRateRadPerS: testMaxSteerRateRadPerS,
		MaxAccelMPS2:        1e6,
		MaxSpeedMPS:         testMaxSpeedMPS,
		RearSteerRatio:      1.0,
		YawGain:             idealYawGain,
		SpeedTauS:           tauS,
	})

	state := kinematics.AckermannState{}
	for range int(math.Round(tauS / dtS)) {
		state = kin.Step(state, target, 0.0, dtS)
	}

	want := target * oneTauFraction
	const relTol = 0.05
	if math.Abs(state.V-want) > relTol*want {
		t.Errorf("speed after one tau = %v, want ~%v", state.V, want)
	}
}

// TestZeroTauReproducesTheOldInstantResponse is what the retired motor's
// profile ships, so its recorded results stay comparable. Matches
// TestMeasuredDeparturesFromTheIdealModel.test_zero_tau_reproduces_the_old_instant_response.
func TestZeroTauReproducesTheOldInstantResponse(t *testing.T) {
	t.Parallel()

	kin := kinematics.NewAckermannKinematics(kinematics.Params{
		WheelbaseM:          testWheelbaseM,
		MaxSteerRad:         testMaxSteerRad,
		MaxSteerRateRadPerS: testMaxSteerRateRadPerS,
		MaxAccelMPS2:        1e6,
		MaxSpeedMPS:         testMaxSpeedMPS,
		RearSteerRatio:      1.0,
		YawGain:             idealYawGain,
		SpeedTauS:           0.0,
	})

	state := kin.Step(kinematics.AckermannState{}, 0.3, 0.0, dtS)

	const tolerance = 1e-9
	if math.Abs(state.V-0.3) > tolerance {
		t.Errorf("speed after one tick = %v, want 0.3", state.V)
	}
}

// radiusOfCurvature is the steady-state turn radius, measured by
// integrating rather than asserted, matching
// test_kinematics_4ws.py's _radius_of_curvature.
//
// Driven to steady state first: Step slews the servo toward the commanded
// angle at maxSteerRate and accelerates toward the commanded speed, so the
// first ticks are a transient that would understate the curvature.
func radiusOfCurvature(kin *kinematics.AckermannKinematics, steerNormValue float64) float64 {
	const settleTicks = 200
	const targetSpeedMPS = 0.1

	state := kinematics.AckermannState{}
	for range settleTicks {
		state = kin.Step(state, targetSpeedMPS, steerNormValue, dtS)
	}

	yawBefore, v := state.Yaw, state.V
	state = kin.Step(state, targetSpeedMPS, steerNormValue, dtS)
	yawRate := (state.Yaw - yawBefore) / dtS
	return v / yawRate
}

// TestCounterPhaseTurnsTwiceAsSharpAsFrontSteer is the headline property,
// and the one that regressed before (fixed 8eb3c38e). Matches
// TestCounterPhaseDoublesTheYawRate.test_counter_phase_turns_twice_as_sharply_as_front_steer.
func TestCounterPhaseTurnsTwiceAsSharpAsFrontSteer(t *testing.T) {
	t.Parallel()

	counter := newTestKinematics(1.0)
	frontOnly := newTestKinematics(0.0)

	got := radiusOfCurvature(counter, steerNorm)
	want := radiusOfCurvature(frontOnly, steerNorm) / 2

	const relTol = 1e-3
	if math.Abs(got-want) > relTol*math.Abs(want) {
		t.Errorf("counter-phase radius = %v, want front-steer radius/2 = %v", got, want)
	}
}

// TestTurnRadiusFollowsTheEffectiveWheelbase: radius = L_eff / tan(steer),
// with L_eff = wheelbase / (1 + rear_ratio). Matches
// TestCounterPhaseDoublesTheYawRate.test_turn_radius_follows_the_effective_wheelbase.
func TestTurnRadiusFollowsTheEffectiveWheelbase(t *testing.T) {
	t.Parallel()

	for _, ratio := range []float64{0.0, 0.5, 1.0} {
		kin := newTestKinematics(ratio)
		expectedLEff := testWheelbaseM / (1.0 + ratio)
		steer := steerNorm * testMaxSteerRad

		got := radiusOfCurvature(kin, steerNorm)
		want := expectedLEff / math.Tan(steer)

		const relTol = 1e-3
		if math.Abs(got-want) > relTol*math.Abs(want) {
			t.Errorf("rear_steer_ratio=%v: radius = %v, want %v", ratio, got, want)
		}
	}
}

// TestTheAxlesSteerInOppositeDirections matches
// TestWheelPosesShowTheCounterPhase.test_the_axles_steer_in_opposite_directions.
func TestTheAxlesSteerInOppositeDirections(t *testing.T) {
	t.Parallel()

	poses := kinematics.WheelPoses(math.Pi/9, testWheelbaseM, 0.1675, 1.0) // 20 degrees

	const tolerance = 1e-9
	want20Deg := math.Pi / 9
	if math.Abs(poses.FrontLeft.Steer-want20Deg) > tolerance {
		t.Errorf("FrontLeft.Steer = %v, want %v", poses.FrontLeft.Steer, want20Deg)
	}
	if math.Abs(poses.RearLeft.Steer-(-want20Deg)) > tolerance {
		t.Errorf("RearLeft.Steer = %v, want %v", poses.RearLeft.Steer, -want20Deg)
	}
	if poses.FrontRight.Steer != poses.FrontLeft.Steer {
		t.Error("FrontRight.Steer != FrontLeft.Steer: one servo per axle")
	}
	if poses.RearRight.Steer != poses.RearLeft.Steer {
		t.Error("RearRight.Steer != RearLeft.Steer: one servo per axle")
	}
}

// TestTheRearAngleScalesWithTheRatio matches
// TestWheelPosesShowTheCounterPhase.test_the_rear_angle_scales_with_the_ratio.
func TestTheRearAngleScalesWithTheRatio(t *testing.T) {
	t.Parallel()

	steer := math.Pi / 9 // 20 degrees
	const tolerance = 1e-9
	for _, ratio := range []float64{0.0, 0.5, 1.0} {
		poses := kinematics.WheelPoses(steer, testWheelbaseM, 0.1675, ratio)
		want := -steer * ratio
		if math.Abs(poses.RearLeft.Steer-want) > tolerance {
			t.Errorf("ratio=%v: RearLeft.Steer = %v, want %v", ratio, poses.RearLeft.Steer, want)
		}
	}
}

// TestTheWheelsSitOnTheMeasuredAxleGeometry matches
// TestWheelPosesShowTheCounterPhase.test_the_wheels_sit_on_the_measured_axle_geometry.
func TestTheWheelsSitOnTheMeasuredAxleGeometry(t *testing.T) {
	t.Parallel()

	const trackWidthM = 0.1675
	poses := kinematics.WheelPoses(0.0, testWheelbaseM, trackWidthM, 1.0)

	const tolerance = 1e-9
	checks := []struct {
		name string
		got  float64
		want float64
	}{
		{"FrontLeft.X", poses.FrontLeft.X, testWheelbaseM / 2},
		{"RearLeft.X", poses.RearLeft.X, -testWheelbaseM / 2},
		{"FrontLeft.Y", poses.FrontLeft.Y, trackWidthM / 2},
		{"FrontRight.Y", poses.FrontRight.Y, -trackWidthM / 2},
		{"RearRight.Y", poses.RearRight.Y, -trackWidthM / 2},
	}
	for _, c := range checks {
		if math.Abs(c.got-c.want) > tolerance {
			t.Errorf("%s = %v, want %v", c.name, c.got, c.want)
		}
	}

	for _, w := range []kinematics.WheelPose{poses.FrontLeft, poses.FrontRight, poses.RearLeft, poses.RearRight} {
		if w.Steer != 0.0 {
			t.Errorf("%s.Steer = %v, want 0 (steer=0 commanded)", w.Name, w.Steer)
		}
	}
}

// TestTheNamesMatchTheURDFLinks matches
// TestWheelPosesShowTheCounterPhase.test_the_names_match_the_urdf_links.
func TestTheNamesMatchTheURDFLinks(t *testing.T) {
	t.Parallel()

	poses := kinematics.WheelPoses(0.0, testWheelbaseM, 0.1675, 1.0)
	got := []string{poses.FrontLeft.Name, poses.FrontRight.Name, poses.RearLeft.Name, poses.RearRight.Name}
	want := []string{"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"}

	for i := range want {
		if got[i] != want[i] {
			t.Errorf("names[%d] = %q, want %q", i, got[i], want[i])
		}
	}
}
