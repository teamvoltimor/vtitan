package natsgw

import (
	"math"
	"testing"

	"google.golang.org/protobuf/types/known/timestamppb"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
)

// testWheelRadiusM is robot.toml's [wheel] radius.
const testWheelRadiusM = 0.035

func TestWheelOdometryFrom_ScalesAngleByWheelRadius(t *testing.T) {
	// One full wheel revolution is 2*pi rad, which is one circumference of
	// travel.
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{2 * math.Pi},
		Velocity: []float64{math.Pi},
	}

	got, ok := wheelOdometryFrom(joints, testWheelRadiusM)
	if !ok {
		t.Fatal("wheelOdometryFrom: ok = false, want true")
	}
	wantDistance := 2 * math.Pi * testWheelRadiusM
	if math.Abs(got.DistanceM-wantDistance) > 1e-12 {
		t.Fatalf("DistanceM = %g, want %g", got.DistanceM, wantDistance)
	}
	if math.Abs(got.SpeedMPS-math.Pi*testWheelRadiusM) > 1e-12 {
		t.Fatalf("SpeedMPS = %g, want %g", got.SpeedMPS, math.Pi*testWheelRadiusM)
	}
}

func TestWheelOdometryFrom_IndexesByNameNotPosition(t *testing.T) {
	// The drive joint is deliberately NOT first. ackermann_motor_node.py
	// publishes [drive, steering] today, but a consumer that assumed index 0
	// would break silently the moment another joint is added ahead of it --
	// which is exactly why the Python callback indexes by name.
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.SteeringJoint, actuationv1.DriveJoint},
		Position: []float64{0.5, 2 * math.Pi},
		Velocity: []float64{0.0, math.Pi},
	}

	got, ok := wheelOdometryFrom(joints, testWheelRadiusM)
	if !ok {
		t.Fatal("wheelOdometryFrom: ok = false, want true")
	}
	if math.Abs(got.DistanceM-2*math.Pi*testWheelRadiusM) > 1e-12 {
		t.Fatalf("DistanceM = %g, want the DRIVE joint's travel", got.DistanceM)
	}
}

func TestWheelOdometryFrom_RejectsMessageWithoutDriveJoint(t *testing.T) {
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.SteeringJoint},
		Position: []float64{0.5},
	}
	if _, ok := wheelOdometryFrom(joints, testWheelRadiusM); ok {
		t.Fatal("wheelOdometryFrom: ok = true for a message with no drive joint")
	}
}

func TestWheelOdometryFrom_RejectsDriveJointWithoutPosition(t *testing.T) {
	// name is longer than position -- the proto allows it ("position may be
	// shorter than name if a joint doesn't report every field"), and reading
	// past the end would panic.
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.SteeringJoint, actuationv1.DriveJoint},
		Position: []float64{0.5},
	}
	if _, ok := wheelOdometryFrom(joints, testWheelRadiusM); ok {
		t.Fatal("wheelOdometryFrom: ok = true for a drive joint with no position")
	}
}

func TestWheelOdometryFrom_MissingVelocityKeepsTravel(t *testing.T) {
	// bayexit differences DISTANCE to bound its legs; withholding the whole
	// sample over an absent rate would deny it the travel it needs.
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{2 * math.Pi},
	}

	got, ok := wheelOdometryFrom(joints, testWheelRadiusM)
	if !ok {
		t.Fatal("wheelOdometryFrom: ok = false, want true")
	}
	if got.SpeedMPS != 0 {
		t.Fatalf("SpeedMPS = %g, want 0", got.SpeedMPS)
	}
	if math.Abs(got.DistanceM-2*math.Pi*testWheelRadiusM) > 1e-12 {
		t.Fatalf("DistanceM = %g, want the travel to survive", got.DistanceM)
	}
}

func TestWheelOdometryFrom_ReverseTravelIsNegative(t *testing.T) {
	// bayexit commands REVERSE out of the pocket; an unsigned distance would
	// report that leg as forward progress and clear the pocket early.
	joints := &actuationv1.JointStates{
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{-2 * math.Pi},
		Velocity: []float64{-math.Pi},
	}

	got, ok := wheelOdometryFrom(joints, testWheelRadiusM)
	if !ok {
		t.Fatal("wheelOdometryFrom: ok = false, want true")
	}
	if got.DistanceM >= 0 || got.SpeedMPS >= 0 {
		t.Fatalf("reverse sample: DistanceM = %g, SpeedMPS = %g, want both negative",
			got.DistanceM, got.SpeedMPS)
	}
}

func TestWheelOdometryFrom_StampIsSecondsSinceEpoch(t *testing.T) {
	stamp := timestamppb.New(timestamppb.Now().AsTime())
	joints := &actuationv1.JointStates{
		Stamp:    stamp,
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{1.0},
	}

	got, ok := wheelOdometryFrom(joints, testWheelRadiusM)
	if !ok {
		t.Fatal("wheelOdometryFrom: ok = false, want true")
	}
	want := float64(stamp.GetSeconds()) + float64(stamp.GetNanos())*1e-9
	if math.Abs(got.StampS-want) > 1e-9 {
		t.Fatalf("StampS = %g, want %g", got.StampS, want)
	}
}

func TestGetWheelOdometry_FalseBeforeAnyMessage(t *testing.T) {
	// The navigator treats ok=false as a normal state and holds; it must not
	// see a zeroed sample it would mistake for "the wheel has not moved".
	g := &Gateway{wheelRadiusM: testWheelRadiusM}
	if _, ok := g.GetWheelOdometry(); ok {
		t.Fatal("GetWheelOdometry: ok = true before any joint_states arrived")
	}
}
