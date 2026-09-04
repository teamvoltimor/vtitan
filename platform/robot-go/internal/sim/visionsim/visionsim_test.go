package visionsim_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/visionsim"
)

// There is no Python oracle test file for vision_emulator.py (none exists
// in the source tree), so these tests pin the module's own documented
// contract directly.

func TestEmulateSignObservations_InFrameSignIsReported(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	signs := []signrouter.SignSpec{{X: 2.0, Y: 0.0, Color: signrouter.SignColorRed}}

	got := visionsim.EmulateSignObservations(signs, 0, 0, 0, cfg, nil)
	if len(got) != 1 {
		t.Fatalf("len(got) = %v, want 1", len(got))
	}
	if math.Abs(got[0].WorldXM-2.0) > 1e-9 || math.Abs(got[0].WorldYM) > 1e-9 {
		t.Errorf("observation = (%v, %v), want (2, 0)", got[0].WorldXM, got[0].WorldYM)
	}
	if got[0].Color != signrouter.SignColorRed {
		t.Errorf("Color = %v, want Red", got[0].Color)
	}
	if got[0].Confidence != cfg.DetectionConfidence {
		t.Errorf("Confidence = %v, want %v", got[0].Confidence, cfg.DetectionConfidence)
	}
}

func TestEmulateSignObservations_OutOfRangeSignIsOmitted(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	signs := []signrouter.SignSpec{{X: cfg.MaxRangeM + 1.0, Y: 0.0, Color: signrouter.SignColorRed}}

	got := visionsim.EmulateSignObservations(signs, 0, 0, 0, cfg, nil)
	if len(got) != 0 {
		t.Errorf("len(got) = %v, want 0 (beyond MaxRangeM)", len(got))
	}
}

func TestEmulateSignObservations_OutOfFOVSignIsOmitted(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	// Directly behind the robot: bearing is pi, robot yaw is 0, so
	// theta_h = pi, comfortably outside any real HFOV.
	signs := []signrouter.SignSpec{{X: -2.0, Y: 0.0, Color: signrouter.SignColorGreen}}

	got := visionsim.EmulateSignObservations(signs, 0, 0, 0, cfg, nil)
	if len(got) != 0 {
		t.Errorf("len(got) = %v, want 0 (behind the robot, outside HFOV)", len(got))
	}
}

func TestEmulateSignObservations_SignAtRobotPositionIsOmitted(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	signs := []signrouter.SignSpec{{X: 1.0, Y: 1.0, Color: signrouter.SignColorRed}}

	// distance == 0 must be excluded (bearing_to is undefined at zero
	// range), matching the Python `distance <= 0.0` guard.
	got := visionsim.EmulateSignObservations(signs, 1.0, 1.0, 0, cfg, nil)
	if len(got) != 0 {
		t.Errorf("len(got) = %v, want 0 (zero-range sign)", len(got))
	}
}

// TestEmulateSignObservations_ReprojectsThroughBelievedPose is the core
// contract distinguishing this from a ground-truth leak: with a believed
// pose OFFSET from the true one, the reported world position must be
// computed from the TRUE relative geometry (bearing/range to the sign from
// the true pose) but PLACED using the believed pose -- not simply equal to
// the sign's own true (X, Y).
func TestEmulateSignObservations_ReprojectsThroughBelievedPose(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	signs := []signrouter.SignSpec{{X: 2.0, Y: 0.0, Color: signrouter.SignColorRed}}

	// True pose at the origin facing east: the sign is 2m ahead (theta_h=0).
	// Believed pose is displaced 1m north of true -- same heading.
	believed := &visionsim.BelievedPose{X: 0.0, Y: 1.0, Yaw: 0.0}
	got := visionsim.EmulateSignObservations(signs, 0, 0, 0, cfg, believed)
	if len(got) != 1 {
		t.Fatalf("len(got) = %v, want 1", len(got))
	}
	// The relative bearing/range (0 rad, 2m) reprojected through the
	// believed pose (0, 1, yaw 0) lands at (2, 1) -- NOT the sign's true
	// (2, 0), and NOT the believed position itself.
	want := struct{ x, y float64 }{2.0, 1.0}
	if math.Abs(got[0].WorldXM-want.x) > 1e-9 || math.Abs(got[0].WorldYM-want.y) > 1e-9 {
		t.Errorf("observation = (%v, %v), want (%v, %v)", got[0].WorldXM, got[0].WorldYM, want.x, want.y)
	}
}

func TestEmulateSignObservations_MultipleSignsEachEvaluatedIndependently(t *testing.T) {
	t.Parallel()

	cfg := visionsim.DefaultConfig()
	signs := []signrouter.SignSpec{
		{X: 2.0, Y: 0.0, Color: signrouter.SignColorRed},               // in frame
		{X: -2.0, Y: 0.0, Color: signrouter.SignColorGreen},            // behind, out of frame
		{X: cfg.MaxRangeM * 2, Y: 0.0, Color: signrouter.SignColorRed}, // too far
	}

	got := visionsim.EmulateSignObservations(signs, 0, 0, 0, cfg, nil)
	if len(got) != 1 {
		t.Fatalf("len(got) = %v, want 1 (only the in-frame sign)", len(got))
	}
	if got[0].Color != signrouter.SignColorRed {
		t.Errorf("Color = %v, want Red", got[0].Color)
	}
}
