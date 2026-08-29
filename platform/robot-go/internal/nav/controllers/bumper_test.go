package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// There is no dedicated Python oracle test file for bumper.py: its two
// functions (bumper_gap_ahead/bumper_gap_behind) are only exercised
// indirectly through CollisionAvoidanceController.assess_risk's contact_dist
// boundary tests in test_collision_avoidance_controller.py (see
// collision_avoidance_test.go's TestForwardPathRisk-derived tests). These
// pin the simple subtraction contract each function's own doc comment
// describes directly, since that arithmetic has no other single point of
// coverage.

// TestBumperGapAhead_SubtractsTheFrontOffset matches bumper_gap_ahead's
// contract: the gap from the front bumper is the raw range minus the
// sensor's forward offset from it.
func TestBumperGapAhead_SubtractsTheFrontOffset(t *testing.T) {
	t.Parallel()

	got := controllers.BumperGapAhead(0.30, 0.0278)
	want := 0.30 - 0.0278
	if math.Abs(got-want) > 1e-12 {
		t.Errorf("BumperGapAhead(0.30, 0.0278) = %v, want %v", got, want)
	}
}

// TestBumperGapBehind_SubtractsTheRearOffset matches bumper_gap_behind's
// contract, using the much larger rear offset the sensor's front mount
// implies (see bumper.go's doc comment: ~0.272m vs ~0.028m front).
func TestBumperGapBehind_SubtractsTheRearOffset(t *testing.T) {
	t.Parallel()

	got := controllers.BumperGapBehind(0.30, 0.2722)
	want := 0.30 - 0.2722
	if math.Abs(got-want) > 1e-12 {
		t.Errorf("BumperGapBehind(0.30, 0.2722) = %v, want %v", got, want)
	}
}

// TestBumperGapBehind_UsesADifferentOffsetThanAhead pins the reason these
// are two separate functions rather than one parameterized helper: a rear
// reading right at the rear bumper (range == LidarToRearBumperM) must report
// zero gap, which bumper_gap_ahead's offset would get wrong by ~0.244m (see
// bumper_gap_behind's doc comment on the reverse guard this fixes).
func TestBumperGapBehind_UsesADifferentOffsetThanAhead(t *testing.T) {
	t.Parallel()

	const lidarToRearBumperM = 0.2722
	if got := controllers.BumperGapBehind(lidarToRearBumperM, lidarToRearBumperM); got != 0.0 {
		t.Errorf("BumperGapBehind(rearOffset, rearOffset) = %v, want 0.0 (touching the rear bumper)", got)
	}
}
