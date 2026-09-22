package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// ApplyBelievedStart must actually move the estimator heading: the belief yaw
// is arbitrary (the IMU has no magnetometer), the measured map yaw is known,
// and the gateway adds the correction to the heading it scores scans with. The
// correction is measured minus belief, so the corrected heading equals the
// measured one. The old code passed belief minus measured, and its one caller
// passed the belief yaw as the measured one, so the offset was always zero and
// the call was skipped.
func TestApplyBelievedStart_CorrectsHeadingByMeasuredMinusBelief(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	belief := trackmodel.Pose{X: 1, Y: 1, Yaw: 0.3}
	measured := trackmodel.Pose{X: 2, Y: 2, Yaw: 1.1}

	nav.ApplyBelievedStart(measured, belief)

	if len(gateway.headingCorrections) != 1 {
		t.Fatalf("heading corrections = %v, want exactly one", gateway.headingCorrections)
	}
	const want = 0.8 // measured.Yaw - belief.Yaw
	if got := gateway.headingCorrections[0]; math.Abs(got-want) > 1e-9 {
		t.Errorf("heading correction = %v, want %v", got, want)
	}
	if got, ok := nav.BelievedYawOffset(); !ok || math.Abs(got-want) > 1e-9 {
		t.Errorf("BelievedYawOffset = (%v, %v), want (%v, true)", got, ok, want)
	}
}

// A zero offset must not call the gateway at all (the boundary the epsilon
// guards), and must still record that the believed start was applied.
func TestApplyBelievedStart_ZeroOffsetSkipsGateway(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	pose := trackmodel.Pose{X: 1, Y: 1, Yaw: 0.7}

	nav.ApplyBelievedStart(pose, pose)

	if len(gateway.headingCorrections) != 0 {
		t.Errorf("heading corrections = %v, want none for a zero offset", gateway.headingCorrections)
	}
	if _, ok := nav.BelievedYawOffset(); !ok {
		t.Error("BelievedYawOffset ok=false after ApplyBelievedStart")
	}
}
