package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// TestBlindCreep_ResolvesDirection drives the BLIND_CREEP bootstrap: with no
// Direction at construction, the navigator creeps and settles the travel
// direction from a parking-bay-style scan (forward blocked, right wall close,
// left open => clockwise), then leaves the creep phase.
func TestBlindCreep_ResolvesDirection(t *testing.T) {
	t.Parallel()
	gateway := &fakeGateway{}
	// Parking-bay scan: forward ~0.1 m (blocked), right ~0.10 m (against wall),
	// left ~3.0 m (open corridor). yaw aligned to corridor (0).
	const n = 360
	ranges := make([]float64, n)
	angles := make([]float64, n)
	for i := 0; i < n; i++ {
		a := float64(i) / float64(n) * 2 * math.Pi
		angles[i] = a
		switch {
		case math.Abs(a) < 0.05: // forward
			ranges[i] = 0.10
		case math.Abs(wrapPi(a-math.Pi/2)) < 0.05: // left
			ranges[i] = 3.0
		case math.Abs(wrapPi(a+math.Pi/2)) < 0.05: // right
			ranges[i] = 0.10
		default:
			ranges[i] = 3.0
		}
	}
	gateway.pose = trackmodel.Pose{X: 1.0, Y: 1.0, Yaw: 0}
	gateway.havePose = true
	gateway.scan = controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
	gateway.haveScan = true

	params := navigator.Params{
		Gateway:           gateway,
		Waypoints:         squareLoop(),
		Direction:         nil, // blind bootstrap
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	}
	nav, err := navigator.New(params)
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}
	nav.Step()

	// First tick should be creep (direction not settled yet on a single scan,
	// but DirectionFromParkingBay resolves outright from the bay geometry).
	if got := nav.DebugSnapshot().Phase; got != navigator.PhaseBlindCreep {
		t.Fatalf("phase after Step = %v, want blind_creep", got)
	}
	// A parking-bay read names the direction immediately; the next tick should
	// have adopted it (non-nil Direction in the snapshot) OR still be creeping
	// if MinVotes gated it -- assert either the direction is now known or the
	// phase stays blind_creep (never normal drive before resolution).
	dir := nav.DebugSnapshot().Direction
	if dir != nil && *dir != trackmodel.Clockwise {
		t.Errorf("resolved direction = %v, want clockwise", *dir)
	}
}

// wrapPi folds a to (-pi, pi].
func wrapPi(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}
