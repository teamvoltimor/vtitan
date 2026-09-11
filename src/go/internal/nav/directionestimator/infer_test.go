package directionestimator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/directionestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// scan builds a minimal scan with just the left/right bearings
// InferDirection actually reads.
func scan(left, right float64) navutil.LidarScan {
	return navutil.LidarScan{
		RangesM:   []float64{left, right},
		AnglesRad: []float64{math.Pi / 2, -math.Pi / 2},
	}
}

func TestInferDirection(t *testing.T) {
	t.Parallel()

	cfg := directionestimator.DefaultConfig()

	tests := []struct {
		name    string
		left    float64
		right   float64
		yaw     float64
		wantDir directionestimator.Direction
		wantOK  bool
	}{
		{
			name: "both walls span the corridor, symmetric: still ambiguous",
			left: 0.5, right: 0.5, yaw: 0, wantOK: false,
		},
		{
			name: "right opened wider than left: block on right, clockwise",
			left: 0.3, right: 3.0, yaw: 0, wantDir: directionestimator.Clockwise, wantOK: true,
		},
		{
			name: "left opened wider than right: block on left, counterclockwise",
			left: 3.0, right: 0.3, yaw: 0, wantDir: directionestimator.Counterclockwise, wantOK: true,
		},
		{
			name: "drifted toward inner block: nearer wall must not decide it",
			// 0.27m to the block on the left, 0.72m to the outer wall on
			// the right -- both still walls of the SAME corridor (span
			// 0.99 <= 1.25 threshold), so this must stay ambiguous, not
			// resolve toward the larger (nearer-outer-wall) reading.
			left: 0.27, right: 0.72, yaw: 0, wantOK: false,
		},
		{
			name: "both dropouts read long: rejected, not treated as open",
			left: 5.0, right: 5.0, yaw: 0, wantOK: false,
		},
		{
			// Default AlignmentToleranceRad is 25deg; 30deg exceeds it.
			name: "off-axis heading rejects an otherwise-decisive reading",
			left: 0.3, right: 3.0, yaw: 30 * math.Pi / 180, wantOK: false,
		},
		{
			name: "within alignment tolerance still resolves",
			left: 0.3, right: 3.0, yaw: 5 * math.Pi / 180, wantDir: directionestimator.Clockwise, wantOK: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			s := scan(tt.left, tt.right)
			dir, ok := directionestimator.InferDirection(s, tt.yaw, cfg)
			if ok != tt.wantOK {
				t.Fatalf("InferDirection() ok = %v, want %v", ok, tt.wantOK)
			}
			if ok && dir != tt.wantDir {
				t.Errorf("InferDirection() dir = %v, want %v", dir, tt.wantDir)
			}
		})
	}
}

func TestDirectionFromParkingBay(t *testing.T) {
	t.Parallel()

	cfg := directionestimator.DefaultConfig()

	tests := []struct {
		name    string
		forward float64
		left    float64
		right   float64
		wantDir directionestimator.Direction
		wantOK  bool
	}{
		{
			name: "boxed in bay, open to the right: clockwise",
			// Forward blocked (< MinForwardClearanceM), hard against the
			// left wall (<= BayWallClearanceM), wide open to the right
			// (> TurnClearanceM).
			forward: 0.10, left: 0.10, right: 2.0, wantDir: directionestimator.Clockwise, wantOK: true,
		},
		{
			name:    "boxed in bay, open to the left: counterclockwise",
			forward: 0.10, left: 2.0, right: 0.10, wantDir: directionestimator.Counterclockwise, wantOK: true,
		},
		{
			name:    "forward clear: not the boxed-in case",
			forward: 1.0, left: 0.10, right: 2.0, wantOK: false,
		},
		{
			name:    "neither side hard against a wall: not the boxed-in case",
			forward: 0.10, left: 0.5, right: 2.0, wantOK: false,
		},
		{
			name:    "neither side reads open: not the boxed-in case",
			forward: 0.10, left: 0.10, right: 0.15, wantOK: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			s := navutil.LidarScan{
				RangesM:   []float64{tt.forward, tt.left, tt.right},
				AnglesRad: []float64{0, math.Pi / 2, -math.Pi / 2},
			}
			dir, ok := directionestimator.DirectionFromParkingBay(s, cfg)
			if ok != tt.wantOK {
				t.Fatalf("DirectionFromParkingBay() ok = %v, want %v", ok, tt.wantOK)
			}
			if ok && dir != tt.wantDir {
				t.Errorf("DirectionFromParkingBay() dir = %v, want %v", dir, tt.wantDir)
			}
		})
	}
}
