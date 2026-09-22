package natsgw

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/wallheading"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

// The heading fusion must pull the IMU yaw toward the wall-derived one a
// fraction at a time, the complementary filter estimator.py applies: a scan
// synthesized from the walls at true yaw 0, scored against a belief yaw of
// 0.3 rad, moves the estimate closer to 0 without overwriting it.
func TestCorrectHeading_PullsTowardWalls(t *testing.T) {
	t.Parallel()

	walls := trackmodel.NewTrackWalls(trackmodel.CorridorGeometry{}, -1.5, 1.5)

	const rays = 360
	angles := make([]float64, rays)
	for i := range angles {
		angles[i] = float64(i) * 2 * math.Pi / rays
	}
	ranges := walls.RaycastFan(0.5, 0.5, 0, trackmodel.NewRayFan(angles), 0.045, 12.0, nil)
	rangesF := make([]float32, len(ranges))
	for i, r := range ranges {
		rangesF[i] = float32(r)
	}
	scan := &sensorv1.Scan{AngleIncrement: 2 * math.Pi / rays, Ranges: rangesF}

	const beliefYaw = 0.3
	half := beliefYaw / 2
	g := &Gateway{
		yawCorrectionGain: 0.05,
		wallCfg:           wallheading.DefaultConfig(),
	}
	g.imu = &sensorv1.Imu{Orientation: &sensorv1.Quaternion{W: math.Cos(half), Z: math.Sin(half)}}

	yaw, ok := g.currentYawLocked()
	if !ok {
		t.Fatal("currentYawLocked ok=false")
	}
	if math.Abs(yaw-beliefYaw) > 1e-9 {
		t.Fatalf("currentYawLocked = %v, want the belief yaw %v", yaw, beliefYaw)
	}

	g.correctHeadingLocked(scan, yaw)
	corrected, ok := g.currentYawLocked()
	if !ok {
		t.Fatal("currentYawLocked ok=false after correction")
	}

	if math.Abs(corrected) >= math.Abs(yaw) {
		t.Errorf("corrected yaw = %v, want it pulled toward the wall yaw (0) from %v", corrected, yaw)
	}
	if math.Abs(corrected-yaw) > 0.05 {
		t.Errorf("one scan moved heading by %v rad, want a small gain-limited step", corrected-yaw)
	}
}
