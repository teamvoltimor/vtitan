package natsgw

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

// TestScanToLidarScan_ExpandsAngles checks that a Scan's angle_min/increment
// are expanded into a per-ray angle slice, and ranges are copied as-is.
func TestScanToLidarScan_ExpandsAngles(t *testing.T) {
	t.Parallel()

	scan := &sensorv1.Scan{
		AngleMin:       -math.Pi,
		AngleIncrement: math.Pi / 2,
		Ranges:         []float32{1.0, 2.0, 3.0, 4.0},
	}
	got := scanToLidarScan(scan)
	if len(got.RangesM) != 4 || len(got.AnglesRad) != 4 {
		t.Fatalf(
			"len(RangesM)=%d len(AnglesRad)=%d, want 4 and 4",
			len(got.RangesM),
			len(got.AnglesRad),
		)
	}
	wantAngles := []float64{-math.Pi, -math.Pi / 2, 0, math.Pi / 2}
	for i, want := range wantAngles {
		if math.Abs(got.AnglesRad[i]-want) > 1e-6 {
			t.Errorf("AnglesRad[%d] = %v, want %v", i, got.AnglesRad[i], want)
		}
	}
	for i, r := range scan.GetRanges() {
		if got.RangesM[i] != float64(r) {
			t.Errorf("RangesM[%d] = %v, want %v", i, got.RangesM[i], r)
		}
	}
}

// TestScanToLidarScan_RetainsZeros checks that a zero scan yields a zero-length
// LidarScan rather than a nil-vs-empty mismatch the navigator must handle.
func TestScanToLidarScan_RetainsZeros(t *testing.T) {
	t.Parallel()

	got := scanToLidarScan(&sensorv1.Scan{})
	if len(got.RangesM) != 0 || len(got.AnglesRad) != 0 {
		t.Errorf(
			"empty scan -> ranges=%d angles=%d, want 0 and 0",
			len(got.RangesM),
			len(got.AnglesRad),
		)
	}
}

// TestImuYawRad_Identity checks a known quaternion (identity orientation) yields
// zero yaw, and a +90 deg about Z yields +pi/2 (left-positive convention).
func TestImuYawRad_Identity(t *testing.T) {
	t.Parallel()

	zero, ok := imuYawRad(&sensorv1.Imu{})
	if ok {
		t.Errorf("imuYawRad(nil orientation) ok = true, want false")
	}
	if zero != 0 {
		t.Errorf("imuYawRad(nil orientation) = %v, want 0", zero)
	}

	identity := &sensorv1.Imu{Orientation: &sensorv1.Quaternion{W: 1, X: 0, Y: 0, Z: 0}}
	yawIdentity, okIdentity := imuYawRad(identity)
	if !okIdentity || math.Abs(yawIdentity) > 1e-9 {
		t.Errorf("imuYawRad(identity) = (%v, %v), want (0, true)", yawIdentity, okIdentity)
	}

	// +90 deg about Z: quaternion (cos45, 0, 0, sin45).
	half := math.Sqrt(2) / 2
	q90 := &sensorv1.Imu{Orientation: &sensorv1.Quaternion{W: half, X: 0, Y: 0, Z: half}}
	yaw, ok := imuYawRad(q90)
	if !ok {
		t.Fatal("imuYawRad(q90) ok = false, want true")
	}
	if math.Abs(yaw-math.Pi/2) > 1e-9 {
		t.Errorf("imuYawRad(+90deg Z) = %v, want %v", yaw, math.Pi/2)
	}
}

// TestPublishDrive_SteeringSign checks SteeringNorm maps + = left to a positive
// (left) AckermannCmd.steering_angle, and is clamped to [-1, 1].
func TestPublishDrive_SteeringSign(t *testing.T) {
	t.Parallel()

	g := &Gateway{}
	cases := []struct {
		name string
		norm float64
		want float64
	}{
		{"full left", 1.0, maxSteeringWheelAngleRad},
		{"full right", -1.0, -maxSteeringWheelAngleRad},
		{"straight", 0.0, 0.0},
		{"over-range clamps", 2.0, maxSteeringWheelAngleRad},
		{"under-range clamps", -2.0, -maxSteeringWheelAngleRad},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			cmd := g.driveCommand(controllers.DriveCommand{SpeedMPS: 0.3, SteeringNorm: tc.norm})
			if math.Abs(float64(cmd.GetSteeringAngle())-tc.want) > 1e-3 {
				t.Errorf("steering_angle = %v, want %v", cmd.GetSteeringAngle(), tc.want)
			}
			if math.Abs(float64(cmd.GetSpeed())-0.3) > 1e-3 {
				t.Errorf("speed = %v, want 0.3", cmd.GetSpeed())
			}
		})
	}
}
