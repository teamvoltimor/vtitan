package imu_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/imu"
)

// quaternionTolerance is the float64 comparison tolerance used throughout
// this file — the golden vectors are scipy float64 output, so anything
// looser would risk masking a real regression, and anything tighter would
// start failing on ordinary floating-point rounding noise.
const quaternionTolerance = 1e-9

// Golden vectors generated directly from the Python driver's own conversion
// call — scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch,
// yaw], degrees=True).as_quat() — not hand-derived, so a passing test here
// means byte-for-byte behavioral parity with
// platform/robot/src/hardware/imu/bno08x/utils.py, which is the actual bar
// for this migration (see go-migration-plan.md's parity-gate testing
// strategy).
func TestQuaternionFromEuler_MatchesScipyGoldenVectors(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name             string
		yaw, pitch, roll float64
		want             imu.Quaternion
	}{
		{
			name: "zero",
			yaw:  0, pitch: 0, roll: 0,
			want: imu.Quaternion{X: 0, Y: 0, Z: 0, W: 1},
		},
		{
			name: "yaw90",
			yaw:  90, pitch: 0, roll: 0,
			want: imu.Quaternion{X: 0, Y: 0, Z: 0.7071067811865476, W: 0.7071067811865476},
		},
		{
			name: "pitch45",
			yaw:  0, pitch: 45, roll: 0,
			want: imu.Quaternion{X: 0, Y: 0.3826834323650898, Z: 0, W: 0.9238795325112867},
		},
		{
			name: "roll30",
			yaw:  0, pitch: 0, roll: 30,
			want: imu.Quaternion{X: 0.25881904510252074, Y: 0, Z: 0, W: 0.9659258262890683},
		},
		{
			name: "combined",
			yaw:  45, pitch: 30, roll: 15,
			want: imu.Quaternion{
				X: 0.01828304624274653,
				Y: 0.28532013309821236,
				Z: 0.33527034435052727,
				W: 0.8976925687940069,
			},
		},
		{
			name: "negative",
			yaw:  -60, pitch: -20, roll: -10,
			want: imu.Quaternion{
				X: -0.16082608733096473,
				Y: -0.10689565208487771,
				Z: -0.5036369370577098,
				W: 0.8420558917496451,
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got := imu.QuaternionFromEuler(tt.yaw, tt.pitch, tt.roll)

			if !closeQuaternion(got, tt.want, quaternionTolerance) {
				t.Errorf("QuaternionFromEuler(%v, %v, %v) = %+v, want %+v",
					tt.yaw, tt.pitch, tt.roll, got, tt.want)
			}
		})
	}
}

func TestQuaternionFromEuler_AlwaysUnitLength(t *testing.T) {
	t.Parallel()

	for yaw := -180.0; yaw <= 180.0; yaw += 37 {
		for pitch := -80.0; pitch <= 80.0; pitch += 23 {
			for roll := -180.0; roll <= 180.0; roll += 41 {
				q := imu.QuaternionFromEuler(yaw, pitch, roll)
				magnitude := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
				if math.Abs(magnitude-1.0) > quaternionTolerance {
					t.Fatalf("QuaternionFromEuler(%v, %v, %v) magnitude = %v, want 1.0",
						yaw, pitch, roll, magnitude)
				}
			}
		}
	}
}

func closeQuaternion(a, b imu.Quaternion, tol float64) bool {
	return math.Abs(a.X-b.X) <= tol &&
		math.Abs(a.Y-b.Y) <= tol &&
		math.Abs(a.Z-b.Z) <= tol &&
		math.Abs(a.W-b.W) <= tol
}
