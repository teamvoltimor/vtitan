//go:build linux

package main

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/imu"
)

const quaternionTolerance = 1e-9

func TestImuMessageFor(t *testing.T) {
	t.Parallel()

	reading := imu.Reading{Yaw: 90, Pitch: 0, Roll: 0, XAccel: 1, YAccel: 2, ZAccel: 3}
	got := imuMessageFor(reading)

	want := imu.QuaternionFromEuler(reading.Yaw, reading.Pitch, reading.Roll)
	if math.Abs(float64(got.GetOrientation().GetX())-want.X) > quaternionTolerance ||
		math.Abs(float64(got.GetOrientation().GetY())-want.Y) > quaternionTolerance ||
		math.Abs(float64(got.GetOrientation().GetZ())-want.Z) > quaternionTolerance ||
		math.Abs(float64(got.GetOrientation().GetW())-want.W) > quaternionTolerance {
		t.Errorf("Orientation = %+v, want %+v", got.GetOrientation(), want)
	}

	if got.GetFrameId() != imuFrameID {
		t.Errorf("FrameId = %q, want %q", got.GetFrameId(), imuFrameID)
	}
	if got.GetOrientationCovariance()[0] != -1 {
		t.Errorf("OrientationCovariance[0] = %v, want -1 (unknown)", got.GetOrientationCovariance()[0])
	}
	if got.GetAngularVelocityCovariance()[0] != -1 {
		t.Errorf(
			"AngularVelocityCovariance[0] = %v, want -1 (not provided by RVC mode)",
			got.GetAngularVelocityCovariance()[0],
		)
	}
	av := got.GetAngularVelocity()
	if av.GetX() != 0 || av.GetY() != 0 || av.GetZ() != 0 {
		t.Errorf("AngularVelocity = %+v, want zero (not provided by RVC mode)", av)
	}

	la := got.GetLinearAcceleration()
	if float64(la.GetX()) != reading.XAccel ||
		float64(la.GetY()) != reading.YAccel ||
		float64(la.GetZ()) != reading.ZAccel {
		t.Errorf("LinearAcceleration = %+v, want (%v, %v, %v)", la, reading.XAccel, reading.YAccel, reading.ZAccel)
	}
}
