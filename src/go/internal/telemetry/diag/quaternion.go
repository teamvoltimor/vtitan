package diag

import (
	"math"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
)

const (
	// radToDegFactor converts radians to degrees.
	radToDegFactor = 180.0 / math.Pi
	// quaternionDouble and quaternionUnity are the two literal coefficients
	// in the standard atan2-based yaw extraction formula (2*(...) and
	// 1-2*(...)) — hoisted to named constants rather than left as inline
	// magic numbers.
	quaternionDouble = 2.0
	quaternionUnity  = 1.0
)

// fillGyroYaw writes the yaw (degrees) extracted from imu's orientation
// quaternion into summary, matching `_publish_ui_summary`'s
// `yaw = math.degrees(self._quaternion_to_yaw(q.x, q.y, q.z, q.w))`. Leaves
// summary.GyroYawDeg at its zero value when imu carries no orientation,
// which a well-formed Imu reading always should but a fake/test Source need
// not guarantee.
func fillGyroYaw(summary *TelemetrySummary, imu *sensorv1.Imu) {
	orientation := imu.GetOrientation()
	if orientation == nil {
		return
	}
	x, y, z, w := orientation.GetX(), orientation.GetY(), orientation.GetZ(), orientation.GetW()
	summary.GyroYawDeg = yawDegFromQuaternion(x, y, z, w)
}

// yawDegFromQuaternion extracts yaw (degrees) from a unit orientation
// quaternion, matching shared.config.coordinate_transform.quaternion_to_yaw
// (src/python/shared/src/shared/config/coordinate_transform.py) — the same
// atan2-based Z-axis extraction telemetry_bridge_node.py's
// `_publish_ui_summary` calls via `_quaternion_to_yaw`, converted to
// degrees to match GyroYawDeg's unit.
func yawDegFromQuaternion(x, y, z, w float64) float64 {
	sinYawCosPitch := quaternionDouble * (w*z + x*y)
	cosYawCosPitch := quaternionUnity - quaternionDouble*(y*y+z*z)
	return math.Atan2(sinYawCosPitch, cosYawCosPitch) * radToDegFactor
}
