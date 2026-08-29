package diag

import (
	"context"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
)

// Source is the narrow read surface Aggregator needs from whatever feeds it
// the input topics — real NATS subscriptions once internal/transport/nats
// is wired up (still a one-line stub as of this pass), a bag-replay
// harness, or a test fake. Defined here, at the point of use (go-architect
// §4: interfaces belong where they're consumed, not where implemented),
// matching internal/sim/scenario.Runner's pattern: Aggregator's logic is
// fully testable against a fake Source today, and the real implementation
// swaps in later without touching this package.
//
// Each method returns the latest received value plus whether one has been
// received at all, mirroring telemetry_bridge_node.py's `_latest_scan` /
// `_latest_imu` / `_latest_vision` caches (nil/None until the first
// callback fires).
type Source interface {
	// LatestScan returns the most recent LIDAR scan, fed by the /scan
	// subscription's callback in the Python original.
	LatestScan(ctx context.Context) (*sensorv1.Scan, bool)

	// LatestIMU returns the most recent IMU reading, fed by /imu/data.
	LatestIMU(ctx context.Context) (*sensorv1.Imu, bool)

	// LatestDetections returns the most recent vision detection batch, fed
	// by /vision/detections.
	LatestDetections(ctx context.Context) ([]Detection, bool)
}
