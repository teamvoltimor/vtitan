package diag

import (
	"context"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
)

// Aggregator computes a TelemetrySummary from whatever Source currently has
// cached, on demand — matching telemetry_bridge_node.py's `_publish_ui_summary`,
// which runs on a fixed-rate timer and reads its node's own `_latest_*`
// instance caches rather than blocking on any one topic. The eventual NATS
// wiring drives Summarize the same way, off its own timer, once
// internal/transport/nats exists.
type Aggregator struct {
	source Source
	config Config
}

// metersToCentimeters converts a clearance in meters to the centimeters
// TelemetrySummary reports, matching `_publish_ui_summary`'s `* 100`.
const metersToCentimeters = 100.0

// NewAggregator builds an Aggregator over source, using config to
// parameterize sector-clearance computation. Call config.Validate() first —
// NewAggregator does not validate on the caller's behalf, matching the
// project's constructor convention of failing fast at the validation call
// site rather than silently inside a constructor.
func NewAggregator(source Source, config Config) *Aggregator {
	return &Aggregator{source: source, config: config}
}

// Summarize builds one TelemetrySummary from the Source's currently cached
// values. Any input that has never arrived yet (LatestScan/LatestIMU/
// LatestDetections returning ok=false) is simply omitted from the summary —
// its fields stay at their zero value, matching `_publish_ui_summary`'s own
// front=left=right=yaw=0.0 defaults and best_detection_class_id=None default.
func (a *Aggregator) Summarize(ctx context.Context) TelemetrySummary {
	var summary TelemetrySummary

	if scan, ok := a.source.LatestScan(ctx); ok {
		a.fillLidarClearances(&summary, scan)
	}
	if imu, ok := a.source.LatestIMU(ctx); ok {
		fillGyroYaw(&summary, imu)
	}
	if detections, ok := a.source.LatestDetections(ctx); ok {
		fillBestDetection(&summary, detections)
	}

	return summary
}

// fillLidarClearances computes the front/left/right sector means (see
// sector.go) and writes them into summary in centimeters, matching
// `_lidar_clearances` -> `clearances_from_scan`. A sector with no valid
// rays reports 0, matching `front = left = right = 0.0`'s fallback.
func (a *Aggregator) fillLidarClearances(summary *TelemetrySummary, scan *sensorv1.Scan) {
	ranges := scan.GetRanges()
	// The bearing origin comes from the message rather than being assumed:
	// see sector.go's fullSweepRad comment for the half-turn error the old
	// hardcoded -pi produced against this stack's [0, 2pi) publisher.
	angleMinRad := float64(scan.GetAngleMin())

	frontM := sectorMeanM(ranges, angleMinRad, a.config, sectorQuery{
		CenterRad:           frontCenterRad,
		HalfFOVRad:          a.config.FrontHalfFOVRad,
		FilterSelfDetection: false,
	})
	leftM := sectorMeanM(ranges, angleMinRad, a.config, sectorQuery{
		CenterRad:           leftCenterRad,
		HalfFOVRad:          a.config.FrontHalfFOVRad,
		FilterSelfDetection: true,
	})
	rightM := sectorMeanM(ranges, angleMinRad, a.config, sectorQuery{
		CenterRad:           rightCenterRad,
		HalfFOVRad:          a.config.FrontHalfFOVRad,
		FilterSelfDetection: true,
	})

	summary.LidarFrontCM = frontM * metersToCentimeters
	summary.LidarLeftCM = leftM * metersToCentimeters
	summary.LidarRightCM = rightM * metersToCentimeters
}
